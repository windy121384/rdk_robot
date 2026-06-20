#!/usr/bin/env python3
"""LLM advisor for the morphing inspection robot.

The LLM explains the scene and suggests high-level actions only. It never sends
commands to STM32, and the local safety rule remains the final authority.
"""
from __future__ import annotations

import argparse
import json
import os
import re
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path
from typing import Any

from ai_decision import decide, load_perception

DEFAULT_SOURCE = "http://127.0.0.1:8091/result.json"
ALLOWED_ACTIONS = {"STOP", "SLOW", "TURN_LEFT", "TURN_RIGHT", "CLEAR"}


def load_env(path: str = ".env") -> None:
    env_path = Path(path)
    if not env_path.exists():
        return
    for raw in env_path.read_text(encoding="utf-8").splitlines():
        line = raw.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, value = line.split("=", 1)
        key = key.strip()
        value = value.strip().strip('"').strip("'")
        os.environ.setdefault(key, value)


def compact_perception(perception: dict[str, Any]) -> dict[str, Any]:
    detections = perception.get("detections") or []
    compact = []
    for det in detections[:5]:
        compact.append(
            {
                "name": det.get("name") or det.get("class") or "object",
                "score": det.get("score"),
                "bbox": det.get("bbox"),
                "distance_m": det.get("distance_m"),
            }
        )
    return {
        "detections": compact,
        "risk": perception.get("risk"),
        "timestamp": perception.get("timestamp"),
    }


def build_messages(perception: dict[str, Any], safety_rule: dict[str, Any]) -> list[dict[str, str]]:
    context = {
        "robot_state": {
            "platform": "RDK X3 morphing inspection robot",
            "llm_control_allowed": False,
        },
        "perception": compact_perception(perception),
        "safety_rule": safety_rule,
        "output_schema": {
            "suggestion": "STOP|SLOW|TURN_LEFT|TURN_RIGHT|CLEAR",
            "explanation": "short Chinese explanation",
            "allowed": "boolean; false when safety_rule.action is STOP",
        },
    }
    return [
        {
            "role": "system",
            "content": (
                "你是机器人副驾驶，只能解释环境和给出高层建议。"
                "禁止输出电机 PWM、串口帧、舵机角度或任何底层控制命令。"
                "本地 safety_rule 是最高优先级；如果 safety_rule.action 是 STOP，"
                "你的 suggestion 必须是 STOP 且 allowed 必须是 false。"
                "只输出一个 JSON 对象，不要 Markdown，不要解释。"
            ),
        },
        {
            "role": "user",
            "content": (
                "根据下面上下文，只返回一个 JSON 对象，例如: "
                '{"suggestion":"STOP","explanation":"前方障碍太近，必须停止。","allowed":false}'
                "\n上下文: " + json.dumps(context, ensure_ascii=False)
            ),
        },
    ]


def extract_json(text: str) -> dict[str, Any]:
    text = text.strip()
    if not text:
        raise ValueError("empty LLM content")
    try:
        return json.loads(text)
    except json.JSONDecodeError:
        decoder = json.JSONDecoder()
        for idx, char in enumerate(text):
            if char != "{":
                continue
            try:
                obj, _ = decoder.raw_decode(text[idx:])
            except json.JSONDecodeError:
                continue
            if isinstance(obj, dict):
                return obj
        raise


def call_llm(messages: list[dict[str, str]]) -> tuple[dict[str, Any], dict[str, Any]]:
    base_url = os.environ["LLM_BASE_URL"].rstrip("/")
    api_key = os.environ["LLM_API_KEY"]
    model = os.environ["LLM_MODEL"]
    payload = {
        "model": model,
        "messages": messages,
        "temperature": 0,
        "max_tokens": 512,
    }
    req = urllib.request.Request(
        base_url + "/chat/completions",
        data=json.dumps(payload, ensure_ascii=False).encode("utf-8"),
        headers={
            "Authorization": "Bearer " + api_key,
            "Content-Type": "application/json",
        },
        method="POST",
    )
    with urllib.request.urlopen(req, timeout=30) as resp:
        raw = json.loads(resp.read().decode("utf-8"))
    choice = raw.get("choices", [{}])[0]
    message = choice.get("message") or {}
    content = message.get("content") or ""
    if not content and message.get("reasoning_content"):
        content = message["reasoning_content"]
    return extract_json(content), raw


def normalize_advice(advice: dict[str, Any], safety_rule: dict[str, Any]) -> dict[str, Any]:
    suggestion = str(advice.get("suggestion") or safety_rule.get("action") or "SLOW").upper()
    if suggestion not in ALLOWED_ACTIONS:
        suggestion = str(safety_rule.get("action") or "SLOW").upper()
    explanation = str(advice.get("explanation") or "大模型未给出解释，采用规则层结果。")
    allowed = bool(advice.get("allowed", True))

    final_action = str(safety_rule.get("action") or suggestion).upper()
    raw_suggestion = suggestion
    if raw_suggestion != final_action:
        explanation = "大模型原始建议为 %s，已采用规则层结果：%s" % (
            raw_suggestion,
            str(safety_rule.get("reason") or ""),
        )
        suggestion = final_action
    allowed = final_action != "STOP"

    return {
        "suggestion": suggestion,
        "explanation": explanation,
        "allowed": allowed,
        "final_action": final_action,
        "max_speed": safety_rule.get("max_speed"),
        "safety_reason": safety_rule.get("reason"),
    }


def make_fallback(exc: Exception, safety_rule: dict[str, Any]) -> dict[str, Any]:
    action = str(safety_rule.get("action") or "SLOW").upper()
    return {
        "suggestion": action,
        "explanation": "大模型暂时不可用，采用本地安全规则。",
        "allowed": action != "STOP",
        "final_action": action,
        "max_speed": safety_rule.get("max_speed"),
        "safety_reason": safety_rule.get("reason"),
        "llm_error": repr(exc),
    }


def run_once(source: str) -> dict[str, Any]:
    perception = load_perception(source)
    decision = decide(perception)
    safety_rule = decision.as_dict()
    messages = build_messages(perception, safety_rule)
    try:
        advice, _ = call_llm(messages)
        llm = normalize_advice(advice, safety_rule)
    except Exception as exc:
        llm = make_fallback(exc, safety_rule)
    return {
        "perception": compact_perception(perception),
        "safety_rule": safety_rule,
        "llm_advice": llm,
    }


def print_human(result: dict[str, Any]) -> None:
    safety = result["safety_rule"]
    advice = result["llm_advice"]
    print(json.dumps(result, ensure_ascii=False, indent=2))
    print("---")
    print(f"规则层动作: {safety['action']}；限速: {safety['max_speed']:.2f}")
    print(f"规则原因: {safety['reason']}")
    print(f"大模型建议: {advice['suggestion']}；允许执行: {advice['allowed']}")
    print(f"大模型解释: {advice['explanation']}")
    print(f"最终动作: {advice['final_action']}；注意：本脚本不向 STM32 发送任何控制帧")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("source", nargs="?", default=DEFAULT_SOURCE)
    parser.add_argument("--env", default=".env")
    parser.add_argument("--watch", action="store_true")
    parser.add_argument("--interval", type=float, default=2.0)
    args = parser.parse_args()

    load_env(args.env)
    missing = [k for k in ("LLM_BASE_URL", "LLM_API_KEY", "LLM_MODEL") if not os.environ.get(k)]
    if missing:
        print(json.dumps({"error": "missing_env", "keys": missing}, ensure_ascii=False), file=sys.stderr)
        raise SystemExit(2)

    while True:
        result = run_once(args.source)
        print_human(result)
        if not args.watch:
            break
        time.sleep(args.interval)


if __name__ == "__main__":
    main()
