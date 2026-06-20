#!/usr/bin/env python3
"""Safety-first decision layer for the morphing inspection robot.

This script reads BPU perception JSON and prints high-level suggestions only.
It does not send commands to STM32; keep this layer safe for tuning.
"""
from __future__ import annotations

import argparse
import json
import time
import urllib.request
from dataclasses import dataclass
from pathlib import Path
from typing import Any

SIDE_IMBALANCE_RATIO = 2.5   # 一侧深度是另一侧 2.5 倍以上时触发转向
STOP_DIST = 0.45
SLOW_DIST = 0.90
CLEAR_DIST = 1.20
TURN_BAND = 0.18
CENTER_MIN = 0.30
CENTER_MAX = 0.70
SIDE_NARROW_DIST = 0.65
CENTER_PASS_DIST = 1.00
LINE_ROBOT_DIAMETER_M = 0.10
PASSAGE_WIDTH_MARGIN_M = 0.05
MIN_PASSAGE_WIDTH_M = LINE_ROBOT_DIAMETER_M + PASSAGE_WIDTH_MARGIN_M


@dataclass
class Decision:
    action: str
    reason: str
    max_speed: float
    target: dict[str, Any] | None = None

    def as_dict(self) -> dict[str, Any]:
        return {
            "action": self.action,
            "reason": self.reason,
            "max_speed": self.max_speed,
            "target": self.target,
        }


def load_perception(source: str) -> dict[str, Any]:
    if source.startswith("http://") or source.startswith("https://"):
        with urllib.request.urlopen(source, timeout=3) as resp:
            return json.loads(resp.read().decode("utf-8"))
    return json.loads(Path(source).read_text())


def bbox_center_x(det: dict[str, Any]) -> float:
    bbox = det.get("bbox") or [0, 0, 0, 0]
    x1, _, x2, _ = bbox
    return (float(x1) + float(x2)) / 2.0 / 640.0


def choose_main_target(detections: list[dict[str, Any]]) -> dict[str, Any] | None:
    valid = [d for d in detections if d.get("distance_m") is not None]
    if not valid:
        return detections[0] if detections else None
    return min(valid, key=lambda d: float(d.get("distance_m", 99.0)))


def decide(perception: dict[str, Any]) -> Decision:
    passage = perception.get("passage") or {}
    left = passage.get("left_m")
    center_pass = passage.get("center_m")
    right = passage.get("right_m")
    center_width = passage.get("center_width_m")
    # None or ≤0.40 = obstructed → treat as 0
    left_v   = 0.0  if (left   is None or float(left)   <= 0.40) else float(left)
    center_v = 0.0  if (center_pass is None or float(center_pass) <= 0.40) else float(center_pass)
    right_v  = 0.0  if (right  is None or float(right)  <= 0.40) else float(right)
    has_left  = left_v  > 0.0
    has_right = right_v > 0.0
    valid_sides = has_left + has_right
    # ── all sides blocked: L/C/R ≤0.40 → STOP ──
    if not has_left and not has_right and center_v == 0.0:
        return Decision("STOP",
            "左/中/右均被遮挡或太近，立即停止。",
            0.0, {"name":"all_blocked","left_m":left_v,"center_m":center_v,"right_m":right_v,"risk":"STOP"})


    # ── partial passage: one side blocked → turn toward open side ──
    if not has_left and has_right and right_v > 0.5:
        if center_pass is None:
            # 左边和中间都为0，右边有空间 → 先后退再右转
            return Decision("BACK_TURN_RIGHT",
                f"左侧和中央太近无法测距，右侧 {right_v:.2f}m 敞亮，先后退再右转向空侧。",
                0.20, {"name":"side_blocked","left_m":None,"center_m":0,"right_m":right_v,"risk":"BACK_TURN_RIGHT"})
        return Decision("TURN_RIGHT",
            f"左侧太近无法测距，右侧 {right_v:.2f}m 敞亮，右转向空侧。",
            0.25, {"name":"side_blocked","left_m":None,"right_m":right_v,"risk":"TURN_RIGHT"})
    if not has_right and has_left and left_v > 0.5:
        if center_pass is None:
            return Decision("BACK_TURN_LEFT",
                f"右侧和中央太近无法测距，左侧 {left_v:.2f}m 敞亮，先后退再左转向空侧。",
                0.20, {"name":"side_blocked","left_m":left_v,"center_m":0,"right_m":None,"risk":"BACK_TURN_LEFT"})
        return Decision("TURN_LEFT",
            f"右侧太近无法测距，左侧 {left_v:.2f}m 敞亮，左转向空侧。",
            0.25, {"name":"side_blocked","left_m":left_v,"right_m":None,"risk":"TURN_LEFT"})

    # ── full passage: all three bands have data ──
    if left is not None and center_pass is not None and right is not None:
        left   = left_v
        center_pass = center_v
        right  = right_v
        width_ok = center_width is not None and float(center_width) >= MIN_PASSAGE_WIDTH_M
        if left < SIDE_NARROW_DIST and right < SIDE_NARROW_DIST and center_pass > CENTER_PASS_DIST and width_ok:
            center_width = float(center_width)
            return Decision(
                "MORPH_LINE",
                f"左右距离较短 L={left:.2f}m/R={right:.2f}m，中间深度 {center_pass:.2f}m，估算宽度 {center_width:.2f}m，大于一字形安全宽度 {MIN_PASSAGE_WIDTH_M:.2f}m，建议切换一字形态。",
                0.16,
                {"name": "narrow_passage", "left_m": left, "center_m": center_pass, "right_m": right, "center_width_m": center_width, "min_width_m": MIN_PASSAGE_WIDTH_M, "risk": "MORPH_LINE"},
            )
        if left < SIDE_NARROW_DIST and right < SIDE_NARROW_DIST and center_pass > CENTER_PASS_DIST and not width_ok:
            width_text = "未知" if center_width is None else f"{float(center_width):.2f}m"
            return Decision(
                "SLOW",
                f"左右距离较短，但中间可通行宽度 {width_text} 未达到一字形安全宽度 {MIN_PASSAGE_WIDTH_M:.2f}m，先减速观察。",
                0.12,
                {"name": "narrow_but_too_thin", "left_m": left, "center_m": center_pass, "right_m": right, "center_width_m": center_width, "min_width_m": MIN_PASSAGE_WIDTH_M, "risk": "SLOW"},
            )
        # Side imbalance: one side close, the other far → turn toward open side
        if left < SIDE_NARROW_DIST and right > left * SIDE_IMBALANCE_RATIO:
            return Decision(
                "TURN_RIGHT",
                f"左侧距离 {left:.2f}m 较近，右侧 {right:.2f}m 敞亮，右转向空侧。",
                0.30,
                {"name": "side_imbalance", "left_m": left, "right_m": right, "risk": "TURN_RIGHT"},
            )
        if right < SIDE_NARROW_DIST and left > right * SIDE_IMBALANCE_RATIO:
            return Decision(
                "TURN_LEFT",
                f"右侧距离 {right:.2f}m 较近，左侧 {left:.2f}m 敞亮，左转向空侧。",
                0.30,
                {"name": "side_imbalance", "left_m": left, "right_m": right, "risk": "TURN_LEFT"},
            )


    front_distance = perception.get("front_distance_m")
    if front_distance is not None:
        front_distance = float(front_distance)
        if front_distance < STOP_DIST:
            return Decision("STOP", f"前方距离 {front_distance:.2f}m，小于停止阈值 {STOP_DIST:.2f}m。", 0.0, {"name": "front_obstacle", "distance_m": front_distance, "risk": "STOP"})
        if front_distance < SLOW_DIST:
            return Decision("SLOW", f"前方距离 {front_distance:.2f}m，进入减速区。", 0.40, {"name": "front_obstacle", "distance_m": front_distance, "risk": "SLOW"})

    detections = perception.get("detections") or []
    if not detections:
        if front_distance is None:
            return Decision("SLOW", "没有检测到目标，且前方距离未知，先减速观察。", 0.20)
        return Decision("CLEAR", f"未检测到目标，前方距离 {front_distance:.2f}m，大于安全距离。", 0.45)

    target = choose_main_target(detections)
    if target is None:
        return Decision("CLEAR", "感知结果为空，保持低速巡检。", 0.35)

    name = target.get("name", "object")
    distance = target.get("distance_m")
    cx = bbox_center_x(target)
    center = CENTER_MIN <= cx <= CENTER_MAX

    if distance is None:
        if center:
            return Decision("SLOW", f"正前方检测到 {name}，但距离未知，先减速观察。", 0.40, target)
        turn = "TURN_RIGHT" if cx < CENTER_MIN else "TURN_LEFT"
        return Decision(turn, f"侧前方检测到 {name}，距离未知，向空侧小角度调整。", 0.40, target)

    distance = float(distance)
    if center and distance < STOP_DIST:
        return Decision("STOP", f"正前方 {name} 距离 {distance:.2f}m，小于停止阈值 {STOP_DIST:.2f}m。", 0.0, target)
    if distance < SLOW_DIST:
        if center:
            return Decision("SLOW", f"正前方 {name} 距离 {distance:.2f}m，进入减速区。", 0.40, target)
        turn = "TURN_RIGHT" if cx < CENTER_MIN else "TURN_LEFT"
        return Decision(turn, f"{name} 位于{'左' if cx < CENTER_MIN else '右'}侧且距离 {distance:.2f}m，建议向空侧转向。", 0.40, target)
    if distance < CLEAR_DIST:
        return Decision("SLOW", f"检测到 {name} 距离 {distance:.2f}m，保持谨慎低速。", 0.40, target)
    return Decision("CLEAR", f"最近目标 {name} 距离 {distance:.2f}m，大于安全距离。", 0.80, target)


def explain(decision: Decision) -> str:
    return f"建议动作: {decision.action}；限速: {decision.max_speed:.2f}；原因: {decision.reason}"


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("source", nargs="?", default="http://127.0.0.1:8091/result.json")
    parser.add_argument("--watch", action="store_true")
    parser.add_argument("--interval", type=float, default=1.0)
    args = parser.parse_args()

    while True:
        try:
            perception = load_perception(args.source)
            decision = decide(perception)
            print(json.dumps(decision.as_dict(), ensure_ascii=False))
            print(explain(decision), flush=True)
        except Exception as exc:
            print(json.dumps({"action": "UNKNOWN", "error": repr(exc)}, ensure_ascii=False), flush=True)
        if not args.watch:
            break
        time.sleep(args.interval)


if __name__ == "__main__":
    main()
