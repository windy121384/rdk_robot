#!/usr/bin/env python3
"""BPU YOLOv5 + Astra depth ROI distance web demo."""
from __future__ import annotations

import argparse
import base64
import ctypes
import json
import os
import socket
import struct
import threading
import time
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import cv2
import numpy as np
from hobot_dnn import pyeasy_dnn as dnn

from ai_decision import decide
from llm_advisor import build_messages, call_llm, load_env, make_fallback, normalize_advice

ASTRA_HFOV_DEG = 60.0
SHARED_DECISION = "/tmp/rdk_robot_decision.json"
def read_shared_decision() -> dict:
    try:
        return json.loads(Path(SHARED_DECISION).read_text())
    except Exception:
        return {}

# ── Cityscapes 19-class palette & passable mask ──
SEG_PALETTE = [
    128,64,128,   244,35,232,  70,70,70,     102,102,156,  190,153,153,
    153,153,153,  250,170,30,  220,220,0,    107,142,35,   152,251,152,
    0,130,180,    220,20,60,   255,0,0,      0,0,142,     0,0,70,
    0,60,100,     0,80,100,   0,0,230,      119,11,32,
]
# Classes: 0=road, 1=sidewalk, 2=building, 3=wall, 4=fence, 5=pole,
# 6=traffic light, 7=traffic sign, 8=vegetation, 9=terrain, 10=sky,
# 11=person, 12=rider, 13=car, 14=truck, 15=bus, 16=train, 17=motorcycle, 18=bicycle
PASSABLE_CLASSES = {0, 1, 8, 9}   # road, sidewalk, vegetation, terrain
BLOCKED_CLASSES  = {2, 3, 4, 11, 12, 13, 14, 15, 16, 17, 18}  # building, wall, fence, person, vehicles


class BpuSegment:
    def __init__(self, model_path: str):
        self.model = dnn.load(model_path)[0]
        h, w = self.model.inputs[0].properties.shape[2], self.model.inputs[0].properties.shape[3]
        self.input_size = (w, h)

    def infer(self, bgr: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Return (class_map, passable_ratio) where class_map is HxW uint8."""
        resized = cv2.resize(bgr, self.input_size, interpolation=cv2.INTER_AREA)
        outputs = self.model.forward(bgr2nv12(resized))
        raw = outputs[0].buffer
        class_map_full = np.argmax(raw[0], axis=-1).astype(np.uint8)
        # Resize back to original
        class_map = cv2.resize(class_map_full, (bgr.shape[1], bgr.shape[0]),
                               interpolation=cv2.INTER_NEAREST)
        total = class_map.size
        passable = np.isin(class_map, list(PASSABLE_CLASSES)).sum()
        blocked  = np.isin(class_map, list(BLOCKED_CLASSES)).sum()
        passable_ratio = passable / total if total > 0 else 0.0
        blocked_ratio  = blocked / total if total > 0 else 0.0
        return class_map, {"passable_ratio": round(passable_ratio, 3),
                           "blocked_ratio": round(blocked_ratio, 3),
                           "total_pixels": total}

def colorize_mask(class_map: np.ndarray) -> np.ndarray:
    """Return BGR image from class_map using SEG_PALETTE."""
    color = np.zeros((*class_map.shape, 3), dtype=np.uint8)
    for cls_id in range(19):
        r, g, b = SEG_PALETTE[cls_id*3:cls_id*3+3]
        color[(class_map == cls_id), 0] = b
        color[(class_map == cls_id), 1] = g
        color[(class_map == cls_id), 2] = r
    return color


class hbSysMem_t(ctypes.Structure):
    _fields_ = [("phyAddr", ctypes.c_double), ("virAddr", ctypes.c_void_p), ("memSize", ctypes.c_int)]


class hbDNNQuantiShift_yt(ctypes.Structure):
    _fields_ = [("shiftLen", ctypes.c_int), ("shiftData", ctypes.c_char_p)]


class hbDNNQuantiScale_t(ctypes.Structure):
    _fields_ = [
        ("scaleLen", ctypes.c_int),
        ("scaleData", ctypes.POINTER(ctypes.c_float)),
        ("zeroPointLen", ctypes.c_int),
        ("zeroPointData", ctypes.c_char_p),
    ]


class hbDNNTensorShape_t(ctypes.Structure):
    _fields_ = [("dimensionSize", ctypes.c_int * 8), ("numDimensions", ctypes.c_int)]


class hbDNNTensorProperties_t(ctypes.Structure):
    _fields_ = [
        ("validShape", hbDNNTensorShape_t),
        ("alignedShape", hbDNNTensorShape_t),
        ("tensorLayout", ctypes.c_int),
        ("tensorType", ctypes.c_int),
        ("shift", hbDNNQuantiShift_yt),
        ("scale", hbDNNQuantiScale_t),
        ("quantiType", ctypes.c_int),
        ("quantizeAxis", ctypes.c_int),
        ("alignedByteSize", ctypes.c_int),
        ("stride", ctypes.c_int * 8),
    ]


class hbDNNTensor_t(ctypes.Structure):
    _fields_ = [("sysMem", hbSysMem_t * 4), ("properties", hbDNNTensorProperties_t)]


class Yolov5PostProcessInfo_t(ctypes.Structure):
    _fields_ = [
        ("height", ctypes.c_int),
        ("width", ctypes.c_int),
        ("ori_height", ctypes.c_int),
        ("ori_width", ctypes.c_int),
        ("score_threshold", ctypes.c_float),
        ("nms_threshold", ctypes.c_float),
        ("nms_top_k", ctypes.c_int),
        ("is_pad_resize", ctypes.c_int),
    ]


LIBPOST = ctypes.CDLL("/usr/lib/libpostprocess.so")
GET_RESULT = LIBPOST.Yolov5PostProcess
GET_RESULT.argtypes = [ctypes.POINTER(Yolov5PostProcessInfo_t)]
GET_RESULT.restype = ctypes.c_char_p


def tensor_layout(layout: str) -> int:
    return 2 if layout == "NCHW" else 0


def get_hw(pro) -> tuple[int, int]:
    if pro.layout == "NCHW":
        return pro.shape[2], pro.shape[3]
    return pro.shape[1], pro.shape[2]


def bgr2nv12(image: np.ndarray) -> np.ndarray:
    height, width = image.shape[:2]
    area = height * width
    yuv420p = cv2.cvtColor(image, cv2.COLOR_BGR2YUV_I420).reshape((area * 3 // 2,))
    y = yuv420p[:area]
    uv_planar = yuv420p[area:].reshape((2, area // 4))
    uv_packed = uv_planar.transpose((1, 0)).reshape((area // 2,))
    nv12 = np.zeros_like(yuv420p)
    nv12[:area] = y
    nv12[area:] = uv_packed
    return nv12


def read_pgm16(path: Path) -> np.ndarray | None:
    data = path.read_bytes()
    if not data.startswith(b"P5\n"):
        return None
    idx = 3
    while data[idx:idx + 1] == b"#":
        idx = data.index(b"\n", idx) + 1
    end = data.index(b"\n", idx)
    width, height = map(int, data[idx:end].split())
    idx = end + 1
    end = data.index(b"\n", idx)
    maxval = int(data[idx:end])
    idx = end + 1
    if maxval != 65535:
        return None
    arr = np.frombuffer(data[idx:], dtype=np.uint16).copy()
    return arr.reshape((height, width))


class BpuYolo:
    def __init__(self, model_path: str, score: float = 0.45):
        self.model = dnn.load(model_path)[0]
        self.input_h, self.input_w = get_hw(self.model.inputs[0].properties)
        self.score = 0.35

    def infer(self, bgr: np.ndarray) -> list[dict]:
        resized = cv2.resize(bgr, (self.input_w, self.input_h), interpolation=cv2.INTER_AREA)
        outputs = self.model.forward(bgr2nv12(resized))
        info = Yolov5PostProcessInfo_t()
        info.height = self.input_h
        info.width = self.input_w
        info.ori_height, info.ori_width = bgr.shape[:2]
        info.score_threshold = self.score
        info.nms_threshold = 0.45
        info.nms_top_k = 20
        info.is_pad_resize = 0

        tensors = (hbDNNTensor_t * len(self.model.outputs))()
        for i, output in enumerate(outputs):
            tensors[i].properties.tensorLayout = tensor_layout(output.properties.layout)
            if len(output.properties.scale_data) == 0:
                tensors[i].properties.quantiType = 0
                tensors[i].sysMem[0].virAddr = ctypes.cast(output.buffer.ctypes.data_as(ctypes.POINTER(ctypes.c_float)), ctypes.c_void_p)
            else:
                tensors[i].properties.quantiType = 2
                tensors[i].properties.scale.scaleData = output.properties.scale_data.ctypes.data_as(ctypes.POINTER(ctypes.c_float))
                tensors[i].sysMem[0].virAddr = ctypes.cast(output.buffer.ctypes.data_as(ctypes.POINTER(ctypes.c_int32)), ctypes.c_void_p)
            for j, dim in enumerate(output.properties.shape):
                tensors[i].properties.validShape.dimensionSize[j] = dim
            LIBPOST.Yolov5doProcess(tensors[i], ctypes.pointer(info), i)

        raw = GET_RESULT(ctypes.pointer(info)).decode("utf-8")
        try:
            return json.loads(raw[16:])
        except Exception:
            return []


def roi_depth_m(depth: np.ndarray | None, bbox: list[float]) -> float | None:
    if depth is None:
        return None
    h, w = depth.shape[:2]
    x1, y1, x2, y2 = [int(round(v)) for v in bbox]
    x1, x2 = sorted((max(0, min(w - 1, x1)), max(0, min(w - 1, x2))))
    y1, y2 = sorted((max(0, min(h - 1, y1)), max(0, min(h - 1, y2))))
    cx1 = x1 + max(1, (x2 - x1) // 3)
    cx2 = x2 - max(1, (x2 - x1) // 3)
    cy1 = y1 + max(1, (y2 - y1) // 3)
    cy2 = y2 - max(1, (y2 - y1) // 3)
    roi = depth[cy1:cy2 + 1, cx1:cx2 + 1]
    vals = roi[(roi > 100) & (roi < 8000)]
    if vals.size == 0:
        return None
    return float(np.median(vals)) / 1000.0


def center_depth_m(depth: np.ndarray | None) -> float | None:
    if depth is None:
        return None
    h, w = depth.shape[:2]
    x1, x2 = int(w * 0.35), int(w * 0.65)
    y1, y2 = int(h * 0.35), int(h * 0.70)
    roi = depth[y1:y2, x1:x2]
    vals = roi[(roi > 100) & (roi < 8000)]
    if vals.size == 0:
        return None
    return float(np.percentile(vals, 20)) / 1000.0


def band_depth_m(depth: np.ndarray | None, x_start: float, x_end: float) -> float | None:
    if depth is None:
        return None
    h, w = depth.shape[:2]
    x1, x2 = int(w * x_start), int(w * x_end)
    y1, y2 = int(h * 0.35), int(h * 0.70)
    roi = depth[y1:y2, x1:x2]
    vals = roi[(roi > 100) & (roi < 8000)]
    if vals.size == 0:
        return None
    return float(np.percentile(vals, 20)) / 1000.0


def passage_depths(depth: np.ndarray | None) -> dict:
    left = band_depth_m(depth, 0.05, 0.30)
    center = band_depth_m(depth, 0.35, 0.65)
    right = band_depth_m(depth, 0.70, 0.95)
    width = None
    if center is not None:
        # Approximate center passable width from the center band's angular span.
        # It is conservative enough for a 100 mm line-shape robot gate.
        center_band_ratio = 0.30
        width = 2.0 * float(center) * np.tan(np.deg2rad(ASTRA_HFOV_DEG * center_band_ratio / 2.0))
    return {
        "left_m": left,
        "center_m": center,
        "right_m": right,
        "center_width_m": width,
    }


def risk_from_distance(distance_m: float | None) -> str:
    if distance_m is None:
        return "UNKNOWN"
    if distance_m < 0.45:
        return "STOP"
    if distance_m < 0.90:
        return "SLOW"
    return "CLEAR"


def target_key_from_perception(perception: dict) -> str:
    detections = perception.get("detections") or []
    if not detections:
        return "none"
    parts = []
    for det in detections[:3]:
        bbox = det.get("bbox") or []
        bbox_key = ",".join(str(round(float(v) / 20.0) * 20) for v in bbox[:4])
        dist = det.get("distance_m")
        dist_key = "null" if dist is None else str(round(float(dist) * 20.0) / 20.0)
        parts.append(":".join([
            str(det.get("name") or det.get("id") or "object"),
            str(round(float(det.get("score") or 0) * 100)),
            str(det.get("risk") or ""),
            dist_key,
            bbox_key,
        ]))
    return "|".join(parts)


def extract_chat_content(data: dict) -> str:
    choice = (data.get("choices") or [{}])[0]
    message = choice.get("message") or {}
    return str(message.get("content") or "").strip()


def call_vlm_with_image(jpeg: bytes, context: dict) -> str:
    base_url = os.environ.get("VLM_BASE_URL", "").rstrip("/")
    api_key = os.environ.get("VLM_API_KEY", "")
    model = os.environ.get("VLM_MODEL", "")
    if not base_url or not api_key or not model:
        raise RuntimeError("VLM_BASE_URL/VLM_API_KEY/VLM_MODEL not configured")
    image_b64 = base64.b64encode(jpeg).decode("ascii")
    prompt = (
        "你是一个机器人场景分析师，正在通过机器人第一视角观察环境。请做一次详细的画面阅读理解，"
        "分四个维度输出：\n"
        "1.【场景概览】描述整体环境类型（室内/室外、走廊/房间/开阔地），光照条件，氛围。\n"
        "2.【检测目标】逐一列出画面中看到的目标物体及其位置关系、远近判断，"
        "如果检测列表为空也请描述你从画面中观察到的内容。\n"
        "3.【通道与距离】分析前方可通行状况——左侧/中央/右侧三个方向的深度距离，"
        "哪个方向最敞亮可行，是否存在窄通道、障碍物密集区、死胡同。\n"
        "4.【决策建议】基于以上观察，给出机器人下一步倾向：继续前进、减速、停止、转向，"
        "并解释为什么。如果是停止，说明需要观察什么才能恢复通过。\n"
        "总长度350-500字，语气自然口语化，像现场实时播报，不要只罗列数据，不要编造画面里没有的东西。\n\n"
        "结构化数据参考：" + json.dumps(context, ensure_ascii=False)
    )
    payload = {
        "model": model,
        "messages": [{
            "role": "user",
            "content": [
                {"type": "image_url", "image_url": {"url": "data:image/jpeg;base64," + image_b64}},
                {"type": "text", "text": prompt},
            ],
        }],
        "temperature": 0.6,
        "max_tokens": 1024,
    }
    req = urllib.request.Request(
        base_url + "/chat/completions",
        data=json.dumps(payload, ensure_ascii=False).encode("utf-8"),
        headers={"Authorization": "Bearer " + api_key, "Content-Type": "application/json"},
        method="POST",
    )
    with urllib.request.urlopen(req, timeout=60) as resp:
        data = json.loads(resp.read().decode("utf-8"))
    text = extract_chat_content(data)
    if not text:
        raise RuntimeError("empty VLM response")
    return text


class PerceptionApp:
    def __init__(self, frames_dir: Path, model_path: str):
        self.frames_dir = frames_dir
        self.rgb_path = frames_dir / "latest_rgb.jpg"
        self.depth_path = frames_dir / "latest_depth16.pgm"
        self.detector = BpuYolo(model_path)
        self.lock = threading.Lock()
        self.latest_jpeg = b""
        self.latest_json = {"status": "starting"}
        self.latest_advisor = {"status": "starting"}
        self.last_advisor_ts = 0.0
        self.last_target_key = ""
        self.advisor_busy = False
        self.work_history = []
        self.latest_work_summary = {"status": "starting", "summary": "等待收集目标与距离数据。"}
        self.last_history_ts = 0.0
        self.last_summary_ts = 0.0
        self.summary_busy = False
        self.summary_min_gap = 2.0
        self.summary_collection = []
        self.running = True

    def loop(self):
        while self.running:
            try:
                if not self.rgb_path.exists() or not self.depth_path.exists():
                    time.sleep(0.1)
                    continue
                bgr = cv2.imread(str(self.rgb_path))
                depth = read_pgm16(self.depth_path)
                if bgr is None:
                    time.sleep(0.1)
                    continue
                front_distance = center_depth_m(depth)
                front_risk = risk_from_distance(front_distance)
                passage = passage_depths(depth)
                results = self.detector.infer(bgr)
                annotated = bgr.copy()
                if front_distance is not None:
                    cv2.putText(annotated, f"FRONT {front_distance:.2f}m {front_risk}", (18, 32),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                                (0, 0, 255) if front_risk == "STOP" else
                                ((0, 255, 255) if front_risk == "SLOW" else (0, 255, 0)), 2)
                cv2.putText(annotated, f"L {passage.get('left_m') or 0:.2f} C {passage.get('center_m') or 0:.2f} R {passage.get('right_m') or 0:.2f} W {passage.get('center_width_m') or 0:.2f}",
                            (18, 62), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 255, 255), 2)
                fused = []
                for det in results[:10]:
                    bbox = det.get("bbox", [0, 0, 0, 0])
                    dist = roi_depth_m(depth, bbox)
                    risk = risk_from_distance(dist)
                    x1, y1, x2, y2 = [int(round(v)) for v in bbox]
                    color = (0, 255, 0) if risk == "CLEAR" else ((0, 255, 255) if risk == "SLOW" else (0, 0, 255))
                    cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)
                    label = f"{det.get('name','obj')} {det.get('score',0):.2f}"
                    if dist is not None:
                        label += f" {dist:.2f}m {risk}"
                    cv2.putText(annotated, label, (x1, max(20, y1 - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
                    fused.append({**det, "distance_m": dist, "risk": risk})
                ok, jpg = cv2.imencode(".jpg", annotated, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
                if ok:
                    snapshot = {"ts": time.time(), "detections": fused,
                               "mode": "AUTO",
                                "front_distance_m": front_distance, "front_risk": front_risk,
                               "passage": passage,
                               "decision": read_shared_decision()}
                    target_key = target_key_from_perception(snapshot)
                    should_update_advisor = False
                    with self.lock:
                        self.latest_jpeg = jpg.tobytes()
                        self.latest_json = snapshot
                        if target_key != self.last_target_key and not self.advisor_busy:
                            self.last_target_key = target_key
                            self.advisor_busy = True
                            should_update_advisor = True
                    self.record_work_history(snapshot, decide(snapshot).action)
                    if should_update_advisor:
                        threading.Thread(target=self.update_advisor, args=(snapshot,), daemon=True).start()
            except Exception as exc:
                with self.lock:
                    self.latest_json = {"status": "error", "error": repr(exc)}
            time.sleep(0.01)

    def record_work_history(self, perception: dict, safety_action: str) -> None:
        now = time.time()
        if now - self.last_history_ts < 1.0:
            return
        detections = []
        for det in (perception.get("detections") or [])[:6]:
            detections.append({
                "name": det.get("name") or det.get("id") or "object",
                "distance_m": det.get("distance_m"),
                "risk": det.get("risk"),
                "score": det.get("score"),
            })
        item = {"ts": perception.get("ts") or now, "action": safety_action, "front_distance_m": perception.get("front_distance_m"), "front_risk": perception.get("front_risk"), "passage": perception.get("passage"), "detections": detections}
        with self.lock:
            self.work_history.append(item)
            self.work_history = self.work_history[-60:]
            self.last_history_ts = now
            if not self.summary_busy and now - self.last_summary_ts >= self.summary_min_gap:
                self.summary_busy = True
                history = list(self.work_history)
                threading.Thread(target=self.update_work_summary, args=(history, now), daemon=True).start()

    def build_rule_summary(self, history: list[dict]) -> str:
        if not history:
            return "暂无工作数据。"
        target_stats = {}
        actions = {}
        for item in history:
            actions[item.get("action") or "UNKNOWN"] = actions.get(item.get("action") or "UNKNOWN", 0) + 1
            for det in item.get("detections") or []:
                name = str(det.get("name") or "object")
                dist = det.get("distance_m")
                stat = target_stats.setdefault(name, {"count": 0, "min_dist": None, "last_dist": None, "risk": det.get("risk")})
                stat["count"] += 1
                stat["risk"] = det.get("risk") or stat.get("risk")
                if dist is not None:
                    dist = float(dist)
                    stat["last_dist"] = round(dist, 2)
                    stat["min_dist"] = round(dist if stat["min_dist"] is None else min(stat["min_dist"], dist), 2)
        if not target_stats:
            return "这段时间没有检测到明显目标，机器人保持低速巡检状态。"
        targets = []
        for name, stat in sorted(target_stats.items(), key=lambda kv: kv[1]["count"], reverse=True)[:5]:
            dist_text = '未知' if stat['last_dist'] is None else f"{stat['last_dist']}m"
            targets.append(f"{name} 出现 {stat['count']} 次，最近距离 {dist_text}，最近风险 {stat.get('risk')}")
        action_text = "、".join(f"{k} {v} 次" for k, v in actions.items())
        return "本阶段工作情况：" + "；".join(targets) + f"。规则动作统计：{action_text}。"

    def update_work_summary(self, history: list[dict], summary_ts: float) -> None:
        window = history[-15:]
        fallback = self.build_rule_summary(window)
        with self.lock:
            jpeg = bytes(self.latest_jpeg)
            latest = dict(self.latest_json)
        context = {
            "period_s": round(time.time() - summary_ts, 1) if summary_ts else 0,
            "latest_perception": latest,
            "work_history": window,
            "rule_fallback_summary": fallback,
        }
        try:
            if not jpeg:
                raise RuntimeError("no latest jpeg frame")
            summary = call_vlm_with_image(jpeg, context)
            if summary.strip() in {"中文白话总结，80字以内", "summary"}:
                raise RuntimeError("template-like VLM summary")
            status = "vlm_ok"
            error = None
        except Exception as exc:
            summary = fallback
            status = "fallback"
            error = repr(exc)
        with self.lock:
            self.latest_work_summary = {
                "status": status,
                "ts": time.time(),
                "period_s": round(time.time() - summary_ts, 1) if summary_ts else 0,
                "sample_count": len(window),
                "summary": summary,
                "error": error,
            }
            self.summary_collection.append(dict(self.latest_work_summary))
            self.summary_collection = self.summary_collection[-20:]
            self.last_summary_ts = summary_ts
            self.summary_busy = False

    def update_advisor(self, perception: dict) -> None:
        now = time.time()
        try:
            decision = decide(perception)
            safety_rule = decision.as_dict()
            advice, _ = call_llm(build_messages(perception, safety_rule))
            advisor = {
                "ts": now,
                "perception_ts": perception.get("ts"),
                "status": "ok",
                "safety_rule": safety_rule,
                "llm_advice": normalize_advice(advice, safety_rule),
            }
        except Exception as exc:
            decision = decide(perception)
            safety_rule = decision.as_dict()
            advisor = {
                "ts": now,
                "perception_ts": perception.get("ts"),
                "status": "fallback",
                "safety_rule": safety_rule,
                "llm_advice": make_fallback(exc, safety_rule),
            }

        with self.lock:
            self.latest_advisor = advisor
            self.last_advisor_ts = now
            self.advisor_busy = False

    def advisor_snapshot(self) -> dict:
        with self.lock:
            perception = dict(self.latest_json)
            cached = dict(self.latest_advisor)
            work_summary = dict(self.latest_work_summary)
            summary_busy = self.summary_busy
            busy = self.advisor_busy

        decision = decide(perception)
        safety_rule = decision.as_dict()
        cached_advice = cached.get("llm_advice") or {}
        explanation = cached_advice.get("explanation") or "大模型解释更新中，当前先采用规则层结果。"
        current_advice = {
            "suggestion": safety_rule.get("action"),
            "explanation": explanation,
            "allowed": safety_rule.get("action") != "STOP",
            "final_action": safety_rule.get("action"),
            "max_speed": safety_rule.get("max_speed"),
            "safety_reason": safety_rule.get("reason"),
            "llm_cached_perception_ts": cached.get("perception_ts"),
        }

        if cached.get("status") == "starting" and not busy:
            with self.lock:
                self.advisor_busy = True
            threading.Thread(target=self.update_advisor, args=(perception,), daemon=True).start()
            busy = True

        return {
            "ts": time.time(),
            "perception_ts": perception.get("ts"),
            "status": "updating" if busy else "live",
            "safety_rule": safety_rule,
            "llm_advice": current_advice,
            "cached_llm": cached,
            "work_summary": {**work_summary, "updating": summary_busy},
            "summary_collection": list(self.summary_collection),
        }


APP: PerceptionApp | None = None


def format_realtime_text(data: dict) -> str:
    rule = data.get("safety_rule") or {}
    advice = data.get("llm_advice") or {}
    target = rule.get("target") or {}
    name = target.get("name") or target.get("id") or "无"
    dist = "未知" if target.get("distance_m") is None else f"{float(target.get('distance_m')):.2f} m"
    risk = target.get("risk") or rule.get("action") or "UNKNOWN"
    return "\n".join([
        "当前动作：" + str(advice.get("final_action") or rule.get("action") or "UNKNOWN"),
        "最大速度：" + (str(advice.get("max_speed")) + " m/s" if advice.get("max_speed") is not None else "未知"),
        "触发原因：" + str(advice.get("safety_reason") or rule.get("reason") or "暂无"),
        "最近目标：" + str(name) + " / " + dist + " / " + str(risk),
        "通道距离：L=" + str(target.get("left_m", "-")) + "m / C=" + str(target.get("center_m", "-")) + "m / R=" + str(target.get("right_m", "-")) + "m" if str(name) == "narrow_passage" else "通道距离：按前方深度优先决策",
        "画面同步：已同步当前感知帧 " + str(round(float(data.get("perception_ts") or 0), 1)),
    ])


def format_summary_text(data: dict) -> str:
    work = data.get("work_summary") or {}
    return "\n".join([
        "阶段工作总结：" + str(work.get("summary") or "正在收集目标和距离数据。"),
        "样本数量：" + str(work.get("sample_count") or 0) + " 帧 | 周期：" + str(work.get("period_s") or "-") + "s",
        "状态：" + ("更新中…" if work.get("updating") else str(work.get("status") or "init")),
    ])

def format_log_text(data: dict) -> str:
    """All accumulated Doubao work summaries."""
    collection = data.get("summary_collection") or []
    lines = []
    if not collection:
        lines.append("暂无工作总结，等待豆包生成…")
    else:
        for idx, entry in enumerate(reversed(collection)):
            ts = entry.get("ts", 0)
            status = entry.get("status", "?")
            summary = entry.get("summary", "")
            lines.append(f"#{len(collection)-idx} [{time.strftime('%H:%M:%S', time.localtime(ts))}] {status}")
            for i in range(0, len(summary), 55):
                lines.append("  " + summary[i:i+55])
            lines.append("")
    return "\n".join(lines)


def render_panels_html() -> bytes:
    data = APP.advisor_snapshot()
    html = f"""
<!doctype html><html><head><meta charset="utf-8"><meta http-equiv="Cache-Control" content="no-store"><meta http-equiv="refresh" content="1">
<style>
body{{background:#111;color:#eee;font-family:sans-serif;margin:0;padding:0 12px 12px}}
.panel{{width:96vw;overflow:auto;margin:6px auto;text-align:left;background:#000;color:#9f9;padding:8px;border:1px solid #333;white-space:pre-wrap}}
#advisor{{color:#ffd479}}
.hint{{color:#aaa;font-size:14px;text-align:center}}
h3{{text-align:center;margin:10px 0 6px}}
</style></head><body>
<div class="hint">信息面板自动刷新：{time.strftime('%H:%M:%S')}</div>
<h3>实时安全决策 / Final Action</h3><pre id="advisor" class="panel">{format_realtime_text(data)}</pre>
<h3>阶段工作总结</h3><pre id="plain" class="panel">{format_summary_text(data)}</pre>
<h3>工作日志</h3><pre id="log" class="panel">{format_log_text(data)}</pre>
</body></html>
""".encode("utf-8")
    return html


class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        global APP
        path = self.path.split("?", 1)[0]
        if path == "/panels.html":
            html = render_panels_html()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(html)))
            self.end_headers()
            self.wfile.write(html)
            return
        if path == "/result.mjpg":
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            while True:
                with APP.lock:
                    data = APP.latest_jpeg
                if data:
                    try:
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " + str(len(data)).encode() + b"\r\n\r\n")
                        self.wfile.write(data)
                        self.wfile.write(b"\r\n")
                        self.wfile.flush()
                    except (BrokenPipeError, ConnectionResetError):
                        break
                time.sleep(0.15)
            return
        if path == "/result.jpg":
            with APP.lock:
                data = APP.latest_jpeg
            if not data:
                self.send_error(503, "no frame yet")
                return
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Cache-Control", "no-cache")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)
            return
        if path == "/result.json":
            with APP.lock:
                data = json.dumps(APP.latest_json, ensure_ascii=False).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Cache-Control", "no-cache")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)
            return
        if path == "/advisor.json":
            data = json.dumps(APP.advisor_snapshot(), ensure_ascii=False).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Cache-Control", "no-cache")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)
            return
        html = b"""
<!doctype html><html><head><title>BPU Perception</title>
        <meta charset="utf-8"><meta http-equiv="Cache-Control" content="no-store"><meta name="viewport" content="width=device-width,initial-scale=1">
<style>
body{background:#111;color:#eee;text-align:center;font-family:sans-serif;margin:0;padding:6px}
#img{display:block;margin:0 auto;max-width:96vw;max-height:48vh;border:1px solid #555;background:#222;object-fit:contain}
#panels{width:98vw;height:52vh;border:0;display:block;margin:8px auto;background:#111}
.hint{color:#aaa;font-size:13px;margin:2px 0}
h2{margin:4px 0;font-size:18px}
</style>
</head><body><h2>BPU YOLO + Astra Depth Distance</h2>
<div id="status" class="hint">video stream stays open; information panel refreshes every 1s</div>
<a href="/result.jpg" target="_blank"><img id="img" src="/result.mjpg" alt="BPU detection result"></a>
<iframe id="panels" src="/panels.html" title="decision and summary panels"></iframe>
</body></html>
"""
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(html)))
        self.end_headers()
        self.wfile.write(html)

    def log_message(self, fmt, *args):
        return


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--frames-dir", default="/tmp/rdk_robot_frames")
    parser.add_argument("--model", default="/opt/hobot/model/x3/basic/yolov5s_672x672_nv12.bin")
    parser.add_argument("--port", type=int, default=8091)
    args = parser.parse_args()
    load_env(str(Path(__file__).with_name(".env")))
    global APP
    APP = PerceptionApp(Path(args.frames_dir), args.model)
    th = threading.Thread(target=APP.loop, daemon=True)
    th.start()
    print(f"BPU perception page: http://0.0.0.0:{args.port}/")
    ThreadingHTTPServer(("0.0.0.0", args.port), Handler).serve_forever()


if __name__ == "__main__":
    main()
