#!/usr/bin/env python3
"""Rule-driven controller: perception → ai_decision → STM32 serial.

Continuously reads BPU/depth perception, runs safety rules, and sends
motor commands to the STM32.  Manual override via joystick is respected
when present in the perception JSON.
"""
from __future__ import annotations

import json
import signal
import sys
import time
import urllib.request
from typing import Any

import serial

from ai_decision import decide
from protocol import motor_many, motor_stop_mask, hexstr

PERCEPTION_URL = "http://127.0.0.1:8091/result.json"
SERIAL_PORT = "/dev/ttyACM0"
SERIAL_BAUD = 1000000
INTERVAL_S = 0.2  # 5 Hz control loop
MOTOR_IDS = (0, 1, 2, 3)
STOP_MASK_ALL = 0x0F
SHARED_DECISION = "/tmp/rdk_robot_decision.json"

running = True


def load_perception() -> dict[str, Any]:
    with urllib.request.urlopen(PERCEPTION_URL, timeout=3) as resp:
        return json.loads(resp.read().decode("utf-8"))


def speed_to_motor_values(action: str, max_speed: float) -> list[float]:
    """Convert decision action + max_speed into per-motor speed values."""
    s = max_speed  # base speed

    if action == "STOP" or max_speed == 0:
        return [0.0, 0.0, 0.0, 0.0]
    if action == "SLOW":
        return [s, -s, s, -s]
    if action == "MORPH_LINE":
        return [s, -s, s, -s]
    if action == "TURN_LEFT":
        # 左侧(1,3)停，右侧(2,4)前进(镜像=负值)
        return [0.0, -s, 0.0, -s]
    if action == "TURN_RIGHT":
        # 左侧(1,3)前进，右侧(2,4)停
        return [s, 0.0, s, 0.0]
    if action == "BACK_TURN_LEFT":
        # 左侧(1,3)后退，右侧(2,4)停
        return [-s, 0.0, -s, 0.0]
    if action == "BACK_TURN_RIGHT":
        # 左侧(1,3)停，右侧(2,4)后退(镜像=正值)
        return [0.0, s, 0.0, s]
    # CLEAR / FORWARD
    return [s, -s, s, -s]


def main():
    global running

    def shutdown(_sig, _frame):
        global running
        running = False
        print("\n[rule_controller] shutting down...")

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    try:
        ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=0.3)
    except Exception as e:
        print(f"[rule_controller] serial open failed: {e}", file=sys.stderr)
        sys.exit(1)

    # Send initial stop
    ser.write(motor_stop_mask(STOP_MASK_ALL))
    ser.flush()

    print(f"[rule_controller] started — reading {PERCEPTION_URL} → rules → {SERIAL_PORT}")
    print(f"[rule_controller] STOP<{0.45}m  SLOW<{0.90}m  interval={INTERVAL_S}s")

    last_action = ""
    while running:
        try:
            p = load_perception()
        except Exception as e:
            print(f"[rule_controller] perception read error: {e}")
            time.sleep(INTERVAL_S)
            continue

        # ── manual override check ──
        override = p.get("manual_override")
        manual_active = p.get("manual_active", False)
        if override is not None or manual_active:
            # Let joystick/teleop handle it; skip rule output
            time.sleep(INTERVAL_S)
            continue

        # ── rule decision ──
        try:
            decision = decide(p)
        except Exception as e:
            print(f"[rule_controller] decide error: {e}")
            time.sleep(INTERVAL_S)
            continue

        action = decision.action
        max_speed = decision.max_speed

        # ── serial output ──
        if action == "STOP" or max_speed == 0.0:
            frame = motor_stop_mask(STOP_MASK_ALL)
        else:
            speeds = speed_to_motor_values(action, max_speed)
            motor_map = dict(zip(MOTOR_IDS, speeds))
            frame = motor_many(motor_map)

        ser.write(frame)
        ser.flush()
        # share decision with web dashboard
        try:
            with open(SHARED_DECISION, "w") as f:
                json.dump({"action": action, "max_speed": max_speed,
                           "reason": decision.reason, "mode": "AUTO",
                           "ts": time.time()}, f)
        except Exception:
            pass


        if action != last_action:
            print(f"[rule_controller] {action:12s} speed={max_speed:.2f}  "
                  f"reason: {decision.reason[:60]}")
            last_action = action

        time.sleep(INTERVAL_S)

    # ── clean exit ──
    ser.write(motor_stop_mask(STOP_MASK_ALL))
    ser.flush()
    time.sleep(0.2)
    ser.close()
    print("[rule_controller] stopped, motors halted.")


if __name__ == "__main__":
    main()
