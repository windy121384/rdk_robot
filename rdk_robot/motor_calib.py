#!/usr/bin/env python3
"""Minimal motor calibration tool.

Rotate one motor for a short duration, then stop with dual-stop protection.
Use this to identify motor ID mapping and sign direction safely.
"""
from __future__ import annotations

import argparse
import os
import time

import serial

from protocol import hexstr, motor_many, motor_one, motor_stop_mask

DEFAULT_PORT = "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B32012490-if00"
STOP_MASK_ALL = 0x0F


def stop_frames() -> list[bytes]:
    return [
        motor_stop_mask(STOP_MASK_ALL),
        motor_many({0: 0.0, 1: 0.0, 2: 0.0, 3: 0.0}),
    ]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--baud", type=int, default=1000000)
    parser.add_argument("--motor", type=int, required=True, choices=[0, 1, 2, 3])
    parser.add_argument("--speed", type=float, default=0.2, help="motor speed set_point, e.g. 0.2 or -0.2")
    parser.add_argument("--duration", type=float, default=1.0, help="seconds to run before stopping")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    if not os.path.exists(args.port):
        raise SystemExit(f"serial port not found: {args.port}")

    run_frame = motor_one(args.motor, args.speed)
    stops = stop_frames()

    print(f"motor={args.motor} speed={args.speed} duration={args.duration}s port={args.port}")
    print("RUN_FRAME:", hexstr(run_frame))
    for i, frame in enumerate(stops):
        print(f"STOP_FRAME_{i}:", hexstr(frame))

    if args.dry_run:
        return 0

    ser = serial.Serial(args.port, args.baud, timeout=0, write_timeout=0.5)
    try:
        # Pre-stop once before motion, just to clear any stale state.
        for frame in stops:
            ser.write(frame)
            ser.flush()
            time.sleep(0.05)

        ser.write(run_frame)
        ser.flush()
        time.sleep(args.duration)

        for frame in stops:
            ser.write(frame)
            ser.flush()
            time.sleep(0.05)
    finally:
        try:
            for frame in stops:
                ser.write(frame)
                ser.flush()
                time.sleep(0.05)
        except Exception:
            pass
        ser.close()

    print("DONE")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
