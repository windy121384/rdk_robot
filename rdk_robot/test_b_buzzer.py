#!/usr/bin/env python3
"""Press B on the gamepad to beep once via STM32 serial."""
from __future__ import annotations

import argparse
import os
import struct
import time

import serial

from control_mapping import BTN_B
from protocol import buzzer, hexstr

JS_EVENT_BUTTON = 0x01
JS_EVENT_INIT = 0x80


def read_events(device: str):
    with open(device, "rb", buffering=0) as js:
        while True:
            data = js.read(8)
            if len(data) != 8:
                continue
            timestamp, value, event_type, number = struct.unpack("IhBB", data)
            yield timestamp, value, event_type, number


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--device", default="/dev/input/js0")
    parser.add_argument("--port", default="/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B32012490-if00")
    parser.add_argument("--baud", type=int, default=1000000)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    if not os.path.exists(args.device):
        raise SystemExit(f"joystick not found: {args.device}")
    if not args.dry_run and not os.path.exists(args.port):
        raise SystemExit(f"serial port not found: {args.port}")

    frame = buzzer(1000, 200, 200, 1)
    stop_frame = buzzer(0, 0, 0, 0)
    print(f"B press will send: {hexstr(frame)}")
    print(f"stop frame: {hexstr(stop_frame)}")
    ser = None if args.dry_run else serial.Serial(args.port, args.baud, timeout=0)
    last_b = 0
    last_beep = 0.0
    try:
        print("listening; press B to beep, Ctrl+C to exit")
        for _, value, event_type, number in read_events(args.device):
            clean_type = event_type & ~JS_EVENT_INIT
            if clean_type != JS_EVENT_BUTTON or number != BTN_B:
                continue
            if value == 1 and last_b == 0 and time.monotonic() - last_beep > 0.5:
                last_beep = time.monotonic()
                if ser:
                    ser.write(frame)
                    ser.flush()
                    time.sleep(0.35)
                    ser.write(stop_frame)
                    ser.flush()
                print(f"B pressed -> {'sent' if ser else 'dry-run'} {hexstr(frame)}", flush=True)
            last_b = value
    finally:
        if ser:
            ser.close()


if __name__ == "__main__":
    raise SystemExit(main())
