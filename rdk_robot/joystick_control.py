#!/usr/bin/env python3
"""Read the gamepad and print project-specific robot control values."""
from __future__ import annotations

import argparse
import os
import struct
import time

from control_mapping import BTN_A, BTN_X, BTN_Y, GamepadState, RobotMode, make_command

JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
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
    parser.add_argument("--rate", type=float, default=10.0, help="print rate in Hz")
    args = parser.parse_args()

    if not os.path.exists(args.device):
        raise SystemExit(f"joystick not found: {args.device}")

    state = GamepadState()
    mode = RobotMode()
    last_buttons = [0] * 11
    last_print = 0.0
    interval = 1.0 / max(1.0, args.rate)

    print(
        f"reading {args.device}; "
        "A=U/line shape, X=mecanum/differential, Y=camera raise, RT=speed"
    )
    for _, value, event_type, number in read_events(args.device):
        clean_type = event_type & ~JS_EVENT_INIT
        if clean_type == JS_EVENT_AXIS and number < len(state.axes):
            state.axes[number] = value
        elif clean_type == JS_EVENT_BUTTON and number < len(state.buttons):
            state.buttons[number] = value
            if value == 1 and last_buttons[number] == 0:
                if number == BTN_A:
                    mode.is_u_shape = not mode.is_u_shape
                elif number == BTN_X:
                    mode.mecanum_mode = not mode.mecanum_mode
                elif number == BTN_Y:
                    mode.camera_raised = not mode.camera_raised
            last_buttons[number] = value

        now = time.monotonic()
        if now - last_print >= interval:
            cmd = make_command(state, mode)
            print(
                f"shape={'U' if cmd.is_u_shape else 'LINE'} "
                f"drive={'MECANUM' if cmd.mecanum_mode else 'DIFF'} "
                f"camera={'UP' if cmd.camera_raised else 'DOWN'} "
                f"vx={cmd.vx:+.2f} vy={cmd.vy:+.2f} wz={cmd.wz:+.2f} "
                f"pan={cmd.camera_pan:+.2f} speed={cmd.speed_scale:.2f}",
                flush=True,
            )
            last_print = now


if __name__ == "__main__":
    raise SystemExit(main())
