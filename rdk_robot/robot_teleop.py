#!/usr/bin/env python3
"""Gamepad teleoperation with optional STM32 serial output."""
from __future__ import annotations

import argparse
import os
import select
import struct
import time

import serial

from control_mapping import AXIS_RX, BTN_A, BTN_B, BTN_X, BTN_Y, GamepadState, RobotMode, make_command
from protocol import hexstr, motor_many, motor_stop_mask, pwm_servo_many, pwm_servo_one, buzzer

JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
JS_EVENT_INIT = 0x80
JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
BUTTON_NAMES = {BTN_A: "A", BTN_B: "B", BTN_X: "X", BTN_Y: "Y"}

# TODO: tune these IDs/pulse widths to match the real STM32 wiring.
MOTOR_IDS = (0, 1, 2, 3)
STOP_MASK_ALL = 0x0F

PWM_SHAPE_LEFT = 1
PWM_SHAPE_RIGHT = 2
PWM_CAMERA_PAN = 3
PWM_CAMERA_TILT = 4
BUS_CAMERA_LIFT = 1

U_SHAPE_PULSES = {PWM_SHAPE_LEFT: 1000, PWM_SHAPE_RIGHT: 2000}
LINE_SHAPE_PULSES = {PWM_SHAPE_LEFT: 2000, PWM_SHAPE_RIGHT: 1000}
CAMERA_TILT_VERTICAL_US = 1500
CAMERA_TILT_HORIZONTAL_US = 2500
CAMERA_LIFT_UP_US = 2000
CAMERA_LIFT_DOWN_US = 1000
CAMERA_PAN_START_US = 1500
CAMERA_PAN_MIN_US = 500
CAMERA_PAN_MAX_US = 2500
CAMERA_PAN_STEP_US = 50
SERVO_MOVE_MS = 600
MAX_MOTOR_SPEED = 1.0
TURN_GAIN = 1.6
DEFAULT_STATUS_RATE_HZ = 5.0
DEFAULT_STOP_RATE_HZ = 2.0
DEFAULT_PAN_RATE_HZ = 12.0


def read_pending_events_js(js):
    """Read evdev events (24 bytes: timeval + type + code + value)"""
    events = []
    fd = js.fileno()
    while True:
        ready, _, _ = select.select([js], [], [], 0)
        if not ready:
            return events
        data = os.read(fd, 24)
        if not data or len(data) != 24:
            return events
        # Parse: timeval(16 bytes) + type(2) + code(2) + value(4)
        tv_sec, tv_usec, typ, code, value = struct.unpack('llHHi', data)
        events.append((0, value, typ, code))  # (time, value, type, number)


def clamp(value: float, low: float = -1.0, high: float = 1.0) -> float:
    return max(low, min(high, value))


def mix_motors(vx: float, vy: float, wz: float, mecanum: bool) -> dict[int, float]:
    wz = clamp(wz * TURN_GAIN)
    if mecanum:
        speeds = [vx - vy - wz, vx + vy + wz, vx + vy - wz, vx - vy + wz]
    else:
        left = vx - wz
        right = vx + wz
        # Right side motors (ID 1, 3) are mirrored, so negate their speed
        speeds = [left, -right, left, -right]
    max_abs = max(1.0, *(abs(s) for s in speeds))
    return {motor_id: clamp(speed / max_abs) * MAX_MOTOR_SPEED for motor_id, speed in zip(MOTOR_IDS, speeds)}


def event_frames(prev: RobotMode, mode: RobotMode):
    frames = []
    if prev.is_u_shape != mode.is_u_shape:
        pulses = U_SHAPE_PULSES if mode.is_u_shape else LINE_SHAPE_PULSES
        frames.append(pwm_servo_many(pulses, time_ms=SERVO_MOVE_MS))
    if prev.camera_raised != mode.camera_raised:
        # Y toggles camera posture between stowed/vertical and raised/horizontal.
        tilt = CAMERA_TILT_HORIZONTAL_US if mode.camera_raised else CAMERA_TILT_VERTICAL_US
        frames.append(pwm_servo_one(PWM_CAMERA_TILT, tilt, time_ms=SERVO_MOVE_MS))
    return frames


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--device", default="/dev/input/js0")
    parser.add_argument("--port", default="/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B32012490-if00")
    parser.add_argument("--baud", type=int, default=1000000)
    parser.add_argument("--rate", type=float, default=20.0)
    parser.add_argument("--status-rate", type=float, default=DEFAULT_STATUS_RATE_HZ)
    parser.add_argument("--stop-rate", type=float, default=DEFAULT_STOP_RATE_HZ)
    parser.add_argument("--pan-rate", type=float, default=DEFAULT_PAN_RATE_HZ)
    parser.add_argument("--send", action="store_true", help="actually write frames to STM32")
    parser.add_argument("--dry-run", action="store_true", help="print frames without sending")
    args = parser.parse_args()

    if not os.path.exists(args.device):
        raise SystemExit(f"joystick not found: {args.device}")
    if args.send and not os.path.exists(args.port):
        raise SystemExit(f"serial port not found: {args.port}")

    ser = serial.Serial(args.port, args.baud, timeout=0, write_timeout=0.2) if args.send else None
    state = GamepadState()
    mode = RobotMode()
    last_mode = RobotMode(mode.is_u_shape, mode.mecanum_mode, mode.camera_raised)
    last_buttons = [0] * 11
    last_send = 0.0
    last_stop_send = 0.0
    last_pan_send = 0.0
    last_status_print = 0.0
    camera_pan_us = CAMERA_PAN_START_US
    interval = 1.0 / max(1.0, args.rate)
    stop_interval = 1.0 / max(0.1, args.stop_rate)
    pan_interval = 1.0 / max(0.1, args.pan_rate)
    status_interval = 1.0 / max(0.1, args.status_rate)

    print(
        f"teleop {args.device}; serial={args.port}@{args.baud}; "
        f"send={bool(args.send)}; A=shape X=drive Y=camera RT=speed"
    )

    def emit(frame: bytes, label: str):
        if ser:
            try:
                ser.write(frame)
                ser.flush()
            except serial.SerialTimeoutException:
                print(f"WARN serial write timeout on {label}; frame dropped", flush=True)
                return
        if args.dry_run or not args.send:
            print(f"{label}: {hexstr(frame)}")

    js = open(args.device, "rb", buffering=0)
    try:
            while True:
                for _, value, event_type, number in read_pending_events_js(js):
                    if event_type == JS_EVENT_AXIS and number < len(state.axes):
                        state.axes[number] = value
                    elif event_type == JS_EVENT_BUTTON and number < len(state.buttons):
                        state.buttons[number] = value
                        if value == 1 and last_buttons[number] == 0:
                            if number == BTN_A:
                                mode.is_u_shape = not mode.is_u_shape
                            elif number == BTN_X:
                                mode.mecanum_mode = not mode.mecanum_mode
                            elif number == BTN_Y:
                                mode.camera_raised = not mode.camera_raised
                            elif number == BTN_B:
                                emit(buzzer(1000, 200, 200, 1), "buzzer")
                            print(
                                f"button {BUTTON_NAMES.get(number, number)} pressed; "
                                f"camera={'UP' if mode.camera_raised else 'DOWN'}",
                                flush=True,
                            )
                        last_buttons[number] = value

                for frame in event_frames(last_mode, mode):
                    emit(frame, "servo")
                last_mode = RobotMode(mode.is_u_shape, mode.mecanum_mode, mode.camera_raised)

                now = time.monotonic()
                if now - last_send >= interval:
                    cmd = make_command(state, mode)
                    speeds = mix_motors(cmd.vx, cmd.vy, cmd.wz, cmd.mecanum_mode and cmd.is_u_shape)
                    motion_active = abs(cmd.vx) > 0.02 or abs(cmd.vy) > 0.02 or abs(cmd.wz) > 0.02
                    if cmd.speed_scale <= 0.02 or not motion_active:
                        if now - last_stop_send >= stop_interval:
                            emit(motor_stop_mask(STOP_MASK_ALL), "motor_stop")
                            last_stop_send = now
                    else:
                        emit(motor_many(speeds), "motor")
                    if cmd.camera_raised:
                        # Right-stick X is velocity control: release to hold, not return to center.
                        pan_input = state.axis_norm(AXIS_RX, deadzone=0.10)
                        if pan_input != 0.0 and now - last_pan_send >= pan_interval:
                            camera_pan_us = int(round(camera_pan_us + pan_input * CAMERA_PAN_STEP_US))
                            camera_pan_us = int(clamp(camera_pan_us, CAMERA_PAN_MIN_US, CAMERA_PAN_MAX_US))
                            emit(pwm_servo_one(PWM_CAMERA_PAN, camera_pan_us, time_ms=80), "camera_pan")
                            last_pan_send = now
                    if now - last_status_print >= status_interval:
                        print(
                            f"shape={'U' if cmd.is_u_shape else 'LINE'} "
                            f"drive={'MECANUM' if cmd.mecanum_mode else 'DIFF'} "
                            f"camera={'UP' if cmd.camera_raised else 'DOWN'} "
                            f"vx={cmd.vx:+.2f} vy={cmd.vy:+.2f} wz={cmd.wz:+.2f} "
                            f"pan={camera_pan_us} speed={cmd.speed_scale:.2f}",
                            flush=True,
                        )
                        last_status_print = now
                    last_send = now
                time.sleep(0.005)
    finally:
        js.close()
        if ser:
            ser.write(motor_stop_mask(STOP_MASK_ALL))
            ser.flush()
            ser.close()


if __name__ == "__main__":
    raise SystemExit(main())
