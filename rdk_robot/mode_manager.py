#!/usr/bin/env python3
"""Manual/auto mode manager for the morphing inspection robot.

Power-on default is MANUAL. Button B toggles AUTO only when the robot is in
U-shape and the camera is raised. This module is intentionally conservative:
by default it prints the final decision and does not write STM32 frames.
"""
from __future__ import annotations

import argparse
import json
import os
import select
import struct
import time
import urllib.request
from dataclasses import dataclass
from enum import Enum

import serial

from ai_decision import decide, load_perception
from control_mapping import AXIS_RX, BTN_A, BTN_B, BTN_X, BTN_Y, GamepadState, RobotMode, make_command
from protocol import buzzer, bus_servo_one, hexstr, motor_many, motor_stop_mask, pwm_servo_many, pwm_servo_one
from robot_teleop import (
    CAMERA_PAN_MAX_US,
    CAMERA_PAN_MIN_US,
    CAMERA_PAN_START_US,
    CAMERA_PAN_STEP_US,
    CAMERA_TILT_HORIZONTAL_US,
    CAMERA_TILT_VERTICAL_US,
    CAMERA_LIFT_UP_US,
    CAMERA_LIFT_DOWN_US,
    BUS_CAMERA_LIFT,
    LINE_SHAPE_PULSES,
    MOTOR_IDS,
    SERVO_MOVE_MS,
    STOP_MASK_ALL,
    TURN_GAIN,
    U_SHAPE_PULSES,
    clamp,
    mix_motors,
)

JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
JS_EVENT_INIT = 0x80
JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
BUTTON_NAMES = {BTN_A: "A", BTN_B: "B", BTN_X: "X", BTN_Y: "Y"}


class RunMode(str, Enum):
    MANUAL = "MANUAL"
    AUTO = "AUTO"
    ESTOP = "ESTOP"


@dataclass
class ManagerState:
    robot_mode: RobotMode
    run_mode: RunMode = RunMode.MANUAL
    last_auto_reject: str = ""


def read_pending_events_js(js):
    """Read js0 events (8 bytes: time + value + type + number)"""
    events = []
    while True:
        ready, _, _ = select.select([js], [], [], 0)
        if not ready:
            return events
        data = js.read(8)
        if not data or len(data) != 8:
            return events
        events.append(struct.unpack("IhBB", data))


def can_enter_auto(robot_mode: RobotMode) -> tuple[bool, str]:
    if not robot_mode.is_u_shape:
        return False, "需要 U 型形态"
    if not robot_mode.camera_raised:
        return False, "需要摄像头展开"
    return True, "ok"


def event_frames(prev: RobotMode, mode: RobotMode):
    frames = []
    if prev.is_u_shape != mode.is_u_shape:
        frames.append(pwm_servo_many(U_SHAPE_PULSES if mode.is_u_shape else LINE_SHAPE_PULSES, time_ms=SERVO_MOVE_MS))
    if prev.camera_raised != mode.camera_raised:
        tilt = CAMERA_TILT_HORIZONTAL_US if mode.camera_raised else CAMERA_TILT_VERTICAL_US
        if mode.camera_raised:
            # 抬升：先总线舵机抬升，再PWM舵机转水平
            frames.append(("lift", bus_servo_one(BUS_CAMERA_LIFT, CAMERA_LIFT_UP_US, time_ms=1500)))
            frames.append(("tilt", pwm_servo_one(4, CAMERA_TILT_HORIZONTAL_US, time_ms=SERVO_MOVE_MS)))
        else:
            # 放下：先云台回中，再PWM舵机转竖直，再总线舵机回落
            frames.append(("pan_center", pwm_servo_one(3, CAMERA_PAN_START_US, time_ms=SERVO_MOVE_MS)))
            frames.append(("tilt", pwm_servo_one(4, CAMERA_TILT_VERTICAL_US, time_ms=SERVO_MOVE_MS)))
            frames.append(("lift", bus_servo_one(BUS_CAMERA_LIFT, CAMERA_LIFT_DOWN_US, time_ms=1500)))
    return frames


def stop_all_frames() -> list[bytes]:
    return [
        motor_stop_mask(STOP_MASK_ALL),
        motor_many({0: 0.0, 1: 0.0, 2: 0.0, 3: 0.0}),
    ]


def auto_motor_speeds(action: str, max_speed: float) -> dict[int, float] | None:
    speed = max(0.0, min(0.35, float(max_speed or 0.0)))
    if action == "STOP":
        return None
    if action in {"SLOW", "CLEAR", "MORPH_LINE"}:
        return mix_motors(speed, 0.0, 0.0, mecanum=False)
    if action == "TURN_LEFT":
        return mix_motors(0.0, 0.0, 0.22, mecanum=False)
    if action == "TURN_RIGHT":
        return mix_motors(0.0, 0.0, -0.22, mecanum=False)
    return None


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--device", default="/dev/input/js0")
    parser.add_argument("--port", default="/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B32012490-if00")
    parser.add_argument("--baud", type=int, default=1000000)
    parser.add_argument("--source", default="http://127.0.0.1:8091/result.json")
    parser.add_argument("--rate", type=float, default=20.0)
    parser.add_argument("--send", action="store_true", help="actually write frames to STM32")
    parser.add_argument("--dry-run", action="store_true", help="print frames without sending")
    parser.add_argument("--allow-auto-motion", action="store_true", help="allow AUTO to send SLOW/TURN motor frames; STOP is always allowed")
    args = parser.parse_args()

    if not os.path.exists(args.device):
        raise SystemExit(f"joystick not found: {args.device}")
    if args.send and not os.path.exists(args.port):
        raise SystemExit(f"serial port not found: {args.port}")

    ser = serial.Serial(args.port, args.baud, timeout=0, write_timeout=1.0) if args.send else None
    gamepad = GamepadState()
    manager = ManagerState(robot_mode=RobotMode(is_u_shape=True, mecanum_mode=False, camera_raised=False))
    last_robot_mode = RobotMode(manager.robot_mode.is_u_shape, manager.robot_mode.mecanum_mode, manager.robot_mode.camera_raised)
    last_buttons = [0] * 11
    last_send = 0.0
    last_status = 0.0
    last_pan_send = 0.0
    interval = 1.0 / max(1.0, args.rate)
    pan_interval = 1.0 / 12.0  # 12 Hz for pan updates
    camera_pan_us = CAMERA_PAN_START_US

    print("mode_manager started; power-on mode=MANUAL; B toggles AUTO only if U-shape + camera raised")

    def emit(frame: bytes, label: str):
        if ser:
            try:
                bytes_written = ser.write(frame)
                ser.flush()
                if label == "manual_motor":
                    print(f"DEBUG emit: wrote {bytes_written}/{len(frame)} bytes for {label}, data={frame.hex()}", flush=True)
            except serial.SerialTimeoutException:
                print(f"WARN serial write timeout on {label}; frame dropped", flush=True)
                return
            except serial.SerialException as exc:
                print(f"WARN serial error on {label}: {exc!r}; frame dropped", flush=True)
                return
        if args.dry_run or not args.send:
            print(f"{label}: {hexstr(frame)}")

    js = open(args.device, "rb", buffering=0)
    try:
            while True:
                for _, value, event_type, number in read_pending_events_js(js):
                    is_init = bool(event_type & JS_EVENT_INIT)
                    event_type &= ~JS_EVENT_INIT  # 去掉 INIT 标志
                    
                    if event_type == JS_EVENT_AXIS and number < len(gamepad.axes):
                        gamepad.axes[number] = value
                    elif event_type == JS_EVENT_BUTTON and number < len(gamepad.buttons):
                        gamepad.buttons[number] = value
                        # INIT 事件不触发按键动作
                        if is_init:
                            last_buttons[number] = value
                            continue
                        pressed = value == 1 and last_buttons[number] == 0
                        if pressed:
                            if number == BTN_A and manager.run_mode == RunMode.MANUAL:
                                manager.robot_mode.is_u_shape = not manager.robot_mode.is_u_shape
                            elif number == BTN_X and manager.run_mode == RunMode.MANUAL:
                                manager.robot_mode.mecanum_mode = not manager.robot_mode.mecanum_mode
                            elif number == BTN_Y and manager.run_mode == RunMode.MANUAL:
                                manager.robot_mode.camera_raised = not manager.robot_mode.camera_raised
                            elif number == BTN_B:
                                if manager.run_mode == RunMode.AUTO:
                                    manager.run_mode = RunMode.MANUAL
                                    manager.last_auto_reject = "B pressed: return MANUAL"
                                    for idx, frame in enumerate(stop_all_frames()):
                                        emit(frame, f"auto_exit_stop_{idx}")
                                else:
                                    ok, reason = can_enter_auto(manager.robot_mode)
                                    if ok:
                                        manager.run_mode = RunMode.AUTO
                                        manager.last_auto_reject = "AUTO enabled"
                                        emit(buzzer(1200, 100, 80, 2), "auto_on_buzzer")
                                    else:
                                        manager.run_mode = RunMode.MANUAL
                                        manager.last_auto_reject = f"AUTO rejected: {reason}"
                                        emit(buzzer(500, 150, 80, 2), "auto_reject_buzzer")
                            print(
                                f"button {BUTTON_NAMES.get(number, number)}; mode={manager.run_mode.value}; "
                                f"shape={'U' if manager.robot_mode.is_u_shape else 'LINE'}; "
                                f"camera={'UP' if manager.robot_mode.camera_raised else 'DOWN'}; {manager.last_auto_reject}",
                                flush=True,
                            )
                        last_buttons[number] = value

                for frame in event_frames(last_robot_mode, manager.robot_mode):
                    if isinstance(frame, tuple):
                        label, data = frame
                        emit(data, f"servo_{label}")
                        if label == "lift":
                            time.sleep(1.5 + 0.2)
                        elif label == "tilt":
                            time.sleep(SERVO_MOVE_MS / 1000.0 + 0.2)
                        elif label == "pan_center":
                            time.sleep(SERVO_MOVE_MS / 1000.0 + 0.2)
                            camera_pan_us = CAMERA_PAN_START_US
                    else:
                        emit(frame, "servo")
                last_robot_mode = RobotMode(manager.robot_mode.is_u_shape, manager.robot_mode.mecanum_mode, manager.robot_mode.camera_raised)

                now = time.monotonic()
                if now - last_send >= interval:
                    if manager.run_mode == RunMode.MANUAL:
                        cmd = make_command(gamepad, manager.robot_mode)
                        speeds = mix_motors(cmd.vx, cmd.vy, cmd.wz, cmd.mecanum_mode and cmd.is_u_shape)
                        motion_active = abs(cmd.vx) > 0.02 or abs(cmd.vy) > 0.02 or abs(cmd.wz) > 0.02
                        if cmd.speed_scale <= 0.02 or not motion_active:
                            print(
                                f"manual_stop decision: speed={cmd.speed_scale:.2f} vx={cmd.vx:+.2f} vy={cmd.vy:+.2f} wz={cmd.wz:+.2f} motion_active={motion_active}",
                                flush=True,
                            )
                            for idx, frame in enumerate(stop_all_frames()):
                                emit(frame, f"manual_stop_{idx}")
                        else:
                            print(
                                f"manual_motor decision: speed={cmd.speed_scale:.2f} vx={cmd.vx:+.2f} vy={cmd.vy:+.2f} wz={cmd.wz:+.2f} motion_active={motion_active} speeds={speeds}",
                                flush=True,
                            )
                            emit(motor_many(speeds), "manual_motor")
                    # 右摇杆控制云台水平旋转（仅摄像头抬升时）
                    if manager.robot_mode.camera_raised:
                        pan_input = gamepad.axis_norm(AXIS_RX, deadzone=0.10)
                        if pan_input != 0.0 and now - last_pan_send >= pan_interval:
                            camera_pan_us = int(round(camera_pan_us + pan_input * CAMERA_PAN_STEP_US))
                            camera_pan_us = int(clamp(camera_pan_us, CAMERA_PAN_MIN_US, CAMERA_PAN_MAX_US))
                            emit(pwm_servo_one(3, camera_pan_us, time_ms=80), "camera_pan")
                            last_pan_send = now
                    else:
                        try:
                            perception = load_perception(args.source)
                            decision = decide(perception)
                        except Exception as exc:
                            decision = None
                            for idx, frame in enumerate(stop_all_frames()):
                                emit(frame, f"auto_sensor_error_stop_{idx}")
                            manager.last_auto_reject = f"AUTO sensor error: {exc!r}"
                        if decision:
                            if decision.action == "STOP" or not args.allow_auto_motion:
                                for idx, frame in enumerate(stop_all_frames()):
                                    emit(frame, f"auto_{decision.action.lower()}_safe_{idx}")
                            else:
                                speeds = auto_motor_speeds(decision.action, decision.max_speed)
                                if speeds is None:
                                    for idx, frame in enumerate(stop_all_frames()):
                                        emit(frame, f"auto_{decision.action.lower()}_stop_{idx}")
                                else:
                                    emit(motor_many(speeds), f"auto_{decision.action.lower()}_motor")
                    last_send = now

                if now - last_status >= 1.0:
                    print(
                        f"run_mode={manager.run_mode.value} "
                        f"shape={'U' if manager.robot_mode.is_u_shape else 'LINE'} "
                        f"camera={'UP' if manager.robot_mode.camera_raised else 'DOWN'} "
                        f"auto_gate={can_enter_auto(manager.robot_mode)[1]} "
                        f"note={manager.last_auto_reject}",
                        flush=True,
                    )
                    last_status = now
                time.sleep(0.005)
    finally:
        js.close()
        if ser:
            for frame in stop_all_frames():
                try:
                    ser.write(frame)
                    ser.flush()
                except Exception:
                    pass
            ser.close()


if __name__ == "__main__":
    raise SystemExit(main())
