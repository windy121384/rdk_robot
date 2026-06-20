#!/usr/bin/env python3
"""
手动模式控制程序 v2
- RT扳机: 油门（0~32767 → 0~1）
- 左摇杆X: 转向
- 左摇杆Y: 前进/后退方向
- A键: 切换形态（U字型/一字型）
- Y键: 摄像头抬升/放下
- B键: 切换手动/自动模式（需U字型+摄像头抬升）
- 右摇杆X: 云台水平旋转（仅摄像头抬升时）
"""

import struct, os, select, time
import serial
import json
import urllib.request
from protocol import (
    motor_many, motor_stop_mask, pwm_servo_one, pwm_servo_many,
    bus_servo_one, buzzer, hexstr
)
from ai_decision import decide, load_perception

# ── 手柄配置 ──
JS_DEVICE = '/dev/input/js0'
JS_EVENT_AXIS = 0x02
JS_EVENT_BUTTON = 0x01
JS_EVENT_INIT = 0x80

AXIS_LX = 0   # 左摇杆X
AXIS_LY = 1   # 左摇杆Y
AXIS_RX = 3   # 右摇杆X
AXIS_RT = 5   # RT扳机
BTN_A = 0
BTN_B = 1
BTN_Y = 3

# ── 串口配置 ──
SERIAL_PORT = '/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B32012490-if00'
BAUD_RATE = 1000000
PERCEPTION_URL = "http://127.0.0.1:8091/result.json"

# ── 控制参数 ──
DEADZONE = 0.10
MAX_SPEED = 0.7
TURN_SCALE = 0.35
CONTROL_INTERVAL = 0.025

# ── 舵机配置 ──
SERVO_MOVE_MS = 600
CAMERA_TILT_UP = 500
CAMERA_TILT_DOWN = 1500
CAMERA_LIFT_UP = 2150
CAMERA_LIFT_DOWN = 1500
U_SHAPE = {1: 1600, 2: 1550}
LINE_SHAPE = {1: 600, 2: 2200}
CAMERA_PAN_CENTER = 1500
CAMERA_PAN_MIN = 500
CAMERA_PAN_MAX = 2500
CAMERA_PAN_STEP = 50


def normalize_trigger(value):
    if abs(value) < 2000:
        return 0.0
    return max(0.0, min(1.0, value / 32767.0))


def normalize_axis(value):
    return value / 32767.0


def deadzone(value, dz=DEADZONE):
    if abs(value) < dz:
        return 0.0
    return value


def sw(ser, data):
    """串口写入+冲刷，write_timeout=500ms失败快速崩"""
    ser.write(data)
    ser.flush()


def main():
    print("=" * 50)
    print("手动模式控制程序 v2")
    print("=" * 50)

    try:
        js = open(JS_DEVICE, "rb", buffering=0)
        print(f"✓ 手柄: {JS_DEVICE}")
    except Exception as e:
        print(f"✗ 手柄打开失败: {e}")
        return

    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1, write_timeout=0.5)
        print(f"✓ 串口: {SERIAL_PORT}")
    except Exception as e:
        print(f"✗ 串口打开失败: {e}")
        js.close()
        return

    axes = [0] * 8
    buttons = [0] * 16
    last_buttons = [0] * 16
    is_u_shape = True
    camera_raised = False
    is_auto = False
    camera_pan = CAMERA_PAN_CENTER
    last_send = 0
    last_pan_send = 0
    last_print = 0
    print_interval = 1.0

    print("\n控制说明:")
    print("  RT扳机 = 油门    左摇杆X = 转向    左摇杆Y = 前进/后退")
    print("  A = 切换形态    Y = 摄像头升降    B = 手动/自动")
    print("  右摇杆X = 云台旋转（摄像头抬升时）")
    print("  Ctrl+C 退出\n")

    try:
        while True:
            ready, _, _ = select.select([js], [], [], 0.01)
            if ready:
                data = js.read(8)
                if data and len(data) == 8:
                    tv, value, etype, num = struct.unpack("IhBB", data)
                    if etype & JS_EVENT_INIT:
                        continue
                    etype &= ~JS_EVENT_INIT

                    if etype == JS_EVENT_AXIS and num < len(axes):
                        axes[num] = value

                    elif etype == JS_EVENT_BUTTON and num < len(buttons):
                        buttons[num] = value
                        if value == 1 and last_buttons[num] == 0:
                            if num == BTN_A:
                                is_u_shape = not is_u_shape
                                sw(ser, pwm_servo_many(U_SHAPE if is_u_shape else LINE_SHAPE, time_ms=SERVO_MOVE_MS))
                                print(f"\n[形态] {'U字型' if is_u_shape else '一字型'}")

                            elif num == BTN_Y:
                                if is_auto:
                                    print("\n[警告] 自动模式下无法操作摄像头")
                                    continue
                                camera_raised = not camera_raised
                                if camera_raised:
                                    sw(ser, bus_servo_one(1, CAMERA_LIFT_UP, 1500))
                                    time.sleep(1.7)
                                    sw(ser, pwm_servo_one(4, CAMERA_TILT_UP, SERVO_MOVE_MS))
                                else:
                                    sw(ser, pwm_servo_one(3, CAMERA_PAN_CENTER, SERVO_MOVE_MS))
                                    camera_pan = CAMERA_PAN_CENTER
                                    time.sleep(0.8)
                                    sw(ser, pwm_servo_one(4, CAMERA_TILT_DOWN, SERVO_MOVE_MS))
                                    time.sleep(0.8)
                                    sw(ser, bus_servo_one(1, CAMERA_LIFT_DOWN, 1500))
                                print(f"\n[摄像头] {'抬升' if camera_raised else '放下'}")

                            elif num == BTN_B:
                                if is_auto:
                                    is_auto = False
                                    sw(ser, motor_stop_mask(0x0F))
                                    sw(ser, buzzer(800, 150, 80, 2))
                                    print("\n[模式] 手动模式")
                                else:
                                    if not is_u_shape:
                                        sw(ser, buzzer(300, 200, 100, 2))
                                        print("\n[拒绝] 需要U字型形态")
                                    elif not camera_raised:
                                        sw(ser, buzzer(300, 200, 100, 2))
                                        print("\n[拒绝] 需要摄像头抬升")
                                    else:
                                        is_auto = True
                                        sw(ser, buzzer(1200, 300, 100, 1))
                                        print("\n[模式] 自动模式")

                        last_buttons[num] = value

            now = time.time()
            if camera_raised and not is_auto:
                rx = deadzone(normalize_axis(axes[AXIS_RX]), 0.15)
                if abs(rx) > 0 and now - last_pan_send >= 0.08:
                    camera_pan += int(rx * CAMERA_PAN_STEP)
                    camera_pan = max(CAMERA_PAN_MIN, min(CAMERA_PAN_MAX, camera_pan))
                    sw(ser, pwm_servo_one(3, camera_pan, 80))
                    last_pan_send = now

            if now - last_send < CONTROL_INTERVAL:
                continue
            last_send = now

            if is_auto:
                try:
                    perception = load_perception(PERCEPTION_URL)
                    decision = decide(perception)
                    action = decision.action
                    max_speed = decision.max_speed
                    if action == "STOP":
                        sw(ser, motor_stop_mask(0x0F))
                        print(f"[自动] STOP 前方障碍  ", end='\r')
                    elif action == "SLOW":
                        speeds = motor_many({0: max_speed, 1: -max_speed if is_u_shape else max_speed, 2: max_speed, 3: -max_speed if is_u_shape else max_speed})
                        sw(ser, speeds)
                        print(f"[自动] SLOW 速度:{max_speed:.2f}  ", end='\r')
                    elif action == "CLEAR":
                        speeds = motor_many({0: max_speed, 1: -max_speed if is_u_shape else max_speed, 2: max_speed, 3: -max_speed if is_u_shape else max_speed})
                        sw(ser, speeds)
                        print(f"[自动] CLEAR 速度:{max_speed:.2f}  ", end='\r')
                    elif action in ("TURN_LEFT", "TURN_RIGHT"):
                        turn_speed = 0.3
                        if action == "TURN_LEFT":
                            speeds = motor_many({0: turn_speed, 1: turn_speed if is_u_shape else -turn_speed, 2: -turn_speed, 3: -turn_speed if is_u_shape else turn_speed})
                        else:
                            speeds = motor_many({0: -turn_speed, 1: -turn_speed if is_u_shape else turn_speed, 2: turn_speed, 3: turn_speed if is_u_shape else -turn_speed})
                        sw(ser, speeds)
                        print(f"[自动] {action}  ", end='\r')
                    else:
                        sw(ser, motor_stop_mask(0x0F))
                        print(f"[自动] {action}  ", end='\r')
                except Exception as e:
                    try:
                        sw(ser, motor_stop_mask(0x0F))
                    except Exception:
                        pass
                    print(f"[自动] 感知错误: {e}  ", end='\r')
                continue

            rt = normalize_trigger(axes[AXIS_RT])
            lx = deadzone(normalize_axis(axes[AXIS_LX]))
            ly = deadzone(normalize_axis(axes[AXIS_LY]))

            speed = rt * MAX_SPEED * (-1.0 if ly > 0 else 1.0)
            turn = -lx * TURN_SCALE * MAX_SPEED

            if is_u_shape:
                m0 = speed + turn
                m1 = -speed + turn
                m2 = speed - turn
                m3 = -speed - turn
            else:
                m0 = speed + turn
                m1 = speed - turn
                m2 = speed - turn
                m3 = speed + turn
            m0 = max(-MAX_SPEED, min(MAX_SPEED, m0))
            m1 = max(-MAX_SPEED, min(MAX_SPEED, m1))
            m2 = max(-MAX_SPEED, min(MAX_SPEED, m2))
            m3 = max(-MAX_SPEED, min(MAX_SPEED, m3))
            speeds = {0: m0, 1: m1, 2: m2, 3: m3}

            if abs(speed) > 0.01 or abs(turn) > 0.01:
                try:
                    sw(ser, motor_many(speeds))
                except Exception:
                    pass
                if now - last_print >= print_interval:
                    d = "前进" if speed > 0 else "后退"
                    print(f"{d} 速度:{abs(speed):.2f} 转向:{turn:+.2f} [{('U' if is_u_shape else 'L')}]", flush=True)
                    last_print = now
            else:
                try:
                    sw(ser, motor_stop_mask(0x0F))
                except Exception:
                    pass
                if now - last_print >= print_interval:
                    print(f"  [{'U' if is_u_shape else 'L'}]", flush=True)
                    last_print = now

    except KeyboardInterrupt:
        print("\n\n退出中...")

    finally:
        try:
            sw(ser, motor_stop_mask(0x0F))
        except Exception:
            pass
        if camera_raised:
            try:
                sw(ser, pwm_servo_one(3, CAMERA_PAN_CENTER, SERVO_MOVE_MS))
            except Exception:
                pass
            camera_pan = CAMERA_PAN_CENTER
            time.sleep(0.8)
            try:
                sw(ser, pwm_servo_one(4, CAMERA_TILT_DOWN, SERVO_MOVE_MS))
            except Exception:
                pass
            time.sleep(0.8)
            try:
                sw(ser, bus_servo_one(1, CAMERA_LIFT_DOWN, 1500))
            except Exception:
                pass
        js.close()
        ser.close()
        print("✓ 已停止")


if __name__ == '__main__':
    main()