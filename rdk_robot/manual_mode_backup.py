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
from protocol import (
    motor_many, motor_stop_mask, pwm_servo_one, pwm_servo_many,
    bus_servo_one, buzzer, hexstr
)

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

# ── 控制参数 ──
DEADZONE = 0.10
MAX_SPEED = 1.0

# ── 舵机配置 ──
SERVO_MOVE_MS = 600
CAMERA_TILT_UP = 2500
CAMERA_TILT_DOWN = 1500
CAMERA_LIFT_UP = 2000
CAMERA_LIFT_DOWN = 1000
U_SHAPE = {1: 1000, 2: 2000}
LINE_SHAPE = {1: 2000, 2: 1000}
CAMERA_PAN_CENTER = 1500
CAMERA_PAN_MIN = 500
CAMERA_PAN_MAX = 2500
CAMERA_PAN_STEP = 50


def normalize_trigger(value):
    """扳机归一化: 松开=0, 按下=1"""
    return max(0.0, min(1.0, value / 32767.0))


def normalize_axis(value):
    """摇杆归一化: 左/上=-1, 右/下=+1"""
    return value / 32767.0


def deadzone(value, dz=DEADZONE):
    if abs(value) < dz:
        return 0.0
    return value


def main():
    print("=" * 50)
    print("手动模式控制程序 v2")
    print("=" * 50)

    # 打开手柄
    try:
        js = open(JS_DEVICE, "rb", buffering=0)
        print(f"✓ 手柄: {JS_DEVICE}")
    except Exception as e:
        print(f"✗ 手柄打开失败: {e}")
        return

    # 打开串口
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1, write_timeout=1.0)
        print(f"✓ 串口: {SERIAL_PORT}")
    except Exception as e:
        print(f"✗ 串口打开失败: {e}")
        js.close()
        return

    # 状态
    axes = [0] * 8
    buttons = [0] * 16
    last_buttons = [0] * 16
    is_u_shape = True
    camera_raised = False
    is_auto = False  # 手动/自动模式
    camera_pan = CAMERA_PAN_CENTER
    last_send = 0
    last_pan_send = 0

    print("\n控制说明:")
    print("  RT扳机 = 油门    左摇杆X = 转向    左摇杆Y = 前进/后退")
    print("  A = 切换形态    Y = 摄像头升降    B = 手动/自动")
    print("  右摇杆X = 云台旋转（摄像头抬升时）")
    print("  Ctrl+C 退出\n")

    try:
        while True:
            # ── 读取手柄事件 ──
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
                        # 按键按下检测
                        if value == 1 and last_buttons[num] == 0:
                            if num == BTN_A:
                                is_u_shape = not is_u_shape
                                pulses = U_SHAPE if is_u_shape else LINE_SHAPE
                                ser.write(pwm_servo_many(pulses, time_ms=SERVO_MOVE_MS))
                                ser.flush()
                                print(f"\n[形态] {'U字型' if is_u_shape else '一字型'}")

                            elif num == BTN_Y:
                                if is_auto:
                                    print("\n[警告] 自动模式下无法操作摄像头")
                                    continue
                                camera_raised = not camera_raised
                                if camera_raised:
                                    ser.write(bus_servo_one(1, CAMERA_LIFT_UP, 1500))
                                    ser.flush()
                                    time.sleep(1.7)
                                    ser.write(pwm_servo_one(4, CAMERA_TILT_UP, SERVO_MOVE_MS))
                                    ser.flush()
                                else:
                                    # 先云台回中
                                    ser.write(pwm_servo_one(3, CAMERA_PAN_CENTER, SERVO_MOVE_MS))
                                    ser.flush()
                                    camera_pan = CAMERA_PAN_CENTER
                                    time.sleep(0.8)
                                    # 再摄像头转竖直
                                    ser.write(pwm_servo_one(4, CAMERA_TILT_DOWN, SERVO_MOVE_MS))
                                    ser.flush()
                                    time.sleep(0.8)
                                    # 最后总线舵机回落
                                    ser.write(bus_servo_one(1, CAMERA_LIFT_DOWN, 1500))
                                    ser.flush()
                                print(f"\n[摄像头] {'抬升' if camera_raised else '放下'}")

                            elif num == BTN_B:
                                if is_auto:
                                    # 退出自动模式
                                    is_auto = False
                                    ser.write(motor_stop_mask(0x0F))
                                    ser.flush()
                                    ser.write(buzzer(800, 150, 80, 2))
                                    ser.flush()
                                    print("\n[模式] 手动模式")
                                else:
                                    # 尝试进入自动模式
                                    if not is_u_shape:
                                        ser.write(buzzer(300, 200, 100, 2))
                                        ser.flush()
                                        print("\n[拒绝] 需要U字型形态")
                                    elif not camera_raised:
                                        ser.write(buzzer(300, 200, 100, 2))
                                        ser.flush()
                                        print("\n[拒绝] 需要摄像头抬升")
                                    else:
                                        is_auto = True
                                        ser.write(buzzer(1200, 300, 100, 1))
                                        ser.flush()
                                        print("\n[模式] 自动模式")

                        last_buttons[num] = value

            # ── 云台控制（仅摄像头抬升时） ──
            if camera_raised and not is_auto:
                rx = deadzone(normalize_axis(axes[AXIS_RX]), 0.15)
                if abs(rx) > 0 and now - last_pan_send >= 0.08:
                    camera_pan += int(rx * CAMERA_PAN_STEP)
                    camera_pan = max(CAMERA_PAN_MIN, min(CAMERA_PAN_MAX, camera_pan))
                    ser.write(pwm_servo_one(3, camera_pan, 80))
                    ser.flush()
                    last_pan_send = now

            # ── 定时发送电机指令 (20Hz) ──
            now = time.time()
            if now - last_send < 0.05:
                continue
            last_send = now

            # 自动模式下不发送手动电机指令
            if is_auto:
                print(f"[自动模式运行中]  [{'U' if is_u_shape else 'L'}]  ", end='\r')
                continue

            # 读取输入
            rt = normalize_trigger(axes[AXIS_RT])
            lx = deadzone(normalize_axis(axes[AXIS_LX]))
            ly = deadzone(normalize_axis(axes[AXIS_LY]))

            # 计算: RT=油门, LY=方向(前正后负), LX=转向
            speed = rt * MAX_SPEED * (-1.0 if ly > 0 else 1.0)  # ly>0=往后推
            turn = -lx * rt  # 左推为正(左侧慢右侧快=左转), 右推为负

            # 差速驱动
            left = speed - turn
            right = speed + turn
            left = max(-MAX_SPEED, min(MAX_SPEED, left))
            right = max(-MAX_SPEED, min(MAX_SPEED, right))

            # 电机映射: 左侧正转, 右侧反转(镜像)
            speeds = {0: left, 1: -right, 2: left, 3: -right}

            if abs(speed) > 0.01 or abs(turn) > 0.01:
                frame = motor_many(speeds)
                ser.write(frame)
                ser.flush()
                d = "前进" if speed > 0 else "后退"
                print(f"{d} 速度:{abs(speed):.2f} 转向:{turn:+.2f} "
                      f"[{'U' if is_u_shape else 'L'}]  ", end='\r')
            else:
                ser.write(motor_stop_mask(0x0F))
                ser.flush()
                print(f"停止  [{'U' if is_u_shape else 'L'}]  ", end='\r')

    except KeyboardInterrupt:
        print("\n\n退出中...")

    finally:
        ser.write(motor_stop_mask(0x0F))
        ser.flush()
        # 摄像头放下
        if camera_raised:
            ser.write(pwm_servo_one(3, CAMERA_PAN_CENTER, SERVO_MOVE_MS))
            ser.flush()
            camera_pan = CAMERA_PAN_CENTER
            time.sleep(0.8)
            ser.write(pwm_servo_one(4, CAMERA_TILT_DOWN, SERVO_MOVE_MS))
            ser.flush()
            time.sleep(0.8)
            ser.write(bus_servo_one(1, CAMERA_LIFT_DOWN, 1500))
            ser.flush()
        js.close()
        ser.close()
        print("✓ 已停止")


if __name__ == '__main__':
    main()
