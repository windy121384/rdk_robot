#!/usr/bin/env python3
"""简洁的手柄控制程序 - 用于调试和测试"""

import struct
import os
import select
import time
import serial
from protocol import motor_many, motor_stop_mask, pwm_servo_one, pwm_servo_many, bus_servo_one, buzzer, hexstr

# 手柄配置
JS_DEVICE = '/dev/input/js0'
JS_EVENT_AXIS = 0x02
JS_EVENT_BUTTON = 0x01
JS_EVENT_INIT = 0x80

# 轴映射
AXIS_LX = 0  # 左摇杆X（左右）
AXIS_LY = 1  # 左摇杆Y（前后）
AXIS_LT = 2  # LT扳机
AXIS_RX = 3  # 右摇杆X
AXIS_RY = 4  # 右摇杆Y
AXIS_RT = 5  # RT扳机

# 按键映射
BTN_A = 0
BTN_B = 1
BTN_X = 2
BTN_Y = 3

# 串口配置
SERIAL_PORT = '/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B32012490-if00'
BAUD_RATE = 1000000

# 控制参数
DEADZONE = 0.1  # 死区
MAX_SPEED = 1.0

# 舵机配置
SERVO_MOVE_MS = 600
CAMERA_TILT_VERTICAL_US = 1500
CAMERA_TILT_HORIZONTAL_US = 2500
CAMERA_LIFT_UP_US = 2000
CAMERA_LIFT_DOWN_US = 1000
U_SHAPE_PULSES = {1: 1000, 2: 2000}  # 左1000, 右2000
LINE_SHAPE_PULSES = {1: 2000, 2: 1000}  # 左2000, 右1000


def apply_deadzone(value, deadzone=DEADZONE):
    """应用死区"""
    if abs(value) < deadzone:
        return 0.0
    return value


def normalize_axis(value):
    """归一化轴值到 [-1.0, 1.0]"""
    return value / 32767.0


def normalize_trigger(value):
    """归一化扳机值到 [0.0, 1.0]"""
    return (value + 32767) / 65534.0


def main():
    print("=" * 60)
    print("简洁手柄控制程序")
    print("=" * 60)
    print(f"手柄设备: {JS_DEVICE}")
    print(f"串口设备: {SERIAL_PORT}")
    print(f"波特率: {BAUD_RATE}")
    print("=" * 60)
    
    # 打开手柄
    try:
        js = open(JS_DEVICE, "rb", buffering=0)
        print("✓ 手柄打开成功")
    except Exception as e:
        print(f"✗ 手柄打开失败: {e}")
        return
    
    # 打开串口
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1, write_timeout=1.0)
        print("✓ 串口打开成功")
    except Exception as e:
        print(f"✗ 串口打开失败: {e}")
        js.close()
        return
    
    # 状态变量
    axes = [0] * 8
    buttons = [0] * 16
    last_buttons = [0] * 16
    is_u_shape = True  # 当前形态
    camera_raised = False  # 摄像头状态
    last_send_time = 0
    send_interval = 0.05  # 20Hz
    
    print("\n开始控制（按 Ctrl+C 退出）...")
    print("-" * 60)
    
    try:
        while True:
            # 读取手柄事件
            ready, _, _ = select.select([js], [], [], 0.01)
            if ready:
                data = js.read(8)
                if data and len(data) == 8:
                    tv_sec, value, event_type, number = struct.unpack("IhBB", data)
                    
                    # 过滤初始化事件
                    if event_type & JS_EVENT_INIT:
                        continue
                    
                    event_type &= ~JS_EVENT_INIT
                    
                    if event_type == JS_EVENT_AXIS and number < len(axes):
                        axes[number] = value
                    elif event_type == JS_EVENT_BUTTON and number < len(buttons):
                        buttons[number] = value
                        # 检测按键按下（从0变1）
                        if value == 1 and last_buttons[number] == 0:
                            if number == BTN_A:
                                # A键：切换形态（U字型/一字型）
                                is_u_shape = not is_u_shape
                                pulses = U_SHAPE_PULSES if is_u_shape else LINE_SHAPE_PULSES
                                frame = pwm_servo_many(pulses, time_ms=SERVO_MOVE_MS)
                                ser.write(frame)
                                ser.flush()
                                print(f"\n切换形态: {'U字型' if is_u_shape else '一字型'}")
                            
                            elif number == BTN_Y:
                                # Y键：摄像头抬升/放下
                                camera_raised = not camera_raised
                                if camera_raised:
                                    # 抬升：先总线舵机，再PWM舵机
                                    lift_frame = bus_servo_one(1, CAMERA_LIFT_UP_US, time_ms=1500)
                                    ser.write(lift_frame)
                                    ser.flush()
                                    time.sleep(1.7)
                                    tilt_frame = pwm_servo_one(4, CAMERA_TILT_HORIZONTAL_US, time_ms=SERVO_MOVE_MS)
                                    ser.write(tilt_frame)
                                    ser.flush()
                                else:
                                    # 放下：先PWM舵机，再总线舵机
                                    tilt_frame = pwm_servo_one(4, CAMERA_TILT_VERTICAL_US, time_ms=SERVO_MOVE_MS)
                                    ser.write(tilt_frame)
                                    ser.flush()
                                    time.sleep(0.8)
                                    lift_frame = bus_servo_one(1, CAMERA_LIFT_DOWN_US, time_ms=1500)
                                    ser.write(lift_frame)
                                    ser.flush()
                                print(f"\n摄像头: {'抬升' if camera_raised else '放下'}")
                            
                            elif number == BTN_B:
                                # B键：蜂鸣器响一声
                                frame = buzzer(1000, 200, 100, 1)
                                ser.write(frame)
                                ser.flush()
                                print(f"\n蜂鸣器")
                        
                        last_buttons[number] = value
            
            # 定时发送电机指令
            now = time.time()
            if now - last_send_time >= send_interval:
                # 读取控制输入
                lx = apply_deadzone(normalize_axis(axes[AXIS_LX]))
                ly = apply_deadzone(normalize_axis(axes[AXIS_LY]))
                rt = normalize_trigger(axes[AXIS_RT])
                
                # 计算速度
                # RT扳机控制速度，左摇杆Y控制前进/后退，左摇杆X控制转向
                forward = -ly  # 左摇杆往前推为负值，所以取反
                speed = rt * MAX_SPEED * (1.0 if forward >= 0 else -1.0)
                turn = lx
                
                # 差速驱动：左轮 = speed - turn, 右轮 = speed + turn
                left_speed = speed - turn
                right_speed = speed + turn
                
                # 限制范围
                left_speed = max(-MAX_SPEED, min(MAX_SPEED, left_speed))
                right_speed = max(-MAX_SPEED, min(MAX_SPEED, right_speed))
                
                # 电机映射：0=左前, 1=右前(反转), 2=左后, 3=右后(反转)
                speeds = {
                    0: left_speed,
                    1: -right_speed,  # 右前反转
                    2: left_speed,
                    3: -right_speed   # 右后反转
                }
                
                # 发送指令
                if abs(speed) > 0.01 or abs(turn) > 0.01:
                    frame = motor_many(speeds)
                    try:
                        ser.write(frame)
                        ser.flush()
                        direction = "前进" if speed > 0 else "后退"
                        print(f"{direction} 速度: {abs(speed):.2f} 转向: {turn:+.2f} | "
                              f"左轮: {left_speed:+.2f} 右轮: {right_speed:+.2f} "
                              f"[{'U' if is_u_shape else 'L'}]", end='\r')
                    except Exception as e:
                        print(f"\n✗ 串口写入失败: {e}")
                else:
                    # 停止
                    frame = motor_stop_mask(0x0F)
                    try:
                        ser.write(frame)
                        ser.flush()
                        print(f"停止                                    ", end='\r')
                    except Exception as e:
                        print(f"\n✗ 串口写入失败: {e}")
                
                last_send_time = now
    
    except KeyboardInterrupt:
        print("\n\n用户中断，正在停止...")
    
    finally:
        # 发送停止指令
        print("发送停止指令...")
        frame = motor_stop_mask(0x0F)
        try:
            ser.write(frame)
            ser.flush()
            print("✓ 电机已停止")
        except:
            pass
        
        # 关闭设备
        js.close()
        ser.close()
        print("✓ 设备已关闭")
        print("=" * 60)


if __name__ == '__main__':
    main()
