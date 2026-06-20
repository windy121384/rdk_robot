#!/usr/bin/env python3
"""预编程序列动作：左转2秒→前进5秒→停止"""
import time, serial
from protocol import motor_many, motor_stop_mask, pwm_servo_many, bus_servo_one

SERIAL_PORT = '/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B32012490-if00'
BAUD_RATE = 1000000
TURN_SPEED = 0.3
FORWARD_SPEED = 0.4

U_SHAPE = {1: 1600, 2: 1550}

def run():
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1, write_timeout=0.5)
    print("✓ 串口已打开")
    time.sleep(0.3)

    # 确保U字型
    ser.write(pwm_servo_many(U_SHAPE, time_ms=600)); ser.flush()
    time.sleep(0.7)

    print("→ 左转 2 秒")
    start = time.time()
    while time.time() - start < 2.0:
        # 左转: 1前 2退 3退 4前（反接适配）
        try:
            ser.write(motor_many({0: TURN_SPEED, 1: TURN_SPEED, 2: -TURN_SPEED, 3: -TURN_SPEED})); ser.flush()
        except Exception:
            pass
        time.sleep(0.025)

    print("→ 前进 5 秒")
    start = time.time()
    while time.time() - start < 5.0:
        # U字型前进: 电机2、4反接
        try:
            ser.write(motor_many({0: FORWARD_SPEED, 1: -FORWARD_SPEED, 2: FORWARD_SPEED, 3: -FORWARD_SPEED})); ser.flush()
        except Exception:
            pass
        time.sleep(0.025)

    print("→ 停止")
    try:
        ser.write(motor_stop_mask(0x0F)); ser.flush()
    except Exception:
        pass
    ser.close()
    print("✓ 完成")

if __name__ == '__main__':
    run()
