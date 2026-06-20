#!/usr/bin/env python3
"""Quick protocol checks and sample frames."""
from protocol import hexstr, led, motor_many, pwm_servo_one, bus_servo_one, crc8_maxim


body = bytes.fromhex("01 07 01 E8 03 E8 03 0A 00")
print("LED known body CRC:", f"0x{crc8_maxim(body):02X}", "expected 0xD9")
print("LED sample:", hexstr(led(1, 100, 100, 5)))
print("Motor 4 sample:", hexstr(motor_many({1: 0.25, 2: 0.25, 3: 0.25, 4: 0.25})))
print("PWM servo sample:", hexstr(pwm_servo_one(1, 1500, 500)))
print("Bus servo sample:", hexstr(bus_servo_one(1, 1500, 500)))
