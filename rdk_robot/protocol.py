#!/usr/bin/env python3
"""STM32 serial protocol helpers for the RDK robot."""
from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import Iterable

HEADER = bytes([0xAA, 0x55])

SYS = 0x00
LED = 0x01
BUZZER = 0x02
MOTOR = 0x03
PWM_SERVO = 0x04
BUS_SERVO = 0x05
KEY = 0x06
IMU = 0x07
GAMEPAD = 0x08
SBUS = 0x09


def crc8_maxim(data: bytes) -> int:
    """CRC8-MAXIM used by the controller.

    The written spec says init 0xFF, but the known-good LED frame ending in
    0xD9 matches the common MAXIM/Dallas init 0x00 variant.
    """
    crc = 0x00
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x01:
                crc = (crc >> 1) ^ 0x8C  # reflected polynomial for 0x31
            else:
                crc >>= 1
            crc &= 0xFF
    return crc


def frame(cmd: int, payload: bytes = b"") -> bytes:
    if not 0 <= cmd <= 0xFF:
        raise ValueError("cmd must fit in one byte")
    if len(payload) > 0xFF:
        raise ValueError("payload too long")
    body = bytes([cmd, len(payload)]) + payload
    return HEADER + body + bytes([crc8_maxim(body)])


def led(led_id: int, on_ms: int, off_ms: int, cycles: int) -> bytes:
    payload = struct.pack("<BHHH", led_id, on_ms, off_ms, cycles)
    return frame(LED, payload)


def buzzer(freq_hz: int, on_ms: int, off_ms: int, cycles: int) -> bytes:
    payload = struct.pack("<HHHH", freq_hz, on_ms, off_ms, cycles)
    return frame(BUZZER, payload)


def motor_one(motor_id: int, speed: float) -> bytes:
    payload = bytes([0x00, motor_id & 0xFF]) + struct.pack("<f", float(speed))
    return frame(MOTOR, payload)


def motor_many(speeds: dict[int, float] | Iterable[tuple[int, float]]) -> bytes:
    items = list(speeds.items() if isinstance(speeds, dict) else speeds)
    payload = bytearray([0x01, len(items) & 0xFF])
    for motor_id, speed in items:
        payload.append(motor_id & 0xFF)
        payload.extend(struct.pack("<f", float(speed)))
    return frame(MOTOR, bytes(payload))


def motor_stop_one(motor_id: int) -> bytes:
    return frame(MOTOR, bytes([0x02, motor_id & 0xFF]))


def motor_stop_mask(mask: int) -> bytes:
    return frame(MOTOR, bytes([0x03, mask & 0xFF]))


def pwm_servo_one(servo_id: int, pulse_us: int, time_ms: int = 500) -> bytes:
    payload = struct.pack("<BHBH", 0x03, time_ms, servo_id & 0xFF, pulse_us)
    return frame(PWM_SERVO, payload)


def pwm_servo_many(servos: dict[int, int] | Iterable[tuple[int, int]], time_ms: int = 500) -> bytes:
    items = list(servos.items() if isinstance(servos, dict) else servos)
    payload = bytearray(struct.pack("<BHB", 0x01, time_ms, len(items) & 0xFF))
    for servo_id, pulse_us in items:
        payload.extend(struct.pack("<BH", servo_id & 0xFF, pulse_us))
    return frame(PWM_SERVO, bytes(payload))


def bus_servo_one(servo_id: int, pulse_us: int, time_ms: int = 500) -> bytes:
    # New protocol: #000P1500T1000! (ASCII)
    # ID: 3 digits zero-padded, PWM: 4 digits zero-padded, TIME: 4 digits zero-padded
    payload = f"#{servo_id:03d}P{pulse_us:04d}T{time_ms:04d}!".encode('ascii')
    return frame(BUS_SERVO, payload)


def bus_servo_many(servos: dict[int, int] | Iterable[tuple[int, int]], time_ms: int = 500) -> bytes:
    # New protocol: concatenate multiple frames (one per servo)
    items = list(servos.items() if isinstance(servos, dict) else servos)
    result = b""
    for servo_id, pulse_us in items:
        result += bus_servo_one(servo_id, pulse_us, time_ms)
    return result


def bus_servo_power(servo_id: int, enable: bool) -> bytes:
    return frame(BUS_SERVO, bytes([0x0C if enable else 0x0B, servo_id & 0xFF]))


def hexstr(data: bytes) -> str:
    return data.hex(" ").upper()


@dataclass(frozen=True)
class ParsedFrame:
    cmd: int
    payload: bytes


def parse_frame(data: bytes) -> ParsedFrame:
    if len(data) < 5 or data[:2] != HEADER:
        raise ValueError("bad frame header or too short")
    cmd = data[2]
    length = data[3]
    expected = 2 + 1 + 1 + length + 1
    if len(data) != expected:
        raise ValueError(f"bad frame length: got {len(data)}, expected {expected}")
    body = data[2:-1]
    got_crc = data[-1]
    want_crc = crc8_maxim(body)
    if got_crc != want_crc:
        raise ValueError(f"bad crc: got 0x{got_crc:02X}, want 0x{want_crc:02X}")
    return ParsedFrame(cmd=cmd, payload=data[4:-1])
