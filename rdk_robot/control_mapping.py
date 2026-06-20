#!/usr/bin/env python3
"""Project-specific control mapping for the morphing inspection robot."""
from __future__ import annotations

from dataclasses import dataclass, field

AXIS_LX = 0
AXIS_LY = 1
AXIS_LT = 2
AXIS_RX = 3
AXIS_RY = 4
AXIS_RT = 5

BTN_A = 0
BTN_B = 1
BTN_X = 2
BTN_Y = 3


@dataclass
class GamepadState:
    axes: list[int] = field(default_factory=lambda: [0] * 8)
    buttons: list[int] = field(default_factory=lambda: [0] * 11)

    def axis_norm(self, idx: int, deadzone: float = 0.08) -> float:
        value = self.axes[idx] / 32767.0
        if abs(value) < deadzone:
            return 0.0
        return max(-1.0, min(1.0, value))

    def trigger_norm(self, idx: int) -> float:
        # js0 trigger: released=-32767, fully pressed=32767
        return max(0.0, min(1.0, (self.axes[idx] + 32767) / 65534.0))


@dataclass
class RobotMode:
    is_u_shape: bool = True
    mecanum_mode: bool = False
    camera_raised: bool = False


@dataclass
class RobotCommand:
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0
    camera_pan: float = 0.0
    speed_scale: float = 0.0
    is_u_shape: bool = True
    mecanum_mode: bool = False
    camera_raised: bool = False


def make_command(state: GamepadState, mode: RobotMode) -> RobotCommand:
    speed_scale = state.trigger_norm(AXIS_RT)
    lx = state.axis_norm(AXIS_LX)
    ly = state.axis_norm(AXIS_LY)
    rx = state.axis_norm(AXIS_RX)

    cmd = RobotCommand(
        speed_scale=speed_scale,
        is_u_shape=mode.is_u_shape,
        mecanum_mode=mode.mecanum_mode,
        camera_raised=mode.camera_raised,
    )

    if mode.is_u_shape:
        # RT controls speed, left stick Y backward = reverse
        if ly > 0.3:  # stick pushed back > 30%
            cmd.vx = -speed_scale  # reverse
        else:
            cmd.vx = speed_scale  # forward
        if mode.mecanum_mode:
            cmd.vy = lx * speed_scale
            cmd.wz = rx * speed_scale
        else:
            # Differential mode cannot strafe; left-stick X becomes yaw.
            cmd.wz = lx
    else:
        # In one-line shape, keep the robot conservative: no lateral motion.
        if ly > 0.3:
            cmd.vx = -speed_scale
        else:
            cmd.vx = speed_scale
        cmd.wz = lx

    if mode.camera_raised:
        cmd.camera_pan = rx

    return cmd
