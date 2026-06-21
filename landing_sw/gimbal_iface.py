"""gimbal_iface.py -- SimpleBGC32 gimbal backend (real + mock).

A common protocol (set_angles / get_angles) lets the FSM command the gimbal
without caring whether it is talking to real SimpleBGC serial hardware or the
mock. MockGimbal models first-order slew lag and is advanced with integrate(dt)
so laptop-only runs behave plausibly. Real serial is hardware-only, hence the
split (mirrors mavlink_iface.py).
"""

from __future__ import annotations

from typing import Protocol, runtime_checkable

import numpy as np

from . import config


@runtime_checkable
class GimbalBackend(Protocol):
    def set_angles(self, pitch: float, roll: float, yaw: float) -> None: ...
    def get_angles(self) -> tuple: ...


class SimpleBGCInterface:
    """Real SimpleBGC32 over serial. Imports pyserial lazily (hardware-only)."""

    def __init__(self, port: str = config.GIMBAL_SERIAL_PORT,
                 baud: int = config.GIMBAL_SERIAL_BAUD):
        import serial  # noqa: F401  (hardware dependency, imported on use)
        self._serial = serial.Serial(port, baud, timeout=0.1)
        self._angles = (config.GIMBAL_PITCH_LAND_DEG, 0.0, 0.0)

    def set_angles(self, pitch, roll, yaw):
        # Encode/transmit a SimpleBGC CMD_CONTROL frame (omitted: hardware-specific).
        self._angles = (pitch, roll, yaw)

    def get_angles(self):
        # Real impl parses the realtime-data stream; placeholder echoes command.
        return self._angles


class MockGimbal:
    """First-order gimbal model; call integrate(dt) to advance toward commands."""

    def __init__(self, tau_s: float = 0.05,
                 slew_deg_s: float = config.GIMBAL_SLEW_DEG_S):
        self.tau_s = tau_s
        self.slew_deg_s = slew_deg_s
        self.pitch = config.GIMBAL_PITCH_LAND_DEG
        self.roll = 0.0
        self.yaw = 0.0
        self._cmd = (config.GIMBAL_PITCH_LAND_DEG, 0.0, 0.0)

    def set_angles(self, pitch, roll, yaw):
        self._cmd = (float(pitch), float(roll), float(yaw))

    def get_angles(self):
        return (self.pitch, self.roll, self.yaw)

    def integrate(self, dt: float):
        max_step = self.slew_deg_s * dt
        cp, cr, cy = self._cmd
        self.pitch += np.clip(cp - self.pitch, -max_step, max_step)
        self.roll += np.clip(cr - self.roll, -max_step, max_step)
        self.yaw += np.clip(cy - self.yaw, -max_step, max_step)
