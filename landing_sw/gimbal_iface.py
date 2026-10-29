"""gimbal_iface.py -- gimbal backends (real ESP32 + mock).

A common protocol (set_angles / get_angles) lets the FSM command the gimbal
without caring whether it is talking to the ESP32 (gimbal_fw over UART) or the
mock. MockGimbal models first-order slew lag and is advanced with integrate(dt)
so laptop-only runs behave plausibly.

Angles at this interface are degrees, (pitch, roll, yaw), pitch 0 = bore forward,
-90 = bore down (plan §2.2). On the wire they are radians.
"""

from __future__ import annotations

import math
import threading
import time
from typing import Protocol, runtime_checkable

import numpy as np

from . import config
from . import gimbal_proto as gp


@runtime_checkable
class GimbalBackend(Protocol):
    def set_angles(self, pitch: float, roll: float, yaw: float) -> None: ...
    def get_angles(self) -> tuple: ...


class ESP32Gimbal:
    """gimbal_fw over UART. A background thread parses TELEM and re-sends the
    current target at tx_hz, which keeps the firmware's 200 ms watchdog fed.

    ser: anything with read(n) / write(b) / in_waiting (tests pass a fake).
    """

    def __init__(self, port: str = config.ESP32_GIMBAL_PORT, baud: int = config.ESP32_GIMBAL_BAUD,
                 ser=None, tx_hz: float = config.ESP32_GIMBAL_TX_HZ, start: bool = True):
        if ser is None:
            import serial  # hardware dependency, imported on use
            ser = serial.Serial(port, baud, timeout=0.01)
        self._ser = ser
        self._parser = gp.Parser()
        self._lock = threading.Lock()
        self._tx_lock = threading.Lock()     # set_angles (caller thread) and the resend share the port
        self._telem = None
        self._telem_t = 0.0
        self._target = (gp.MODE_NADIR, -math.pi / 2, 0.0, 0.0)
        self._seq = 0
        self._sent_t = {}                    # seq -> send time, for the round-trip latency
        self.latency_s = float("nan")        # SET_TARGET -> first TELEM that acks it (T14)
        self.version_mismatch = 0
        self._tx_period = 1.0 / tx_hz
        self._next_tx = 0.0
        self._stop = threading.Event()
        self._thread = None
        if start:
            self._thread = threading.Thread(target=self._run, daemon=True)
            self._thread.start()

    # --- GimbalBackend ---------------------------------------------------- #
    def set_angles(self, pitch, roll, yaw):
        """HOLD: pitch/roll world-referenced camera tilt, yaw relative to the body."""
        self._set_target(gp.MODE_HOLD, math.radians(pitch), math.radians(roll), math.radians(yaw))

    def get_angles(self):
        """MEASURED joint angles (pitch, roll, yaw) deg; the commanded target until TELEM arrives."""
        with self._lock:
            t, target = self._telem, self._target
        if t is None:
            return tuple(math.degrees(a) for a in target[1:])
        yaw, roll, pitch = t["q_rad"]
        return (math.degrees(pitch), math.degrees(roll), math.degrees(yaw))

    # --- extras ------------------------------------------------------------ #
    def nadir(self):
        self._set_target(gp.MODE_NADIR, -math.pi / 2, 0.0, 0.0)

    def park(self):
        """Servo torque off."""
        self._set_target(gp.MODE_PARK, -math.pi / 2, 0.0, 0.0)

    def telem(self):
        """Latest TELEM as a dict plus 'age_s', or None if nothing arrived yet."""
        with self._lock:
            if self._telem is None:
                return None
            d = dict(self._telem)
            d["age_s"] = time.monotonic() - self._telem_t
        return d

    def close(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=1.0)
        self._ser.close()

    # --- internals --------------------------------------------------------- #
    def _set_target(self, mode, pitch_rad, roll_rad, yaw_rad):
        with self._lock:
            self._target = (mode, pitch_rad, roll_rad, yaw_rad)
        self._send()                          # now, not at the next resend

    def _send(self):
        with self._lock:
            self._seq = (self._seq + 1) & 0xFFFF
            seq, (mode, p, r, y) = self._seq, self._target
            self._sent_t[seq] = time.monotonic()
            if len(self._sent_t) > 64:
                self._sent_t.pop(next(iter(self._sent_t)))
        with self._tx_lock:
            self._ser.write(gp.encode_set_target(seq, mode, p, r, y))

    def poll_once(self):
        data = self._ser.read(self._ser.in_waiting or 1)
        now = time.monotonic()
        for msg_type, _, payload in self._parser.feed(data):
            if msg_type != gp.MSG_TELEM or len(payload) != gp.TELEM.size:
                continue
            t = gp.decode_telem(payload)
            if t["proto_version"] != gp.PROTO_VERSION:
                self.version_mismatch += 1
                continue
            with self._lock:
                self._telem, self._telem_t = t, now
                sent = self._sent_t.pop(t["ack_seq"], None)
            if sent is not None:
                self.latency_s = now - sent
        if now >= self._next_tx:
            self._next_tx = now + self._tx_period
            self._send()

    def _run(self):
        while not self._stop.is_set():
            self.poll_once()


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
