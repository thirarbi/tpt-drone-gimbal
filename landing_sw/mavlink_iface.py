"""mavlink_iface.py -- flight-controller backend (real pymavlink + mock).

The companion talks to the SpeedyBee F405 (ArduCopter) over MAVLink via
pymavlink. MavlinkInterface sends body-frame velocity setpoints
(SET_POSITION_TARGET_LOCAL_NED, velocity mask) and reads the FC EKF3 velocity
(NED->body via yaw), RC mode, AGL, and attitude. read_velocity_body() is what the
EKF predict step folds in as ego-motion (principle 3) -- crucially it comes from
the FC's own filter (principle 4: no custom high-rate vehicle filter here).

MockMavlink records the last setpoint and lets tests inject agl/rc/attitude and a
drone velocity (returned from read_velocity_body). pymavlink is imported lazily so
the package runs on a laptop with no autopilot.
"""

from __future__ import annotations

from typing import Protocol, runtime_checkable

import numpy as np

from . import config, frames


@runtime_checkable
class VehicleBackend(Protocol):
    def send_velocity_body(self, vx: float, vy: float, vz: float) -> None: ...
    def read_velocity_body(self) -> np.ndarray: ...
    def read_rc_mode(self) -> str: ...
    def read_agl(self) -> float: ...
    def read_attitude(self) -> tuple: ...


class MavlinkInterface:
    """Real ArduCopter link over pymavlink (imported lazily; hardware/SITL only)."""

    def __init__(self, conn: str = config.MAVLINK_CONN, baud: int = config.MAVLINK_BAUD):
        from pymavlink import mavutil
        self._mavutil = mavutil
        self.master = mavutil.mavlink_connection(conn, baud=baud)
        self.master.wait_heartbeat()
        self.last_setpoint = (0.0, 0.0, 0.0)

    def send_velocity_body(self, vx, vy, vz):
        m = self.master
        # MAV_FRAME_BODY_NED + velocity-only type_mask.
        m.mav.set_position_target_local_ned_send(
            0, m.target_system, m.target_component,
            self._mavutil.mavlink.MAV_FRAME_BODY_NED,
            0b0000111111000111, 0, 0, 0, vx, vy, vz, 0, 0, 0, 0, 0)
        self.last_setpoint = (vx, vy, vz)

    def read_velocity_body(self):
        msg = self.master.recv_match(type="LOCAL_POSITION_NED", blocking=False)
        att = self.master.recv_match(type="ATTITUDE", blocking=False)
        if msg is None:
            return np.zeros(3)
        vned = np.array([msg.vx, msg.vy, msg.vz])
        yaw = np.degrees(att.yaw) if att is not None else 0.0
        return frames.ned_to_body(vned, yaw)

    def read_rc_mode(self):
        msg = self.master.recv_match(type="HEARTBEAT", blocking=False)
        return "LAND" if msg and msg.custom_mode == 9 else "LOITER"

    def read_agl(self):
        msg = self.master.recv_match(type="RANGEFINDER", blocking=False)
        return float(msg.distance) if msg else float("nan")

    def read_attitude(self):
        msg = self.master.recv_match(type="ATTITUDE", blocking=False)
        if msg is None:
            return (0.0, 0.0, 0.0)
        return (np.degrees(msg.roll), np.degrees(msg.pitch), np.degrees(msg.yaw))


class MockMavlink:
    """Software backend: integrates velocity setpoints and serves injected state."""

    def __init__(self, agl=10.0, rc="LAND", attitude=(0.0, 0.0, 0.0),
                 drone_vel=(0.0, 0.0, 0.0)):
        self._agl = agl
        self._rc = rc
        self._attitude = attitude
        self._drone_vel = np.array(drone_vel, dtype=np.float64)
        self.last_setpoint = (0.0, 0.0, 0.0)

    # --- injection helpers for tests/sims ---
    def set_agl(self, agl): self._agl = agl
    def set_rc(self, rc): self._rc = rc
    def set_attitude(self, att): self._attitude = att
    def set_drone_vel(self, v): self._drone_vel = np.asarray(v, float)

    # --- backend protocol ---
    def send_velocity_body(self, vx, vy, vz):
        self.last_setpoint = (vx, vy, vz)

    def read_velocity_body(self):
        return self._drone_vel.copy()

    def read_rc_mode(self):
        return self._rc

    def read_agl(self):
        return self._agl

    def read_attitude(self):
        return self._attitude
