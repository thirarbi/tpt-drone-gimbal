"""pointing.py -- PointingController for the SimpleBGC gimbal.

Keeps the camera bore on the marker by commanding gimbal (pitch, roll, yaw)
angles, slew-rate limited to GIMBAL_SLEW_DEG_S so commands stay mechanically
achievable. desired_angles_from_marker() turns a relative marker position into
the pitch/yaw that points the bore at it; step() moves the current command toward
that target under the slew limit.
"""

from __future__ import annotations

import numpy as np

from . import config


class PointingController:
    def __init__(self, slew_deg_s: float = config.GIMBAL_SLEW_DEG_S):
        self.slew_deg_s = slew_deg_s
        self.pitch = config.GIMBAL_PITCH_LAND_DEG
        self.roll = 0.0
        self.yaw = 0.0

    def desired_angles_from_marker(self, marker_pos_body) -> tuple:
        """Return (pitch, roll, yaw) deg to point the bore at a body-frame marker.

        Body FRD: pitch from horizon (down negative), yaw about Down toward the
        marker bearing.
        """
        fx, ry, dz = (float(v) for v in marker_pos_body)
        horiz = np.hypot(fx, ry)
        pitch = -np.degrees(np.arctan2(dz, horiz))   # straight down -> -90
        yaw = np.degrees(np.arctan2(ry, fx))
        return pitch, 0.0, yaw

    def step(self, dt: float, ekf_position) -> tuple:
        tgt_pitch, tgt_roll, tgt_yaw = self.desired_angles_from_marker(ekf_position)
        max_step = self.slew_deg_s * dt
        self.pitch = self._slew(self.pitch, tgt_pitch, max_step)
        self.roll = self._slew(self.roll, tgt_roll, max_step)
        self.yaw = self._slew(self.yaw, tgt_yaw, max_step)
        self.pitch = float(np.clip(self.pitch, config.GIMBAL_PITCH_LAND_DEG,
                                   config.GIMBAL_PITCH_NAV_DEG))
        return self.pitch, self.roll, self.yaw

    def pitch_roll_yaw(self) -> tuple:
        return self.pitch, self.roll, self.yaw

    @staticmethod
    def _slew(cur, tgt, max_step):
        return cur + np.clip(tgt - cur, -max_step, max_step)
