"""landing.py -- LandingController implementing design principle 2.

Control law:  v_cmd = Kp * position_error + G_ff * v_target_absolute
The proportional term alone lags a moving target by v_target * latency; the
velocity feedforward cancels that lag. The feedforward uses the EKF's
target-absolute-velocity estimate DIRECTLY -- it must NOT be recomputed from the
FC velocity here, because the EKF has already folded ego-motion into its predict
step on a single time base (principle 3).

Vertical: descend at DESCENT_VEL_MS only when well-centred and above the flare
altitude; a slow creep otherwise; zero at/below flare. Body FRD: +z is down.
"""

from __future__ import annotations

import numpy as np

from . import config


class LandingController:
    def __init__(self, mav):
        self.mav = mav

    def compute_velocity_setpoint(self, marker_pos_body, marker_vel_body,
                                  est_agl_m) -> tuple:
        px, py, _ = (float(v) for v in marker_pos_body)
        vtx, vty, _ = (float(v) for v in marker_vel_body)   # target absolute vel

        # Horizontal: proportional-to-error + velocity feedforward (principle 2).
        vx = config.KP_HORIZ * px + config.FEEDFORWARD_GAIN * vtx
        vy = config.KP_HORIZ * py + config.FEEDFORWARD_GAIN * vty
        vmag = np.hypot(vx, vy)
        if vmag > config.MAX_HORIZ_VEL_MS:
            s = config.MAX_HORIZ_VEL_MS / vmag
            vx, vy = vx * s, vy * s

        # Vertical: descend only when centred and above flare.
        centred = np.hypot(px, py) < 0.25 * max(est_agl_m, 0.1)
        if est_agl_m <= config.FLARE_AGL_M:
            vz = 0.0
        elif centred:
            vz = config.DESCENT_VEL_MS
        else:
            vz = 0.3 * config.DESCENT_VEL_MS      # slow creep while recentring
        vz = float(np.clip(vz, -config.MAX_DESCENT_VEL_MS, config.MAX_DESCENT_VEL_MS))
        return float(vx), float(vy), vz

    def send(self, marker_pos_body, marker_vel_body, est_agl_m):
        vx, vy, vz = self.compute_velocity_setpoint(marker_pos_body, marker_vel_body,
                                                    est_agl_m)
        self.mav.send_velocity_body(vx, vy, vz)
        return vx, vy, vz

    def abort(self):
        """Climb away and hand control back to the pilot."""
        self.mav.send_velocity_body(0.0, 0.0, -config.ABORT_CLIMB_RATE_MS)
