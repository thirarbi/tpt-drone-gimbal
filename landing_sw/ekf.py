"""ekf.py -- RelativeMarkerEKF (6-state) implementing design principle 3.

State (body FRD frame):  x = [px, py, pz, vtx, vty, vtz]
    [px,py,pz]   relative marker position w.r.t. the drone.
    [vtx,vty,vtz] target ABSOLUTE velocity.

Principle 3 (load-bearing): the target's absolute velocity is estimated by
folding the drone's own velocity (ego-motion) into the predict step as a known
control input. After the constant-velocity propagation we subtract ego-motion
from the relative position:  x[:3] -= drone_vel_body * dt.

The tempting WRONG alternative -- keep a relative-velocity state and add the FC
velocity back afterwards (v_target = v_rel + v_drone) -- puts the lagged
perception estimate and the fresh FC velocity on different time bases and injects
positive feedback that destabilises the loop. We do NOT do that; ego-motion goes
inside predict so everything shares one time base.

This is the ONLY companion-side filter: it estimates just the relative marker
state and bridges gaps between 30 fps camera updates with a constant-velocity
prediction (principle 4). Vehicle dead-reckoning during marker loss is
ArduCopter's on-board EKF3, not this filter.
"""

from __future__ import annotations

import numpy as np

from . import config


class RelativeMarkerEKF:
    def __init__(self,
                 meas_pos_std: float = config.EKF_MEAS_POS_STD,
                 init_pos_std: float = config.EKF_INIT_POS_STD,
                 init_vel_std: float = config.EKF_INIT_VEL_STD,
                 proc_accel_std: float = config.EKF_PROC_ACCEL_STD,
                 gate_chi2: float = config.EKF_GATE_CHI2):
        self.x = np.zeros(6)
        self.P = np.diag([init_pos_std**2] * 3 + [init_vel_std**2] * 3).astype(float)
        self.H = np.hstack([np.eye(3), np.zeros((3, 3))])
        self.R = np.eye(3) * meas_pos_std**2
        self._q = proc_accel_std**2
        self._init_pos_std = init_pos_std
        self._init_vel_std = init_vel_std
        self.gate_chi2 = gate_chi2
        self.initialized = False
        self.last_innov_chi2 = None

    # ------------------------------------------------------------------ #
    def reset(self):
        """Re-initialise (used by the FSM on reacquisition so a fresh lock does
        not inherit a stale velocity estimate)."""
        self.x = np.zeros(6)
        self.P = np.diag([self._init_pos_std**2] * 3
                         + [self._init_vel_std**2] * 3).astype(float)
        self.initialized = False
        self.last_innov_chi2 = None

    # ------------------------------------------------------------------ #
    def predict(self, dt: float, drone_vel_body=(0.0, 0.0, 0.0)):
        if not self.initialized:
            return
        F = np.eye(6)
        for i in range(3):
            F[i, i + 3] = dt
        self.x = F @ self.x
        # Fold ego-motion in as a known control input (principle 3).
        self.x[:3] -= np.asarray(drone_vel_body, dtype=np.float64) * dt
        self.P = F @ self.P @ F.T + self._Q(dt)

    def _Q(self, dt):
        # Piecewise-constant white-acceleration model.
        q = self._q
        Qpp = dt**4 / 4.0 * q
        Qpv = dt**3 / 2.0 * q
        Qvv = dt**2 * q
        Q = np.zeros((6, 6))
        for i in range(3):
            Q[i, i] = Qpp
            Q[i, i + 3] = Qpv
            Q[i + 3, i] = Qpv
            Q[i + 3, i + 3] = Qvv
        return Q

    # ------------------------------------------------------------------ #
    def update(self, z_pos_body) -> bool:
        """Gated position update. Returns True if accepted, False if gated out."""
        z = np.asarray(z_pos_body, dtype=np.float64).reshape(3)
        if not self.initialized:
            self.x[:3] = z
            self.x[3:] = 0.0
            self.initialized = True
            self.last_innov_chi2 = 0.0
            return True
        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        Sinv = np.linalg.inv(S)
        self.last_innov_chi2 = float(y @ Sinv @ y)
        if self.last_innov_chi2 > self.gate_chi2:
            return False
        K = self.P @ self.H.T @ Sinv
        self.x = self.x + K @ y
        self.P = (np.eye(6) - K @ self.H) @ self.P
        return True

    def mahalanobis(self, z_pos_body) -> float:
        """Chi^2 distance of a candidate measurement (no state change)."""
        z = np.asarray(z_pos_body, dtype=np.float64).reshape(3)
        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        return float(y @ np.linalg.solve(S, y))

    # ------------------------------------------------------------------ #
    @property
    def position(self) -> np.ndarray:
        return self.x[:3].copy()

    @property
    def velocity(self) -> np.ndarray:
        """Target ABSOLUTE velocity estimate (body frame)."""
        return self.x[3:].copy()
