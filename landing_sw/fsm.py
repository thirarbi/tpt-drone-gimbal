"""fsm.py -- LandingFSM, the mission state machine and per-frame orchestrator.

Seven states: IDLE, NAVIGATE, TRANSITION, ACTIVE_LANDING, COASTING,
GIMBAL_SEARCH, ABORT. step() runs the per-frame chain:
    detect -> camera->body transform (live gimbal angles)
    -> ekf.predict(dt, mav.read_velocity_body())   (ego-motion folded in)
    -> ekf.update -> pointing -> landing setpoint.

Transition policy (principles 3 & 4):
  * marker loss < MARKER_LOSS_COAST_S -> COASTING: keep flying on the EKF
    constant-velocity prediction (the companion bridges 30 fps gaps; it does NOT
    run a high-rate vehicle filter -- EKF3 dead-reckons the vehicle).
  * loss >= MARKER_LOSS_COAST_S and GIMBAL_SEARCH_MIN_AGL <= agl <=
    GIMBAL_SEARCH_MAX_AGL -> GIMBAL_SEARCH (altitude-gated: never sweep near ground).
  * reacquisition from search -> ekf.reset() then ACTIVE_LANDING, so a fresh lock
    does not inherit a stale velocity.
  * loss at unsafe AGL, search timeout, or camera fault -> ABORT.
"""

from __future__ import annotations

from enum import Enum

import numpy as np

from . import config, frames


class State(Enum):
    IDLE = "IDLE"
    NAVIGATE = "NAVIGATE"
    TRANSITION = "TRANSITION"
    ACTIVE_LANDING = "ACTIVE_LANDING"
    COASTING = "COASTING"
    GIMBAL_SEARCH = "GIMBAL_SEARCH"
    ABORT = "ABORT"


class LandingFSM:
    SEARCH_TIMEOUT_S = 8.0

    def __init__(self, detector, ekf, pointing, landing, gimbal, mav, db=None,
                 flight_id=None, dt=1.0 / config.CAM_FPS):
        self.detector = detector
        self.ekf = ekf
        self.pointing = pointing
        self.landing = landing
        self.gimbal = gimbal
        self.mav = mav
        self.db = db
        self.flight_id = flight_id
        self.dt = dt
        self.state = State.IDLE
        self.t = 0.0
        self._since_seen = 0.0
        self._search_elapsed = 0.0
        self._search_dir = 1.0

    def _set_state(self, s):
        if s != self.state:
            if self.db and self.flight_id is not None:
                self.db.log_event(self.flight_id, self.t, "state",
                                  f"{self.state.value}->{s.value}")
            self.state = s

    def step(self, frame, agl, rc, attitude, camera_ok=True):
        self.t += self.dt
        dt = self.dt
        veh_yaw = attitude[2] if attitude is not None else 0.0

        # --- perception + filter (principle 3: ego-motion into predict) ---
        drone_vel_body = self.mav.read_velocity_body()
        self.ekf.predict(dt, drone_vel_body)

        pose = config.Pose(valid=False)
        accepted = False
        if camera_ok and frame is not None:
            pose = self.detector.detect(frame, est_agl_m=agl)
        if pose.valid:
            g = self.gimbal.get_angles()
            pos_body = frames.camera_point_to_body(pose.t, g)
            accepted = self.ekf.update(pos_body)
            if accepted:
                self._since_seen = 0.0
        if not (pose.valid and accepted):
            self._since_seen += dt

        # --- state logic ---
        landing_requested = (rc == "LAND")
        if not camera_ok:
            self._set_state(State.ABORT)
        elif not landing_requested:
            self._set_state(State.IDLE if agl < 1.0 else State.NAVIGATE)
        else:
            self._transition_landing(agl)

        # --- act ---
        self._act(agl, veh_yaw)

        if self.db and self.flight_id is not None:
            self._log_tick(agl, pose, accepted)
        return self.state

    def _transition_landing(self, agl):
        seen = self._since_seen < 1e-6
        if self.state in (State.IDLE, State.NAVIGATE):
            self._set_state(State.TRANSITION)
        if self.state == State.TRANSITION:
            if self.ekf.initialized and seen:
                self._set_state(State.ACTIVE_LANDING)
            return
        if self.state == State.GIMBAL_SEARCH:
            self._search_elapsed += self.dt
            if seen:
                self.ekf.reset()
                self._search_elapsed = 0.0
                self._set_state(State.ACTIVE_LANDING)
            elif self._search_elapsed > self.SEARCH_TIMEOUT_S:
                self._set_state(State.ABORT)
            return
        # ACTIVE_LANDING / COASTING
        if self._since_seen < config.MARKER_LOSS_COAST_S:
            self._set_state(State.ACTIVE_LANDING if seen else State.COASTING)
        else:
            if config.GIMBAL_SEARCH_MIN_AGL <= agl <= config.GIMBAL_SEARCH_MAX_AGL:
                self._search_elapsed = 0.0
                self._set_state(State.GIMBAL_SEARCH)
            else:
                self._set_state(State.ABORT)   # loss at unsafe AGL

    def _act(self, agl, veh_yaw):
        if self.state in (State.ACTIVE_LANDING, State.COASTING, State.TRANSITION):
            if self.ekf.initialized:
                # During landing the camera is locked nadir: lateral marker error
                # is nulled by the vehicle velocity controller, not by slewing the
                # gimbal (slewing would rotate the camera->body axes and destabilise
                # the loop). The pointing controller is exercised for telemetry but
                # the bore stays down. Gimbal tracking is used in GIMBAL_SEARCH.
                self.pointing.step(self.dt, self.ekf.position)
                self.gimbal.set_angles(config.GIMBAL_PITCH_LAND_DEG, 0.0, 0.0)
                self.landing.send(self.ekf.position, self.ekf.velocity, agl)
        elif self.state == State.GIMBAL_SEARCH:
            # Altitude-gated sweep in yaw; hold position (no descent).
            self._search_dir *= -1.0 if abs(self.pointing.yaw) > 45 else 1.0
            self.pointing.yaw += self._search_dir * config.GIMBAL_SLEW_DEG_S * self.dt
            self.gimbal.set_angles(config.GIMBAL_PITCH_LAND_DEG, 0.0, self.pointing.yaw)
            self.mav.send_velocity_body(0.0, 0.0, 0.0)
        elif self.state == State.ABORT:
            self.landing.abort()

    def _log_tick(self, agl, pose, accepted):
        p = self.ekf.position if self.ekf.initialized else np.full(3, np.nan)
        v = self.ekf.velocity if self.ekf.initialized else np.full(3, np.nan)
        g = self.gimbal.get_angles()
        sp = self.mav.last_setpoint
        self.db.log_tick(self.flight_id, self.t, self.state.value,
                         pose.marker_id, pose.t[0], pose.t[1], pose.t[2],
                         self.detector.last_reproj_err,
                         np.nan, np.nan, np.nan, int(accepted),
                         self.ekf.last_innov_chi2,
                         p[0], p[1], p[2], v[0], v[1], v[2],
                         g[0], g[1], g[2], sp[0], sp[1], sp[2], agl, np.nan)
