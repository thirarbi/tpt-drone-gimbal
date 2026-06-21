"""sitl_harness.py -- closed-loop moving-target sim: feedforward ON vs OFF.

Risk question (Bab 4.2 row 5): does the velocity feedforward make 30 fps
perception sufficient to track a MOVING pad (principle 2)? This is the control-law
validation, not a perception result. Pass criterion: steady-state horizontal
tracking error with feedforward ON must be under
config.FEEDFORWARD_MAX_STEADY_ERR_CM (and ~2 orders of magnitude below OFF).

The pad moves at ~1.4 m/s; perception is modelled as true relative position +
Gaussian noise + occasional dropout at 30 fps. The EKF estimates the target's
ABSOLUTE velocity by folding the drone's own velocity into predict (principle 3),
and the landing law applies that estimate as feedforward. With FF OFF the
proportional loop lags by v_target/Kp (~117 cm for the defaults) and stalls; with
FF ON it tracks to a few cm and descends to flare. Output: console + CSV.
"""

from __future__ import annotations

import csv

import numpy as np

from . import config, ekf, landing
from .mavlink_iface import MockMavlink

TARGET_SPEED_MS = 1.4
SIM_T = 25.0
DT = 1.0 / config.CAM_FPS
DROPOUT_PROB = 0.05
MEAS_NOISE_M = 0.03
DRONE_TAU_S = 0.20
STEADY_FRACTION = 0.4
MAX_STEADY_ERR_CM = config.FEEDFORWARD_MAX_STEADY_ERR_CM


def _run_once(feedforward: bool, seed: int = 1):
    rng = np.random.default_rng(seed)
    kf = ekf.RelativeMarkerEKF()
    mav = MockMavlink(agl=8.0, rc="LAND")
    land = landing.LandingController(mav)

    drone = np.zeros(3)          # world pos (yaw 0 => body == world horizontally)
    drone_vel = np.zeros(3)
    agl = 8.0
    n = int(SIM_T / DT)
    errs, agls = [], []
    for k in range(n):
        t = k * DT
        target = np.array([TARGET_SPEED_MS * t, 0.0, 0.0])
        rel = target - drone                       # body == world (yaw 0)
        rel[2] = -agl                              # marker is below (FRD +z down)

        kf.predict(DT, drone_vel_body=drone_vel)
        if rng.random() > DROPOUT_PROB:
            meas = rel + rng.normal(0, MEAS_NOISE_M, 3)
            kf.update(meas)

        vel_ff = kf.velocity if feedforward else np.zeros(3)
        vx, vy, vz = land.compute_velocity_setpoint(kf.position, vel_ff, agl)

        # Drone first-order velocity tracking + integrate.
        cmd = np.array([vx, vy, vz])
        drone_vel += (DT / (DRONE_TAU_S + DT)) * (cmd - drone_vel)
        drone += drone_vel * DT
        agl = max(agl - vz * DT, 0.0)              # +vz is descent

        errs.append(float(np.hypot(target[0] - drone[0], target[1] - drone[1])))
        agls.append(agl)

    errs = np.array(errs)
    steady = errs[int((1 - STEADY_FRACTION) * n):]
    return float(steady.mean()), float(agls[-1]), errs, agls


def run(verbose: bool = True):
    err_ff, agl_ff, trace_ff, _ = _run_once(feedforward=True)
    err_no, agl_no, trace_no, _ = _run_once(feedforward=False)
    err_ff_cm, err_no_cm = err_ff * 100.0, err_no * 100.0
    passed = err_ff_cm < MAX_STEADY_ERR_CM and err_ff_cm < 0.1 * err_no_cm

    with open(config.results_path("sitl_harness.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["frame", "err_with_ff_m", "err_without_ff_m"])
        for i, (a, b) in enumerate(zip(trace_ff, trace_no)):
            w.writerow([i, f"{a:.4f}", f"{b:.4f}"])

    if verbose:
        print("\n=== SITL moving-target tracking (Bab 4.2 row 5) ===")
        print(f"  target {TARGET_SPEED_MS} m/s  criterion < {MAX_STEADY_ERR_CM} cm")
        print(f"  feedforward ON : steady err {err_ff_cm:7.2f} cm   final AGL {agl_ff:.2f} m")
        print(f"  feedforward OFF: steady err {err_no_cm:7.2f} cm   final AGL {agl_no:.2f} m")
        print(f"  ratio OFF/ON = {err_no_cm / max(err_ff_cm, 1e-6):.0f}x  "
              f"[{'PASS' if passed else 'FAIL'}]")

    return {"steady_err_with_ff_cm": round(err_ff_cm, 2),
            "steady_err_without_ff_cm": round(err_no_cm, 2),
            "final_agl_ff_m": round(agl_ff, 2),
            "criterion_cm": MAX_STEADY_ERR_CM,
            "source": "SYNTHETIC_MODEL",
            "pass": bool(passed)}


if __name__ == "__main__":
    run()
