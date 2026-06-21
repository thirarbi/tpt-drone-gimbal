"""latency_bench.py -- photon-to-setpoint pipeline latency (principle 1).

Risk question: can the perception->control chain produce a setpoint within the
real-time budget? The benchmark is END-TO-END latency, not loop frequency:
target LATENCY_TARGET_MS (33 ms = one frame), hard ceiling LATENCY_BUDGET_MS
(66 ms = two frames). PASS if the projected full p95 <= LATENCY_BUDGET_MS.

It times the genuine compute stages (detect -> PnP+transform -> EKF -> pointing+
setpoint) with perf_counter over many synthetic frames, then adds the allocated
capture_exposure_readout + mavlink_serial_tx from config (stages this harness
cannot time on a laptop) to project a full photon-to-setpoint p95.

NOTE: measured on the dev CPU; MUST be re-run on the Pi 5 (source SYNTHETIC_MODEL).
Output: console table + results/latency_bench.csv.
"""

from __future__ import annotations

import csv
import time

import numpy as np

from . import config, ekf, frames, landing, markers, pointing, synthetic_markers
from .mavlink_iface import MockMavlink

N_CYCLES = 300
BENCH_AGL_M = 5.0
GIMBAL_NADIR = (config.GIMBAL_PITCH_LAND_DEG, 0.0, 0.0)
# Stages added from the budget allocation (not timeable on a laptop).
ADDED_MS = (config.LATENCY_ALLOC_MS["capture_exposure_readout"]
            + config.LATENCY_ALLOC_MS["mavlink_serial_tx"])


def run(n_cycles: int = N_CYCLES, verbose: bool = True):
    rng = np.random.default_rng(7)
    detector = markers.MarkerDetector()
    kf = ekf.RelativeMarkerEKF()
    mav = MockMavlink()
    point = pointing.PointingController()
    land = landing.LandingController(mav)

    stages = {"detect": [], "pnp_transform": [], "ekf": [], "pointing_setpoint": [],
              "compute_e2e": []}
    for _ in range(n_cycles):
        off = rng.uniform(-0.4, 0.4, size=2)
        img, _ = synthetic_markers.render_view(BENCH_AGL_M, offset_xy=off,
                                               tilt_deg=rng.uniform(0, 3), rng=rng)
        t0 = time.perf_counter()
        pose = detector.detect(img, est_agl_m=BENCH_AGL_M)
        t1 = time.perf_counter()
        if pose.valid:
            pos_body = frames.camera_point_to_body(pose.t, GIMBAL_NADIR)
        else:
            pos_body = np.zeros(3)
        t2 = time.perf_counter()
        kf.predict(1.0 / config.CAM_FPS, mav.read_velocity_body())
        if pose.valid:
            kf.update(pos_body)
        t3 = time.perf_counter()
        if kf.initialized:
            point.step(1.0 / config.POINTING_RATE_HZ, kf.position)
            land.compute_velocity_setpoint(kf.position, kf.velocity, BENCH_AGL_M)
        t4 = time.perf_counter()

        stages["detect"].append((t1 - t0) * 1e3)
        stages["pnp_transform"].append((t2 - t1) * 1e3)
        stages["ekf"].append((t3 - t2) * 1e3)
        stages["pointing_setpoint"].append((t4 - t3) * 1e3)
        stages["compute_e2e"].append((t4 - t0) * 1e3)

    summary = {n: {"p50": float(np.percentile(v, 50)),
                   "p95": float(np.percentile(v, 95)),
                   "mean": float(np.mean(v)), "max": float(np.max(v))}
               for n, v in stages.items()}
    compute_p95 = summary["compute_e2e"]["p95"]
    projected_p95 = compute_p95 + ADDED_MS
    passed = projected_p95 <= config.LATENCY_BUDGET_MS

    with open(config.results_path("latency_bench.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["stage", "p50_ms", "p95_ms", "mean_ms", "max_ms"])
        for n, s in summary.items():
            w.writerow([n, f"{s['p50']:.3f}", f"{s['p95']:.3f}",
                        f"{s['mean']:.3f}", f"{s['max']:.3f}"])
        w.writerow(["added_alloc(capture+tx)", "", f"{ADDED_MS:.3f}", "", ""])
        w.writerow(["projected_full_p95", "", f"{projected_p95:.3f}", "", ""])

    if verbose:
        print("\n=== Photon-to-setpoint latency (Bab 4.2 row: latency) ===")
        print(f"  cycles={n_cycles}  target={config.LATENCY_TARGET_MS} ms  "
              f"ceiling={config.LATENCY_BUDGET_MS} ms  (DEV CPU -- re-run on Pi 5)")
        print(f"  {'stage':<20}{'p50':>8}{'p95':>8}{'mean':>8}{'max':>8}  (ms)")
        for n in ("detect", "pnp_transform", "ekf", "pointing_setpoint", "compute_e2e"):
            s = summary[n]
            print(f"  {n:<20}{s['p50']:8.2f}{s['p95']:8.2f}{s['mean']:8.2f}{s['max']:8.2f}")
        print(f"  + allocated capture+mavlink_tx = {ADDED_MS:.1f} ms")
        print(f"  --> projected full p95 = {projected_p95:.2f} ms  "
              f"[{'PASS' if passed else 'FAIL'}]")

    return {"compute_p95_ms": round(compute_p95, 3),
            "added_alloc_ms": round(ADDED_MS, 3),
            "projected_full_p95_ms": round(projected_p95, 3),
            "target_ms": config.LATENCY_TARGET_MS,
            "budget_ms": config.LATENCY_BUDGET_MS,
            "source": "SYNTHETIC_MODEL",
            "pass": bool(passed)}


if __name__ == "__main__":
    run()
