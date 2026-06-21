"""exp_bec_current.py -- BEC current margin (supports Bab 4.2, BEC row).

Risk question: does the BEC have headroom to power the Pi 5 + payload at worst-
case load? Pass criterion: peak current must leave at least config.BEC_MARGIN_MIN_A
headroom under config.BEC_RATED_CURRENT_A.

No BEC hardware is in the loop, so this is a MEASUREMENT HARNESS plus a worst-case
model, not a pure simulation. A CurrentSensor protocol (read_amps) mirrors the
real/mock split in mavlink_iface.py; the real backend would wrap an INA219/INA226
shunt monitor. The shipped MockCurrentSensor models Pi 5 + payload 5 V-rail draw
across a load profile (idle / camera streaming / full CPU + gimbal serial + Wi-Fi)
from the # TYPICAL figures in config. A run fed by a real sensor is tagged
source = HARDWARE; the mock is tagged SYNTHETIC_MODEL.

Seam for real hardware: implement read_amps() against the shunt and pass it to run().
Output: peak/mean/margin + results/exp_bec_current.csv current trace.
"""

from __future__ import annotations

import csv
from typing import Protocol, runtime_checkable

import numpy as np

from . import config

RATED_A = config.BEC_RATED_CURRENT_A
MARGIN_MIN_A = config.BEC_MARGIN_MIN_A
SAMPLE_HZ = 50.0
DURATION_S = 20.0
PROFILE = [("idle", 0.20), ("camera", 0.20), ("full", 0.60)]


@runtime_checkable
class CurrentSensor(Protocol):
    def read_amps(self) -> float: ...


class MockCurrentSensor:
    """Model Pi 5 + payload 5 V-rail current for a named load phase (# TYPICAL)."""

    def __init__(self, rng=None, noise_a=0.05):
        self.rng = rng or np.random.default_rng(99)
        self.noise_a = noise_a
        self._phase = "idle"

    def set_phase(self, phase):
        self._phase = phase

    def _base(self):
        a = config.PI5_IDLE_A
        if self._phase in ("camera", "full"):
            a += config.PI5_CAMERA_A
        if self._phase == "full":
            a += (config.PI5_CPU_FULL_A + config.PI5_WIFI_A
                  + config.PI5_GIMBAL_SERIAL_A)
        return a

    def read_amps(self):
        a = self._base() + self.rng.normal(0, self.noise_a)
        if self._phase == "full" and self.rng.random() < 0.05:
            a += abs(self.rng.normal(0.25, 0.1))   # CPU/Wi-Fi transient
        return max(a, 0.0)


def run(sensor=None, verbose: bool = True):
    is_mock = sensor is None
    sensor = sensor or MockCurrentSensor()
    n = int(DURATION_S * SAMPLE_HZ)
    bounds = np.cumsum([f for _, f in PROFILE]) * n
    trace = np.zeros(n)
    phases = []
    for i in range(n):
        phase = PROFILE[-1][0]
        for (name, _), b in zip(PROFILE, bounds):
            if i < b:
                phase = name
                break
        if isinstance(sensor, MockCurrentSensor):
            sensor.set_phase(phase)
        phases.append(phase)
        trace[i] = sensor.read_amps()

    peak = float(trace.max())
    mean = float(trace.mean())
    margin = RATED_A - peak
    passed = margin >= MARGIN_MIN_A

    with open(config.results_path("exp_bec_current.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["sample", "t_s", "phase", "current_a"])
        for i in range(n):
            w.writerow([i, f"{i/SAMPLE_HZ:.3f}", phases[i], f"{trace[i]:.4f}"])

    if verbose:
        print("\n=== BEC current margin (Bab 4.2 BEC row) ===")
        print(f"  model=Pi5+payload  rated={RATED_A} A  required margin>="
              f"{MARGIN_MIN_A} A  source={'SYNTHETIC_MODEL' if is_mock else 'HARDWARE'}")
        for name, frac in PROFILE:
            seg = trace[[i for i in range(n) if phases[i] == name]]
            if len(seg):
                print(f"  phase {name:<7} ({frac:>4.0%}): mean={seg.mean():.2f} A  "
                      f"peak={seg.max():.2f} A")
        print(f"  overall peak={peak:.2f} A  mean={mean:.2f} A  margin={margin:.2f} A  "
              f"[{'PASS' if passed else 'FAIL'}]")

    return {"peak_a": round(peak, 3), "mean_a": round(mean, 3),
            "rated_a": RATED_A, "margin_a": round(margin, 3),
            "required_margin_a": MARGIN_MIN_A,
            "source": "SYNTHETIC_MODEL" if is_mock else "HARDWARE",
            "pass": bool(passed)}


if __name__ == "__main__":
    run()
