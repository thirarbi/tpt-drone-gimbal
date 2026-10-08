"""exp_detection_rate.py -- detection rate vs AGL (supports Bab 4.2, row 2).

Risk question: across the operating altitude band, does the nested-ArUco pad stay
detectable, and does the outer->inner marker handoff happen as designed
(principle 5)? Pass criterion: every AGL band's detection rate must meet
config.DETECTION_RATE_MIN.

Method: sweep AGL AGL_MIN..AGL_MAX; per band render N synthetic frames with
randomized in-plane offset and small randomized tilt/yaw, run the genuine
MarkerDetector, record hit/miss and which marker id (outer/inner) produced the
fix. Output: console table + sparkline, flagged failing bands,
results/exp_detection_rate.csv, and (if matplotlib present)
results/detection_rate_vs_agl.png. source = SYNTHETIC_MODEL.
"""

from __future__ import annotations

import csv

import numpy as np

from . import config, markers, synthetic_markers

AGL_MIN_M = 0.30
AGL_MAX_M = 30.0
N_BANDS = 16
FRAMES_PER_BAND = 60
MAX_OFFSET_FRAC = 0.10   # worst-case pad offset the precision approach must hold (frac of AGL)
MAX_TILT_DEG = 5.0
RATE_MIN = config.DETECTION_RATE_MIN
SPARK = " .:-=+*#@"


def _sparkline(vals):
    return "".join(SPARK[max(0, min(len(SPARK) - 1, int(round(v * (len(SPARK) - 1)))))]
                   for v in vals)


def run(frames_per_band: int = FRAMES_PER_BAND, verbose: bool = True):
    rng = np.random.default_rng(12345)
    detector = markers.MarkerDetector()
    agls = np.geomspace(AGL_MIN_M, AGL_MAX_M, N_BANDS)

    rows = []
    for agl in agls:
        hits = inner = outer = 0
        for _ in range(frames_per_band):
            r = max(0.03, MAX_OFFSET_FRAC * agl)
            off = rng.uniform(-r, r, size=2)
            img, _ = synthetic_markers.render_view(
                float(agl), offset_xy=off, tilt_deg=rng.uniform(0, MAX_TILT_DEG),
                yaw_deg=rng.uniform(-180, 180), rng=rng)
            pose = detector.detect(img, est_agl_m=float(agl))
            if pose.valid:
                hits += 1
                if pose.marker_id == config.INNER_MARKER_ID:
                    inner += 1
                else:
                    outer += 1
        rate = hits / frames_per_band
        rows.append({"agl": float(agl), "rate": rate,
                     "inner_frac": inner / max(hits, 1),
                     "outer_frac": outer / max(hits, 1)})

    min_rate = min(r["rate"] for r in rows)
    failing = [round(r["agl"], 2) for r in rows if r["rate"] < RATE_MIN]
    # Observed source crossover: highest AGL where the inner still dominates fixes.
    crossover = None
    for r in sorted(rows, key=lambda x: x["agl"]):
        if r["rate"] > 0 and r["inner_frac"] >= 0.5:
            crossover = r["agl"]
    passed = len(failing) == 0

    with open(config.results_path("exp_detection_rate.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["agl_m", "detection_rate", "inner_frac", "outer_frac"])
        for r in rows:
            w.writerow([f"{r['agl']:.3f}", f"{r['rate']:.3f}",
                        f"{r['inner_frac']:.3f}", f"{r['outer_frac']:.3f}"])
    _maybe_plot(rows)

    if verbose:
        print("\n=== Detection rate vs AGL (Bab 4.2 row 2) ===")
        print(f"  frames/band={frames_per_band}  threshold={RATE_MIN:.0%}  "
              f"policy handoff={config.HANDOFF_AGL_M} m")
        print(f"  {'AGL(m)':>7} {'rate':>6} {'src':>6}   bar")
        for r in rows:
            src = ("inner" if r["inner_frac"] >= 0.5 else
                   "outer" if r["outer_frac"] >= 0.5 else "-")
            flag = "  <-- below threshold" if r["rate"] < RATE_MIN else ""
            print(f"  {r['agl']:7.2f} {r['rate']:6.2f} {src:>6}   "
                  f"{'#' * int(round(r['rate'] * 20)):<20}{flag}")
        print(f"  sparkline: {_sparkline([r['rate'] for r in rows])}")
        print(f"  observed inner->outer source crossover ~ "
              f"{crossover if crossover else float('nan'):.2f} m")
        print(f"  min rate={min_rate:.2f}  failing bands={failing}  "
              f"[{'PASS' if passed else 'FAIL'}]")

    return {"by_agl_band": {f"{r['agl']:.2f}": round(r["rate"], 3) for r in rows},
            "min_rate": round(min_rate, 3), "threshold": RATE_MIN,
            "observed_source_crossover_m": None if crossover is None else round(crossover, 2),
            "policy_handoff_agl_m": config.HANDOFF_AGL_M,
            "failing_bands_m": failing, "source": "SYNTHETIC_MODEL",
            "pass": bool(passed)}


def _maybe_plot(rows):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception:
        return
    agls = [r["agl"] for r in rows]
    fig, ax = plt.subplots(figsize=(7, 4))
    ax.plot(agls, [r["rate"] for r in rows], "o-", label="detection rate")
    ax.axhline(RATE_MIN, ls="--", color="r", label=f"threshold {RATE_MIN:.0%}")
    ax.axvline(config.HANDOFF_AGL_M, ls=":", color="g", label="policy handoff")
    ax.set_xscale("log"); ax.set_xlabel("AGL (m)"); ax.set_ylabel("detection rate")
    ax.set_ylim(-0.02, 1.02); ax.set_title("Detection rate vs AGL"); ax.legend()
    ax.grid(True, which="both", alpha=0.3); fig.tight_layout()
    fig.savefig(config.results_path("detection_rate_vs_agl.png"), dpi=110)
    plt.close(fig)


if __name__ == "__main__":
    run()
