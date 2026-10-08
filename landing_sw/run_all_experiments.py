"""run_all_experiments.py -- single entry point for the Bab 4.2 experiment suite.

Runs every experiment in order, isolating failures so one broken experiment does
not abort the rest, then writes a consolidated machine-readable
results/experiment_summary.json (each metric: measured value, criterion, verdict,
and a "source" field of SYNTHETIC_MODEL or HARDWARE so synthetic predictions are
never mistaken for real measurements) and a human-readable
results/experiment_summary.md (risk question | metric/criterion | result | source
| pass/fail). Non-zero exit code if any experiment fails its criterion (CI-able).

Run: python -m landing_sw.run_all_experiments
"""

from __future__ import annotations

import json
import sys
import traceback

from . import (config, latency_bench, sitl_harness, exp_detection_rate,
               exp_motion_blur, exp_pose_flip, exp_bec_current)

EXPERIMENTS = [
    ("latency", "Latency budget", latency_bench.run,
     "Can the perception+control loop close within the photon-to-setpoint budget?",
     "projected full p95 vs 66 ms ceiling"),
    ("detection_rate", "Detection rate vs AGL", exp_detection_rate.run,
     "Is the nested pad detectable across the whole AGL band (incl. handoff)?",
     "min detection rate vs 95% threshold"),
    ("motion_blur", "Rolling vs global shutter", exp_motion_blur.run,
     "Would rolling shutter corrupt pose during gimbal slew (vs chosen global)?",
     "rolling-shutter corner error at SEARCH slew"),
    ("pose_flip", "Pose-flip ambiguity", exp_pose_flip.run,
     "Does the PnP pose-flip ambiguity corrupt the landing position near nadir?",
     "worst-case position impact vs 5 cm (+ reproj/chi2 discrimination)"),
    ("bec_current", "BEC current margin", exp_bec_current.run,
     "Does the BEC have headroom for the Pi 5 + payload worst case?",
     "peak current vs rated current"),
    ("feedforward", "Moving-target feedforward", sitl_harness.run,
     "Does velocity feedforward keep the vehicle over a moving pad?",
     "steady tracking error with FF on"),
]


def _line(key, res):
    if key == "latency":
        s = (f"projected p95 {res['projected_full_p95_ms']} ms "
             f"(ceiling {res['budget_ms']} ms; compute {res['compute_p95_ms']} ms)")
    elif key == "detection_rate":
        s = (f"min {res['min_rate']:.0%} (thr {res['threshold']:.0%}); "
             f"failing {res['failing_bands_m']} m; "
             f"src crossover ~{res['observed_source_crossover_m']} m")
    elif key == "motion_blur":
        s = (f"global 0 px (chosen HW); rolling {res['rolling_err_at_search_px']} px @ "
             f"{res['at_search_slew_deg_s']} deg/s vs {res['budget_px']} px budget "
             f"({'would exceed' if res['rolling_would_exceed_budget'] else 'within'})")
    elif key == "pose_flip":
        s = (f"position impact {res['max_pos_impact_cm']} cm "
             f"(< {res['pos_impact_criterion_cm']} cm); flip is rotational, "
             f"chi2-reject {res['chi2_reject_rate_safe_band']:.0%}; "
             f"orient-ambiguous {res['orientation_ambiguous_elev_deg']} deg")
    elif key == "bec_current":
        s = (f"peak {res['peak_a']} A / rated {res['rated_a']} A; "
             f"margin {res['margin_a']} A")
    elif key == "feedforward":
        s = (f"{res['steady_err_with_ff_cm']} cm FF-on vs "
             f"{res['steady_err_without_ff_cm']} cm FF-off")
    else:
        s = json.dumps(res)
    return s, bool(res.get("pass")), res.get("source", "SYNTHETIC_MODEL")


# Static design footer: the MINIMUM conditions each criterion needs to hold, and
# the safety margin the current design carries over that minimum. Appended to the
# summary so the "what must stay true for this to keep passing" envelope travels
# with the results. Keep in sync with config.py if tunables move.
MIN_CRITERIA_FOOTER = """
---

## Minimum criteria to succeed + safety margins

These are the binding conditions for each risk question and the headroom the
current design holds over them. If a condition is violated, the corresponding row
flips to FAIL.

| # | Minimum required to pass | Current design | Safety margin |
|---|--------------------------|----------------|---------------|
| 1 | photon-to-setpoint p95 <= 66 ms ceiling | projected p95 ~17.8 ms | ~48 ms (~73% of budget unused) |
| 2 | every AGL band detects >= 95%; needs outer marker big enough to clear the ~20 px detector floor at 30 m **and** inner marker + quiet ring inside the FOV at 0.30 m, **and** the approach holds the pad within <= 10% of AGL off-nadir | min band 100%; outer 1.25 m -> ~23 px @ 30 m; inner 0.10 m -> ~1.2 cm (~22 px) FOV clearance @ 0.30 m; handoff/crossover ~4.75 m sits mid-band | +5 pp over the 95% floor; ~17% over the pixel floor at 30 m; ~11% FOV clearance at 0.30 m |
| 3 | pose-corner error <= 1.0 px during the 60 deg/s SEARCH slew | global shutter -> 0 px (rolling would be ~1.7 px) | full 1.0 px budget unused; choosing global shutter is the load-bearing requirement |
| 4 | pose-flip position impact <= 5.0 cm, gated by reproj (<=3 px) + chi2 | worst-case ~0.11 cm (the flip is rotational, not translational) | ~44x (4.89 cm) |
| 5 | BEC headroom >= 1.0 A over the Pi 5 + payload peak | peak 3.63 A on a 5.0 A rail | 1.37 A (27% of rated) |
| 6 | steady moving-target error <= 10 cm with feedforward on | 4.95 cm (off: 112 cm) | ~2x (5.05 cm); feedforward must be enabled |

**Load-bearing assumptions for row 2 (detection):** lo-res 640x400 detect stream
(f = 560 px), nested pad of a 1.25 m outer + 0.10 m inner marker, and a precision
approach that keeps the pad within ~0.1 x AGL of nadir. At the original 18%-of-AGL
worst-case offset, a two-marker pad cannot span 0.3-30 m on the lo-res stream (the
outer overflows the frame before the inner is large enough at the handoff); the
fix was to size both markers to the band ends and bound the offset envelope, not
to add hardware.
"""


def main():
    results = {}
    errors = {}
    for key, label, fn, _, _ in EXPERIMENTS:
        print(f"\n{'='*64}\n[ {label} ]\n{'='*64}", flush=True)
        try:
            results[key] = fn()
        except Exception as e:
            errors[key] = "".join(traceback.format_exception(type(e), e, e.__traceback__))
            results[key] = {"pass": False, "error": str(e), "source": "SYNTHETIC_MODEL"}
            print(f"  !! {label} raised: {e}")

    with open(config.results_path("experiment_summary.json"), "w") as f:
        json.dump(results, f, indent=2)

    lines = ["# Bab 4.2 -- Hasil Eksperimen", "",
             "| # | Risk question | Metric / criteria | Result | Source | Pass |",
             "|---|---------------|-------------------|--------|--------|------|"]
    all_pass = True
    for i, (key, label, _, question, metric) in enumerate(EXPERIMENTS, 1):
        res = results.get(key, {})
        if "error" in res:
            s, ok, src = f"ERROR: {res['error']}", False, res.get("source", "")
        else:
            s, ok, src = _line(key, res)
        all_pass = all_pass and ok
        lines.append(f"| {i} | {question} | {metric} | {s} | {src} | "
                     f"{'PASS' if ok else 'FAIL'} |")
    npass = sum(1 for r in results.values() if r.get("pass"))
    lines += ["", f"**Overall: {'PASS' if all_pass else 'FAIL'}** "
              f"({npass}/{len(EXPERIMENTS)} criteria met)", ""]
    md = "\n".join(lines) + MIN_CRITERIA_FOOTER
    with open(config.results_path("experiment_summary.md"), "w") as f:
        f.write(md)

    print("\n" + "=" * 64 + "\n" + md)
    if errors:
        print("\nExperiments that raised (bugs, not criterion failures):",
              list(errors.keys()))
    return 0 if all_pass else 1


if __name__ == "__main__":
    sys.exit(main())
