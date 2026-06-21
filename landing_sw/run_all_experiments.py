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
    md = "\n".join(lines)
    with open(config.results_path("experiment_summary.md"), "w") as f:
        f.write(md)

    print("\n" + "=" * 64 + "\n" + md)
    if errors:
        print("\nExperiments that raised (bugs, not criterion failures):",
              list(errors.keys()))
    return 0 if all_pass else 1


if __name__ == "__main__":
    sys.exit(main())
