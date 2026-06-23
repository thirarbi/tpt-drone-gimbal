# Module 08 — Telemetry & the Bab 4.2 Experiment Suite

> This codebase exists to *answer a thesis's risk questions with evidence*. This
> module is about the **validation methodology**: how each design risk is turned into
> a falsifiable experiment with a numeric pass/fail criterion, how synthetic
> predictions are honestly distinguished from hardware measurements, and how the
> per-flight telemetry is logged. This is the part that makes it a *Tugas Akhir* and
> not just a program — and the part an examiner will probe hardest.

## Learning objectives

- Explain the **experiment harness** structure: one risk question → one experiment →
  one criterion from `config.py` → a `SYNTHETIC_MODEL` or `HARDWARE` verdict.
- Summarize what each of the six experiments tests, its method, and its result.
- Explain the **latency budget** (principle 1) and how `latency_bench.py` projects a
  full photon-to-setpoint p95.
- Read the SQLite telemetry schema and explain the off-hot-path logging (principle 7).
- Critically discuss the **one genuine FAIL** (detection rate) and what it means.

## Read, in this order

1. [`landing_sw/run_all_experiments.py`](../landing_sw/run_all_experiments.py) — the
   orchestrator; read `EXPERIMENTS` (the risk→experiment table) and `main()`.
2. [`landing_sw/results/experiment_summary.md`](../landing_sw/results/experiment_summary.md)
   — the output you're learning to produce and read.
3. The six experiments, in this order (easiest methodology first):
   [`exp_bec_current.py`](../landing_sw/exp_bec_current.py),
   [`latency_bench.py`](../landing_sw/latency_bench.py),
   [`exp_detection_rate.py`](../landing_sw/exp_detection_rate.py),
   [`exp_motion_blur.py`](../landing_sw/exp_motion_blur.py),
   [`exp_pose_flip.py`](../landing_sw/exp_pose_flip.py),
   [`sitl_harness.py`](../landing_sw/sitl_harness.py).
4. [`landing_sw/telemetry_db.py`](../landing_sw/telemetry_db.py).

## The harness philosophy

`run_all_experiments.py` is a small but exemplary piece of research engineering.
Internalize three habits from it:

1. **Each experiment maps to a stated risk question.** The `EXPERIMENTS` list
   (lines 23–42) pairs every experiment with the *question it answers* and the
   *metric/criterion*. Science, not vibes.
2. **The criterion is a config constant, not a magic number in the experiment.** Each
   pass/fail threshold lives in `config.py`'s "Experiment acceptance thresholds" block
   (e.g. `DETECTION_RATE_MIN`, `POSE_FLIP_MAX_POS_IMPACT_CM`). The experiment *reads*
   it. So the spec and the test can't drift apart.
3. **Provenance is tracked.** Every result carries `source: SYNTHETIC_MODEL` or
   `HARDWARE`. A simulated prediction is *never* allowed to masquerade as a real
   measurement. The whole suite is also **CI-able**: non-zero exit code if any
   criterion fails (lines 98, 112). This is exactly the discipline a thesis defense
   rewards: "which of your numbers are measured, and which are modeled?"

The output is both machine-readable (`experiment_summary.json`) and human-readable
(`experiment_summary.md`, the table you read in Module 01).

## The six experiments

Read each docstring — they're excellent. Summary, with the principle each defends:

| Experiment | Risk question | Method (synthetic) | Criterion | Result |
|------------|---------------|--------------------|-----------|--------|
| **latency** (`latency_bench.py`) | Can the loop close within the photon-to-setpoint budget? (principle 1) | Time the *real* compute stages (detect→PnP→EKF→setpoint) over 300 frames; **add** the allocated capture + serial-tx times that can't be timed on a laptop; take p95. | projected full p95 ≤ 66 ms | **PASS** (~17 ms) |
| **detection_rate** (`exp_detection_rate.py`) | Is the nested pad detectable across 0.3–30 m, incl. handoff? (principle 5) | Sweep 16 AGL bands, 60 synthetic frames each with random offset/tilt/yaw; run the genuine detector; record hit rate + which marker. | every band ≥ 95% | **FAIL** (min 57%) |
| **motion_blur** (`exp_motion_blur.py`) | Would a rolling shutter corrupt pose during gimbal slew? (justifies the global-shutter choice) | Geometric model of per-row readout skew vs. a global shutter, reprojecting corners at search slew rates. | global-shutter ≤ 1 px (it's 0 by construction) | **PASS** (rolling would hit 1.07 px → choice vindicated) |
| **pose_flip** (`exp_pose_flip.py`) | Does the PnP pose-flip ambiguity corrupt the landing position? (principle 6) | Sweep elevation/altitude; take both IPPE solutions; measure tvec separation, reproj separation, and EKF chi2 reject rate. | worst-case position impact < 5 cm | **PASS** (~0.1 cm; flip is *rotational*) |
| **bec_current** (`exp_bec_current.py`) | Does the BEC have headroom for Pi 5 + payload worst case? | Measurement *harness* (real `CurrentSensor` seam) + a worst-case load model across idle/camera/full phases. | peak leaves ≥ 1 A margin under 5 A rated | **PASS** (peak 3.6 A, margin 1.4 A) |
| **feedforward** (`sitl_harness.py`) | Does velocity feedforward keep the vehicle over a *moving* pad? (principles 2 & 3) | Closed-loop sim: pad at 1.4 m/s, perception = truth + noise + dropout at 30 fps, EKF + landing law, FF on vs off. | steady tracking error < 10 cm with FF on | **PASS** (~5 cm vs ~112 cm off) |

### The latency budget worth dwelling on (principle 1)

`latency_bench.py` embodies the project's thesis: **end-to-end latency, not loop
rate, is the benchmark.** Target 33 ms (one frame), ceiling 66 ms (two). It times the
*genuine* compute path with `perf_counter`, then transparently *adds* the budgeted
stages it cannot measure on a laptop (`capture_exposure_readout`, `mavlink_serial_tx`
from `config.LATENCY_ALLOC_MS`) to project a full p95 — and the docstring flags
"DEV CPU — MUST be re-run on the Pi 5," with `source = SYNTHETIC_MODEL`. That honesty
about what is and isn't measured is the methodological heart of the whole suite.

### The honest FAIL (read the memory + the docstring)

The suite reports **5/6 PASS**, and the one FAIL — detection rate — is *real*, not a
bug: at 30 m the 0.78 m outer marker is only ~15 px across (Module 03 Exercise 1) and
detection drops to 57%; near 0.3 m it dips too. The right thesis response is *not* to
loosen the threshold but to **discuss the operating-envelope implication** (the
reliable detection band is narrower than the full 0.3–30 m sweep, suggesting either a
larger outer marker, an additional mid-size marker, or a constrained approach
profile). Being able to *defend a failing result as informative* is a sign you
understand the methodology rather than just the code.

## Telemetry (principle 7)

`telemetry_db.py` is a small SQLite logger: one `flight` row per run, many `tick`
rows (the full per-frame perception→control state — see `TICK_COLS`, lines 16–22),
and `event` rows for state transitions/faults. The FSM's `_log_tick`/`log_event`
(Module 07) feed it. The principle-7 point: logging is the **off-hot-path telemetry
sink** — the control loop must never block on disk I/O. After a flight you can query
this DB (or the per-experiment CSVs in `results/`) to reconstruct exactly what the
estimator and controller did each frame — invaluable for debugging and for thesis
plots.

## External resources

- **Real-time / latency in perception-control loops:** the "sense-plan-act latency"
  literature; a concrete drone-relevant read is the **agile autonomy / high-speed
  flight** work (Davide Scaramuzza's group, UZH RPG) on why latency dominates —
  e.g. their *Vision-based agile flight* surveys: https://rpg.ifi.uzh.ch/
- **Percentiles (p50/p95) and why tail latency matters** rather than the mean: Gil
  Tene's "How NOT to Measure Latency" talk (the coordinated-omission idea) — directly
  relevant to why the suite reports p95.
- **ArduPilot SITL** for turning the `sitl_harness` synthetic loop into a real
  software-in-the-loop test against ArduCopter:
  https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html
- **Experimental method / pre-registered criteria** mindset — any research-methods
  guide on stating acceptance criteria *before* running; this suite is a clean example
  of "criterion in `config`, measured value from the run, verdict computed."
- **SQLite** docs (the telemetry store): https://www.sqlite.org/docs.html ; query the
  produced `results/flightlog.sqlite` with the `sqlite3` CLI or DB Browser for SQLite.

## Exercises

1. **Run the suite.** `python -m landing_sw.run_all_experiments`. Reproduce the 5/6
   table. Open `results/experiment_summary.json` and find the `source` field on each
   result — which one *could* become `HARDWARE` without code changes, and how
   (`exp_bec_current`'s sensor seam)?
2. **Move the goalpost — and resist it.** Lower `DETECTION_RATE_MIN` to 0.5 so the
   suite "passes," then write two sentences on why that's the *wrong* fix and what the
   right engineering responses are. Revert.
3. **Latency sensitivity.** In `config.LATENCY_ALLOC_MS`, double `detect_aruco` and
   re-run `latency_bench`. Does the projected p95 still clear 66 ms? How much headroom
   is there for a slower real Pi 5?
4. **Query telemetry.** Run `python -m landing_sw.main --sim`, then open
   `results/flightlog.sqlite` and `SELECT state, COUNT(*) FROM tick GROUP BY state;`.
   Which states did the sim spend time in? Cross-check against Module 07.
5. **Pose-flip claim.** From `exp_pose_flip.py`'s output, confirm the central claim:
   the two PnP branches differ in *rotation* but barely in *translation*. Why does
   that make the failure mode harmless *for landing specifically*?
