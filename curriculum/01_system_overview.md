# Module 01 — System Overview & the 7 Design Principles

## Learning objectives

By the end you can:

- Draw the full perception → estimation → control → actuation loop and name the
  module responsible for each block.
- State the **7 design principles** the codebase repeatedly cites, and say *why*
  each is load-bearing (i.e., what breaks if you violate it).
- Read `config.py` and explain what every block of constants controls.
- Run the simulator and the experiment suite and interpret the output.

## Read, in this order

1. `curriculum/00_README.md` (if you haven't).
2. [`landing_sw/config.py`](../landing_sw/config.py) — top to bottom. It is the
   single source of truth; every other module imports its numbers from here.
3. [`landing_sw/main.py`](../landing_sw/main.py) — the `run_sim()` loop is the
   whole system in 40 lines.
4. [`landing_sw/results/experiment_summary.md`](../landing_sw/results/experiment_summary.md)
   — what "done and validated" looks like for this project.

## The loop, as a control engineer sees it

You already know the canonical feedback diagram: reference → error → controller →
plant → sensor → back to error. Map this system onto it:

```
 pad position (reference = 0 relative error, "be directly above the pad")
        │
        ▼
   ┌──────────┐   image    ┌───────────┐  marker pos   ┌──────────┐ rel. pos+vel ┌───────────┐
   │  camera  │──────────▶ │ MarkerDet │─────────────▶ │   EKF    │─────────────▶│  Landing  │
   │ (sensor) │  640x400   │ +PnP      │  (camera fr.) │ (filter) │   (body fr.) │ controller│
   └──────────┘   gray     └───────────┘               └──────────┘              └─────┬─────┘
        ▲                       │frames.py                                              │ v_cmd
        │                       │(camera→body)                                          ▼
        │                                                                        ┌───────────┐
        │                                                                        │ ArduCopter│
        └──────────────── drone physically moves ◀──────────────────────────────│ (plant)   │
                                                                                  └───────────┘
```

- **Sensor**: `markers.py` (detect) + `synthetic_markers.py` (the fake camera in sim).
- **Coordinate plumbing**: `frames.py` rotates the measurement out of the camera
  frame into the drone body frame. This step has no analog in a classroom SISO
  problem, and it is where a surprising amount of the engineering lives (Module 02).
- **Estimator**: `ekf.py` — your Kalman filter, specialized. (Module 04.)
- **Controller**: `landing.py` (vehicle velocity) + `pointing.py` (gimbal). (Module 05.)
- **Plant**: the drone, simulated by a kinematic integrator in `main.py`/`sitl_harness.py`,
  or real ArduCopter via `mavlink_iface.py`.
- **Supervisor**: `fsm.py` decides *which* of the above run, based on whether the
  marker is visible, the altitude, and the pilot's mode switch. (Module 07.)

Crucially, this is a **cascaded / nested** architecture, which your Multivariable
Control course gives you the vocabulary for: a fast inner loop (ArduCopter's own
attitude + EKF3 controllers, running on the F405 at hundreds of Hz) wrapped by a
slow outer loop (this companion software, running at the 30 fps camera rate). The
companion never touches motors directly; it hands ArduCopter *velocity setpoints*
and lets the autopilot's tuned inner loops execute them.

## The 7 design principles

The docstrings repeatedly cite "principle N". Here is the consolidated list,
reconstructed from the code, with the engineering reason each matters. Keep this
table next to you for the whole course — every module is an elaboration of one or
more of these.

| # | Principle | Where enforced | Why it's load-bearing |
|---|-----------|----------------|------------------------|
| 1 | **Latency, not loop rate, is the benchmark.** The metric is *photon-to-setpoint* end-to-end delay: target 33 ms (one frame), ceiling 66 ms. | `config.LATENCY_*`, `latency_bench.py` | A 30 Hz loop with 200 ms of latency flies blind. Phase lag from latency is what destabilizes a tracking loop — you know this as the phase margin story from Automatic Control. |
| 2 | **Velocity feedforward, not just proportional.** `v_cmd = Kp·error + G_ff·v_target_absolute`. | `landing.py`, `config.FEEDFORWARD_GAIN` | A pure P controller *lags a moving target* by `v_target/Kp` (≈1.1 m for the defaults — it never catches up). Feedforward cancels that lag. This is the single idea that makes landing on a *moving* pad work at 30 fps. |
| 3 | **The EKF estimates the target's ABSOLUTE velocity by folding ego-motion into `predict`.** State is relative position + target absolute velocity. | `ekf.py`, `fsm.py` | The "obvious" alternative (`v_target = v_rel + v_drone` computed *after* the filter) mixes a lagged perception estimate with a fresh autopilot velocity on *different time bases* and injects positive feedback. Putting ego-motion inside `predict` keeps one time base. |
| 4 | **The companion only bridges 30 fps gaps; it does NOT run a high-rate vehicle filter.** During marker loss, ArduCopter's onboard EKF3 dead-reckons the vehicle; this filter only coasts the *marker* estimate. | `ekf.py`, `fsm.py` COASTING state | Don't reinvent the autopilot's job. The companion's filter is small (6 states) and single-purpose. |
| 5 | **Nested concentric ArUco markers.** A large outer marker dominates at altitude; a small inner one takes over up close once the outer overflows the field of view. | `config.*_MARKER_*`, `markers.py`, `synthetic_markers.py` | One marker size can't span 30 m → 0.3 m. At 30 m a small marker is invisible; at 0.3 m a big marker overflows the frame. Hand off between them. |
| 6 | **Pose-flip mitigation is two-layered.** A planar marker's PnP has a near-degenerate "flip" solution; reject it with (a) a detector reprojection-error gate AND (b) an EKF chi-square innovation gate. | `markers.py` (reproj gate), `ekf.py` (chi2 gate) | A wrong pose flip could command a lateral lunge near the ground. Belt and suspenders. (Module 03 + 04.) |
| 7 | **Two camera streams + off-hot-path telemetry.** Low-res Y stream for detection (fast), full-res stream for recording; SQLite logging is off the control hot path. | `config` cam block, `telemetry_db.py` | Detecting on a small image is faster (principle 1); logging must never block control. |

If you remember nothing else from Module 01, remember principles **1, 2, 3**:
latency is the enemy, feedforward beats the lag, and ego-motion goes *inside* the
filter. They are the intellectual core of the thesis.

## Reading `config.py`

Notice the discipline: **there are no bare numeric literals anywhere else in the
codebase** — every tunable lives here, tagged with its provenance:

- `# MEASURE` — must be confirmed against real hardware/calibration (e.g. the
  camera matrix `CAM_K`, `EKF_MEAS_POS_STD`).
- `# TYPICAL` — a datasheet/handbook stand-in until measured (e.g. the Pi 5
  current figures).
- `# DESIGN` — a chosen design value (e.g. `REPROJ_ERR_MAX_PX`).

This is good engineering practice you should internalize: **separate the things
you measured from the things you guessed.** When this code runs an experiment, the
output is even tagged `SYNTHETIC_MODEL` vs `HARDWARE` so a simulated prediction is
never mistaken for a real measurement (Module 08).

Skim each block and connect it to a module:

- Camera block → Module 03.
- Nested ArUco block → Module 03.
- Latency budget → principle 1, Module 08.
- EKF block → Module 04.
- FSM thresholds → Module 07.
- Pointing/gimbal → Module 05/06.
- Landing controller gains (`KP_*`, `FEEDFORWARD_GAIN`) → Module 05.
- Experiment acceptance thresholds → Module 08.

## External resources

- **Drone software architecture overview (companion + autopilot):**
  ArduPilot "Companion Computers" docs — https://ardupilot.org/dev/docs/companion-computers.html
- **The big-picture textbook for the whole field** (frames, sensors, guidance,
  control of small UAVs), and the one most worth owning:
  *Beard & McLain, "Small Unmanned Aircraft: Theory and Practice"* (Princeton UP).
  Free supplementary material + simulation code:
  https://github.com/randybeard/mavsim_public
- **A real, readable ArUco precision-landing project to compare against** — Tiziano
  Fiorenzani's "How Do Drones Work":
  https://github.com/tizianofiorenzani/how_do_drones_work (see the
  `opencv`/`precision landing` examples) and his YouTube series of the same name.
  This codebase is a more rigorous cousin of that project.

## Exercises

1. Run `python -m landing_sw.main --sim`. It should report a touchdown with a
   centimeter-scale residual. Now open `main.py` and change `start_xyz` so the
   drone starts offset by `(3.0, 2.0, 6.0)`. Does it still land? What residual?
2. In `config.py`, set `FEEDFORWARD_GAIN = 0.0` and re-run
   `python -m landing_sw.sitl_harness`. Read the steady-state tracking error with
   and without feedforward. This is principle 2 in one number — note both values.
3. Without reading ahead, write down which module you think would have to change
   if the team switched from ArUco markers to an AprilTag pad. (Check yourself
   after Module 03.)
