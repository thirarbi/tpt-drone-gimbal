# Curriculum: Understanding the `landing_sw` Vision Precision-Landing Codebase

A self-paced reading course built around **this** repository. It takes you from
the parts you already know (control loops, Kalman filtering) to the parts that
are likely new (camera geometry, ArUco/PnP, the ArduPilot + MAVLink ecosystem,
companion-computer software architecture), then back to the closed loop and the
experimental validation that this code exists to produce.

## Who this is for

You — a 3rd-year Engineering Physics student at ITB who has passed:

- **Automatic Control** — block diagrams, stability, feedback, proportional control.
- **Multivariable Control** — LQR, intro to the Kalman filter.

That background is a real head start: this system is, at its core, a perception
front-end feeding a feedback controller, with a Kalman filter (an EKF) in the
middle. The modules below deliberately *anchor new ideas on the control/estimation
language you already speak*, and spend most of their effort on the genuinely new
material: turning pixels into a metric position, the coordinate-frame bookkeeping,
and how a "companion computer" cooperates with a flight controller.

## What this codebase actually is

`landing_sw/` is the **companion-computer software** for a precision-landing drone,
the software artifact of your Tugas Akhir (ITB Teknik Fisika, pad TF3001). The
physical system:

```
   ArduCam OV9281  ──┐
   (global-shutter   │  CSI
    mono camera)      ├──▶  Raspberry Pi 5  ──MAVLink/serial──▶  SpeedyBee F405
   SimpleBGC32 ───────┘    (companion: runs              (ArduCopter flight
   gimbal (serial)          landing_sw)                   controller / autopilot)
```

The Pi watches the ground through a gimbal-stabilized camera, finds a **nested
ArUco** landing pad (a big marker for altitude, a small concentric one for the
final touchdown), estimates where the pad is relative to the drone *and how fast
it is moving*, and streams velocity commands to ArduCopter so the drone descends
onto it — even if the pad is moving (~1.4 m/s).

Everything runs **with no hardware** via `--sim` and a synthetic camera, which is
why you can study, run, and modify all of it on a laptop.

## How to use this curriculum

Read the modules **in order**. Each one:

1. States **learning objectives**.
2. Tells you which **source files to read**, in what order.
3. Explains the concepts, *bridging from your coursework where possible*.
4. Lists **external resources** — GitHub repos, books, papers, docs — to go deeper.
5. Gives **exercises**: small experiments you run or code changes you make to
   confirm understanding. Do them. Reading drone-control code is not the same as
   understanding it until you've perturbed it and watched what breaks.

Estimated pace: one module per study session (~1.5–3 h each). The whole course is
roughly a focused week, or a comfortable two to three weeks alongside classes.

## Module map

| #  | Module | Core source files | New vs. familiar |
|----|--------|-------------------|------------------|
| 01 | [System overview & the 7 design principles](01_system_overview.md) | `config.py`, `main.py` | Orientation |
| 02 | [Coordinate frames & rotations](02_frames_and_rotations.md) | `frames.py` | **New** |
| 03 | [Computer vision: cameras, ArUco, PnP](03_vision_aruco_pnp.md) | `markers.py`, `synthetic_markers.py` | **New** |
| 04 | [The relative-marker EKF](04_ekf.md) | `ekf.py` | Familiar → extended |
| 05 | [Control: pointing & landing laws](05_control_laws.md) | `landing.py`, `pointing.py` | **Familiar** |
| 06 | [The drone stack: MAVLink, ArduPilot, interfaces](06_drone_stack.md) | `mavlink_iface.py`, `gimbal_iface.py` | **New** |
| 07 | [Orchestration: the landing FSM](07_fsm_orchestration.md) | `fsm.py`, `main.py` | Mostly new |
| 08 | [Telemetry & the Bab 4.2 experiment suite](08_telemetry_and_experiments.md) | `telemetry_db.py`, `run_all_experiments.py`, `exp_*.py`, `sitl_harness.py`, `latency_bench.py` | Methodology |
| 09 | [Capstone: read the loop end-to-end & extend it](09_capstone.md) | all | Synthesis |

## One-time setup (do this before Module 01)

The dependencies live in a virtualenv at the repo root (system Python is locked
down — PEP 668). From the repo root:

```bash
. .venv/bin/activate                       # numpy, opencv, scipy, pymavlink, matplotlib
python -m landing_sw.main --sim            # should print a touchdown with ~0 cm residual
python -m landing_sw.run_all_experiments   # runs the whole Bab 4.2 suite (~5/6 pass)
```

Note the `python -m landing_sw.<module>` form: the package uses **relative imports**,
so you must run modules through the package, not as `python landing_sw/foo.py`.

## A reading order note

The code has unusually good docstrings — every module opens with a paragraph
explaining its job and which "design principle" it serves. **Read the module
docstring first, every time.** This curriculum is largely a guided tour through
those docstrings with the missing background filled in.
