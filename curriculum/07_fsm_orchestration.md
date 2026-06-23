# Module 07 — Orchestration: the Landing FSM

> Now you assemble the pieces. The **finite state machine** in `fsm.py` is the
> conductor: it decides, every frame, *which* of the subsystems you've studied run,
> and how the system responds to marker loss, altitude, mode switches, and faults.
> This is the **supervisory control** layer — the logic that turns a pile of
> controllers into a mission.

## Learning objectives

- Enumerate the 7 states and the transitions between them, and explain the *policy*
  behind each transition (especially marker-loss handling).
- Walk through the per-frame `step()` pipeline: detect → transform → predict →
  update → decide → act → log.
- Explain `COASTING` vs `GIMBAL_SEARCH` vs `ABORT` and the altitude gating that
  chooses among them (principles 3 & 4).
- Read `main.py run_sim()` and see the entire closed loop run.

## Read, in this order

1. [`landing_sw/fsm.py`](../landing_sw/fsm.py) — the whole file; the docstring is a
   complete spec.
2. [`landing_sw/main.py`](../landing_sw/main.py) — `run_sim()` drives the FSM with the
   synthetic camera + mocks; this is the integration point of the whole course.

## The seven states

```
IDLE ──landing requested──▶ TRANSITION ──ekf locked──▶ ACTIVE_LANDING ──▶ (flare/touchdown)
 ▲                              │                          │  ▲
 │ not landing                  │                          │  │ reacquired
 │                              ▼                          ▼  │
NAVIGATE                    (still acquiring)          COASTING (marker lost < 2 s)
                                                           │
                                       lost ≥ 2 s & 2.5–30 m │
                                                           ▼
                                                     GIMBAL_SEARCH ──reacquire──▶ ACTIVE_LANDING
                                                           │ timeout / unsafe AGL / cam fault
                                                           ▼
                                                         ABORT  (climb away, hand to pilot)
```

State definitions (`State` enum, fsm.py lines 29–37):

- **IDLE** — on the ground / not commanded to land.
- **NAVIGATE** — airborne, not yet in landing mode.
- **TRANSITION** — landing requested, acquiring the marker / waiting for EKF lock.
- **ACTIVE_LANDING** — marker locked, running the landing law and descending.
- **COASTING** — marker briefly lost (< `MARKER_LOSS_COAST_S = 2 s`); keep flying on
  the EKF's constant-velocity prediction (principle 4: bridge the gap, don't panic).
- **GIMBAL_SEARCH** — marker lost longer; sweep the gimbal in yaw to reacquire, but
  **only** at safe altitude (`2.5–30 m`); hold position, don't descend.
- **ABORT** — unrecoverable (loss at unsafe AGL, search timeout, camera fault): climb
  and return control to the pilot.

## The per-frame pipeline: `step()`

`step(frame, agl, rc, attitude, camera_ok)` (fsm.py lines 66–102) is the **whole
system in one method**. Trace it — this is the payoff of Modules 02–06:

1. **Advance time, read ego-motion** — `drone_vel_body = mav.read_velocity_body()`
   (Module 06), then **`ekf.predict(dt, drone_vel_body)`** (Module 04, principle 3).
   Predict runs *every* frame, even when the marker is invisible — that's what makes
   COASTING possible.
2. **Perceive** — if the camera is OK, `detector.detect(frame, agl)` → a `Pose` in
   the **camera** frame (Module 03).
3. **Transform + update** — if the pose is valid, rotate it to body with the **live**
   gimbal angles `frames.camera_point_to_body(pose.t, gimbal.get_angles())` (Module
   02), then **`ekf.update(pos_body)`** — which may **gate it out** (Module 04,
   principle 6). Track "time since last accepted measurement" (`_since_seen`).
4. **Decide** — pick the next state from `rc` (landing requested?), `camera_ok`, and
   `agl`. The marker-loss policy lives in `_transition_landing` (lines 104–129).
5. **Act** — `_act()` (lines 131–149) invokes the controllers appropriate to the
   current state (Module 05).
6. **Log** — `_log_tick()` writes the full per-frame state to SQLite (Module 08),
   *off the hot path* (principle 7).

That ordering — **predict before you look, update only if you see, act on the
estimate** — is the canonical predictor-corrector control loop. You've now seen every
block; the FSM is where they're wired in sequence.

## The marker-loss policy (principles 3 & 4)

`_transition_landing` (lines 104–129) is the most important policy in the file. Once
in landing:

- **Seen this frame** (`_since_seen ≈ 0`) → `ACTIVE_LANDING`.
- **Lost < 2 s** → `COASTING`: keep commanding from the EKF's constant-velocity
  prediction. The companion bridges short 30 fps gaps; it does *not* hand off to a
  vehicle filter, because ArduCopter's EKF3 is already dead-reckoning the vehicle
  (principle 4).
- **Lost ≥ 2 s** and at safe altitude `2.5–30 m` → `GIMBAL_SEARCH`: sweep to
  reacquire. The **altitude gate** is a safety rule — never sweep near the ground,
  where a search maneuver is dangerous and there's no time to recover.
- **Lost ≥ 2 s** at *unsafe* AGL, **search timeout** (`SEARCH_TIMEOUT_S = 8 s`), or
  **camera fault** → `ABORT`.

On **reacquisition from search**, the FSM calls `ekf.reset()` *then* enters
`ACTIVE_LANDING` (lines 114–117) — so the fresh lock doesn't inherit a stale velocity
(the reason `reset()` exists, Module 04). Every one of these branches is a design
principle made executable; being able to narrate them is a good test of whether the
whole course has landed.

## `_act()` and the gimbal-hold decision

`_act()` (lines 131–149) ties Module 05 to the states:

- In `ACTIVE_LANDING`/`COASTING`/`TRANSITION` with a lock: step the pointing
  controller *(telemetry only)*, **hold the gimbal at nadir**, and send the landing
  velocity setpoint. (Re-read the gimbal-hold reasoning, Module 05 — this is where it
  lives in code.)
- In `GIMBAL_SEARCH`: sweep the gimbal yaw back and forth (`±` past 45°) and command
  **zero vehicle velocity** (hold position while searching).
- In `ABORT`: `landing.abort()` — climb away.

## `main.py`: the closed loop

`run_sim()` (main.py lines 25–69) is the smallest complete demonstration of
everything:

- It constructs the **real** detector + EKF + controllers, but **mock** mav + gimbal
  and a **synthetic** camera (the seam from Module 06).
- Each iteration: render the pad as the camera would see it through the *current
  gimbal attitude* (so the synthetic image and the camera→body transform share one
  convention — note `tilt_deg = gimbal_pitch + 90`, the same nadir anchor as Module
  02), step the FSM, integrate the mock gimbal, then **integrate the drone's world
  pose** from the velocity setpoint the controller produced.
- It stops at flare and reports the **touchdown residual** — the horizontal distance
  from the pad at touchdown, the headline accuracy number of the whole system.

Reading this loop after Modules 01–06 should feel like watching all the pieces click
together. If any line is still opaque, that line points at the module to revisit.

## External resources

- **Finite state machines for robot behavior / mission management:** the supervisory
  control idea is covered in any robotics-systems course; a practical drone-flavored
  treatment is in the **PX4/ArduPilot flight-mode** state logic (read about
  ArduCopter flight modes: https://ardupilot.org/copter/docs/flight-modes.html — LAND,
  LOITER, GUIDED are exactly the modes this FSM straddles).
- **Behavior coordination patterns** (states vs. behavior trees, for when you wonder
  "why not a behavior tree?"): https://www.behaviortree.dev/ and the comparison
  literature — useful context, not required.
- **Predictor-corrector loop framing:** Thrun, *Probabilistic Robotics* (the Bayes
  filter chapter) frames predict/update exactly as the FSM sequences them.
- **Beard & McLain**, *Small Unmanned Aircraft*, the "state machine / path manager"
  chapters — the closest textbook analog to what `fsm.py` does for a UAV mission.

## Exercises

1. **Draw it yourself.** Without looking at the diagram above, draw the 7-state graph
   with the *condition* on every edge, from `_transition_landing`. Then check.
2. **Force a search.** In `main.py`'s sim loop, blank the camera for ~2.5 s (pass
   `img=None`, or set `camera_ok=False` briefly) once the drone is between 2.5–30 m.
   Which states does it pass through? Add a `print(machine.state.value)` each step to
   watch. Does it reacquire or abort?
3. **Coast vs. search boundary.** Change `MARKER_LOSS_COAST_S` to 0.2 s and rerun the
   experiment above. How does the behavior change, and why?
4. **Narrate a frame.** Pick one `step()` call mid-landing and write the value of
   `pose.valid`, `accepted`, `_since_seen`, the chosen state, and the commanded
   setpoint — i.e., tell the story of one 33 ms tick end to end.
