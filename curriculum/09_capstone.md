# Module 09 — Capstone: Read the Loop End-to-End & Extend It

> No new files. This module is where understanding consolidates into mastery: you
> trace the entire system in one sitting, you can explain any design decision in
> terms of the 7 principles, and you make a non-trivial change and validate it.

## Learning objectives

- Trace one frame of `--sim` through *every* module without notes.
- Explain each of the 7 design principles, the failure it prevents, and the
  experiment that defends it.
- Make a meaningful modification and validate it with the experiment suite.
- Articulate the system's limitations and a credible improvement roadmap (thesis
  "future work").

## Part A — The grand trace (do this from memory)

Start a stopwatch. Without re-reading, narrate one 33 ms tick of
`python -m landing_sw.main --sim` while the drone is at ~5 m, locked on the pad,
descending. Name the module and the key line for each step:

1. Synthetic camera renders the pad through the current gimbal attitude →
   grayscale image. *(synthetic_markers.render_view — Module 03)*
2. `fsm.step()` reads ego-motion velocity from the (mock) autopilot.
   *(mavlink_iface.read_velocity_body — Module 06)*
3. EKF `predict(dt, drone_vel_body)` — constant-velocity propagation **with
   ego-motion folded in**. *(ekf.predict — Module 04, principle 3)*
4. Detector finds the ArUco corners, picks outer/inner by AGL, solves PnP, gates on
   reprojection error → `Pose` in the camera frame. *(markers.detect — Module 03,
   principles 5 & 6)*
5. Transform the pose to the body frame using **live** gimbal angles.
   *(frames.camera_point_to_body — Module 02)*
6. EKF `update()` — chi-square gate, then Kalman correction; velocity state tracks the
   target's absolute velocity. *(ekf.update — Module 04, principle 6)*
7. FSM decides the state is `ACTIVE_LANDING` (seen this frame). *(fsm.\_transition_landing
   — Module 07)*
8. `_act()`: hold gimbal at nadir; landing law computes
   `v = Kp·p + G_ff·v_target`, saturates, gates descent on centering.
   *(landing.compute_velocity_setpoint — Module 05, principle 2)*
9. Setpoint sent to the (mock) autopilot; the sim integrates the drone's world pose.
   *(mavlink_iface.send_velocity_body + main.run_sim — Modules 06, 07)*
10. Tick logged to SQLite, off the hot path. *(telemetry_db — Module 08, principle 7)*

If you stumble on a step, that's your signal for which module to revisit. Aim to do
the full trace cleanly twice.

## Part B — Principle defense (oral-exam rehearsal)

For each principle, state it, the failure it prevents, and the experiment that
validates it. Practice saying these out loud — this is precisely the kind of
question a thesis examiner asks.

| Principle | Failure prevented | Defended by |
|-----------|-------------------|-------------|
| 1 — latency is the benchmark | phase lag → unstable tracking | `latency_bench.py` |
| 2 — velocity feedforward | permanent `vt/Kp` lag behind a moving pad | `sitl_harness.py` |
| 3 — ego-motion inside `predict` | two-time-base positive feedback → instability | `sitl_harness.py` |
| 4 — companion bridges gaps, EKF3 dead-reckons | duplicating/ fighting the autopilot's filter | `fsm.py` COASTING logic |
| 5 — nested ArUco handoff | one marker can't span 30 m → 0.3 m | `exp_detection_rate.py` |
| 6 — two-layer pose-flip gate | wrong PnP branch → lateral lunge | `exp_pose_flip.py` |
| 7 — dual stream + off-path logging | logging blocks control; detect too slow | `telemetry_db.py`, latency |

## Part C — Make a change and validate it

Pick one. Each is sized to take an afternoon and forces real understanding. Use the
experiment suite as your validation harness, and keep `source` honesty in mind.

1. **Add a third marker tier.** Introduce a *mid-size* marker (e.g. 0.30 m) between
   the 0.78 m outer and 0.09 m inner, with a second handoff altitude. Update
   `config`, `synthetic_markers._pad_board`, and `markers.detect`'s selection logic.
   Re-run `exp_detection_rate` — does the mid-AGL band improve? This directly attacks
   the one genuine FAIL (Module 08).
2. **Integral action.** Add a small integral term to the horizontal landing law
   (watch for windup under `MAX_HORIZ_VEL_MS` saturation — use your control-course
   anti-windup knowledge). Does steady error in `sitl_harness` improve or does it
   destabilize? Explain in phase-margin terms.
3. **Tougher perception.** Add lens distortion (`CAM_DIST`) to the synthetic camera
   and a non-zero value to `config`, with and without giving the detector the matching
   coefficients. Quantify the pose error when calibration is wrong. (Motivates the
   `# MEASURE` tag on `CAM_K`.)
4. **A new risk experiment.** Wind/turbulence isn't tested. Add `exp_wind.py`: inject a
   lateral disturbance velocity into the `sitl_harness` plant and find the wind speed
   at which steady error exceeds tolerance. Wire it into `run_all_experiments`. This is
   genuine thesis contribution.
5. **Real SITL.** Stand up ArduPilot SITL (Module 06), point `MAVLINK_CONN` at it, and
   close the loop through the *real* `MavlinkInterface` instead of `MockMavlink`. This
   converts a `SYNTHETIC_MODEL` result toward `HARDWARE`.

Whatever you pick: state a hypothesis first, change the code, run the relevant
experiment, and write down whether the result matched your prediction. That loop —
predict, perturb, measure — is the whole point.

## Part D — Limitations & future work (write this up)

Draft half a page you could drop into a thesis "future work" section. Seed ideas,
grounded in what you've now read:

- **Perception envelope.** Detection fails at the band extremes (57% at 30 m). The
  reliable envelope is narrower than the design sweep → bigger/extra markers, or a
  constrained approach profile, or a learned detector.
- **All key numbers are `SYNTHETIC_MODEL`.** Latency, detection, current, tracking are
  modeled; the immediate next step is Pi 5 + real-camera + SITL/flight measurement to
  convert them to `HARDWARE`.
- **Calibration.** `CAM_K`/`CAM_DIST` are placeholders (`# MEASURE`); real pose
  accuracy depends on a proper calibration.
- **Single fiducial family.** ArUco only; AprilTag or IR beacons (ArduPilot IRLock)
  could complement it in poor lighting.
- **Wind/disturbance rejection** is untested (see Part C #4).
- **No formal stability proof** of the coupled vision-EKF-velocity loop with latency;
  a describing-function or Lyapunov argument would strengthen the thesis (and uses
  your Multivariable Control background).

## Where to go next (beyond this codebase)

- **Build the rigorous controls/estimation foundation:** Beard & McLain, *Small
  Unmanned Aircraft* (cover to cover now that you have a concrete system in mind) +
  Labbe's *Kalman and Bayesian Filters in Python* (work the notebooks).
- **Go deeper on vision-based flight:** UZH-RPG publications and their open datasets/
  code (https://rpg.ifi.uzh.ch/), and the **VINS-Mono / OpenVINS** projects for how
  real visual-inertial estimation scales up from this 6-state filter.
- **Live in the ecosystem:** run ArduPilot SITL, read the precision-landing source in
  `ardupilot/`, and compare its IRLock approach to this ArUco approach.
- **Contribute back:** the change you made in Part C, written up with before/after
  experiment numbers, is a real result. That's the bridge from "understanding the
  codebase" to "extending it."

## Final self-check

You understand this codebase when you can, without notes:

- [ ] Do the Part A grand trace, twice, cleanly.
- [ ] State all 7 principles, their failure modes, and their defending experiments.
- [ ] Explain principle 3 (ego-motion in `predict`) and *why the alternative is
      unstable* to a peer in under two minutes.
- [ ] Explain why the gimbal is held nadir during landing (Modules 02 + 05).
- [ ] Defend the detection-rate FAIL as an informative result, not a bug.
- [ ] Point to the exact hardware seams (`Protocol` + `Mock*`) that let it all run on
      a laptop.

When every box is checked, you don't just understand the code — you can defend the
thesis it implements.
