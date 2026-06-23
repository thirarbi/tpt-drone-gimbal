# Module 05 — Control: the Pointing & Landing Laws

> **Your strongest home turf.** This is the controls course, applied. Two
> controllers: the **landing controller** (commands the *vehicle's* velocity to null
> the marker error — a P controller plus feedforward) and the **pointing controller**
> (commands the *gimbal's* angles — a slew-rate-limited tracker). Neither is more
> than a page of code. The interesting content is *why* feedforward, the saturation
> limits, and the subtle decision to **not** slew the gimbal during landing.

## Learning objectives

- Read the landing control law `v_cmd = Kp·err + G_ff·v_target` and explain each
  term in feedback-control language (P term, feedforward term, lag cancellation).
- Explain the velocity saturation (`MAX_HORIZ_VEL_MS`) and the gated vertical
  descent logic.
- Explain the gimbal pointing controller and slew-rate limiting.
- Explain why, during `ACTIVE_LANDING`, the gimbal is *held nadir* rather than
  slewed to track — a stability argument that uses Module 02.

## Read, in this order

1. [`landing_sw/landing.py`](../landing_sw/landing.py) — the vehicle controller.
2. [`landing_sw/pointing.py`](../landing_sw/pointing.py) — the gimbal controller.
3. The landing-controller and pointing/gimbal blocks of `config.py` (the gains).
4. `fsm.py` `_act()` lines ~131–147 — to see *when* each controller is invoked.

## The landing control law (principle 2)

`LandingController.compute_velocity_setpoint` (landing.py lines 25–47):

```
vx = KP_HORIZ · px  +  FEEDFORWARD_GAIN · vtx
vy = KP_HORIZ · py  +  FEEDFORWARD_GAIN · vty
```

Two terms, and you have the tools to analyze both:

- **Proportional term** `KP_HORIZ · p`: a classic P controller driving the relative
  position error `p` to zero. `KP_HORIZ = 1.2` has units of 1/s — it sets the
  closed-loop time constant of the horizontal centering loop. Bigger = snappier but
  (with latency, principle 1) less phase margin. This is exactly the gain/bandwidth
  trade you studied.
- **Feedforward term** `FEEDFORWARD_GAIN · vt`: adds the *target's absolute velocity*
  (from the EKF, Module 04). Here's the analysis that motivates the whole project:

  For a pure-P controller chasing a target moving at constant `vt`, the steady state
  is where the commanded velocity equals the target velocity: `Kp · p_ss = vt`, i.e.
  a **standing position error** `p_ss = vt / Kp`. For `vt = 1.4 m/s` and `Kp = 1.2`
  that's ≈ **1.17 m** of permanent lag — the drone trails the pad by over a meter and
  never lands. (This is the classic "type-0 system has finite steady-state error to a
  ramp" result from Automatic Control, in disguise.) Adding feedforward `+1·vt`
  supplies that velocity *directly*, so the P term only has to correct residual error
  → steady error collapses to a few cm. `sitl_harness.py` measures exactly this:
  ~112 cm without FF vs ~5 cm with FF (Module 08).

**Saturation** (lines 33–36): the horizontal command magnitude is clipped to
`MAX_HORIZ_VEL_MS = 4 m/s`. Actuator saturation is a real nonlinearity — worth
connecting to anti-windup discussions from your control course (there's no integral
term here, so no windup, but the saturation still bounds aggressiveness).

**Vertical logic** (lines 38–46): descend at `DESCENT_VEL_MS` only when **centred**
(`hypot(px,py) < 0.25·AGL` — a cone that tightens as you near the ground) and above
the **flare** altitude; a slow creep while re-centring; **zero at/below flare**
(`FLARE_AGL_M = 0.28 m`, the cutoff where the autopilot finishes the touchdown).
This "only descend when lined up" gating is a simple, robust supervisory rule — the
controls equivalent of "don't commit until you're sure."

**Abort** (lines 55–57): command a pure climb (`-ABORT_CLIMB_RATE_MS` in FRD where
+z is down) and hand back to the pilot. Read this together with the FSM's ABORT
state (Module 07).

> Note where feedforward must come from. The docstring (landing.py lines 5–8) insists
> the feedforward use the EKF's velocity **directly** and *not* be recomputed from the
> autopilot velocity here — because the EKF already folded ego-motion in on one time
> base (principle 3, Module 04). Recomputing it here would reintroduce the two-time-base
> instability. This is the same idea as Module 04, viewed from the controller side.

## The pointing controller

`PointingController` (pointing.py) keeps the camera **bore** on the marker by
commanding gimbal `(pitch, roll, yaw)`:

- `desired_angles_from_marker` (lines 24–34): pure geometry. Given the marker in
  body coordinates `[forward, right, down]`, the pitch to look at it is
  `−atan2(down, horizontal_range)` (straight down → −90°) and the yaw is
  `atan2(right, forward)`. This is just spherical pointing — no dynamics.
- `step` (lines 36–44): moves the *current* command toward the desired angles under
  a **slew-rate limit** `GIMBAL_SLEW_DEG_S = 60°/s` (the `_slew` helper clips the
  per-step change). Rate limiting keeps commands mechanically achievable — a
  saturation on the *derivative* rather than the value. Pitch is additionally clamped
  to `[−90°, 0°]`.

This is a **rate-limited tracking controller**, conceptually a first-order pursuit.
The mock gimbal (`gimbal_iface.py`, Module 06) adds first-order lag so the simulated
gimbal responds plausibly.

## The subtle decision: don't slew during landing

Read `fsm.py` `_act` (lines 131–147) and its comment carefully. During
`ACTIVE_LANDING`/`COASTING`:

- the gimbal is **held at nadir** (`set_angles(GIMBAL_PITCH_LAND_DEG=−90, 0, 0)`),
- lateral marker error is nulled by commanding the **vehicle** velocity,
- the pointing controller is still *stepped*, but only **for telemetry** — its output
  doesn't move the bore.

Why? Because (Module 02) the camera→body transform depends on the *live* gimbal
angles. If the gimbal slewed mid-landing, the camera→body axes would rotate every
frame, changing the measurement frame underneath the controller — coupling the
estimator and the gimbal loop and risking instability. Holding the gimbal fixed
keeps the measurement frame constant so the vehicle velocity loop is clean. Gimbal
*tracking* is therefore reserved for `GIMBAL_SEARCH` (reacquiring a lost marker),
where the vehicle is holding position and a moving bore is safe.

This is a lovely example of the kind of cross-coupling reasoning Multivariable
Control trains you for: two loops that are each fine alone can destabilize when
coupled, so the design *decouples* them by freezing one.

## External resources

- **Feedforward + feedback tracking, the rigorous version:** Åström & Murray,
  *Feedback Systems* (free PDF: https://fbswiki.org/) — chapters on PID/feedforward
  and on the two-degree-of-freedom controller structure. The "feedforward cancels the
  lag to a reference" story is formalized there.
- **Steady-state error to a ramp / system type** (the `vt/Kp` lag result): any
  classical-control text; e.g. Ogata, *Modern Control Engineering*, the steady-state
  error chapter — directly explains the 1.17 m number.
- **Guidance & velocity control of multirotors** in the UAV context: Beard & McLain,
  *Small Unmanned Aircraft*, the guidance/autopilot chapters; and ArduPilot's
  guided-mode velocity control docs:
  https://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html
- **Gimbal control background** (for the pointing loop): SimpleBGC / BaseCam manuals,
  and any "camera gimbal stabilization" tutorial; the control idea is a rate-limited
  servo loop.

## Exercises

1. **Quantify the lag.** Compute `vt/Kp` for `vt = 1.4 m/s` and the default
   `KP_HORIZ`. Then run `python -m landing_sw.sitl_harness` and compare the *FF-off*
   steady error to your hand calculation. Do they match? (They should be close.)
2. **Gain sweep.** Set `FEEDFORWARD_GAIN = 0.5` (half-cancellation) in `config.py`,
   rerun the harness, and predict the steady error *before* you look. Revert.
3. **Saturation.** Start the sim far off-pad (`start_xyz=(10, 0, 6)` in `main.py`).
   Does the horizontal command saturate at `MAX_HORIZ_VEL_MS`? Add a print of the
   command magnitude in `compute_velocity_setpoint` to check, then remove it.
4. **Decoupling argument.** In one paragraph, explain to an examiner why slewing the
   gimbal to track the marker *during descent* could destabilize the landing loop,
   referencing the camera→body transform from Module 02.
