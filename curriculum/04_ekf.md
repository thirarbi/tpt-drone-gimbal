# Module 04 — The Relative-Marker EKF

> **Your home turf.** You've seen the Kalman filter in Multivariable Control. This
> module reuses that knowledge and adds three things you may not have done before:
> a *concrete* `predict`/`update` implementation, a **control input in the predict
> step** (this is the heart of principle 3), and **innovation gating** (chi-square
> outlier rejection). The "EKF" here is barely nonlinear — the measurement is
> linear — so don't be intimidated by the "E".

## Learning objectives

- Map the code in `ekf.py` onto the standard KF equations you learned.
- Explain the **6-state** choice: relative position + *target absolute velocity*.
- Explain principle 3 precisely: why ego-motion is folded into `predict` as a
  control input, and why the "obvious" alternative is unstable.
- Explain the **process noise** model `_Q(dt)` (white-acceleration / constant-velocity).
- Explain the **chi-square innovation gate** and how it implements half of the
  pose-flip mitigation (principle 6).

## Read, in this order

1. [`landing_sw/ekf.py`](../landing_sw/ekf.py) — the whole file, slowly, with the
   KF equations beside you.
2. The EKF block of `config.py` (the noise std-devs and the gate threshold).
3. How the FSM drives it: `fsm.py` `step()` lines ~71–86 (`predict` then `update`).

## The standard KF, refreshed

For a linear system `x_{k+1} = F x_k + B u_k + w`, `z_k = H x_k + v`, with
`w ~ N(0,Q)`, `v ~ N(0,R)`:

**Predict**
```
x ← F x  (+ B u)
P ← F P Fᵀ + Q
```
**Update** (given measurement `z`)
```
y = z − H x              # innovation (measurement residual)
S = H P Hᵀ + R           # innovation covariance
K = P Hᵀ S⁻¹             # Kalman gain
x ← x + K y
P ← (I − K H) P
```

Now read `ekf.py` and you'll find *exactly* these lines (predict at 60–69, update
at 95–103). It is a textbook linear KF. Confirm the mapping yourself — it's the
single best way to lock in both the theory and this code.

## The state vector (principle 3)

```
x = [px, py, pz,  vtx, vty, vtz]      (body FRD frame)
     └─ relative marker position ─┘    └─ target ABSOLUTE velocity ─┘
```

- `[px,py,pz]` — where the marker is *relative to the drone*, in body coordinates.
  This is what the controller drives to zero.
- `[vtx,vty,vtz]` — the **target's absolute velocity** (e.g. a moving pad), *not*
  the relative velocity. This distinction is the crux of the thesis and feeds the
  feedforward term in Module 05.

`H = [I₃ | 0₃]` (line 40): the camera measures **position only**; velocity is never
measured directly — it's *inferred* by the filter from a sequence of positions. (You
saw this idea in class: the KF estimates unmeasured states. Here, velocity is the
unmeasured state, observable through the position dynamics.)

### The constant-velocity model and `_Q`

`F` (lines 63–65) is a constant-velocity propagation: `p ← p + v·dt`, `v ← v`. The
filter *assumes the target moves at roughly constant velocity* between updates, and
`predict` extrapolates the marker position forward during the gaps between 30 fps
camera frames (principle 4 — the companion "bridges 30 fps gaps").

`_Q(dt)` (lines 71–83) is the **piecewise-constant white-acceleration** process-noise
model: we admit the target *can* accelerate, modeled as white noise on acceleration
with intensity `q = EKF_PROC_ACCEL_STD²` (1.5 m/s²)². The resulting block structure
(`dt⁴/4` on position, `dt³/2` cross, `dt²` on velocity) is the standard discretized
continuous-white-noise-acceleration `Q`. This is a model you may not have derived in
class but is *the* canonical tracking-filter process model — see the Labbe and
Bar-Shalom references.

## Principle 3 in detail: ego-motion as a control input

This is the idea to truly understand. Look at `predict` (lines 60–69):

```python
self.x = F @ self.x
self.x[:3] -= np.asarray(drone_vel_body) * dt   # fold ego-motion in
self.P = F @ self.P @ F.T + self._Q(dt)
```

The relative position changes for two reasons: the target moves, *and the drone
moves*. The drone's own velocity (`drone_vel_body`, read from ArduCopter's EKF3 over
MAVLink) is a **known input** `u`, so it belongs in the predict step — the
`x[:3] -= drone_vel_body·dt` term is the `+B u` of the standard equations, with
`B` picking out position and the sign saying "if the drone moves +forward, the
marker's relative position moves −forward."

Because ego-motion is removed in `predict`, the velocity state `[vtx,vty,vtz]`
converges to the target's **absolute** velocity (relative velocity + drone velocity,
but computed *consistently* inside one time-update). Read the docstring's warning
(ekf.py lines 12–16): the tempting WRONG alternative — keep a *relative*-velocity
state and add the autopilot velocity back *afterwards* (`v_target = v_rel + v_drone`)
— puts the **lagged** perception estimate and the **fresh** autopilot velocity on
different time bases and injects positive feedback that destabilizes the loop. The
control-theory translation: you'd be summing two signals with a relative time delay
inside a feedback path → phase lag → instability. The single-time-base predict
avoids it. `sitl_harness.py` is the experiment that demonstrates the stable version
works (Module 08).

## Initialization, reset, and the chi-square gate

- **Lazy init** (`update`, lines 89–94): the first measurement *sets* the position
  state to the measurement and zeroes velocity, then flips `initialized = True`. Before
  that, `predict` is a no-op (lines 61–62). Clean way to avoid needing a prior.
- **`reset()`** (lines 50–57): the FSM calls this on *reacquisition* after losing the
  marker, so a fresh lock doesn't inherit a **stale velocity** estimate. (Why this
  matters: an old velocity, fed to feedforward, would command a phantom lunge.)
- **Chi-square innovation gate** (lines 99–100, principle 6): compute the squared
  Mahalanobis distance of the innovation, `d² = yᵀ S⁻¹ y`, and **reject** the
  measurement if `d² > EKF_GATE_CHI2` (16.27 ≈ 99.9% of χ² with 3 dof). This rejects
  outliers — a wildly wrong PnP solution, a misdetection — *before* they corrupt the
  state. `mahalanobis()` (lines 106–111) exposes the same distance *without* updating,
  which `exp_pose_flip.py` uses to test whether the gate would reject a flipped pose.

The χ² threshold is a probabilistic decision rule: under the assumption that the
filter is consistent, the normalized innovation squared follows a χ² distribution
with `dim(z)=3` degrees of freedom, so a fixed quantile gives a known false-reject
rate. This connects your statistics to your estimation course; it's worth being able
to explain the "16.27 = χ²₃ at 99.9%" number to an examiner.

## Why it's called an EKF if everything looks linear

Fair question. The measurement and propagation here are linear, so this is
effectively a *linear* KF. The "EKF" naming is because the surrounding system (the
projection from pose, the frame rotations) is nonlinear, and the architecture is
written to allow a nonlinear `H` later; but as implemented, `H` is constant. Don't
let the name make you look for Jacobians that aren't here.

## External resources

- **The single best practical Kalman resource, free, in Python** — Roger Labbe,
  *Kalman and Bayesian Filters in Python*:
  https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python
  Read: the KF chapter, the **"Kalman Filter Math"** chapter (derives the
  white-noise `Q` you see in `_Q`), and the **"Adaptive Filtering / gating"**
  material for the chi-square gate. This maps 1:1 onto `ekf.py`.
- **The tracking-filter bible** (constant-velocity models, process noise, gating):
  Bar-Shalom, Li & Kirubarajan, *Estimation with Applications to Tracking and
  Navigation*. Chapter 6 (state estimation) and the gating/association sections.
- **Probabilistic Robotics** — Thrun, Burgard, Fox — the EKF chapter, for the
  robotics framing and the "control input in predict" view (`u` in `predict` is
  central to their formulation).
- **Refresher / lecture notes** mapping LQR-era linear-systems language to the KF:
  Stephen Boyd's EE263 notes (Stanford) on the Kalman filter, if you want the
  controls-flavored derivation you saw in Multivariable Control.

## Exercises

1. **Prove the mapping.** Annotate every line of `predict` and `update` with the
   corresponding symbol from the standard KF equations above. Identify `F`, `H`,
   `Q`, `R`, `S`, `K` in the code.
2. **Velocity is inferred, not measured.** In a REPL, build a `RelativeMarkerEKF`,
   feed it a sequence of positions consistent with a target moving at `[1.4, 0, 0]`
   m/s (call `predict(dt)` then `update(pos)` each step, with `drone_vel_body=0`),
   and watch `kf.velocity` converge. How many updates until it's within 10%?
3. **The gate at work.** After establishing a lock, call `kf.mahalanobis(z)` for a
   `z` that's 1 m off in x. Is `d²` above `EKF_GATE_CHI2`? Now make it 5 cm off —
   accepted? This is the outlier rejection that protects the landing.
4. **Conceptual:** in two sentences, explain to a classmate why `reset()` exists and
   what specifically goes wrong on reacquisition without it.
