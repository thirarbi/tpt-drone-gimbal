# Module 02 — Coordinate Frames & Rotations

> This is likely the most unfamiliar *small* topic in the course, and the one that
> trips up almost everyone new to robotics. Spend real time here; everything
> downstream (the EKF measurement, the control law signs, the gimbal pointing)
> depends on getting frames right.

## Learning objectives

- Define the three frames this system uses — **camera optical**, **body FRD**,
  **NED** — and the gimbal angle convention.
- Read a rotation matrix and know which way it converts (`v_body = R · v_camera`).
- Explain *why* the camera→body transform must use the **live** gimbal angles, not
  a fixed mounting assumption.
- Trace a marker measurement from "pixels" to "a vector in the drone body frame."

## Read, in this order

1. [`landing_sw/frames.py`](../landing_sw/frames.py) — the whole file; it's 70 lines.
2. Re-read the comment block at lines 37–43 (`_NADIR_CAM_TO_BODY`) slowly.
3. Peek at how it's used: `fsm.py` line ~81 (`frames.camera_point_to_body`) and
   `mavlink_iface.py` `read_velocity_body` (`frames.ned_to_body`).

## Background you may be missing

### Rotation matrices as change-of-basis

You met rotation matrices in mechanics/linear algebra. The one fact that matters
here: a 3×3 rotation `R` is **orthonormal** (`Rᵀ = R⁻¹`, `det R = +1`), and the
*same* matrix can be read two ways depending on intent:

- **Rotating a vector** within one frame, or
- **Re-expressing the same physical vector in a different frame** (change of basis).

This codebase uses them as **change of basis**: `camera_to_body_rotation(g)`
returns `R` such that `v_body = R · v_camera` — the *same arrow in space*, written
in body-frame components instead of camera-frame components. Note the naming
convention `R_wc` in `synthetic_markers.py`: "world from camera," i.e. it maps
camera-frame coordinates to world-frame. Read subscripts as `R_(to)(from)`.

The elementary building blocks are the principal-axis rotations `_Rx, _Ry, _Rz`
(frames.py lines 22–34) — memorize the shape of one and you can reconstruct the
others by cycling indices.

### The three frames (and the gimbal)

| Frame | Axes | Used by |
|-------|------|---------|
| **Camera optical** (OpenCV convention) | x **right**, y **down**, z **forward** (out the lens / "bore") | PnP output in `markers.py` |
| **Body FRD** | x **Forward**, y **Right**, z **Down** | the EKF, the control law |
| **NED** (navigation) | x **North**, y **East**, z **Down** | ArduCopter's velocity reports |

> ⚠️ Two different "x". In the camera frame x is *image-right*; in the body frame x
> is *forward*. They are not the same axis. Mixing them up is the classic frames
> bug, and the whole point of `frames.py` is to convert between them correctly.

**Gimbal convention** (frames.py lines 14, 37–55): pitch `0°` = bore points
forward (at the horizon); pitch `-90°` = bore points straight **down** (nadir).
During landing the gimbal is locked at nadir.

### The "nadir anchor" trick

Rather than derive the camera→body rotation from scratch for an arbitrary gimbal
attitude (error-prone), the code does something clever and worth understanding
(lines 37–55):

1. It hand-verifies *one* configuration — the camera at **nadir** — giving the
   constant map `_NADIR_CAM_TO_BODY = diag([1, -1, 1])`. Read the comment: at
   nadir, body-Forward = +x_cam (image right), body-Right = −y_cam (−image down),
   body-Down = +z_cam (the bore). Convince yourself of each of those three by
   picturing a downward camera over a pad.
2. For any other gimbal attitude, it *lifts* away from nadir by `(pitch + 90°)`
   about body-Right, then applies gimbal yaw about body-Down and roll about
   body-Forward:
   `R = Rz(yaw) · Rx(roll) · Ry(pitch+90°) · _NADIR_CAM_TO_BODY`.

This "anchor on a verified special case, then rotate" pattern is robust and a good
habit. The reason the *sign* of `_NADIR_CAM_TO_BODY` matters: it is what makes the
proportional control loop **stabilizing** rather than destabilizing. If you flipped
a sign here, the drone would fly *away* from the pad — a positive feedback loop.
(Connect to your Automatic Control intuition: a sign error turns negative feedback
into positive feedback.)

### Why *live* gimbal angles?

The camera hangs on a **moving** gimbal. If you assumed a fixed mounting and the
gimbal slewed 20°, every marker position would be mis-transformed by 20°. So
`camera_to_body_rotation` takes the gimbal's *currently streamed* `(pitch, roll,
yaw)`. In `fsm.py` the measurement transform reads `self.gimbal.get_angles()` every
single frame. This also explains a subtle decision you'll meet in Module 07: during
landing the gimbal is *held still at nadir* and lateral error is corrected by moving
the *vehicle*, precisely so the camera→body axes don't keep rotating mid-loop.

### body ↔ NED

`body_to_ned`/`ned_to_body` (lines 64–71) are a single `Rz(yaw)` — a rotation about
the shared Down axis by the vehicle heading. ArduCopter reports velocity in NED;
the EKF wants it in body, so `mavlink_iface.read_velocity_body()` calls
`ned_to_body(v_ned, yaw)`. (Note `ned_to_body` uses the *transpose*, since it's the
inverse rotation — orthonormality at work.)

## External resources

- **The definitive UAV frames reference, written for exactly your level:**
  Beard & McLain, *Small Unmanned Aircraft*, **Chapter 2 ("Coordinate Frames")**.
  This single chapter covers inertial/vehicle/body/stability/wind frames and the
  rotation conventions. If you read one external thing for this module, read this.
- **OpenCV camera coordinate convention** (x-right, y-down, z-forward) and how it
  relates to pose: https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html
  (the "Detailed Description" diagram). You'll revisit this in Module 03.
- **Rotation conventions, done carefully** (if you want the rigorous version,
  including quaternions, used by real autopilots): Joan Solà, *Quaternion kinematics
  for the error-state Kalman filter* — https://arxiv.org/abs/1711.02508 (sections
  on rotation matrices and frame conventions; skip the quaternion algebra on a
  first pass).
- **NED / aircraft axes background:** ArduPilot "Copter attitude/heading" + the
  general "ENU vs NED" discussion — https://ardupilot.org/copter/docs/common-coordinate-frames.html
- **Interactive intuition:** "Visualizing rotations" — 3Blue1Brown's linear algebra
  series, chapter on linear transformations and change of basis (YouTube), if the
  change-of-basis idea still feels slippery.

## Exercises

1. By hand (or in a Python REPL with `numpy`), evaluate
   `frames.camera_to_body_rotation((-90, 0, 0))` and check you get `diag([1,-1,1])`.
   Then evaluate it at `(0, 0, 0)` (bore forward) and describe in words where a
   marker straight ahead of the camera ends up in body coordinates.
2. A marker is detected at `t_cam = [0.2, 0.5, 4.0]` m (camera frame: a bit right,
   a bit down, 4 m out along the bore) with the gimbal at nadir. Compute
   `camera_point_to_body(t_cam, (-90, 0, 0))` and interpret the three body
   components physically (forward? right? down?). Does the sign of the "down"
   component make sense for a pad below the drone?
3. **Break it on purpose.** Temporarily change `_NADIR_CAM_TO_BODY` to
   `diag([1, 1, 1])` (remove the y-flip) and run `python -m landing_sw.main --sim`.
   What happens to the landing? Explain it in feedback-sign terms, then revert.
