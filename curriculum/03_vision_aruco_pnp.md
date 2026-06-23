# Module 03 — Computer Vision: Cameras, ArUco, and PnP

> The biggest "new" block of the course. You don't need to become a vision
> researcher — you need to understand the *pinhole camera model*, what an *ArUco
> marker* gives you, and how *PnP* turns 2D corners into a 3D pose. Everything in
> this codebase's perception front-end is those three ideas.

## Learning objectives

- Write down the pinhole projection equation and identify the camera intrinsic
  matrix `K` and distortion coefficients in `config.py`.
- Explain what an ArUco marker is and what `detectMarkers` returns.
- Explain the **PnP (Perspective-n-Point)** problem and why the planar
  `SOLVEPNP_IPPE_SQUARE` solver is the right tool for a square marker.
- Explain the **nested-marker handoff** (principle 5) and the **reprojection-error
  gate** + **pose-flip ambiguity** (principle 6).
- Read `synthetic_markers.py` and explain how a fake-but-honest camera image is
  rendered for testing.

## Read, in this order

1. [`landing_sw/markers.py`](../landing_sw/markers.py) — the real detector.
2. [`landing_sw/synthetic_markers.py`](../landing_sw/synthetic_markers.py) — the
   simulated camera (read `camera_extrinsic`, `project`, `render_view`).
3. Re-read `config.py` camera + nested-ArUco blocks with this module's vocabulary.

## Background: the pinhole camera model

A camera maps a 3D point `X_cam = [X, Y, Z]` (in the camera frame, Z forward) to a
pixel `[u, v]`:

```
[u]         [X]          [fx  0  cx]
[v] ~  K ·  [Y]  ,  K =  [ 0 fy  cy]
[1]         [Z]          [ 0  0   1]
```

`~` means "up to scale": you divide by Z. That division is *the* nonlinearity of
vision — depth and image position are entangled, which is exactly why recovering 3D
from a single image needs extra structure (here, a marker of *known physical size*).

- `fx, fy` — focal lengths in pixels. `cx, cy` — the principal point (≈ image
  center). In this repo: `CAM_K = [[560,0,320],[0,560,200],[0,0,1]]` for the
  640×400 detection stream. See `config.cam_matrix()`.
- **Distortion** `CAM_DIST = [k1,k2,p1,p2,k3]` — lens radial/tangential distortion.
  Here zeros (`# MEASURE`: must be filled from a real calibration). The synthetic
  camera renders an ideal pinhole, so zero distortion is self-consistent in sim.

`project()` in `synthetic_markers.py` (lines 78–88) is literally this equation:
transform world→camera with `(R_wc, t_wc)`, drop points behind the camera, multiply
by `K`, divide by z. Reading those 10 lines is the fastest way to *feel* the model.

> **Calibration** is how you get `K` and the distortion from a real camera (shoot a
> checkerboard from many angles, solve). The `# MEASURE` tag on `CAM_K` is a note
> that this must be done before flying. See resources.

## Background: ArUco markers

An **ArUco marker** is a square black-and-white 2D barcode — a thick black border
around an `n×n` grid of bits encoding an integer ID, drawn from a **dictionary**
(here `DICT_5X5_100`: 5×5 bit markers, 100 of them). They're popular for robotics
because detection is fast, robust, and gives you **four corner pixels with known
correspondence** (you know which corner is which), plus an ID.

`cv2.aruco.ArucoDetector.detectMarkers(gray)` returns, per marker: the 4 corner
pixel coordinates and the decoded ID. That's what `MarkerDetector._detect_raw`
wraps (markers.py lines 34–40). Corner refinement to **subpixel** accuracy
(`CORNER_REFINE_SUBPIX`, line 30) matters because pose accuracy scales with corner
accuracy.

### Why *nested* markers (principle 5)

A single marker can't cover the whole descent. `config` defines:

- **Outer**: ID 23, side **0.78 m** — big, seen from altitude (up to ~30 m).
- **Inner**: ID 7, side **0.09 m** — small, concentric in the center, used for the
  final approach once the outer marker overflows the camera's field of view.
- `HANDOFF_AGL_M = 0.90` — below this altitude, trust the inner marker.

`MarkerDetector.detect` (markers.py lines 47–75) implements the selection: use
inner if it's visible *and* (we're low *or* the outer isn't visible), else outer,
else inner, else "no pose." Read those ~10 lines as a decision table.
`synthetic_markers._pad_board` renders the physically nested pad (a small marker on
a white "quiet ring" in the center of a big one) so the simulated images exercise
the same handoff.

## Background: PnP — pixels to pose

**Perspective-n-Point**: given `n` 3D points in an object's own frame, their
corresponding 2D pixel projections, and the camera intrinsics `K`, recover the
object's pose `(R, t)` relative to the camera. For a square marker we have `n = 4`
points (the corners), and we know their object coordinates exactly: a square of
known side, centered at the origin, lying in its own z=0 plane
(`markers._square_obj`, lines 19–21).

The code uses `cv2.solvePnP(..., flags=cv2.SOLVEPNP_IPPE_SQUARE)`. **IPPE** =
*Infinitesimal Plane-based Pose Estimation* — an analytic solver specialized for
*planar* targets (all points coplanar), which is exactly a flat marker. It's faster
and better-conditioned than the general iterative solver for this case.

### The planar pose-flip ambiguity (principle 6)

Here's the subtlety that the whole `exp_pose_flip.py` experiment exists to study: a
**planar** marker viewed near **head-on (nadir)** has *two* PnP solutions that are
near-mirror-images — the marker could be tilted "toward" or "away" and project to
nearly the same pixels. This is the **pose-flip ambiguity**. `SOLVEPNP_IPPE_SQUARE`
even returns *both* (see `detect_with_ambiguity`, lines 77–92, via
`solvePnPGeneric`).

Two defenses, both in the code:

1. **Detector-side reprojection-error gate** (markers.py lines 42–45, 71–73):
   take the estimated pose, *re-project* the known 3D corners back to pixels with
   `cv2.projectPoints`, and measure the RMS pixel error vs. the detected corners. A
   bad/ambiguous solution reprojects worse. Reject if error > `REPROJ_ERR_MAX_PX`
   (3 px). This is the detector's branch-selection signal.
2. **EKF-side chi-square innovation gate** — Module 04.

A *key finding* the experiment surfaces (read the `exp_pose_flip.py` docstring,
even now): for a square marker the flip is a **purely rotational** degeneracy — both
branches return nearly the *same translation* `tvec`. Since landing only cares about
*position*, the flip barely moves the landing point (~0.1 cm). So the EKF position
gate is *insensitive to it by construction*, and that's fine — the reprojection gate
handles the orientation question. This is a genuinely nice piece of analysis and a
good model for how to reason about whether a "scary" failure mode actually matters
for *your* objective.

## How `synthetic_markers.py` keeps the test honest

The clever bit (read the module docstring): `render_view` produces a **real grayscale
image** by perspective-warping the rendered pad to where the camera would see it,
*and* returns the **ground-truth pose**. So:

- the **genuine** `cv2.aruco` + PnP pipeline runs on a real image (not a shortcut),
- but you can *score* its output against truth.

Swapping `render_view()` for a real camera frame is a drop-in change — every
consumer just takes an `HxW uint8` image. That's principle-7 hygiene (the detection
stream is a plain mono image) and good test design: test the real algorithm,
against known truth, with the seam to real hardware kept clean.

## External resources

- **OpenCV ArUco tutorial** (detection, dictionaries, pose): 
  https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html and the
  "Detection of ArUco Markers" + "Pose estimation" pages.
- **OpenCV camera calibration tutorial** (how `K` and distortion are obtained):
  https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html
- **The original ArUco paper** (Garrido-Jurado et al., 2014, *Pattern Recognition*)
  — "Automatic generation and detection of highly reliable fiducial markers." Good
  for understanding why these markers are robust.
- **IPPE, the planar PnP solver this code uses** — Collins & Bartoli, *Infinitesimal
  Plane-Based Pose Estimation*, IJCV 2014: https://link.springer.com/article/10.1007/s11263-014-0725-5
  (the ambiguity discussion is directly relevant to principle 6). Reference code:
  https://github.com/tobycollins/IPPE
- **Textbook, the standard reference for projective geometry / PnP / the camera
  model:** Hartley & Zisserman, *Multiple View Geometry in Computer Vision*
  (Cambridge). Read the camera-model chapter; it's dense but authoritative.
- **Gentler textbook, free PDF:** Szeliski, *Computer Vision: Algorithms and
  Applications* (2nd ed.) — https://szeliski.org/Book/ — chapters on image
  formation and feature-based alignment / pose.
- **AprilTag**, the main alternative fiducial system (for Exercise 3 / the Module 01
  exercise): https://github.com/AprilRobotics/apriltag

## Exercises

1. From `CAM_K`, the outer marker side 0.78 m, and the pinhole equation, estimate
   the marker's apparent side **in pixels** at AGL = 30 m (nadir). Then check it
   against the detection-rate result (Module 08): the suite *fails* the high-AGL
   band partly because the marker shrinks to ~15 px. Do your numbers agree?
2. Run `python -m landing_sw.exp_detection_rate`. Look at where the *inner→outer
   source crossover* happens vs. the `HANDOFF_AGL_M` policy value. Are the policy
   handoff and the *observed* crossover the same? Why might they differ?
3. In a REPL: `from landing_sw import synthetic_markers as sm; img, gt = sm.render_view(5.0, offset_xy=(0.3,-0.2))`.
   Run `from landing_sw.markers import MarkerDetector; MarkerDetector().detect(img, 5.0)`
   and compare the returned `t` (camera-frame translation) against `gt.t`. How close
   is the genuine pipeline to ground truth?
