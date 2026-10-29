# landing_sw/: the Pi side

Runs on the laptop (sim, tests) and on the Pi 5 (camera + ESP32 gimbal). Hardware-only imports
(`picamera2`, `serial`) happen inside the functions that need them, so everything else runs anywhere.

## Entry points

| Command | What |
|---|---|
| `python -m landing_sw.main --sim` | closed-loop landing sim, synthetic camera, mock gimbal/MAVLink |
| `python -m landing_sw.run_all_experiments` | Bab 4.2 experiments → `results/` |
| `python -m landing_sw.demo_vision --camera imx477` | **demo**: camera → marker → body frame with live gimbal angles (`main` without `--sim` runs this) |
| `python -m landing_sw.gimbal_console --watch` | talk to the ESP32 by hand: nadir / park / hold, telemetry, round trip |
| `python -m landing_sw.calibrate_camera --camera imx477` | checkerboard → `K`, `dist` for `config.py` |
| `pytest` (repo root) | frames, kinematics, C↔Python protocol, sim regression |

## Files added or changed for the October demo

| File | What |
|---|---|
| `gimbal_proto.py` | mirror of `gimbal_fw/components/proto/proto.h`: CRC, `encode`, byte-wise `Parser`, `decode_telem` |
| `gimbal_iface.py` | `ESP32Gimbal` (replaces the SimpleBGC stub) + `MockGimbal` |
| `camera.py` | picamera2 lo-res Y stream; profiles `ov9281` (project camera) and `imx477` (ArduCam HQ, testing) |
| `demo_vision.py` | the T18 demo loop |
| `gimbal_console.py` | T13/T14 bench console |
| `calibrate_camera.py` | T18 camera calibration |
| `config.py` | `ESP32_GIMBAL_*`, `IMX477_*` |
| `main.py` | hardware path → `demo_vision`; sim world-axis fix (below) |

### ESP32Gimbal
Same `set_angles(pitch, roll, yaw)` / `get_angles()` as before, so `fsm.py` didn't change. Degrees at
this interface, radians on the wire. Semantics (plan §2.2):
- `set_angles`: pitch/roll are **world-referenced** camera tilt (the ESP32 stabilises them), yaw is
  relative to the drone body. `nadir()` and `park()` are shortcuts.
- `get_angles`: the **measured** joint angles from TELEM, as `(pitch, roll, yaw)`. That is what
  `frames.camera_to_body_rotation` needs: the marker has to be rotated by where the camera really is.
  TELEM sends joints in firmware order `(yaw, roll, pitch)`; `test_gimbal_proto.py` pins the reorder.

A daemon thread reads TELEM and re-sends the current target at 25 Hz, which keeps the firmware's
200 ms watchdog fed. `latency_s` = time from a SET_TARGET to the first TELEM that acks its `seq`.

### camera.py and the IMX477
The main stream picks the sensor mode and field of view (IMX477: 2028×1520, the 2×2-binned full
sensor). The detector only sees the lo-res YUV420 stream, of which the first H rows are the luma
(Y) plane, so a colour sensor gives the same grayscale image a mono one does. `K` must be the one
**at detect_res**. For the IMX477 it's a 6 mm-lens estimate until `calibrate_camera.py` is run.
The IMX477 has a rolling shutter: fast hand motion skews the marker more than with the OV9281.

### The sim fix (T3c step 3)
After `frames.py` lost its mirror (T3c), the sim stopped landing. `synthetic_markers` renders
image-right = world x and image-down = −world y. At nadir, `frames.py` (correctly) says image-right =
body Right and image-down = body Back. So the sim's world is x = Right, y = Forward, but `main.py` was
integrating body Forward into x. That swap is the mirror, and it turns one lateral axis into positive feedback.
`main.py` now integrates `p[0] += vy`, `p[1] += vx`. `tests/test_main_sim.py` keeps it fixed.
