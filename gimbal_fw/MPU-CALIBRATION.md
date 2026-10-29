# MPU6050 calibration record

Every MPU6050 board has its own zero-g offsets and gyro bias, so the corrections belong to
**one physical board**, not to the code. This file explains how to re-measure them for a new
board and keeps a log of what was measured.

## What is per-board, per-mounting, per-boot

| Thing | Depends on | Where it lives |
|---|---|---|
| Accel offset (`acc_bias`) | the **chip** | `components/calib/mpu_unit.h` |
| Gyro bias | the **chip**, and its **temperature** | `mpu_unit.h` as fallback; measured at every boot in T9 |
| `R_imu_cam` | how the board is **mounted** on the camera, not the chip | `components/calib/calib.c` |

Swapping the board means editing `mpu_unit.h` only. Re-mounting it on the camera means
updating `R_imu_cam` only. Keep the two apart.

The values flow: `mpu_unit.h` -> `calib.c` (compile-time defaults) -> `calib_t` -> `ctrl_task.c`
subtracts them from each sample **before** `R_imu_cam` is applied.

## Why it matters

The attitude filter skips accel correction when `| |a| - 1 g | > 0.15 g`. An uncorrected
offset of 0.2 g makes a still, flat board read 0.8 g, so every sample is rejected and the filter
becomes a gyro-only integrator that drifts. An offset on a horizontal axis also tilts the
estimate by roughly `atan(offset / 1 g)` (0.2 g is about 12 degrees).

## Re-measuring the accelerometer offset (about 45 s)

1. Mount the board **rigidly**: tape it to a hardcover book or a small box so it can rest flat
   on each of six faces without a hand holding it. Hand tremor ruins the edge poses.
2. In `main/config_defaults.h` set `MPU_CALIB_SIXPOSE` to `1` (it is `0` by default, so a
   normal flash never starts calibration), then build and flash.
3. At boot the firmware announces `POSE 1/6 ... 6/6`. Move the board during each 6 s countdown,
   then leave it perfectly still. Poses: +Z up (flat), -Z up (upside down), then each of the
   chip's X and Y arrows pointing at the ceiling and at the floor.
4. Each pose prints `mean` and `spread`. Noise alone gives a spread of up to about 150-200
   counts. A spread of several hundred or more means it moved: reset (EN) and redo.
5. Read the three `AXIS` lines. Copy each `offset=` value (counts) into
   `MPU_UNIT_ACC_OFFSET_X_COUNTS`, `_Y_COUNTS` and `_Z_COUNTS` in `mpu_unit.h`.
6. Check `sens=` is within about 3 % of 8192 for all axes. If not, the chip's scale is off
   (possibly a clone): do not just subtract offsets, ask first.
7. Set `MPU_CALIB_SIXPOSE` back to `0`, rebuild, flash, and add a row to the log below.

Formulas used by the test, per axis, from the readings at +1 g and -1 g:

    offset = (reading_+1g + reading_-1g) / 2
    sens   = (reading_+1g - reading_-1g) / 2      (counts per g, nominal 8192)

## Gyro bias

Measure with the board warmed up (about 5 minutes, bias drifts with temperature) and perfectly
still for 60 s. The runtime boot calibration is planned for T9 (plan section 4.6 step 2). Until
it exists, record the mean of each gyro axis here by hand and paste it into
`MPU_UNIT_GYRO_BIAS_RADS`.

## Log

| Date | Board | Who | Accel offset X / Y / Z (counts) | Sens X / Y / Z (counts/g) | Gyro bias (rad/s) | Notes |
|---|---|---|---|---|---|---|
| 2026-10-30 | bench board #1 | Fathir | +196 / -115 / **-1699** | 8042 / 8153 / 8280 | not measured | Z offset is about -0.21 g, which is why a flat board read 0.82 g. X/Y provisional: poses 3-5 were hand-held and moved (spread up to 2600), Z pose 2 spread 417. Redo on a rigid fixture. |
