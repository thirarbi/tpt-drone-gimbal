# tpt-drone-gimbal

Vision precision landing (TF3001): a Raspberry Pi 5 finds a nested ArUco pad with a camera on a
3-axis gimbal; an ESP32 keeps that camera pointed at the ground with XL-320 servos and an MPU6050.

## How to start
```bash
. ~/.espressif/v6.1/esp-idf/export.sh
```
or use `scripts/fw.sh <idf.py args>`, which also picks the right Python env.

## Repo map

| Path | What | Read first |
|---|---|---|
| `gimbal_fw/` | ESP32 firmware (C, ESP-IDF 6.1) | [gimbal_fw/README.md](gimbal_fw/README.md) |
| `gimbal_fw/main/` | tasks + boot sequence | [gimbal_fw/main/README.md](gimbal_fw/main/README.md) |
| `gimbal_fw/components/` | maths, servo bus, Pi protocol, IMU, calibration | [gimbal_fw/components/README.md](gimbal_fw/components/README.md) |
| `landing_sw/` | Pi software (Python): vision, EKF, FSM, sim, gimbal link | [landing_sw/README.md](landing_sw/README.md) |
| `scripts/` | `fw.sh` (idf.py wrapper), `demo_sim.sh` (laptop demo), `demo_pi.sh` (Pi demo) | |
| `TESTING.md` | step-by-step hardware bring-up and demo run order | **before touching hardware** |
| `october_demo_plan.md` | the task plan; each task carries its status | |
| `curriculum/`, `*.md` at root | study notes, derivations, decisions | |

## Three commands

```bash
scripts/demo_sim.sh                                   # laptop: host tests + pytest + sim landing
scripts/fw.sh -p /dev/ttyUSB0 flash monitor           # ESP32: build, flash, watch the status line
scripts/demo_pi.sh imx477                             # Pi: camera → marker → body frame, gimbal at nadir
```

## Tests

- `make -C gimbal_fw/host_test`: every PURE C module (kinematics vs Python golden vectors, Mahony
  vs Python, Dynamixel packets, `stab_step` in a closed-loop sim, Pi protocol). Also writes the golden
  frame the Python side decodes.
- `pytest` (repo root): frames, kinematics, C↔Python protocol agreement, `ESP32Gimbal` conversions,
  sim-landing regression.
- Hardware: [TESTING.md](TESTING.md).
