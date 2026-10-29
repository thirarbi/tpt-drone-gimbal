## To initiate
```bash
source .venv/bin/activate
source ~/.espressif/v6.1/esp-idf/export.sh
```

If there were any changes regarding CMake or whatever, do `deactivate` first, and then `source ~/.espressif/v6.1/esp-idf/export.sh` again

Or skip all of that: `scripts/fw.sh build` / `scripts/fw.sh -p /dev/ttyUSB0 flash monitor` from the repo
root does it with the Python env this build was configured with.

Host tests (no ESP32 needed): `make -C gimbal_fw/host_test`. Hardware bring-up: [../TESTING.md](../TESTING.md).

## What runs where

```
              core 1                                   core 0
 ┌─────────────────────────────────┐   q_cmd   ┌──────────────────────────────┐  UART2 1/2-duplex
 │ ctrl_task  1 kHz (1 ms tick*)    │ ───────►  │ servo_task 100 Hz            │ ─── 74HC126 ─── XL-320 x2..3
 │  MPU6050 read → bias → R_imu_cam │           │  SYNC WRITE goals            │
 │  → Mahony                        │  q_meas   │  READ Present Position x n   │
 │  every 10th tick: stab_step()    │ ◄───────  │                              │
 └─────────────────────────────────┘           └──────────────────────────────┘
        ▲ target            │ ctrl_out (attitude, q_cmd, flags)
        │                   ▼
 ┌─────────────────────────────────┐  UART1 115200   ┌──────────────┐
 │ comms_task  TELEM 50 Hz          │ ◄─────────────► │ Pi: landing_sw│
 │  SET_TARGET parser, 200 ms WD    │                 └──────────────┘
 └─────────────────────────────────┘
 app_main: boot sequence, then a 1 Hz status line on the USB console (921600)
 * Rev 4: MPU INT is not wired, so a 1 ms FreeRTOS tick clocks the loop (MPU_USE_INT 0)
```

Tasks never share variables: each arrow is a **1-slot mailbox** (`gimbal_state.h`): the writer calls
`xQueueOverwrite`, readers call `xQueuePeek`. A reader always gets the latest value, and nobody
blocks or needs a mutex. `servo_task` is the only code that touches UART2, so the bus needs no lock either.

### The control law in one paragraph (`components/gimbal_math/stab.c`, plan §4.2)

The IMU sits on the camera, so Mahony gives the **camera** attitude `R_wc`. The servos report the joint
angles `q_meas`, so `FK(q_meas)` is camera-relative-to-body. Then the **body** attitude is
`R_wb = R_wc · FK(q_meas)ᵀ`, with no sensor on the body. The target camera attitude is built with the
body's own heading (`Rz(yaw(R_wb)) · Ry(pitch_t) · Rx(roll_t)`). The MPU6050's yaw drifts, but it
drifts identically in `R_wc` and `R_wb`, so it cancels. IK of `R_wbᵀ · R_target` gives the joint goal.
`q_cmd` moves a fraction `K` of the way there each tick, limited to `ω_max·dt` and the joint limits.
With yaw absent (2 servos), roll + pitch still point the bore exactly at nadir: for a pure tilt the IK
yaw comes out 0.

## File layout
```
gimbal_fw/
├── main/                     ESP-IDF glue: tasks and boot (see main/README.md)
│   ├── app_main.c            boot sequence §4.6, 1 Hz status / CSV log
│   ├── ctrl_task.c           1 kHz IMU + Mahony, 100 Hz stab_step, boot gyro calibration
│   ├── servo_task.c          servo bring-up + 100 Hz bus loop, rad <-> tick
│   ├── comms_task.c          UART1 to the Pi, watchdog
│   ├── gimbal_state.c/.h     the mailboxes
│   ├── bench_tests.c         APP_* test modes used by TESTING.md
│   ├── board.h               pins (only place pins live)
│   └── config_defaults.h     every tunable + APP_MODE
├── components/               (see components/README.md)
│   ├── gimbal_math/          PURE C: rot, kinematics, mahony, stab   ← host-tested
│   ├── xl320/                dxl_packet (PURE C, host-tested) + xl320 UART driver
│   ├── proto/                Pi frame: CRC, encode, parser (PURE C) ← host-tested + golden file
│   ├── mpu6050/              I²C driver (new i2c_master.h API only)
│   └── calib/                calib_t defaults: servo IDs, dir, zero_tick, IMU offsets, R_imu_cam
├── host_test/                gcc tests of everything PURE C; `make` runs them all
└── tools/                    golden-vector generators, Mahony reference, serial_log.py
```

## APP_MODE (in `main/config_defaults.h`)

| Mode | What it does | When |
|---|---|---|
| `APP_GIMBAL` | full boot, stabilise, talk to the Pi | normal / demo |
| `APP_IMU_ONLY` | IMU + Mahony + status, servos untouched | IMU checks |
| `APP_PIN_TEST` | TX/DIR as GPIOs, read the DATA line | bus wiring, no servo power |
| `APP_BUS_SCAN` | ping IDs 0–252 at all four XL-320 bauds | find servos |
| `APP_SERVO_SWEEP` | ±20° per joint, then SYNC WRITE all | servos move, bus timings |
| `APP_JOINT_MONITOR` | torque off, live ticks/angles + camera attitude | zero_tick, dir, R_imu_cam |
| `APP_MPU_SIXPOSE` | accel offset measurement | new MPU board (MPU-CALIBRATION.md) |
| `APP_SERVO_SPEEDUP` | one-shot: 1 Mbit/s, return delay 0 | after the demo |

## Knobs you will actually touch

| Where | What |
|---|---|
| `calib.c` `servo_id` | which Dynamixel ID is yaw / roll / pitch; missing ID = joint held at 0 |
| `calib.c` `dir`, `zero_tick` | from `APP_JOINT_MONITOR` (TESTING.md step 6) |
| `calib.c` `R_imu_cam` | IMU mounting, see below |
| `config_defaults.h` `STAB_GAIN_K`, `STAB_OMEGA_MAX_RAD_S` | stiffness / max slew |
| `config_defaults.h` `STAB_MODE` | `STAB_MODE_PID` = per-axis fallback |
| `config_defaults.h` `SERVO_RATE_HZ` | 50 if 3 servos stay at 115200 |
| `config_defaults.h` `LOG_CSV_HZ` | 50 to log for `tools/serial_log.py` |

## R_imu_cam

`v_cam = R_imu_cam · v_imu`. The camera ("link") frame is defined **at joint zero**: x = bore
(forward), y = camera right, z = camera down. **Column k of `R_imu_cam` is chip axis k (the X, Y, Z
silkscreen arrows, Z out of the chip face) written in (bore, right, down)**, with the gimbal at joint zero.

Two common cases:

| Mounting | R_imu_cam |
|---|---|
| chip flat, face up, X arrow along the bore, at joint zero (bench default) | `{{1,0,0},{0,-1,0},{0,0,-1}}` |
| chip flat, face up, X arrow toward the drone's front **when the camera points down** | `{{0,0,-1},{0,-1,0},{-1,0,0}}` |

How the second one comes out: at nadir the bore points down, so chip Z (up) = −bore. Chip X (forward)
is the link's −z there, and Y = Z × X = left = −right. Columns X, Y, Z = (0,0,−1), (0,−1,0), (−1,0,0).
Always check: `det = +1`, and in `APP_JOINT_MONITOR` the camera pointing down reads pitch ≈ −90°.

## Block Diagram (whole system)
```
Pi 5 — landing_sw (Python)                    ESP32 — gimbal_fw (C, ESP-IDF)
─────────────────────────────                 ───────────────────────────────────────────────
camera.py (picamera2, OV9281 / IMX477)        1 ms tick (Rev 4; INT wire later) ─► ctrl_task
  → markers.py → frames.py → ekf.py            core 1: ctrl_task (1 kHz)
  → fsm.py → landing.py / pointing.py            imu read → mahony; every 10th tick:
                    │                            stab_step() → q_cmd mailbox ──┐
                    │                         core 0: servo_task (100 Hz) ◄────┘
                    │                            SYNC_WRITE goals, READ present pos ×n
                    │                            → q_meas mailbox (back to ctrl_task)
                    │                              │ UART2, half-duplex via 74HC126
                    │                              ▼
                    │                            XL-320 ×3 (yaw, roll, pitch) @ 7.4 V
gimbal_iface.ESP32Gimbal                      core 0: comms_task (UART1 parser, telemetry 50 Hz,
  SET_TARGET (25 Hz) ─────── UART 3.3 V ───►          watchdog)
  ◄──────────────────────── TELEM (50 Hz)
```

## Not done (needs hardware time or was cut)
- NVS persistence of `calib_t` (compile-time defaults are used, plan T10 allows it).
- Servo fault auto-recovery (bring-up reboots a faulted servo; at run time the error byte is only
  reported in TELEM). First on the plan's cut list.
- 1 Mbit/s bus (`APP_SERVO_SPEEDUP` exists; not needed for 2 servos at 115200, about 7.5 ms per cycle).
- MPU INT wire (Rev 4).
