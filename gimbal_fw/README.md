## To initiate
```bash
source .venv/bin/activate
. ~/.espressif/v6.1/esp-idf/export.sh

```

## Block Diagram
```
Pi 5 — landing_sw (Python)                    ESP32 — gimbal_fw (C, ESP-IDF)
─────────────────────────────                 ───────────────────────────────────────────────
camera.py (picamera2, OV9281)                 MPU6050 INT (1 kHz data-ready) ─► GPIO ISR
  → markers.py → frames.py → ekf.py                                               │ notify
  → fsm.py → landing.py / pointing.py         core 1: ctrl_task (1 kHz)  ◄────────┘
                    │                            imu read → mahony → stab_step() → foc ×3
gimbal_iface.ESP32Gimbal                      core 0: comms_task (UART2 parser, telemetry 50 Hz,
  SET_TARGET (20–50 Hz) ──── UART 3.3 V ───►          watchdog), log_task (USB console)
  ◄──────────────────────── TELEM (50 Hz)
```

## File Layout
```
gimbal_fw/
├── CMakeLists.txt, sdkconfig.defaults, README.md, NOTES.md
├── main/
│   ├── app_main.c        boot sequence (§4.6), creates tasks
│   ├── board.h           pin map + motor-to-MCPWM mapping (ONLY place pins live)
│   ├── ctrl_task.c       1 kHz loop: imu → mahony → stab_step → foc
│   ├── comms_task.c      UART2 to Pi: parse SET_TARGET, send TELEM, watchdog
│   └── log_task.c        ring buffer → binary dump on USB console (dev only)
├── components/
│   ├── gimbal_math/      PURE C, NO ESP-IDF HEADERS (compiles on laptop)
│   │   ├── rot.c/.h          mat3/quat types, Rx/Ry/Rz, mul, transpose, wrap_pi
│   │   ├── kinematics.c/.h   gimbal_fk(), gimbal_ik(), joint limit clamp
│   │   ├── mahony.c/.h       attitude filter (quaternion)
│   │   ├── pid.c/.h          PID with anti-windup + output clamp (fallback mode)
│   │   └── stab.c/.h         stab_step(): the WHOLE per-tick control law
│   ├── mpu6050/          init, burst read 14 B, scale to SI, gyro bias cal
│   ├── foc/              MCPWM setup, open-loop sine drive, enable/fault
│   ├── proto/            frame encode/parse state machine, crc16
│   └── calib/            calib_t load/save in NVS
├── host_test/            Makefile + test_*.c, links gimbal_math only
└── tools/
    ├── gen_golden.py     Python reference → golden_*.csv test vectors
    └── serial_log.py     read dev log stream → CSV → plots
```