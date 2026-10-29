## To initiate
```bash
source .venv/bin/activate
source ~/.espressif/v6.1/esp-idf/export.sh
```

If there were any changes regarding CMake or whatever, do `deactivate` first, and then `source ~/.espressif/v6.1/esp-idf/export.sh` again

## Block Diagram
```
Pi 5 — landing_sw (Python)                    ESP32 — gimbal_fw (C, ESP-IDF)
─────────────────────────────                 ───────────────────────────────────────────────
camera.py (picamera2, OV9281)                 MPU6050 INT (1 kHz data-ready) ─► GPIO ISR
  → markers.py → frames.py → ekf.py                                               │ notify
  → fsm.py → landing.py / pointing.py         core 1: ctrl_task (1 kHz)  ◄────────┘
                    │                            imu read → mahony; every 10th tick:
                    │                            stab_step() → q_cmd mailbox ──┐
                    │                         core 0: servo_task (100 Hz) ◄────┘
                    │                            SYNC_WRITE goals, READ present pos ×3
                    │                            → q_meas mailbox (back to ctrl_task)
                    │                              │ UART2, half-duplex via 74HC126
                    │                              ▼
                    │                            XL-320 ×3 (yaw, roll, pitch) @ 7.4 V
gimbal_iface.ESP32Gimbal                      core 0: comms_task (UART1 parser, telemetry 50 Hz,
  SET_TARGET (20–50 Hz) ──── UART 3.3 V ───►          watchdog), log_task (USB console)
  ◄──────────────────────── TELEM (50 Hz)
```

## File Layout
```
gimbal_fw/
├── CMakeLists.txt, sdkconfig.defaults, README.md, NOTES.md
├── main/
│   ├── app_main.c        boot sequence (§4.6), creates tasks
│   ├── board.h           pin map (ONLY place pins live); servo IDs live in calib_t
│   ├── ctrl_task.c       1 kHz: imu → mahony; 100 Hz: stab_step → q_cmd mailbox
│   ├── servo_task.c      100 Hz, sole owner of the servo bus: q_cmd → Sync Write, read q_meas
│   ├── comms_task.c      UART1 to Pi: parse SET_TARGET, send TELEM, watchdog
│   └── log_task.c        ring buffer → binary dump on USB console (dev only)
├── components/
│   ├── gimbal_math/      PURE C, NO ESP-IDF HEADERS (compiles on laptop)
│   │   ├── rot.c/.h          mat3/quat types, Rx/Ry/Rz, mul, transpose, wrap_pi
│   │   ├── kinematics.c/.h   gimbal_fk(), gimbal_ik(), joint limit clamp
│   │   ├── mahony.c/.h       attitude filter (quaternion)
│   │   ├── pid.c/.h          PID with anti-windup + output clamp (fallback mode)
│   │   └── stab.c/.h         stab_step(): the WHOLE per-tick control law
│   ├── mpu6050/          init, burst read 14 B, scale to SI, gyro bias cal
│   ├── xl320/            dxl_packet.c/.h: PURE C (crc16_dxl, packet build, status parser),
│   │                     host-tested · xl320.c/.h: UART2 half-duplex, ping/read/write/sync write
│   ├── proto/            frame encode/parse state machine, crc16
│   └── calib/            calib_t load/save in NVS
├── host_test/            Makefile + test_*.c, links gimbal_math only
└── tools/
    ├── gen_golden.py     Python reference → golden_*.csv test vectors
    └── serial_log.py     read dev log stream → CSV → plots
```