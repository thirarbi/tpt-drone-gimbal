# Software Work Plan — October Demo (8 → 27 Oct 2026)

> **To:** Fathir · **Re:** gimbal firmware + landing_sw for the 27 Oct demo
>
> This is your tasking for the next three weeks. Each task has an estimate, a
> "done when", and tips/cautions from me. Estimates are *focused* hours; hardware
> days usually overrun by ~30%, so the stretch items are your buffer. If a task goes
> **50% over its estimate, stop and flag it** — don't grind silently, we re-plan.
> Phase 0 (6–7 Oct) slipped, which is fine: it's folded into today and Friday.
>
> **Rev 2 (26 Oct): actuator change.** The gimbal is driven by three **Dynamixel
> XL-320 smart servos** on a half-duplex TTL bus, not gimbal BLDCs + DRV8313. Aga's
> working Arduino sketch (XL-320 chain + MPU-6050 console) is the hardware reference;
> its constants are folded into §2.5, §2.6 and §4.3. Everything MCPWM / open-loop /
> pole-pair related is gone. The maths (§4.1, §4.2) is unchanged, except that joint
> angles are now *measured* instead of assumed.

---

## 0. Ground rules

**Working style**
- Commit at the end of every task, small commits, messages that say *why*. Tag
  `demo-rc1` on Sun 25 and don't touch `main` after that except for bug fixes.
- Every task ends with something you can **show**: a plot, a passing test, a
  motor moving. "I wrote the code" is not done.
- Keep a running `gimbal_fw/NOTES.md`: measured values (servo IDs/baud, zero ticks,
  bus timings, loop times), weird bugs and their causes. This becomes your report
  material and the poster numbers.
- **Units in names**: `pitch_rad`, `rate_dps`, `t_us`, `goal_tick`. Radians everywhere
  in C and on the wire; degrees only at the `landing_sw` interface and in plots.

**Using your AI assistant (for learning, not just output)**
- For the maths (IK, filter): **write the first version yourself**, then ask the AI
  to review it. The derivation is the thing you need to be able to defend in front
  of the lecturer.
- Ask for *explanations and tests first*, code second: "explain how a Mahony filter
  corrects gyro drift, then give me 3 test cases it must pass".
- **Never trust AI-written register values or peripheral setup without the
  datasheet open.** Specific traps it *will* fall into here:
  - ESP-IDF has **two I²C APIs** (legacy `driver/i2c.h` vs new
    `driver/i2c_master.h`). AI happily mixes them. We use **only the new one**.
    If you see `i2c_master_write_to_device()` or `i2c_param_config()`, reject it.
  - **Dynamixel has two protocols and many control tables.** The XL-320 speaks
    **Protocol 2.0 only**, with its *own* control table (Goal Position = addr 30,
    2 bytes, 0–1023). The AX-12 (Protocol 1.0) and the X-series (XL330/XL430: Goal
    Position = addr 116, 4 bytes) look similar and AI blends them. Every address
    comes from the XL-320 e-manual page, nothing else.
  - Rotation order and sign conventions (ZXY vs ZYX, FRD vs FLU). Always verify
    numerically against the Python reference, never by reading.
  - MPU6050 clones: register map mostly identical, `WHO_AM_I` differs.
- Paste the actual compiler error / log output into the AI, not your summary of it.

**Datasheets to have open (download today)**
- MPU-6000/MPU-6050 *Product Specification* **and** *Register Map and Descriptions*
- ROBOTIS e-manual: the **XL-320** page (specs + control table) and the **DYNAMIXEL
  Protocol 2.0** page (packet format, CRC, byte stuffing, instructions, error byte)
- 74HC126 datasheet (quad tri-state buffer; output enable is active-**high**)
- ESP-IDF Programming Guide (match your installed version, v6.1): I2C master,
  UART (incl. **RS485 half-duplex mode**), GPIO (ISR), NVS, FreeRTOS (task
  notifications, queues)

**Arduino → ESP-IDF cheat sheet (you'll want this the first week)**

| Arduino | ESP-IDF (C) |
|---|---|
| `setup()` / `loop()` | `app_main()` creates tasks and **returns**; each task is its own `for(;;)` |
| `delay(ms)` | `vTaskDelay(pdMS_TO_TICKS(ms))` (set tick rate to 1000 Hz, see §4.7) |
| `millis()` / `micros()` | `esp_timer_get_time()` → `int64_t` µs |
| `Serial2.begin()` / `.write()` / `.read()` | `driver/uart.h`: `uart_driver_install`, `uart_write_bytes`, `uart_read_bytes` |
| `digitalWrite(DIR)` around a UART write | `uart_set_mode(..., UART_MODE_RS485_HALF_DUPLEX)`: the UART hardware drives DIR (as RTS) for you |
| `Wire` | `driver/i2c_master.h` |
| `Serial.print` | `ESP_LOGI(TAG, ...)` / `printf` |
| `attachInterrupt()` | `gpio_install_isr_service()` + `gpio_isr_handler_add()` |
| `EEPROM` | NVS (`nvs_flash_init`, `nvs_set_blob`) |
| Busy-wait loop | **Not allowed**: starves the idle task → task watchdog reset. Block on notifications/queues instead |

---

## 1. Demo scope

| Tier       | Demo item                                                                                                                          | Fallback if it slips                                   |
| ---------- | ---------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------ |
| **Must**   | Gimbal holds camera attitude (nadir) while the base is tilted by hand — IMU + IK on the ESP32                                      | Single-axis bench rig doing the same                   |
| **Must**   | `landing_sw` closed-loop sim landing live + Bab 4.2 results table                                                                  | Pre-recorded run + figures                             |
| **Should** | Pi detects a hand-held ArUco pad → commands gimbal over UART → gimbal tracks / reports angles                                      | Gimbal driven by a scripted angle sequence from the Pi |
| **Could**  | Sim landing with the real gimbal in the loop                                                                                       | Skip                                                   |
| **Over**   | The gimbal is NOT asked to lock-on to the ArUco board and follow it if it tilts! The gimbal is only asked to be pointed downwards. | -                                                      |

**Cut order if behind** (cut from the top): gimbal-in-the-loop sim → `ctypes`
gimbal sim (T7) → servo fault auto-recovery (reboot on overload) → active yaw
(hold yaw at 0, demo roll+pitch only) → servo position readback (run `stab_step`
on `q_cmd` only) → protocol speed-ups.

Explicitly **after** the demo: ArduPilot SITL integration (Aga, Nov), real MAVLink
flight hardware, flight tests, MAE/landing-radius analysis (Dec–Jan).

---

## 2. Architecture & interface contract

### 2.1 Block diagram

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

### 2.2 Interface semantics: read this twice

This is where integration bugs come from, so it's written down now and not changed
without telling the team.

- **Pi → ESP32 `SET_TARGET`** = *what the camera should look at*:
  - `pitch_rad`, `roll_rad`: **world-referenced** (gravity-stabilised) camera tilt.
    Nadir = pitch −π/2, roll 0.
  - `yaw_rel_rad`: **relative to the drone body** (we have no magnetometer, so
    there is no world yaw).
  - `mode`: `PARK` (servo torque off), `HOLD` (stabilise to target), `NADIR` (shortcut).
- **ESP32 → Pi `TELEM`** = *where the camera actually is relative to the body*:
  - `q_rad[3]` = **joint angles** (yaw, roll, pitch), body-relative, **measured**
    from the servos' position sensors (not the commanded value). This is what
    `frames.camera_to_body_rotation()` needs, because the marker must be
    transformed with the *live* camera→body rotation.
  - plus camera attitude from the IMU, status/fault flags (incl. per-servo
    hardware-error bits), loop-time max, echo of last command `seq`.
- `ESP32Gimbal.get_angles()` returns joint angles in the **existing**
  `landing_sw` convention: `(pitch, roll, yaw)` in degrees, pitch 0 = bore forward,
  −90 = bore down. `set_angles()` keeps its signature, so `fsm.py` doesn't change.

### 2.3 Frames & kinematic chain

- Joint order **yaw → roll → pitch** (base → camera), i.e. `R_body_link = Rz(ψ)·Rx(φ)·Ry(θ)`.
  This is **the same order `frames.py` already uses**, so keep it. Confirm with
  Pasha that the mechanical chain is yaw motor on the frame, then roll, then pitch
  holding the camera. If the CAD differs, the IK changes: tell me before
  Saturday.
- Body = FRD. Pitch θ = 0 → bore forward, θ = −90° → bore down (consistent with
  `config.GIMBAL_PITCH_LAND_DEG`).
- Singularity of this chain is at **roll = ±90°**, far from where a gimbal operates.
  That's why this order is good. Nadir (pitch −90°) is *not* singular here.
- The XL-320 only turns **300°** (ticks 0–1023, then a 60° dead zone). Have Pasha
  mount every horn so joint zero ≈ tick 512 (servo centre). That caps each joint at
  ±150° minus margin, so yaw ±170° is no longer possible.

### 2.4 Timing budget

| Item | Rate | Note |
|---|---|---|
| MPU6050 sample (data-ready IRQ) | 1 kHz | Sensor clocks the loop, so no aliasing and no duplicate samples |
| Attitude loop (`ctrl_task`) | 1 kHz | IMU read + Mahony. Budget ≤ 400 µs; the 14-byte I²C read alone is a few hundred µs at 400 kHz, so measure it |
| `stab_step` + servo bus (`servo_task`) | 100 Hz | The XL-320 runs its own position loop, so commanding it faster buys nothing. Per cycle: 1 Sync Write + 3 Reads |
| Servo bus baud | 1 Mbit/s (target) | Aga's servos are at 115200: one cycle there ≈ 2 ms write + 3 × ≈ 2.8 ms reads ≈ 10.5 ms, which **doesn't fit 100 Hz**. At 1 M with Return Delay 0 it's ≈ 1.2 ms. See §4.3 |
| Telemetry ESP32 → Pi | 50 Hz | ~40 B/frame → 2 kB/s, fine at 115200 |
| Commands Pi → ESP32 | 20–50 Hz | Watchdog: 200 ms without a valid frame → `NADIR` hold |
| Dev logging over USB | 200 Hz–1 kHz | Binary at 921600 baud on UART0 |

### 2.5 Pin map (servo + IMU pins from Aga's working sketch; Pi UART pins are a proposal, confirm with Aga)

| Function | GPIO | Note |
|---|---|---|
| Servo bus TX (UART2) | 17 | → 74HC126 pin 2 (1A) |
| Servo bus RX (UART2) | 16 | ← DATA line directly, so you receive your own echo (§4.3) |
| Servo bus DIR | 4 | → 74HC126 pin 1 (1OE), high = ESP32 drives the line. Used as UART2 **RTS** in RS485 half-duplex mode. Add a 10 k pull-**down** so the buffer is off during boot/reset |
| Servo DATA line | n/a | 74HC126 pin 3 (1Y) → DATA, 10 k pull-up DATA → 3V3, daisy-chained to the 3 servos |
| MPU6050 SDA / SCL | 21 / 22 | AD0 → GND (addr 0x68). Pull-ups to **3.3 V only**. Aga's sketch runs 100 kHz; we need 400 kHz (§2.4), so keep wires short (< 20 cm) |
| MPU6050 INT | 35 | input-only, push-pull active-high by default. **Not wired in Aga's sketch** (it polls), so add this wire |
| UART1 RX / TX ↔ Pi | 26 / 25 | Moved: UART2 now belongs to the servos. UART1's default pins are flash pins, so it **must** be remapped with `uart_set_pin`. Pi GPIO14 TX → ESP 26, Pi GPIO15 RX ← ESP 25, **common GND** |
| Status LED | 2 | onboard on most DevKits |

Freed by the actuator change: 13, 18, 19, 23, 27, 32, 33, 34.

Why the servo bus needs the 74HC126 at all: the XL-320 has **one** data wire for
both directions. The ESP32 TX pin always drives (idle = high), so it would fight
the servo's reply. The buffer disconnects TX from the line (high-Z) whenever DIR is
low; RX listens to the line all the time.

**Never use:** GPIO 6–11 (flash). **Avoid for outputs:** 0, 2, 5, 12, 15
(strapping; **GPIO 12 high at boot breaks the boot**), 1/3 (USB console).
34–39 are input-only with no internal pull-ups. **If the module is a WROVER**,
GPIO 16/17 are taken by PSRAM, so move the servo bus. `esptool chip_id` today tells
you what you have.

### 2.6 Power

- **Servos on their own 7.4 V supply** (XL-320 range is 6–8.4 V: a 2S pack, or a
  ≥ 3 A buck from the 3S pack set to 7.4 V). **A 3S pack (11.1–12.6 V) straight on
  the servo bus kills them.** ESP32 from a 5 V buck into the DevKit 5V pin. **One
  common ground** for LiPo−, servo supply, ESP32 GND, Pi GND.
- A few hundred µF electrolytic across the servo supply near the first servo: three
  servos accelerating together pull current spikes that otherwise brown out the bus.
- Size the supply for 3 × the XL-320 stall current (e-manual) plus margin: a servo
  pushing against a limit or an unbalanced camera *is* a stall.
- Bench work: lab PSU at 7.4 V, **current limit ~1–1.5 A** while bringing servos up.

---

## 3. Code map (what you'll write)

### 3.1 Firmware: `gimbal_fw/` (ESP-IDF project)

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

Key APIs (signatures only, you write the bodies):

- `kinematics.h`: `void gimbal_fk(const float q_rad[3], mat3_t *R_body_link);`
  `int gimbal_ik(const mat3_t *R_body_link, float q_rad_out[3]);` (returns non-zero
  near the roll singularity)
- `mahony.h`: `void mahony_init(mahony_t*, float kp, float ki);`
  `void mahony_update(mahony_t*, const float gyro_rads[3], const float acc_g[3], float dt);`
  `void mahony_get_R(const mahony_t*, mat3_t *R_world_cam);`
- `stab.h`: `void stab_step(stab_t*, const stab_in_t *in, stab_out_t *out);`
  `stab_in_t` = camera attitude, current `q_cmd`, measured `q_meas` (+ valid flag), target, dt; `stab_out_t` = new
  `q_cmd`, flags. **Keeping the control law a pure function** is what lets you
  unit-test it on the laptop and call the *exact same code* from the Python sim.
- `mpu6050.h`: `mpu6050_init()`, `mpu6050_read(mpu6050_sample_t*)`,
  `mpu6050_calibrate_gyro(n_samples)`
- `dxl_packet.h` (pure C): `uint16_t crc16_dxl(const uint8_t*, size_t)`,
  `size_t dxl_build(uint8_t id, uint8_t inst, const uint8_t *params, uint16_t n, uint8_t *out)`,
  `int dxl_parser_feed(dxl_parser_t*, uint8_t byte)` → 1 when a valid **status** packet is complete
- `xl320.h`: `xl320_init(uart_port_t, int baud)`, `xl320_ping(uint8_t id)`,
  `xl320_read_u16(uint8_t id, uint16_t addr, uint16_t *v)`, `xl320_write_u8(...)`,
  `xl320_sync_write_goal(const uint8_t ids[], const uint16_t ticks[], int n)`,
  `xl320_torque(uint8_t id, bool on)`, `xl320_reboot(uint8_t id)`. Return 0, a
  negative `XL320_ERR_*` (timeout, CRC, bad packet), or the servo's error byte.
- `proto.h`: `proto_parser_feed(proto_parser_t*, uint8_t byte)` → callback per valid
  frame; `size_t proto_encode(uint8_t type, const void *payload, uint8_t len, uint8_t *out)`;
  `uint16_t crc16_ccitt(const uint8_t*, size_t)`
- `calib.h`: `calib_t { servo_id[3], dir[3], zero_tick[3], gyro_bias[3], R_imu_cam }`,
  `calib_load()`, `calib_save()`

### 3.2 Pi side: `landing_sw/`

| File | New/changed | Purpose |
|---|---|---|
| `gimbal_kinematics.py` | new | Python reference FK/IK, mirrors the C exactly |
| `gimbal_proto.py` | new | frame encode/decode with `struct` + `binascii.crc_hqx` |
| `gimbal_iface.py` | add `ESP32Gimbal` | background reader thread, latest TELEM under a lock, `set_angles/get_angles` |
| `camera.py` | new | picamera2 capture of the lo-res Y stream (OV9281 is mono). Real mode has **no capture code yet** (`main.py` exits) |
| `main.py` | real mode | wire `camera.py` + `ESP32Gimbal` (+ mock MAVLink for the demo) |
| `frames.py` | **fix** | mirror bug (T3c) |
| `config.py` | changed | ESP32 port/baud/protocol constants; remove SimpleBGC; real `CAM_K` |
| `exp_gimbal_stab.py` | new (stretch) | gimbal sim via `ctypes` → Bab 4.2 row |
| `tests/` | new | `pytest` for kinematics, proto round-trip, frames determinant |

---

## 4. Key technical notes (with the few snippets that matter)

### 4.1 Inverse kinematics (yaw→roll→pitch)

Expanding `M = Rz(ψ)·Rx(φ)·Ry(θ)` gives `M[2][1] = sin φ`,
`M[0][1] = −sin ψ cos φ`, `M[1][1] = cos ψ cos φ`, `M[2][0] = −cos φ sin θ`,
`M[2][2] = cos φ cos θ`. So:

```c
// R = desired camera ("link") attitude relative to the body
float s_roll = fminf(fmaxf(R->m[2][1], -1.0f), 1.0f);   // clamp: asinf(1.0000001f) = NaN
q[ROLL]  = asinf(s_roll);
q[YAW]   = atan2f(-R->m[0][1], R->m[1][1]);
q[PITCH] = atan2f(-R->m[2][0], R->m[2][2]);
if (fabsf(s_roll) > 0.995f) return GIMBAL_IK_NEAR_SINGULAR;
```

**Do the derivation on paper yourself first** (Sat 10), then check against this.
The test that matters: for 10 000 random joint vectors inside the limits,
`ik(fk(q)) == q` to 1e-4 rad.

### 4.2 The per-tick control law (`stab_step`)

The IMU measures the camera's attitude in the world, `R_wc`. We know the joint angles
we commanded, `q_cmd`. So we can *infer the drone body attitude* and solve for the
joints that put the camera where we want it:

1. `R_wb = R_wc · FK(q_meas)ᵀ`: base attitude estimate (no extra sensor needed).
   Use the **measured** joint angles from the servos; fall back to `q_cmd` if the
   readback is stale (> 30 ms) or was cut.
2. Build target `R_wc*` **using the base's own heading**: `R_wc* = Rz(yaw(R_wb) + yaw_rel) · Ry(pitch_t) · Rx(roll_t)`.
   The MPU6050's yaw drifts, but it drifts *identically* in `R_wc` and `R_wb`, so
   it **cancels**. This is the trick that makes a no-magnetometer IMU usable.
3. `q_ik = IK(R_wbᵀ · R_wc*)`
4. `q_cmd += clamp(K · wrap_pi(q_ik − q_cmd), ±ω_max·dt)`, then clamp to joint limits.

Why `K < 1` (low-pass) and not just `q_cmd = q_ik`: the servo *lags* its goal (its
own position loop + gear train), and `q_meas` is up to one servo cycle (10 ms) old
while `R_wc` is fresh. Step 1 is then briefly inconsistent, and a deadbeat update
oscillates. Tune `K`: start at a ~5 Hz equivalent, raise until it rings, then back
off by half. `K` is applied per **100 Hz** tick now.

The **fallback mode** (`STAB_MODE_PID`), worth 1 h to implement: for small angles
each joint mostly corrects its own camera axis, so a plain per-axis PID on camera
roll/pitch error (classic hobby-gimbal style) also works. Implement it so you have
something that stabilises even if the IK loop fights you on the day.

### 4.3 XL-320 smart servos: what the servo does for you, and the bus

You no longer generate the motor drive. Each XL-320 has its own MCU, a potentiometer
and a position PID: you send a *goal* in ticks, it drives there and holds.
Consequences:

- **Absolute position**: Present Position is readable any time → no snap on
  enable, no slip, no hunting for a yaw home mark at boot. You still need a one-time
  `zero_tick` per joint (T16).
- **Resolution 300°/1023 ≈ 0.29°/tick**, plus plastic-gear backlash. That is a floor
  on pointing accuracy; measure both in T12 and quote them honestly.
- **Torque is limited** (stall torque per e-manual, < 0.4 N·m at 7.4 V): balance
  still matters, now because of **overload shutdown** instead of slip.

Ticks ↔ joint radians (the *only* place this conversion lives):

```c
#define XL320_RAD_PER_TICK  (300.0f / 1023.0f * 0.017453293f)   // ≈ 0.0051183 rad
q_rad = (float)c->dir[j] * (float)((int)tick - (int)c->zero_tick[j]) * XL320_RAD_PER_TICK;
// inverse: tick = zero_tick + dir * q_rad / XL320_RAD_PER_TICK, rounded, clamped to 0..1023
```

**Control table** (XL-320, decimal addresses, as used in Aga's sketch; verify every
row in the e-manual):

| Addr | Size | Name | Area | Use |
|---|---|---|---|---|
| 3 | 1 | ID | EEPROM | 1, 2, 3 on Aga's bus |
| 4 | 1 | Baud Rate | EEPROM | 115200 now → 1 M (§2.4) |
| 5 | 1 | Return Delay Time | EEPROM | × 2 µs, default 250 (= 500 µs) → set 0 |
| 6 / 8 | 2 | CW / CCW Angle Limit | EEPROM | = joint limits in ticks: servo-side safety net |
| 11 | 1 | Control Mode | EEPROM | 2 = joint (1 = wheel) |
| 17 | 1 | Status Return Level | EEPROM | default replies to everything; 1 = reply to PING/READ only |
| 24 | 1 | Torque Enable | RAM | EEPROM writes are only accepted with torque **off** |
| 25 | 1 | LED | RAM | bits R=1 G=2 B=4: free status light per joint |
| 27–29 | 1 each | D / I / P gain | RAM | servo stiffness; tune P in T12 |
| 30 | 2 | Goal Position | RAM | 0–1023 = 0–300° |
| 32 | 2 | Moving Speed | RAM | 0 = max (unregulated); Aga used 300 |
| 35 | 2 | Torque Limit | RAM | low during first bring-up |
| 37 | 2 | Present Position | RAM | → `q_meas` |
| 41 | 2 | Present Load | RAM | bits 0–9 magnitude, bit 10 direction |
| 45 | 1 | Present Voltage | RAM | × 0.1 V: check 6–8.4 V at boot |
| 46 | 1 | Present Temperature | RAM | °C |
| 50 | 1 | Hardware Error Status | RAM | overload / overheat / input voltage |

**Packet (Protocol 2.0)**

```
[FF FF FD 00][ID][LEN_L LEN_H][INST][PARAMS...][CRC_L CRC_H]
LEN = n_params + 3            (instruction + params + 2 CRC bytes)
CRC = CRC-16, poly 0x8005, init 0x0000, not reflected (a.k.a. CRC-16/BUYPASS),
      over everything from the first FF to the last param
Status reply: INST = 0x55, then 1 ERROR byte, then the data
Instructions: PING 0x01, READ 0x02, WRITE 0x03, REBOOT 0x08, SYNC_WRITE 0x83; broadcast ID 0xFE (never replies)
```

- This is a **different CRC** from the Pi protocol's CCITT one (§4.5). Name them
  `crc16_dxl` and `crc16_ccitt`, never plain `crc16`. Host-test `crc16_dxl` against
  the e-manual's PING example (`FF FF FD 00 01 03 00 01` → CRC bytes `19 4E`) and the
  standard check value (`"123456789"` → `0xFEE8`).
- **Byte stuffing**: Protocol 2.0 inserts an extra `FD` whenever `FF FF FD` appears
  inside the params. With goals ≤ 1023 (high byte ≤ `0x03`) and IDs 1–3 that can't
  happen, so Aga skipped it and so can we, **as long as you `assert(tick <= 1023)`**
  before building a packet. Write that invariant in a comment next to `dxl_build`.
- **One Sync Write per servo cycle** moves all three servos at the same instant and
  gets no reply. Reads are one round trip per servo.
- **Echo**: RX sits on the same wire as TX, so you receive every packet you send.
  Aga's `readStatus()` already handles this the right way: parse everything, **drop
  packets whose instruction isn't `0x55` or whose ID is wrong**. Make
  `dxl_parser_feed()` a byte-wise state machine like `proto.c`; then you can
  host-test it with a recorded "echo + reply" byte stream.
- **DIR in ESP-IDF**: don't port `digitalWrite(DIR); write; flush; digitalWrite(DIR)`.
  Put UART2 in `UART_MODE_RS485_HALF_DUPLEX` with GPIO4 as its **RTS** pin; the UART
  hardware asserts RTS while bytes are on the wire and releases it right after the
  stop bit. That matters at 1 M with Return Delay 0, where the reply starts within
  µs. **Check the polarity on the scope** (RTS high during TX = 74HC126 OE high)
  before connecting servos. API reference: ESP-IDF's `uart/uart_echo_rs485` example.
- **Faults**: if bit 7 (Alert) of a status packet's error byte is set, read Hardware
  Error Status (50). Overload/overheat switches torque off by itself; recover with
  REBOOT → torque on → flag it in TELEM. A servo that silently went limp is the new
  "slip".

### 4.4 Loop timing: sensor-driven, ISR does nothing but notify

```c
static TaskHandle_t s_ctrl;
static void IRAM_ATTR imu_drdy_isr(void *arg) {
    BaseType_t woken = pdFALSE;
    vTaskNotifyGiveFromISR(s_ctrl, &woken);
    portYIELD_FROM_ISR(woken);
}
static void ctrl_task(void *arg) {
    for (;;) {
        if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(5)) == 0) { /* IMU dead → safe stop */ }
        int64_t t0 = esp_timer_get_time();
        /* imu read → mahony; every 10th tick: stab_step → xQueueOverwrite(q_cmd) */
        /* record esp_timer_get_time() - t0 into loop stats */
    }
}
// xTaskCreatePinnedToCore(ctrl_task, "ctrl", 4096, NULL, configMAX_PRIORITIES-2, &s_ctrl, 1);
```

- **No float maths in ISRs** on the ESP32 (the FPU isn't usable there, so it'll crash
  or corrupt). The ISR only notifies.
- **`float` everywhere, never `double`.** The ESP32 FPU is single-precision;
  `double` is emulated in software (much slower). Use `sinf/cosf/atan2f/sqrtf` and
  `1.0f` literals; compile with `-Wdouble-promotion` to catch slips.
- Don't call `ESP_LOGx`/`printf` inside `ctrl_task`: it blocks on the UART. Push
  to the log ring buffer instead.
- **No servo bus I/O in `ctrl_task` either.** A servo READ blocks until the reply
  (~0.3 ms at 1 M, ~3 ms at 115200). `servo_task` owns UART2 alone (so no mutex),
  takes `q_cmd` from a 1-slot queue (`xQueueOverwrite` / `xQueuePeek`), and
  publishes `q_meas` + timestamp + per-servo status the same way. Aga's sketch notes
  the IMU "may skip samples" while servos talk: this split is what prevents that.

### 4.5 Protocol frame

```c
// [0xA5][0x5A][type u8][len u8][seq u16][payload...][crc16 u16], little-endian
// crc = CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) over type..payload
typedef struct __attribute__((packed)) {
    uint8_t mode; float pitch_rad, roll_rad, yaw_rel_rad;
} msg_set_target_t;
```

Python side gets the same CRC for free: `binascii.crc_hqx(data, 0xFFFF)`.
Write the C CRC yourself (10 lines, bitwise), and test it against a value from
Python. Don't use ESP-IDF's ROM CRC helpers; their init/xor conventions differ.
Python mirrors the packed structs with `struct.Struct('<B3f')`. **One struct
definition change = change both sides + bump a `PROTO_VERSION` byte in TELEM.**

### 4.6 Boot sequence (`app_main`)

1. DIR low (pull-down) → servo bus released. Init NVS, load `calib_t` (if missing →
   refuse `HOLD`, allow calibration only).
2. Init MPU6050; **gyro bias calibration: 2 s, gimbal must be still**; reject if
   gyro variance too high ("someone's touching it") and retry. (Servo torque is
   still off, so the camera hangs freely: that's fine, it just has to be still.)
3. Mahony converges for ~1 s (high `kp` during start-up, then normal).
4. **Servo bring-up**: PING all 3 IDs (missing → fault, refuse `HOLD`). Check
   Present Voltage is 6–8.4 V. Read Hardware Error Status; REBOOT any servo with an
   error. Read Present Position → `q_meas`. **No home mark needed**: the servos know
   where they are.
5. Goal = present position, Moving Speed low (e.g. 50), torque on → nothing moves.
   Then goal = NADIR; the servos walk there slowly. Once reached (or after 2 s), set
   the operating Moving Speed.
6. Start `ctrl_task`, `servo_task`, `comms_task`. Mode = `NADIR` until the Pi says
   otherwise.

### 4.7 Config defaults

```
# sdkconfig.defaults
CONFIG_FREERTOS_HZ=1000
CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ_240=y
CONFIG_COMPILER_OPTIMIZATION_PERF=y
```

Host tests: `gcc -std=c11 -O2 -Wall -Wextra -Werror -Wdouble-promotion -Wconversion … -lm`.
For `ctypes` also build `libgimbal_math.so` with `-shared -fPIC`.

---

## 5. Day-by-day tasking

Week 2 (8–14 Oct) is blocked on the team sheet. Take the **ESP32 + MPU6050 + USB
cable home**; everything from Fri 9 to Wed 14 needs only those and your laptop.
Total ≈ 75 h over 20 days.

| Date | Task | Est. |
|---|---|---|
| Thu 8 | T0 Lab smoke tests | 3–4 h |
| Fri 9 | T1 Design freeze · T2 Skeleton + host tests | 2 h + 2 h |
| Sat 10 | T3a Kinematics (Python + golden vectors) | 3 h |
| Sun 11 | T3b Kinematics in C · T3c frames.py mirror bug | 3 h + 1–2 h |
| Mon 12 | T4 MPU6050 driver + 1 kHz loop + logger | 4 h |
| Tue 13 | T5 Attitude filter | 4 h |
| Wed 14 | T6 Progress report · T7 ctypes sim (stretch) | 2 h + 3 h |
| Thu 15 | T8 XL-320 bus driver, one servo (lab) | 5 h |
| Fri 16 | T9 Control task + boot sequence | 4 h |
| Sat 17 | T10 3 servos + NVS calib + on-target tests | 4 h |
| Sun 18 | T11 Single-axis stabilisation | 4 h |
| Mon 19 | T12 Tuning + characterisation | 4 h |
| Tue 20 | T13 Protocol, both sides | 5 h |
| Wed 21 | T14 Link robustness · T15 Pi camera capture | 2 h + 3 h |
| Thu 22 | T16 3-axis bring-up on real gimbal (lab) | 4–5 h |
| Fri 23 | T17 3-axis tuning + measurement | 4 h |
| Sat 24 | T18 Vision → gimbal end-to-end | 5 h |
| Sun 25 | T19 Demo path + **feature freeze** | 4 h |
| Mon 26 | T20 Rehearsal + fallbacks | 3–4 h |
| Tue 27 | **DEMO** | — |

---

### T0 · Thu 8 (today, lab) · Hardware smoke tests · 3–4 h

**Goal:** prove every link works *before* writing firmware, using only stock
examples. This is the most valuable lab time you'll get this week.

**T0.1 Permissions + ESP-IDF install (start first, it runs in background, ~30 min)**
```bash
sudo usermod -aG dialout $USER          # then log out/in; you're NOT in dialout now
sudo apt-get install git wget flex bison gperf python3 python3-pip python3-venv \
     cmake ninja-build ccache libffi-dev libssl-dev dfu-util libusb-1.0-0
mkdir -p ~/esp && cd ~/esp
git clone -b <latest stable v5.x tag> --recursive https://github.com/espressif/esp-idf.git
cd esp-idf && ./install.sh esp32
. ~/esp/esp-idf/export.sh               # in every new terminal (make an alias)
```
Need ≥ v5.2 for `i2c_master.h`; take the latest stable release tag from GitHub.
(The VS Code "ESP-IDF" extension wizard does the same thing if you prefer.)

**T0.2 ESP32 identity (10 min)**: plug in, `sudo dmesg -w` → expect `cp210x`
or `ch341` → `/dev/ttyUSB0`. `pip install esptool` (in the venv), then
`esptool.py -p /dev/ttyUSB0 chip_id` and `flash_id`. Record chip type (ESP32 vs
S3!), module (WROOM/WROVER), flash size. Won't connect → hold BOOT, tap EN.
Nothing in dmesg → charge-only cable.

**T0.3 Inventory (20 min)**: photograph every board both sides. Write in
`NOTES.md`: MPU board model (on-board pull-ups? which value?), the 74HC126 wiring
as built, the servo supply. Flash **Aga's sketch** (Arduino) and run `find`: record
each servo's **ID and baud** (currently 115200), then `read` each one (position,
voltage, temperature). Label every servo with its ID and the joint it goes on.

**T0.4 Hello world flash (15 min, once IDF is ready)**: copy
`$IDF_PATH/examples/get-started/hello_world`, `idf.py set-target esp32`,
`idf.py -p /dev/ttyUSB0 flash monitor` (exit monitor: `Ctrl+]`).

**T0.5 MPU6050 with zero code (30 min)**: wire VCC→3V3, GND, SDA→21, SCL→22,
AD0→GND. Flash `examples/peripherals/i2c/i2c_tools` and use its console:
```
i2cconfig --port=0 --sda=21 --scl=22 --freq=400000
i2cdetect                              # expect 0x68
i2cget -c 0x68 -r 0x75 -l 1            # WHO_AM_I: 0x68 genuine; 0x70/0x71/0x98 = clone → note it
i2cset -c 0x68 -r 0x6B 0x00            # wake (PWR_MGMT_1)
i2cget -c 0x68 -r 0x3B -l 14           # accel XYZ, temp, gyro XYZ (big-endian int16)
```
Flat on the table at default ±2 g → accel Z ≈ `0x40 0x00` (16384 = 1 g). Tilt it
90° and watch g move to another axis. **Note which IMU axis points where
relative to the camera**; you need this mounting matrix later.

**T0.6 Pi ↔ ESP32 UART (45 min)**
- Pi first, alone: in `/boot/firmware/config.txt` add `dtparam=uart0=on`;
  `raspi-config` → Interface → Serial: login shell **No**, hardware **Yes**; reboot.
  `ls -l /dev/ttyAMA* /dev/serial*`. On a Pi 5, `serial0` may point at the
  debug-header UART, not GPIO14/15, so use `/dev/ttyAMA0`. **Loopback test**:
  jumper pin 8 (GPIO14) ↔ pin 10 (GPIO15), run
  `python3 -m serial.tools.miniterm /dev/ttyAMA0 115200`, and typed chars echo back.
- Then ESP32: flash `examples/peripherals/uart/uart_echo`, in `idf.py menuconfig`
  set UART port 1, RX 26, TX 25, 115200 (UART2 is the servo bus now, §2.5). Wire Pi
  TX→ESP 26, Pi RX←ESP 25, **GND↔GND**.
  Typing in miniterm on the Pi echoes back.
- Both sides are 3.3 V logic, so direct wiring is fine. **Never** connect a 5 V
  USB-serial adapter's TX to either.

**T0.7 (only with a current-limited bench PSU) Servo bus scope trace (20 min)**:
7.4 V, limit 1 A. With Aga's sketch, scope DATA and DIR together during `ping`:
DIR must be high exactly while the ESP32's bytes are on the line and low before the
servo's reply starts. Note the gap (= Return Delay Time). Save the screenshot: it's
your reference when the ESP-IDF version misbehaves.

**Done when:** `NOTES.md` has chip ID, IMU WHO_AM_I + axis mapping, servo IDs + baud,
the DIR/DATA scope trace, UART echo working Pi↔ESP32.
**Watch out:** don't put the IMU on 5 V I²C lines; keep everything 3.3 V logic.
Unplug servo power before rewiring, *every* time.

---

### T1 · Fri 9 · Design freeze + interface contract · 2 h

- Put §2 of this doc into `gimbal_fw/README.md` (you can trim it). Get **axis order,
  servo-ID-to-joint mapping, horn orientation (which tick = joint zero), and joint
  mechanical limits** from Pasha; **pin map and power** from Aga. Update §2.5 to the
  real pins.
- Decide joint limits now (e.g. pitch −110°…+20°, roll ±40°, yaw ±145°) because they go in
  the IK clamp, the servos' angle-limit registers, and protect the cables. The
  XL-320's 300° travel caps every joint at ±150° around its centre tick.
- **Done when:** README committed; Pasha and Aga have seen the pin map and axis order.

### ✅ T2 · Fri 9 · Project skeleton + host test harness · 2 h

- `idf.py create-project gimbal_fw`, create the component folders from §3.1 with
  empty `.c/.h` + `CMakeLists.txt` (`idf_component_register(SRCS ... INCLUDE_DIRS include REQUIRES ...)`),
  `sdkconfig.defaults` (§4.7). Builds and flashes an empty `app_main`.
- `host_test/Makefile` builds `gimbal_math` + a trivial test with `gcc` and runs it.
  `make -C gimbal_fw/host_test` must work from the repo root.
- **Tip:** `gimbal_math` must never `#include` anything from ESP-IDF. The host
  build enforces it for you: if it breaks, you've leaked a dependency.
- **Done when:** both `idf.py build` and `make -C gimbal_fw/host_test` pass.

### ✅ T3a · Sat 10 · Kinematics: Python reference + golden vectors · 3 h

- Derive FK/IK for ZXY **on paper** (§4.1 is your answer key, so don't peek first).
- `landing_sw/gimbal_kinematics.py`: `fk(q)`, `ik(R)`, `clamp_limits(q)`, with
  `pytest`: round-trip on 10 000 random q, singularity detection, limits.
- `gimbal_fw/tools/gen_golden.py`: writes `golden_kinematics.csv` (q, R) for the C tests.
- **AI use:** ask it to *check your derivation*, and to suggest edge cases (angles at
  ±π, at limits, near singularity). Don't ask it to "write IK".
- **Done when:** pytest green; CSV committed.

### T3b · Sun 11 · Kinematics in C + host tests · 3 h

- `rot.c`, `kinematics.c`; `host_test/test_kinematics.c` reads the CSV and checks
  FK and IK against Python (tolerance 1e-4 rad / 1e-5 on matrix entries).
- **Watch out:** `asinf` of 1.0000001 is NaN, so clamp first. `atan2f` argument order
  is (y, x). Angle wrap: compare with `wrap_pi(a−b)`, not `a−b`.
- **Done when:** host test green, no warnings with `-Wdouble-promotion -Wconversion`.

### T3c · Sun 11 · frames.py mirror bug · 1–2 h · **high priority**

- `frames.camera_to_body_rotation()` returns a matrix with **determinant −1 for every
  gimbal angle** (`_NADIR_CAM_TO_BODY = diag(1,-1,1)` is a reflection). Camera
  (OpenCV) and body (FRD) frames are both right-handed, so the map *must* be a proper
  rotation. On the real drone this mirrors one lateral axis, which turns that axis of
  the landing loop into **positive feedback**.
  The sim doesn't catch it, probably because the synthetic camera and sim are
  consistent with the same convention.
- Steps: (1) add a pytest `det(R) == +1` for random angles (it'll fail, good);
  (2) define the true camera mounting at nadir (image-right / image-down vs
  body F/R/D) and fix it; (3) make `synthetic_markers`/sim agree; (4)
  `run_all_experiments` and `main --sim` must still pass/land; (5) add a test that
  `frames.camera_to_body_rotation(q) == gimbal_kinematics.fk(q) @ R_link_optical`.
- Physical confirmation comes in T18 (marker to the drone's right → body y > 0).
- **Done when:** determinant test + consistency test green, suite still passes.

### T4 · Mon 12 · MPU6050 driver + 1 kHz data-ready loop + dev logger · 4 h

- `mpu6050.c` on `i2c_master.h`: wake (0x6B), `SMPLRT_DIV`=0, `CONFIG` DLPF=2
  (~94 Hz BW, ~3 ms delay; DLPF must be on for 1 kHz output), gyro ±1000 dps,
  accel ±4 g, INT data-ready enable. Burst read 14 bytes from 0x3B, big-endian →
  `int16_t` → SI units.
- GPIO ISR on INT → notify (§4.4). Measure the actual rate (count per second) and the
  read duration.
- `log_task.c` + `tools/serial_log.py`: stream binary samples at 921600 baud → CSV →
  plot. You'll use this tool every day until the demo, so make it nice.
- Record **two logs**: 60 s perfectly still (bias/noise), and 60 s hand-tilting. These
  feed T5.
- **Watch out:** every register value from AI → verify in the Register Map PDF.
  The DLPF setting changes the gyro's base output rate (8 kHz vs 1 kHz), so check it.
- **Done when:** 1000 ± 5 Hz measured, logs saved, still-log noise in `NOTES.md`.

### T5 · Tue 13 · Attitude filter (Python → C) · 4 h

- Prototype Mahony in Python on yesterday's logs: gyro bias subtraction, quaternion
  integration, accel correction with **gating** (skip accel correction when
  `| |a| − 1 g | > 0.15 g`, since drone accelerations would tilt your "down").
  Starting gains: `kp ≈ 1.0`, `ki ≈ 0.02`.
- Port to `mahony.c`; host test: C on the same log == Python output (1e-4).
- Apply the IMU→camera mounting matrix `R_imu_cam` (from T0.5) **before** the filter.
- **Tip:** renormalise the quaternion every step. Check: at rest for 60 s, roll/pitch
  drift < 0.5°.
- **Done when:** plots of roll/pitch for both logs; C matches Python.

### T6 · Wed 14 · Progress-report input · 2 h

- To Aga: architecture diagram (§2.1), interface contract, Bab 4.2 table, what's
  done/next, risks. Plots from T4/T5 make good figures.

### T7 · Wed 14 (stretch) · Gimbal sim via ctypes · 3 h

- `stab.c` first version (§4.2). Build `libgimbal_math.so`; `exp_gimbal_stab.py`
  drives a simulated base (roll/pitch sinusoid 0.5–3 Hz, ±10°) + simple servo model
  (first-order lag + rate limit + 0.29° quantisation), calls the *real* `stab_step` → camera pointing error vs time → new Bab 4.2
  row (e.g. RMS < 1°).
- First thing to cut if behind; you can still write `stab.c` on Sun 18.

---

### T8 · Thu 15 (lab) · XL-320 bus driver, one servo · 5 h

- `dxl_packet.c` first, **on the laptop**: `crc16_dxl`, `dxl_build`,
  `dxl_parser_feed` (§4.3). Host tests: CRC vs the e-manual PING example; PING /
  WRITE / SYNC_WRITE packets byte-for-byte identical to what Aga's sketch sends
  (print its `pkt[]` over Serial once); the parser fed *echo + reply* returns only
  the reply; garbage before the header and a wrong CRC are rejected.
- `xl320.c` on the ESP32: UART2 in RS485 half-duplex mode (§4.3). Sequence: **scope
  TX + DIR with no servo connected** → PING one servo → READ Present Position →
  WRITE Goal Position → SYNC_WRITE with one ID.
- Then the **bus speed-up**, one servo at a time (others unplugged): Return Delay
  Time → 0, Status Return Level → 1, Baud Rate → 1 M; re-PING at the new baud. If
  1 M looks bad on the scope, stay at 115200 and drop the servo rate to 50 Hz.
- Measure: PING round trip, READ round trip, Sync Write duration
  (`esp_timer_get_time()` around each call, or a logic analyser).
- **Watch out:** EEPROM registers only accept writes with torque off. A servo whose
  baud you just changed "disappears" at the old baud: expected, not dead. Keep Aga's
  sketch as the known-good recovery tool (edit `DXL_BAUD` to match).
- **Done when:** one servo goes to commanded angles repeatably from ESP-IDF, bus
  timings in `NOTES.md`, host tests green.

### T9 · Fri 16 · Control task, boot sequence, timing · 4 h

- `ctrl_task` + `servo_task` / `app_main` per §4.4 and §4.6 with real IMU + one
  servo. Loop stats: min/avg/max loop time and missed notifications; servo cycle
  time, read timeouts, CRC failures. Reported every second via log.
- Stub `comms_task` (watchdog only for now).
- **Done when:** max loop time < 1 ms over 10 min, servo cycle ≤ 10 ms, no
  task-watchdog resets, boot sequence walks the servo to NADIR without a jerk.

### T10 · Sat 17 · Three servos + calibration in NVS + on-target tests · 4 h

- Extend to 3 servos: one Sync Write for all goals, three Reads. (Check whether the
  XL-320 e-manual lists **Sync Read**; if it does, 3 round trips become 1.)
- `calib.c`: `calib_t` in NVS + a tiny calibration console command (or compile-time
  defaults for now): per-axis servo ID, direction, zero tick.
- Write each servo's CW/CCW Angle Limit (EEPROM) from the joint limits.
- Flash a test build that runs the golden-vector kinematics and filter tests **on the
  ESP32** (proves float behaviour matches the laptop).
- **Done when:** 3 servos respond independently; on-target tests pass.

### T11 · Sun 18 · Single-axis stabilisation (pitch) · 4 h

- `stab_step` with only pitch active (other joints held). Hand-tilt the base:
  camera pitch should stay put. Implement the PID fallback mode too (§4.2).
- **Tip:** log `q_cmd`, IMU pitch, target at 200 Hz in every run; tuning without
  plots is guessing.
- **Done when:** ±15° base tilt → camera pitch error < ~2°, logged plot.

### T12 · Mon 19 · Tuning + open-loop characterisation · 4 h

- Tune `K`, `ω_max`, servo P gain (addr 29) and Moving Speed. Characterise: servo
  step response (10° goal step, Present Position logged at 100 Hz), dead time,
  **backlash** (reverse in 1-tick steps until the IMU sees the camera move), max base
  tilt rate before the servo saturates, temperature and load while holding.
- (Stretch) fault recovery: Alert bit or no reply → REBOOT → torque on → resume;
  log every event.
- **Done when:** numbers in `NOTES.md`; tuning committed as defaults.

### T13 · Tue 20 · Protocol, both sides · 5 h

- C: `proto.c` parser as a byte-wise state machine (SYNC1→SYNC2→HDR→PAYLOAD→CRC),
  resync on bad CRC, `comms_task` on the ESP-IDF UART driver (`uart_driver_install`
  with an event queue, UART2 pins from §2.5). TELEM at 50 Hz.
- Python: `gimbal_proto.py` + `ESP32Gimbal` (reader thread, latest TELEM + timestamp
  under a lock, `set_angles()` → `SET_TARGET`). `config.py`: add
  `ESP32_GIMBAL_PORT = "/dev/ttyAMA0"`, baud, remove SimpleBGC bits.
- pytest: encode→decode round-trip, CRC vs C golden value, garbage-byte resync.
- **Done when:** from a Pi Python shell, `g.set_angles(-90, 0, 0)` moves the gimbal and
  `g.get_angles()` returns live joint angles.

### T14 · Wed 21 · Link robustness · 2 h

- Measure command→TELEM round-trip latency (echo `seq`), CRC error rate over 10 min,
  behaviour on cable pull (watchdog → NADIR within 200 ms) and reconnect.
- **Done when:** latency number in `NOTES.md`; cable-pull test passes.

### T15 · Wed 21 · Pi camera capture module · 3 h

- `camera.py` with picamera2: lo-res stream at `DETECT_RES`, 30 fps, mono/Y plane →
  `numpy` → `markers.py`. On Pi 5 the OV9281 may need `camera_auto_detect=0` +
  `dtoverlay=ov9281` in `config.txt`; check Arducam's docs for your exact module.
- Measure capture-to-detection latency (feeds Bab 4.2 row 1 with a *real* number).
- **Done when:** live detection of a printed marker printing pose at 30 fps.

---

### T16 · Thu 22 (lab) · 3-axis bring-up on Pasha's gimbal · 4–5 h

- Mount, then **with torque off** move each joint by hand through its range and check
  cables, watching Present Position live (it must never enter the 60° dead zone).
  Then one axis at a time: direction sign, `zero_tick` (read Present Position at the
  mechanical zero), joint limits in firmware **and** in the servo's angle-limit
  registers. Save to NVS.
- Verify the **IMU mounting matrix** on the real camera mount (it may differ from
  your bench).
- **Watch out:** first power-up of each axis with low Moving Speed and Torque Limit,
  your hand near the power switch. Unbalanced camera = servo near stall, hot, and
  overload shutdowns, so tell Pasha immediately.
- **Done when:** all 3 joints track commanded angles, no collisions at limits.

### T17 · Fri 23 · 3-axis stabilisation tuning + measurement · 4 h

- Full `stab_step`. Hand-disturb the base in roll, pitch, yaw. Log and compute
  **pointing error RMS and max** under a repeatable disturbance (e.g. tilt ±15° at
  ~1 Hz by hand for 30 s; do it 3 times).
- **Done when:** poster numbers + plot; tuned values committed.

### T18 · Sat 24 · Vision → gimbal end-to-end · 5 h

- **Camera calibration** if the Sept K isn't real (`config.CAM_K` is still a
  placeholder): checkerboard, OpenCV `calibrateCamera` at **DETECT_RES** (or
  calibrate full-res and halve fx, fy, cx, cy, *only* if the lo-res stream is a scale
  and not a crop; check).
- Wire real mode in `main.py`: `camera.py` + `ESP32Gimbal` + MockMavlink (no
  autopilot needed for the demo).
- **Physical sign test for T3c:** hold the rig, move the marker to the drone's
  right → body-frame y must be positive; forward → x positive.
- **Done when:** hand-held marker → live body-frame position printed with correct
  signs; gimbal search/hold modes driven by the FSM.

### T19 · Sun 25 · Demo path + feature freeze · 4 h

- One-command launchers (`scripts/demo_gimbal.sh`, `scripts/demo_sim.sh`), rerun
  `run_all_experiments`, update the results table. (Could: gimbal-in-the-loop sim.)
- Cold-boot test: power everything off, start from zero using only the README.
- Tag `demo-rc1`. **Feature freeze tonight.**

### T20 · Mon 26 · Rehearsal + fallbacks · 3–4 h

- Two full rehearsals with the team; a written demo script (who says what, 3–5 min).
- Record backup videos of every Must/Should item. Spares: ESP32 (pre-flashed),
  XL-320 (ID and baud already set!), USB cables, jumpers, charged batteries. Hand numbers/figures to Pasha (poster).

### Tue 27 · DEMO

Pre-demo checklist: batteries charged · firmware = `demo-rc1` · gimbal power-up on a
still surface (gyro cal) · logs recording · backup videos on laptop.

---

## 6. Dependencies on partners

| Needed from | What | By |
|---|---|---|
| Pasha | Axis order, servo-ID-to-joint mapping, horn orientation / joint zero, joint limits, camera balance | Fri 9 (info), Thu 22 (hardware) |
| Aga | Pin map sign-off (Pi on UART1 25/26, DIR pull-down, MPU INT wire), servo power (7.4 V, caps, common ground), Pi↔ESP32 harness | Fri 9 (draft), Wed 21 (final) |
| Aga | Progress-report format/deadline | before Wed 14 |
| You → Pasha | Results, plots, architecture diagram | Mon 26 |

## 7. Risks

| Risk | Mitigation |
|---|---|
| frames.py mirror reaches hardware | T3c before any vision-in-the-loop work; physical sign test T18 |
| Gimbal frame late / unbalanced | Single-axis bench rig is the fallback demo; firmware axis-count agnostic |
| Servo overload shutdown (goes limp mid-demo) | Balance, rate limit, Torque Limit, error-bit monitoring + REBOOT recovery, TELEM flag |
| 3S pack on the 7.4 V servo bus | Separate, labelled connector/buck; check Present Voltage at boot, refuse torque outside 6–8.4 V |
| Servo bus too slow / flaky at 1 M | Return Delay 0, Status Return Level 1, Sync Write; fallback 115200 + 50 Hz servo rate |
| Pointing floor from 0.29° ticks + gear backlash | Measure in T12, quote honestly on the poster |
| Writing the bus driver takes > 1 day | Aga's sketch is a working reference; pure-C packet layer host-tested first; T9 has slack |
| AI mixes legacy/new ESP-IDF APIs or Dynamixel control tables | Only `i2c_master.h`; XL-320 addresses only from its own e-manual page |
| `float`/ISR mistakes | `-Wdouble-promotion`; ISR only notifies |
| C and Python maths drift apart | Golden vectors on host + on target; `ctypes` sim uses the C lib |
| IMU vibration on the drone | DLPF, soft mount, accel gating in Mahony |
| Live demo failure | Backup videos, sim-only mode, scripted gimbal motion |
