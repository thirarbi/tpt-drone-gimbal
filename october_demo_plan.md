# Software Work Plan — October Demo (8 → 27 Oct 2026)

> **To:** Fathir · **Re:** gimbal firmware + landing_sw for the 27 Oct demo
>
> This is your tasking for the next three weeks. Each task has an estimate, a
> "done when", and tips/cautions from me. Estimates are *focused* hours; hardware
> days usually overrun by ~30%, so the stretch items are your buffer. If a task goes
> **50% over its estimate, stop and flag it** — don't grind silently, we re-plan.
> Phase 0 (6–7 Oct) slipped, which is fine: it's folded into today and Friday.

---

## 0. Ground rules

**Working style**
- Commit at the end of every task, small commits, messages that say *why*. Tag
  `demo-rc1` on Sun 25 and don't touch `main` after that except for bug fixes.
- Every task ends with something you can **show**: a plot, a passing test, a
  motor moving. "I wrote the code" is not done.
- Keep a running `gimbal_fw/NOTES.md`: measured values (resistances, pole pairs,
  offsets, loop times), weird bugs and their causes. This becomes your report
  material and the poster numbers.
- **Units in names**: `pitch_rad`, `rate_dps`, `t_us`, `u_volts`. Radians everywhere
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
    `driver/i2c_master.h`) and **two MCPWM APIs** (legacy `mcpwm_init()` vs v5
    `mcpwm_new_timer()`…). AI happily mixes them. We use **only the new ones**.
    If you see `i2c_master_write_to_device()` or `mcpwm_init()`, reject it.
  - Rotation order and sign conventions (ZXY vs ZYX, FRD vs FLU). Always verify
    numerically against the Python reference, never by reading.
  - MPU6050 clones: register map mostly identical, `WHO_AM_I` differs.
- Paste the actual compiler error / log output into the AI, not your summary of it.

**Datasheets to have open (download today)**
- MPU-6000/MPU-6050 *Product Specification* **and** *Register Map and Descriptions*
- TI DRV8313 datasheet (pin functions, EN/nSLEEP/nRESET/nFAULT, VM range, current limits)
- ESP32 Technical Reference Manual: MCPWM chapter
- ESP-IDF Programming Guide (match your installed version): MCPWM, I2C master,
  UART, GPIO (ISR), NVS, FreeRTOS (task notifications)
- Your gimbal motor's spec sheet (pole pairs, phase resistance, rated current)

**Arduino → ESP-IDF cheat sheet (you'll want this the first week)**

| Arduino | ESP-IDF (C) |
|---|---|
| `setup()` / `loop()` | `app_main()` creates tasks and **returns**; each task is its own `for(;;)` |
| `delay(ms)` | `vTaskDelay(pdMS_TO_TICKS(ms))` (set tick rate to 1000 Hz, see §4.7) |
| `millis()` / `micros()` | `esp_timer_get_time()` → `int64_t` µs |
| `analogWrite()` | MCPWM (our motors) or LEDC |
| `Wire` | `driver/i2c_master.h` |
| `Serial.print` | `ESP_LOGI(TAG, ...)` / `printf` |
| `attachInterrupt()` | `gpio_install_isr_service()` + `gpio_isr_handler_add()` |
| `EEPROM` | NVS (`nvs_flash_init`, `nvs_set_blob`) |
| Busy-wait loop | **Not allowed**: starves the idle task → task watchdog reset. Block on notifications/queues instead |

---

## 1. Demo scope

| Tier | Demo item | Fallback if it slips |
|------|-----------|----------------------|
| **Must** | Gimbal holds camera attitude (nadir) while the base is tilted by hand — IMU + IK on the ESP32 | Single-axis bench rig doing the same |
| **Must** | `landing_sw` closed-loop sim landing live + Bab 4.2 results table | Pre-recorded run + figures |
| **Should** | Pi detects a hand-held ArUco pad → commands gimbal over UART → gimbal tracks / reports angles | Gimbal driven by a scripted angle sequence from the Pi |
| **Could** | Sim landing with the real gimbal in the loop | Skip |

**Cut order if behind** (cut from the top): gimbal-in-the-loop sim → `ctypes`
gimbal sim (T7) → slip detection → active yaw (hold yaw open-loop at 0, demo
roll+pitch only) → protocol speed-ups.

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
                    │                            imu read → mahony → stab_step() → foc ×3
gimbal_iface.ESP32Gimbal                      core 0: comms_task (UART2 parser, telemetry 50 Hz,
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
  - `mode`: `PARK` (motors off), `HOLD` (stabilise to target), `NADIR` (shortcut).
- **ESP32 → Pi `TELEM`** = *where the camera actually is relative to the body*:
  - `q_rad[3]` = **joint angles** (yaw, roll, pitch), body-relative. This is what
    `frames.camera_to_body_rotation()` needs, because the marker must be
    transformed with the *live* camera→body rotation.
  - plus camera attitude from the IMU, status/fault flags, loop-time max, echo of
    last command `seq`.
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

### 2.4 Timing budget

| Item | Rate | Note |
|---|---|---|
| MPU6050 sample (data-ready IRQ) | 1 kHz | Sensor clocks the loop, so no aliasing and no duplicate samples |
| Control loop (`ctrl_task`) | 1 kHz | Budget ≤ 400 µs compute; the 14-byte I²C read alone is a few hundred µs at 400 kHz, so measure it |
| PWM carrier | 25 kHz | Above audible; center-aligned |
| Telemetry ESP32 → Pi | 50 Hz | ~40 B/frame → 2 kB/s, fine at 115200 |
| Commands Pi → ESP32 | 20–50 Hz | Watchdog: 200 ms without a valid frame → `NADIR` hold |
| Dev logging over USB | 200 Hz–1 kHz | Binary at 921600 baud on UART0 |

### 2.5 Pin map (draft: confirm with Aga, assumes ESP32-WROOM-32 DevKit)

| Function | GPIO | Note |
|---|---|---|
| Pitch motor IN1/IN2/IN3 | 25, 26, 27 | MCPWM unit 0 |
| Roll motor IN1/IN2/IN3 | 32, 33, 4 | MCPWM unit 0 |
| Yaw motor IN1/IN2/IN3 | 18, 19, 23 | MCPWM unit 1 |
| DRV8313 EN (shared) | 13 | + external 10 k pull-**down** → motors off during boot/reset |
| DRV8313 nFAULT (wired-OR) | 34 | input-only pin, open-drain fault → external 10 k pull-**up** to 3V3 |
| MPU6050 SDA / SCL | 21 / 22 | short wires (< 20 cm) at 400 kHz |
| MPU6050 INT | 35 | input-only, INT is push-pull active-high by default |
| UART2 RX / TX ↔ Pi | 16 / 17 | Pi GPIO14 TX → ESP 16, Pi GPIO15 RX ← ESP 17, **common GND** |
| Status LED | 2 | onboard on most DevKits |

**Never use:** GPIO 6–11 (flash). **Avoid for outputs:** 0, 2, 5, 12, 15
(strapping; **GPIO 12 high at boot breaks the boot**), 1/3 (USB console).
34–39 are input-only with no internal pull-ups. **If the module is a WROVER**,
GPIO 16/17 are taken by PSRAM, so move UART2. `esptool chip_id` today tells you
what you have.

MCPWM allocation: unit 0 has 3 operators × 2 generators = 6 outputs → pitch
(op0 A, op0 B, op1 A) + roll (op1 B, op2 A, op2 B). All three operators on unit
0's timer 0 → all 6 phases synchronous. Yaw on unit 1. 9 of 12 outputs used.

### 2.6 Power

- DRV8313 VM from the 3S pack (11.1–12.6 V; DRV8313 needs ≥ 8 V). ESP32 from a 5 V
  buck into the DevKit 5V pin. **One common ground** for LiPo−, DRV GND, ESP32 GND,
  Pi GND.
- 100–470 µF electrolytic across VM at each DRV board. The open-loop drive
  draws constant current even when holding still.
- **Do not** power anything from the DRV8313's V3P3OUT pin (≈10 mA max).
- Bench work: lab PSU at 12 V, **current limit 0.5–1 A**, never straight from the LiPo
  until T10 is done.

---

## 3. Code map (what you'll write)

### 3.1 Firmware: `gimbal_fw/` (ESP-IDF project)

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

Key APIs (signatures only, you write the bodies):

- `kinematics.h`: `void gimbal_fk(const float q_rad[3], mat3_t *R_body_link);`
  `int gimbal_ik(const mat3_t *R_body_link, float q_rad_out[3]);` (returns non-zero
  near the roll singularity)
- `mahony.h`: `void mahony_init(mahony_t*, float kp, float ki);`
  `void mahony_update(mahony_t*, const float gyro_rads[3], const float acc_g[3], float dt);`
  `void mahony_get_R(const mahony_t*, mat3_t *R_world_cam);`
- `stab.h`: `void stab_step(stab_t*, const stab_in_t *in, stab_out_t *out);`
  `stab_in_t` = camera attitude, current `q_cmd`, target, dt; `stab_out_t` = new
  `q_cmd`, flags. **Keeping the control law a pure function** is what lets you
  unit-test it on the laptop and call the *exact same code* from the Python sim.
- `mpu6050.h`: `mpu6050_init()`, `mpu6050_read(mpu6050_sample_t*)`,
  `mpu6050_calibrate_gyro(n_samples)`
- `foc.h`: `foc_init(const foc_motor_cfg_t*)`, `foc_enable(bool)`,
  `foc_set_angle(int motor, float mech_rad, float u_volts)`, `foc_fault_active()`
- `proto.h`: `proto_parser_feed(proto_parser_t*, uint8_t byte)` → callback per valid
  frame; `size_t proto_encode(uint8_t type, const void *payload, uint8_t len, uint8_t *out)`;
  `uint16_t crc16_ccitt(const uint8_t*, size_t)`
- `calib.h`: `calib_t { pole_pairs[3], dir[3], zero_offset_rad[3], gyro_bias[3], R_imu_cam }`,
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

1. `R_wb = R_wc · FK(q_cmd)ᵀ`: base attitude estimate (no extra sensor needed)
2. Build target `R_wc*` **using the base's own heading**: `R_wc* = Rz(yaw(R_wb) + yaw_rel) · Ry(pitch_t) · Rx(roll_t)`.
   The MPU6050's yaw drifts, but it drifts *identically* in `R_wc` and `R_wb`, so
   it **cancels**. This is the trick that makes a no-magnetometer IMU usable.
3. `q_ik = IK(R_wbᵀ · R_wc*)`
4. `q_cmd += clamp(K · wrap_pi(q_ik − q_cmd), ±ω_max·dt)`, then clamp to joint limits.

Why `K < 1` (low-pass) and not just `q_cmd = q_ik`: the rotor *lags* the commanded
field, so for a few ms `R_wc` doesn't yet reflect `q_cmd`. Step 1 is then briefly
wrong, and a deadbeat update oscillates. Tune `K`: start at a ~5 Hz equivalent,
raise until it rings, then back off by half.

The **fallback mode** (`STAB_MODE_PID`), worth 1 h to implement: for small angles
each joint mostly corrects its own camera axis, so a plain per-axis PID on camera
roll/pitch error (classic hobby-gimbal style) also works. Implement it so you have
something that stabilises even if the IK loop fights you on the day.

### 4.3 Open-loop "FOC": think microstepping a stepper

You've driven motors before; this is the new concept. A gimbal BLDC driven open-loop
behaves like a **stepper motor with very fine microstepping**: you rotate the
magnetic field and the rotor follows. With *p* pole pairs, one electrical revolution
= 360°/p mechanical (e.g. 7 pole pairs → 51.4°).

```c
// u = phase voltage amplitude (V), vbus = supply (V), duties in [0,1]
float th_e = calib.dir[m] * calib.pole_pairs[m] * mech_rad + calib.zero_offset_rad[m];
d[0] = 0.5f + u * sinf(th_e)               / vbus;
d[1] = 0.5f + u * sinf(th_e - 2.0943951f)  / vbus;   // -2π/3
d[2] = 0.5f + u * sinf(th_e + 2.0943951f)  / vbus;   // +2π/3
// comparator ticks = d[i] * peak_ticks
```

- **Voltage limit `u`** sets torque *and* heat. Phase current ≈ `u / R_phase`, where
  `R_phase` ≈ half the phase-to-phase resistance you measure today. Start at
  `u = 1 V`; aim for < ~0.5 A unless the motor spec says more. Touch-test the motor
  after 5 min holding.
- **Slip**: if the load torque exceeds what `u` gives, the rotor skips to the next
  electrical position, and your joint-angle belief is now off by 360°/p. Causes:
  imbalance, too-fast `ω_max`, too-low `u`. Mitigations: Pasha balances the
  camera, you rate-limit `q_cmd`, and (stretch) detect it via IMU-vs-FK mismatch.
- **No absolute position at boot**: the rotor snaps to the nearest electrical
  position on enable. See §4.6.

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
        /* imu read → mahony → stab_step → foc_set_angle ×3 */
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

1. EN low (motors off). Init NVS, load `calib_t` (if missing → refuse `HOLD`, allow
   calibration only).
2. Init MPU6050; **gyro bias calibration: 2 s, gimbal must be still**; reject if
   gyro variance too high ("someone's touching it") and retry.
3. Mahony converges for ~1 s (high `kp` during start-up, then normal).
4. **Initial joint estimate**: assume the drone is level at power-on (it's on the
   ground). Then the camera tilt from the IMU ≈ joint roll/pitch. Yaw: **require a
   home mark** (the yaw joint is centred at power-on; ask Pasha for a physical
   alignment mark).
5. Set `θ_e` to that estimate, then **ramp `u` from 0 to its target over ~0.5 s**
   → rotor eases in instead of snapping.
6. Start `ctrl_task`, `comms_task`. Mode = `NADIR` until the Pi says otherwise.

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
| Thu 15 | T8 Open-loop drive, one motor (lab) | 5 h |
| Fri 16 | T9 Control task + boot sequence | 4 h |
| Sat 17 | T10 3 motors + NVS calib + on-target tests | 4 h |
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

**T0.3 Inventory + multimeter (20 min)**: photograph every board both sides. Write
in `NOTES.md`: DRV8313 board model and which pins it breaks out (separate EN1-3 or
one EN? nSLEEP/nRESET pulled up on-board?), MPU board model, motor model. **Measure
phase-to-phase resistance of each motor (AB, BC, CA)**: all three should match
within ~5%; a mismatch means a damaged winding. **Pole pairs**: count rotor magnets
÷ 2, or with a scope on two phase wires spin the motor by hand once and count
electrical cycles.

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
  set UART port 2, RX 16, TX 17, 115200. Wire Pi TX→ESP 16, Pi RX←ESP 17, **GND↔GND**.
  Typing in miniterm on the Pi echoes back.
- Both sides are 3.3 V logic, so direct wiring is fine. **Never** connect a 5 V
  USB-serial adapter's TX to either.

**T0.7 (only with a current-limited bench PSU) DRV8313 power-on, no motor (20 min)**:
12 V, limit 0.3 A. Measure: idle current (should be mA), on-board 3.3 V reg,
nFAULT high (no fault), nSLEEP/nRESET high. Don't spin anything today.

**Done when:** `NOTES.md` has chip ID, IMU WHO_AM_I + axis mapping, motor
resistances + pole pairs, UART echo working Pi↔ESP32.
**Watch out:** don't put the IMU on 5 V I²C lines; keep everything 3.3 V logic.
Unplug motor power before rewiring, *every* time.

---

### T1 · Fri 9 · Design freeze + interface contract · 2 h

- Put §2 of this doc into `gimbal_fw/README.md` (you can trim it). Get **axis order,
  motor model, joint mechanical limits, and a yaw home mark** from Pasha; **pin
  map and power** from Aga. Update §2.5 to the real pins.
- Decide joint limits now (e.g. pitch −110°…+20°, roll ±40°, yaw ±170°) because they go in
  the IK clamp and protect the cables.
- **Done when:** README committed; Pasha and Aga have seen the pin map and axis order.

### T2 · Fri 9 · Project skeleton + host test harness · 2 h

- `idf.py create-project gimbal_fw`, create the component folders from §3.1 with
  empty `.c/.h` + `CMakeLists.txt` (`idf_component_register(SRCS ... INCLUDE_DIRS include REQUIRES ...)`),
  `sdkconfig.defaults` (§4.7). Builds and flashes an empty `app_main`.
- `host_test/Makefile` builds `gimbal_math` + a trivial test with `gcc` and runs it.
  `make -C gimbal_fw/host_test` must work from the repo root.
- **Tip:** `gimbal_math` must never `#include` anything from ESP-IDF. The host
  build enforces it for you: if it breaks, you've leaked a dependency.
- **Done when:** both `idf.py build` and `make -C gimbal_fw/host_test` pass.

### T3a · Sat 10 · Kinematics: Python reference + golden vectors · 3 h

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
  drives a simulated base (roll/pitch sinusoid 0.5–3 Hz, ±10°) + simple rotor-lag
  model, calls the *real* `stab_step` → camera pointing error vs time → new Bab 4.2
  row (e.g. RMS < 1°).
- First thing to cut if behind; you can still write `stab.c` on Sun 18.

---

### T8 · Thu 15 (lab) · Open-loop drive, one motor · 5 h

- `foc.c`: MCPWM v5 chain per motor: `mcpwm_new_timer` → `mcpwm_new_operator` +
  `mcpwm_operator_connect_timer` → `mcpwm_new_comparator` (set
  `update_cmp_on_tez = true` for glitch-free updates) → `mcpwm_new_generator` →
  set generator actions for center-aligned PWM. Use ESP-IDF's
  `examples/peripherals/mcpwm/` as the API reference, not the AI's memory.
  ```c
  mcpwm_timer_config_t t = {
      .group_id = 0, .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
      .resolution_hz = 20000000,                // 20 MHz tick
      .count_mode = MCPWM_TIMER_COUNT_MODE_UP_DOWN,
      .period_ticks = 800,                      // target 25 kHz, VERIFY ON SCOPE
  };
  ```
- Sequence: **scope the 3 outputs with no driver connected** (frequency, duty
  follows command, 120° phase shift) → DRV8313 with motor on bench PSU (limit 0.5 A)
  → slow open-loop rotation (`mech_rad += small step` at 1 kHz) → then hold
  commanded angles.
- Find: correct pole pairs (command 360° mech → exactly one turn?), direction sign,
  minimum `u` that holds against a finger, temperature after 5 min.
- **Watch out:** EN low whenever you change wiring. If the motor buzzes but doesn't
  turn → wrong pole pairs or a phase disconnected. Hot within a minute → `u` too high.
- **Done when:** motor goes to commanded angles repeatably, values in `NOTES.md`.

### T9 · Fri 16 · Control task, boot sequence, timing · 4 h

- `ctrl_task` / `app_main` per §4.4 and §4.6 with real IMU + one motor. Loop stats:
  min/avg/max loop time and missed notifications, reported every second via log.
- Stub `comms_task` (watchdog only for now).
- **Done when:** max loop time < 1 ms over 10 min, no task-watchdog resets, boot
  sequence eases the motor in without a jerk.

### T10 · Sat 17 · Three motors + calibration in NVS + on-target tests · 4 h

- Extend `foc` to 3 motors with the MCPWM allocation from §2.5. Scope-check all 9 outputs.
- `calib.c`: `calib_t` in NVS + a tiny calibration console command (or compile-time
  defaults for now): per-axis pole pairs, direction, zero offset.
- Flash a test build that runs the golden-vector kinematics and filter tests **on the
  ESP32** (proves float behaviour matches the laptop).
- **Done when:** 3 motors respond independently; on-target tests pass.

### T11 · Sun 18 · Single-axis stabilisation (pitch) · 4 h

- `stab_step` with only pitch active (other joints held). Hand-tilt the base:
  camera pitch should stay put. Implement the PID fallback mode too (§4.2).
- **Tip:** log `q_cmd`, IMU pitch, target at 200 Hz in every run; tuning without
  plots is guessing.
- **Done when:** ±15° base tilt → camera pitch error < ~2°, logged plot.

### T12 · Mon 19 · Tuning + open-loop characterisation · 4 h

- Tune `K`, `ω_max`, `u`. Characterise: max base tilt rate before slip, holding
  temperature at the chosen `u`, settling time for a 10° step.
- (Stretch) slip detector: when IMU-measured camera tilt disagrees with the expected
  value by > 360°/p/2 → flag + re-home.
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

- Mount, then **with motors off** move each joint by hand through its range and check
  cables. Then one axis at a time: direction sign, zero offset (align to the home
  marks), joint limits in firmware. Save to NVS.
- Verify the **IMU mounting matrix** on the real camera mount (it may differ from
  your bench).
- **Watch out:** first power-up of each axis with low `u` and your hand near the
  power switch. Unbalanced camera = slip, so tell Pasha immediately.
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
  DRV8313, USB cables, jumpers, charged batteries. Hand numbers/figures to Pasha (poster).

### Tue 27 · DEMO

Pre-demo checklist: batteries charged · firmware = `demo-rc1` · gimbal power-up on a
still surface (gyro cal) · logs recording · backup videos on laptop.

---

## 6. Dependencies on partners

| Needed from | What | By |
|---|---|---|
| Pasha | Axis order, motor model/pole pairs, joint limits, **yaw home mark**, camera balance | Fri 9 (info), Thu 22 (hardware) |
| Aga | Pin map sign-off, power plan (buck, caps, common ground), Pi↔ESP32 harness | Fri 9 (draft), Wed 21 (final) |
| Aga | Progress-report format/deadline | before Wed 14 |
| You → Pasha | Results, plots, architecture diagram | Mon 26 |

## 7. Risks

| Risk | Mitigation |
|---|---|
| frames.py mirror reaches hardware | T3c before any vision-in-the-loop work; physical sign test T18 |
| Gimbal frame late / unbalanced | Single-axis bench rig is the fallback demo; firmware axis-count agnostic |
| Open-loop slip → wrong joint belief | Balance, rate limit, sufficient `u`, home mark, (stretch) slip detection |
| Motor/driver heat | Measured `u` limit (T8/T12), bench PSU current limit |
| Writing the drive code takes > 1 day | Open-loop only, no current control; ESP-IDF examples as API reference; T9 has slack |
| AI mixes legacy/new ESP-IDF APIs | Only `i2c_master.h` + `mcpwm_prelude.h`; reject legacy calls on sight |
| `float`/ISR mistakes | `-Wdouble-promotion`; ISR only notifies |
| C and Python maths drift apart | Golden vectors on host + on target; `ctypes` sim uses the C lib |
| IMU vibration on the drone | DLPF, soft mount, accel gating in Mahony |
| Live demo failure | Backup videos, sim-only mode, scripted gimbal motion |
