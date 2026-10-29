Some explanations and notes regarding the files inside this `main/` folder.

`main/` is the only ESP-IDF-specific glue: tasks, boot order, UARTs. All the maths lives in
`components/gimbal_math` so it can be tested on the laptop.

### `app_main.c`
Runs the boot sequence of plan §4.6 in order, then becomes the 1 Hz status printer:
1. `mailboxes_init()` posts the boot target, NADIR.
2. `mpu6050_init`, then `gyro_calibrate` (2 s, retried while the gimbal moves).
3. `ctrl_task_start`, wait 1 s while Mahony converges with the high boot gain.
4. `servo_task_start`: ping, voltage, error check, goal = present position, then the 100 Hz loop.
5. `comms_task_start`: the Pi link.

Any `APP_*` bench mode other than `APP_GIMBAL`/`APP_IMU_ONLY` jumps to `bench_run()` instead and never returns.

### `ctrl_task.h` and `ctrl_task.c`
In INT mode (`MPU_USE_INT 1`) the ISR only notifies and the task takes the notification count. A count `1` is normal and `n>1 ` means that the task was late and some interrupts were missed. Stats live in a shared struct that `app_main` reads for the status line. There should be no logging in the control loop.

Rev 4 (no INT wire): `xTaskDelayUntil(1 ms)` clocks the loop instead. Each tick:
read 14 bytes → subtract gyro and accel offsets (**chip axes**, so before rotating) → `R_imu_cam` →
`mahony_update`. Every `1000 / SERVO_RATE_HZ`-th tick, `stab_tick()`:
- peeks the target and the latest `q_meas`
- seeds `stab_t` from `q_meas` the first time (and the whole time in PARK), so torque-on never jumps
- runs `stab_step` and overwrites the `q_cmd` mailbox
- packs the TELEM flags (`GFLAG_*` in `proto.h`) into `ctrl_out`

`gyro_calibrate` lives here too: mean of 2000 samples, rejected if any axis' std is above
`GYRO_CAL_MAX_STD_RADS`. It computes the variance as `E[x²] − E[x]²`, which is fine in float here
because the bias (~0.02 rad/s) is small.

### `servo_task.c`
`servo_task_start` does the bring-up on the caller's task: ping every joint's ID (a silent one is
dropped from `present`, so its joint is held at 0), reboot on a hardware error, check 6–8.4 V, torque
off, (optionally) write angle limits to EEPROM, then **goal = present position** and a slow Moving
Speed. It posts one `q_meas` before starting the loop. That's what `ctrl_task` seeds from.

The loop, every 10 ms: peek `q_cmd` → SYNC WRITE all goals (one packet, no reply) → toggle torque if
asked (goals first, so torque-on holds the right place) → READ each Present Position → overwrite
`q_meas`. A READ that comes back with a servo error byte still carries valid data; the byte goes into
`err[]` and ends up in TELEM.

`servo_rad_to_tick` / `servo_tick_to_rad` are the only place ticks and radians meet:
`q = dir · (tick − zero_tick) · 300°/1023`.

### `comms_task.c`
Blocks on UART1 until the next TELEM is due, feeding bytes into the `proto` parser. A valid
SET_TARGET overwrites the target mailbox and becomes `ack_seq` (the Pi times the round trip with it).
200 ms without one, after the Pi has spoken at least once, sets target = NADIR with the watchdog flag.

### `gimbal_state.h`
The four mailbox payloads. Each one says who writes it and who reads it.

### `bench_tests.c`
One function per `APP_*` test mode. What each prints and what "good" looks like: `TESTING.md`.

### `config_defaults.h` / `board.h`
Every number and every pin. If a value appears as a literal in a `.c` file, it either belongs here or is a
fixed fact from a datasheet (register address, protocol constant).

Why use .h? Its a C header. It doesn't contain logic and only acts as variable declarations.
