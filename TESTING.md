# Hardware tests: bring-up to demo

For the rig as it stands on 30 Oct: ESP32 + MPU6050 + **2 XL-320** on the 74HC126 bus, servo power
switch **OFF**. Work through the steps in order. Each step has an *expect* line. If it doesn't match,
stop there: the next step depends on it.

**How to switch what the firmware does.** Set `APP_MODE` in
[gimbal_fw/main/config_defaults.h](gimbal_fw/main/config_defaults.h), then:

```bash
scripts/fw.sh -p /dev/ttyUSB0 flash monitor      # exit the monitor with Ctrl+]
```

`scripts/fw.sh` is `idf.py` with the right Python env (it avoids the "project was configured with a
different python" error). Console baud is 921600; `idf.py monitor` reads it from sdkconfig.

**Before you power the servos:** use a lab PSU at 7.4 V with a 1–1.5 A current limit, **not** the 3S pack.
Keep a hand near the power switch.

---

## 0. Laptop only (1 min)

```bash
scripts/demo_sim.sh
```
**Expect:** every host test `OK`, `17 passed`, then `touchdown at AGL=0.28 m ... (true)`.

## 1. IMU (`APP_IMU_ONLY`, servo power OFF)

Leave the board still for the first ~2 s (gyro calibration).

**Expect:**
```
ctrl: gyro bias 0.0xxxx ... rad/s (std 0.000xx)
app_main: imu 1000 Hz missed=0 err=0 read ~300/… us loop …/… us | cam roll  +0.x pitch  +0.x deg
```
- Board flat, chip face up, X arrow forward (the bench `R_imu_cam`): roll ≈ 0, pitch ≈ 0.
- Lift the X-arrow end: **pitch goes positive**. Lower the edge opposite the Y arrow (with this
  mounting +Y points left, so that edge is the camera's right): **roll goes positive**.
- `err` climbing = I²C wiring. `read` near 1500 µs = the bus fell back to 100 kHz.
- `gyro cal ... moving` repeating = something is shaking the table.

## 2. Bus wiring (`APP_PIN_TEST`, servo power OFF)

This is the check that failed on 29 Oct. No servo power needed.

**Expect:** four `OK` lines and `pin test PASS`. Each FAIL line names the wire:

| FAIL on | Check |
|---|---|
| buffer ON, TX low/high | GPIO17 → 74HC126 pin 2, GPIO4 → pin 1, pin 14 = 3.3 V, pin 7 = GND |
| buffer OFF → must stay HIGH | 10 k pull-up DATA → 3V3 |
| internal pull-down | GPIO16 is not actually on the DATA net (the 29 Oct symptom) |

## 3. Who is on the bus (`APP_BUS_SCAN`, servo power **ON**)

**Expect:** for each servo `FOUND ID n @ 115200 baud: tick …, 7.x V`. Write the IDs down. The scan
repeats every few seconds, so you can toggle power and watch.

- 0 replies but step 2 passed: servo power really on (the servo LED blinks once at power-up)? Servo
  GND common with the ESP32 GND? DATA reaching the servo connector's data pin?
- Found at another baud: either set `DXL_BAUD` to it, or change the servo back with Aga's sketch.

## 4. Tell the firmware which servo is which joint

Edit `servo_id` in [gimbal_fw/components/calib/calib.c](gimbal_fw/components/calib/calib.c),
ordered **yaw, roll, pitch**. Default `{3, 1, 2}`: on the 2-servo rig yaw (ID 3) is absent, so it
is held at 0 and roll + pitch do the work. That's enough to point at nadir (`test_stab` case 2 checks it).
A joint whose ID doesn't answer at boot is skipped, so `0` or any missing ID means "not fitted".

## 5. Servos move (`APP_SERVO_SWEEP`, servo power ON, horns free to move)

Each joint goes to −20°, +20° and 0° around its `zero_tick`, then all joints together with one SYNC WRITE.

**Expect:** `joint j ID n: ping=0 (… us) read=0 (… us)`, then `err` within about ±3 ticks after each move.
Write the ping/read µs into `gimbal_fw/notes.md` (T8 bus timings).

## 6. Calibrate joints and IMU mounting (`APP_JOINT_MONITOR`, power ON, torque is switched OFF)

Move the joints by hand and read the 5 Hz line `yaw --- | roll  512 = +0.0 deg | pitch … | cam roll … pitch …`.

1. **zero_tick**: hold each joint at its mechanical zero (camera level, bore forward, centred), read
   the tick, put it in `zero_tick`.
2. **dir**: move the joint the positive way (pitch: bore up; roll: camera's right side down; yaw:
   bore turns right). The degrees must increase; if they decrease, set that joint's `dir` to −1.
3. **R_imu_cam**: point the camera straight **down**, base level. Expect **cam pitch ≈ −90, roll ≈ 0**.
   Tilt the bore forward: pitch heads toward 0. If not, fix `R_imu_cam` (recipe in
   [gimbal_fw/README.md](gimbal_fw/README.md#r_imu_cam)). **Don't skip this**: with a wrong
   `R_imu_cam` the gimbal confidently points the wrong way.
4. Joints never show the 60° dead zone (ticks stay within 0–1023 over the whole range).
5. Optional (T10): set `SERVO_WRITE_ANGLE_LIMITS 1` now that `zero_tick`/`dir` are real.

Reflash after editing `calib.c`.

## 7. Stabilisation (`APP_GIMBAL`, power ON)

Keep it still for the first ~3 s after reset (gyro calibration, then Mahony converges).

**Expect at boot:** `joint 1: ID 1, 7.4 V, tick …` per servo (their LEDs turn green), then the servos
walk slowly to nadir. Then once a second:
```
imu 1000 Hz missed=0 … | cam roll +0.x pitch -90.x deg
servo 100 Hz cyc ~7500/… us ovr=0 tmo=0 bad=0 ok=6/6 torque=1 | mode 2 flags 0x003 | q_cmd … | q_meas …
pi rx=0 crc=0 watchdog=0 tx=…
```
`ok=6/6` = joints 1 and 2 (bits) answered. `flags 0x003` = IMU_OK + SERVOS_OK. Other bits:
0x004 stale readback, 0x020 servo error byte, 0x040 torque refused (voltage), 0x080 at a joint limit.

**Test (T11):** tilt the base ±15° slowly, then quicker. The camera pitch stays near −90 and q_cmd moves
against the tilt. For the plot and poster numbers set `LOG_CSV_HZ 50`, reflash, close the monitor, then:
```bash
.venv/bin/python gimbal_fw/tools/serial_log.py record /dev/ttyUSB0 --seconds 30 --out tilt1.csv
.venv/bin/python gimbal_fw/tools/serial_log.py plot tilt1.csv      # prints error RMS / max, writes tilt1.png
```
Done when: camera pitch error < ~2° for ±15° tilt.

| Symptom | Fix |
|---|---|
| Rings / oscillates | `STAB_GAIN_K` 0.3 → 0.15 |
| Sluggish | `STAB_GAIN_K` up to 0.5; `SERVO_SPEED_RUN` 0 (= max) |
| Runs away to a limit | sign error: `dir` or `R_imu_cam` (step 6) |
| `tmo` climbing | bus timing or wiring; with 3 servos at 115200 set `SERVO_RATE_HZ 50` |
| `torque=0`, flags 0x040 | supply outside 6–8.4 V |
| IK loop fights you on the day | `STAB_MODE STAB_MODE_PID` (fallback) |

## 8. Pi ↔ ESP32 link (T13/T14)

Wiring: Pi pin 8 (GPIO14 TX) → ESP32 GPIO26, Pi pin 10 (GPIO15 RX) ← ESP32 GPIO25, **GND ↔ GND**.
Both are 3.3 V, so wire them directly. Never use a 5 V adapter.

Pi, once: in `/boot/firmware/config.txt` add `dtparam=uart0=on`; `sudo raspi-config` → Interface →
Serial: login shell **No**, hardware **Yes**; reboot. Python env (picamera2 comes from apt, so the venv
must see system packages; apt's OpenCV is too old for `cv2.aruco.ArucoDetector`):
```bash
sudo apt install -y python3-picamera2 python3-serial
python3 -m venv --system-site-packages ~/venv && ~/venv/bin/pip install opencv-python-headless
```
Copy the repo over (or `git clone`), then from the repo root:
```bash
~/venv/bin/python -m landing_sw.gimbal_console --watch
```
**Expect:** a status line every second: `NADIR  joints p/r/y  -90.x … | IMU_OK,SERVOS_OK | … | rtt 2x ms`.
Commands: `h -60 0 0` (camera to −60°), `p` (park: servos go limp), `n` (back to nadir).
Cable-pull test: unplug the Pi TX wire. The ESP32 summary shows `watchdog=1` and the gimbal holds nadir.
Write the `rtt` into notes (T14 latency).

No TELEM: TX/RX swapped, GND missing, or the firmware is not in `APP_GIMBAL`.

## 9. Camera + marker (T15/T18), ArduCam HQ IMX477 or OV9281

Print marker **ID 7** from `DICT_5X5_100` so that the black square is exactly **100 mm**
(`INNER_MARKER_SIDE_M`; distances scale with it):
```bash
~/venv/bin/python -c "import cv2; d=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100); cv2.imwrite('marker7.png', cv2.aruco.generateImageMarker(d, 7, 800))"
```
1. Camera alone: `scripts/demo_pi.sh imx477 --no-gimbal` (use `~/venv/bin/python` if `python3` isn't the venv).
   **Expect:** `id  7  body x … y … z … m | reproj 0.xx px | … | NN ms` and about 30 fps.
2. Sign check, camera pointing down: move the marker toward the drone's **right** → **y > 0**,
   **forward** (image top) → **x > 0**, distance below → z > 0.
3. Distances off? The IMX477 `K` is a 6 mm-lens estimate. Calibrate with a printed checkerboard
   (9×6 inner corners, 25 mm squares), then paste into `IMX477_K` / `IMX477_DIST` in `landing_sw/config.py`:
   `~/venv/bin/python -m landing_sw.calibrate_camera --camera imx477`
4. With the gimbal: `scripts/demo_pi.sh imx477`. `gimbal p/r/y` now shows the **measured** joint angles,
   and the body-frame position stays put when you tilt the gimbal base.

For the OV9281 it's the same with `ov9281`.

## Demo run order

1. Laptop: `scripts/demo_sim.sh` (Must: sim landing + tests).
2. Gimbal: power on still, wait 3 s, tilt the base by hand (Must: holds nadir).
3. Pi: `scripts/demo_pi.sh imx477` with the hand-held marker (Should).

Record a backup video of 2 and 3 as soon as they work.

## Numbers to write down (report / poster)

| What | Where it shows |
|---|---|
| IMU rate, read time, loop max | step 1 / 7 summary line |
| PING / READ round trip, SYNC WRITE time | step 5 |
| Servo cycle time, timeouts | step 7 `servo` line |
| Camera error RMS / max under ±15° tilt | step 7 `serial_log.py plot` |
| Command → TELEM round trip | step 8 `rtt` |
| Photon → pose latency, fps | step 9 |
