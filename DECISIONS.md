## T4 decisions: MPU6050 driver, 1 kHz loop, logger

Date: 2026-10-28. Sources: `october_demo_plan.md` T4 and §4.4, the MPU-6000/6050
Register Map and Product Specification, ESP-IDF 6.1 headers and Kconfig.

Register values below were checked against the PDF (all nine bytes and both scales
matched). Page numbers are from that check.

### 1. Build configuration (stage A)

| Decision | Why |
|---|---|
| `sdkconfig.defaults` has **no spaces around `=`** | With `CONFIG_X = y` the parser reads the symbol as `X ` (trailing space), reports `unknown kconfig symbol`, and drops the line. The build still passes, so the only symptom is the wrong value in `sdkconfig`. |
| `CONFIG_FREERTOS_HZ=1000` | At the default 100 Hz one tick is 10 ms, so `pdMS_TO_TICKS(5)` is 0 ticks and the "IMU dead after 5 ms" timeout in §4.4 would not wait at all. |
| Console baud needs `CONFIG_ESP_CONSOLE_UART_CUSTOM=y` first | In IDF 6.1 `ESP_CONSOLE_UART_BAUDRATE` has a prompt only when the console is "custom UART". Without it, the assignment is ignored silently. TX/RX stay on GPIO 1/3. |
| An existing `sdkconfig` beats `sdkconfig.defaults` | After editing the defaults: `rm sdkconfig && idf.py reconfigure`, then grep to confirm. |

Final `sdkconfig.defaults`:

```
CONFIG_FREERTOS_HZ=1000
CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ_240=y
CONFIG_COMPILER_OPTIMIZATION_PERF=y
CONFIG_ESP_CONSOLE_UART_CUSTOM=y
CONFIG_ESP_CONSOLE_UART_NUM=0
CONFIG_ESP_CONSOLE_UART_BAUDRATE=921600
```

Check: `grep -E "FREERTOS_HZ|CPU_FREQ_MHZ=|CONSOLE_UART_BAUDRATE" sdkconfig`
must show 1000 / 240 / 921600. If a CH340 board is unreliable at 921600, 460800
still carries the 21 kB/s log stream.

## 2. Driver structure

- New component at `gimbal_fw/components/mpu6050/` (CMakeLists.txt, mpu6050.c, mpu6050.h).
  Add `mpu6050` to `REQUIRES` in `main/CMakeLists.txt`.
- `mpu6050_init(sda, scl, hz)` takes the pins as arguments. A component cannot
  include `main/board.h`.
- The driver owns the sensor ranges. The register value and the LSB scale change
  together, so they live in one file. The `MPU_*_RANGE` macros in `config_defaults.h`
  are redundant: delete them or keep them as documentation.
- `mpu6050_read_raw()` returns the 14-byte burst as `int16` (the logger stores raw
  counts). `mpu6050_scale()` converts to g and rad/s (Mahony in T5 uses those).
- I²C API: only `driver/i2c_master.h` (component `esp_driver_i2c`). No legacy
  `i2c_param_config()` / `i2c_master_write_to_device()`.

## 3. Register values (verified against the PDF)

| Register | Addr | Value | Meaning |
|---|---|---|---|
| SMPLRT_DIV | 0x19 | 0x00 | 1 kHz / (1 + 0), valid only with the DLPF on |
| CONFIG | 0x1A | 0x02 | DLPF_CFG = 2 (~94 Hz, ~3 ms delay) |
| GYRO_CONFIG | 0x1B | 0x10 | FS_SEL = 2 (±1000 °/s) |
| ACCEL_CONFIG | 0x1C | 0x08 | AFS_SEL = 1 (±4 g); bits [2:0] stay 0 |
| INT_PIN_CFG | 0x37 | 0x00 | active high, push-pull, 50 µs pulse |
| INT_ENABLE | 0x38 | 0x01 | DATA_RDY_EN |
| ACCEL_XOUT_H | 0x3B | 14-byte burst | accel 6, temp 2, gyro 6, big-endian |
| PWR_MGMT_1 | 0x6B | 0x80, then 0x01 | reset, then wake with CLKSEL = 1 (PLL, X gyro) |
| WHO_AM_I | 0x75 | 0x68 | part identity, not the bus address (reads 0x68 even with AD0 high) |

Init order: reset, wait, wake, `WHO_AM_I`, `CONFIG` / `SMPLRT_DIV` / `GYRO_CONFIG` /
`ACCEL_CONFIG`, **`INT_ENABLE` last** (it resets to 0, and enabling it last means
nothing interrupts before the sensor is configured).

Always use the 14-byte burst. The PDF only guarantees a single sampling instant for
burst reads. Decode as `(int16_t)(((uint16_t)H << 8) | L)`.

## 4. Decisions made after the register check

1. **Gyro sensitivity: 32.8 LSB/(°/s)** (the datasheet value; the exact figure is
   32.768). The 0.1 % difference is far below the sensor's own tolerance. Firmware
   and `tools/serial_log.py` must use the same number. Accel: 8192 LSB/g.
2. **Reset wait: plain 100 ms delay**, not polling `PWR_MGMT_1`. The 100 ms figure
   comes from the SPI note in the PDF (p.41), not from an I²C requirement. It relies
   on the 1000 Hz tick rate from §1. Cite it that way in the report.
3. **400 kHz is checked by measurement**, not assumed. One burst is about 0.4 ms at
   400 kHz and about 1.5 ms at 100 kHz, so a read time near 1.5 ms means the bus
   fell back and samples will be missed.
4. **The pulse-versus-status question is answered by the rate counter**, no scope
   needed. If the rate holds at 1000 Hz, pulse mode works. Only if it stops or drops:
   - set `INT_RD_CLEAR` (bit 4 of `INT_PIN_CFG`, so 0x10; verify in the PDF), or
   - read `INT_STATUS` (0x3A) each cycle (costs one extra transaction).

   Do not add either preemptively.
5. **Interrupt on the rising edge** (`GPIO_INTR_POSEDGE`). The pulse is 50 µs, so
   polling the GPIO level would miss it.
6. **Accel scale sanity check at rest:** flat, raw Z should read about ±8192 counts.
   About 4096 or 16384 means the `ACCEL_CONFIG` write did not take or the part is a
   clone that passes `WHO_AM_I`. Look at the raw counts, not only the scaled value.

## 5. Loop and logger design

- The sensor clocks the loop. The ISR only calls `vTaskNotifyGiveFromISR`; no maths,
  no logging in the ISR or in `ctrl_task`.
- `ulTaskNotifyTake(pdTRUE, ...)` returns the count since the last take: `1` is
  normal, `n > 1` adds `n - 1` to the `missed` counter, `0` is a timeout.
- Start order: create the task (so the handle is valid), then enable the GPIO
  interrupt.
- `ctrl_task` pinned to core 1; `log_task` on core 0 at low priority. They talk
  through a queue (about 1024 frames, roughly 1 s of buffering). Queue-full is counted
  as `queue_drops`.
- Two modes behind one switch (for example `T4_STREAM` in `config_defaults.h`):
  measure mode prints stats once a second, record mode streams binary and silences
  logs. Never mix text and binary on one stream.
- Log frame, little-endian, 21 bytes, about 21 kB/s (23 % of 921600 baud):
  `[0xA5][0x5A][t_us u32][7 x int16: acc xyz, temp, gyro xyz][xor u8]`.
  The Python reader resyncs on the sync bytes and the checksum, so garbage before the
  first valid frame (boot messages) is harmless.
- The timestamp is taken when the task wakes. If the timestamps look jittery in the
  plot, move the `esp_timer_get_time()` call into the ISR.

## 6. Done when

- Measured rate 1000 ± 5 Hz, `missed` = 0.
- Mean and max read time in µs recorded (budget ≤ 400 µs, §2.4).
- Two 60 s logs saved: one still, one hand-tilted ending in the starting pose.
- Still-log numbers written in `NOTES.md`: gyro bias and std per axis, accel std,
  mean |a|, read time.

Commit message: "T4: MPU6050 driver, data-ready-driven 1 kHz loop, binary dev logger
(the sensor clocks the loop, so no aliasing)".
