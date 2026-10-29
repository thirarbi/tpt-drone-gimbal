

### The 7 Design Principles

1. Latency is the benchmark. The system budgets photon-to-setpoint latency (target 33 ms, ceiling 66 ms), not just camera FPS. It's set on `config.LATENCY_ALLOC_MS`, `latency_bench.py`. 
2. **Velocity feedforward**. `v_cmd = Kp.error G_ff.v_target_absolute`. Written in `landing.py`, `config.FEEDFORWARD_GAIN`.

### Reading `config.py`
There are no bare numeric literals anywhere else in the codebase, except here. Everything you tune is in this file, tagged with provenance:
- `# MEASURE`: Must be confirmed against real hardware / calibration (e.g. the camera matrix `CAM_K`,`EKF_MEAS_POS_STD`)
- `# TYPICAL`: Datasheet/handbook stand-in until measured.
- `# DESIGN`: Chosen design value.

---
At first I followed along Claude's recommended workflow and mavlink's workflow of building the systems bit by bit and then creating tests on it. However, I've realized this would take too much time, especially on the encroaching deadline and my approach of studying deep on the system. Tests doesn't help this verymuch. So then I changed it to a more "Regression testing" approach of trial and error. If there's a silent or unknown error, I'd try to reproduce and identify it using the tests.

There's a risk to this however, so I will mark the "critical codes", the ones that'd be costly to reimplement / if anything goes wrong to always be tested first. The rest are just marked as junctions where "a test was supposed to be here, but now skipped".

---
**Some `sdkconfig.defaults` shenanigans**
Updating the file and doing `idf.py reconfigure` doesn't automatically replaces the file (check by `grep -E "FREERTOS_HZ|CPU_FREQ_MHZ=|CONSOLE_UART_BAUDRATE" sdkconfig`). First, the formatting MUST NOT have any spaces. Yes, even between the equals = signs.

IDF 6.1 actually ignores some settings, such as `CONFIG_ESP_CONSOLE_UART_BAUDRATE` if the console type isn't set to CUSTOM. It's written in `compoenents/esp_stdio/Kconfig`:
```
config ESP_CONSOLE_UART_BAUDRATE
    int
    prompt "UART console baud rate" if ESP_CONSOLE_UART_CUSTOM
    default 115200
```
Thus, in `sdkconfig.defaults` it needs to be added a new line:
```
CONFIG_ESP_CONSOLE_UART_CUSTOM=y
```

---
### [Why use ESP-IDF?](https://www.reddit.com/r/esp32/comments/1hyvh8m/hi_what_are_the_benefits_of_using_idf_instead_of/)

So like there was an oversight of using ESP-IDF as a framework for the project without considering *any* alternatives. I thought that this was the *only* way to do it.

The Arduino-ESP32 core is a layer built ontop of ESP-IDF. Their functions call IDF functions underneath, and the `loop()` runs inside a FreeRTOS task. Nothing in this project is say, *impossible* to do in Arduino. The concern is really where the abstraction layer starts costing for each requirements. The plan demands are:
1. **Sensor-clocked 1kHz loop**
   Arduino does not let you create FreeRTOS task, but you'd be using IDF's API through a layer meant to hide it.
2. **Hardware-driven half-duplex servo bus**
   At 1 Mbit/s with Return Delay 0, the servo replies within microseconds. The electronic's sketch toggles `digitalWrite(DIR)` around the write, which makes the software timing too slow and jittery. 

Right now, the call is just to continue using it because changing frameworks would change very little on the pace of the project to be honest. For the requirements, ESP-IDF gives a much better flexibility on the configs, but those are unecessary for the TPT scope.

---
### [What is I2C?](https://docs.arduino.cc/learn/communication/wire/)
It allows you to connect several peripheral devices, such as sensors, displays, motor drivers, and so on, with only a few wires. Giving you lots of flexibility and speeding up your prototyping, without an abundancy of wires. It allows the coexistence of multiple masters and slaves on the same bus,

The I2C (Inter-integrated Circuit) protocol involves using two lines to send and receive data: a serial clock pin **(SCL)** that the board pulses at a regular interval

https://www.youtube.com/watch?v=CAvawEcxoPU&t=14s

![[Pasted image 20261029033245.png]]

A high-level look of I2C frames is the following:
The master claims the bus using the `start` condition. It then sends the address of the slave it wishes to communicate with, and gives an indication either to `read` or `write`. The slave `Ack`nowledge this and then the data is transmitted and `Ack`. The communication is terminated by a `stop` condition.
![[Pasted image 20261029033613.png]]

Our code doesn't *follow* this step, they are generated inside `i2c_master_transmit()` and `i2c_master_transmit_receive()` by the ESP-IDF driver. The code supplies the inputs (address, bytes, direction) and the function call marks the transaction boundary.

**Frame 1:** `write_regs()`
```c
uint8_t buf[2] = { reg, val };
return i2c_master_transmit(s_dev, buf, sizeof buf, I2C_TIMEOUT_MS);
```

```
S | 0xD0 | A | 0x6B | A | 0x80 | A | P
    addr+W    reg       value
```


| I2C element       | Who does it                               | Where it comes from in the code                                                                                                  |
| ----------------- | ----------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------- |
| **Start**         | Driver                                    | Beginning of every `i2c_master_transmit` call                                                                                    |
| **Slave address** | Driver                                    | `.device_address = MPU_ADDR` (0x68) in `dev_cfg`, set once in `mpu6050_init`. The driver shifts it left by 1: `0x68 << 1 = 0xD0` |
| **R/W bit**       | Driver                                    | Implied by the function. `transmit` means **W** (bit = 0), so you never write this bit yourself                                  |
| **Acknowledge**   | Slave pulls SDA low; the driver checks it | No explicit code. A missing ACK makes the call return an error, which `ESP_RETURN_ON_ERROR` catches                              |
| **Data**          | You                                       | `buf[0] = reg`, `buf[1] = val`                                                                                                   |
| **Stop**          | Driver                                    | End of the call                                                                                                                  |
**Frame 2:** `read_regs()`

```c
return i2c_master_transmit_receive(s_dev, &reg, 1, dst, len, I2C_TIMEOUT_MS);
```

Example: `read_regs(0x3B, b, 14)`

```
S | 0xD0 | A | 0x3B | A | Sr | 0xD1 | A | D0 | A | D1 | A | ... | D13 | NA | P
    addr+W      reg         addr+R         b[0]     b[1]          b[13]
```

|I2C element|Who does it|Where it comes from in your code|
|---|---|---|
|**Start**|Driver|Start of the call|
|**Slave address**|Driver|Sent twice, both times from `MPU_ADDR`|
|**R/W bit**|Driver|`transmit_receive` always does a **W phase** (0xD0) and then an **R phase** (0xD1)|
|**Acknowledge (slave)**|Slave|After the address bytes and after the register byte|
|**Data, write phase**|You|`&reg, 1`: the register address you want to read from|
|**Repeated Start (Sr)**|Driver|Between the two phases. The bus is never released, so nothing can interleave|
|**Data, read phase**|Slave sends, driver stores|Bytes land in `dst` (your `b[14]`)|
|**Acknowledge (master)**|Driver|It ACKs every received byte except the last, where it sends **NACK** to tell the slave “that’s enough”|
|**Stop**|Driver||
I2C itself has no concept of registers. From the bus’s point of view, the register number `0x3B` is just the **first data byte** of the write phase. The MPU6050 interprets that byte as “set my internal register pointer here”. That is a chip convention layered on top of I2C, and it is why reading needs the write-then-read pair.

---
GPIO8 in RASPI connects to GPIO26 in ESP
GPIO10 - GPIO25



---
### What is Endianness?
