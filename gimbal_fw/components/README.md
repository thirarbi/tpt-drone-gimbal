# components/

Rule: anything marked PURE C includes no ESP-IDF header. `host_test/Makefile` compiles those files
with plain `gcc -Wall -Wextra -Werror -Wdouble-promotion -Wconversion`; if one of them leaks an ESP-IDF
dependency, the host build breaks. That is the point.

## gimbal_math/ (PURE C)

| File | What |
|---|---|
| `rot.c/.h` | `mat3_t`, `quat_t`, `rot_x/y/z`, `mat3_mul`, `mat3_transpose`, `mat3_vec`, `wrap_pi` |
| `kinematics.c/.h` | `gimbal_fk` = `Rz(yaw)·Rx(roll)·Ry(pitch)`, `gimbal_ik` (§4.1), singular at roll ±90° |
| `mahony.c/.h` | quaternion attitude filter with accel gating and integral clamp (`tools/MAHONY-FILTER.md`) |
| `stab.c/.h` | `stab_step()`: the whole per-tick control law. Two modes: |

- **`STAB_MODE_IK`** (main): infer the body attitude, build the target with the body's heading, IK,
  then `q_cmd += clamp(K · wrap_pi(q_goal − q_cmd), ±ω_max·dt)`, clamped to joint limits. The steps
  are commented 1–4 to match plan §4.2.
- **`STAB_MODE_PID`** (fallback): small-angle shortcut. The roll joint corrects camera roll and the pitch
  joint corrects camera pitch, each with its own PID on the IMU error. It needs no kinematics, so it is
  the thing to switch to if the IK loop misbehaves on the day.

`stab_step` takes everything through `stab_in_t` and keeps its only state (`q_cmd`, PID integrators)
in `stab_t`, so the same call can be tested in `host_test/test_stab.c` with a simulated base.
`joint_en` is a bitmask: a disabled joint is held at exactly 0, and the IK uses 0 for it. That is how
the 2-servo rig works.

## xl320/

| File | What |
|---|---|
| `dxl_packet.c/.h` (PURE C) | `crc16_dxl` (CRC-16/BUYPASS), `dxl_build`, byte-wise status parser that drops our own echo |
| `xl320.c/.h` | UART2 in RS485 half-duplex mode (the UART drives DIR = RTS), `ping`, `read_u8/u16`, `write_u8/u16`, `sync_write_goal`, `torque`, `reboot`, control-table addresses |

`txrx()` is the core: build → flush RX → write → (if a reply is expected) wait until the bytes have
left → feed RX bytes to the parser until a status packet from the right ID arrives or 5 ms pass.
Return: 0 OK, the servo's error byte (> 0), or `XL320_ERR_*` (< 0). See `README` for what a CRC is.

## proto/ (PURE C)

The Pi frame (§4.5): `[A5 5A][type][len][seq u16][payload][crc u16]`, CRC-16/CCITT-FALSE, which Python
gets for free as `binascii.crc_hqx(data, 0xFFFF)`. `msg_set_target_t` and `msg_telem_t` are packed
structs; `landing_sw/gimbal_proto.py` has the matching `struct` format strings next to them.
`host_test/test_proto.c` writes `golden_proto.txt`, and a pytest decodes it. That catches the silent
failures: wrong CRC convention, or two struct fields swapped. Changing a struct means changing both
sides and bumping `PROTO_VERSION`.

## mpu6050/

New `driver/i2c_master.h` API only. Register values were checked against the register map (see
`DECISIONS.md`). One 14-byte burst per sample, big-endian → `int16`. `mpu6050_scale` → g and rad/s.

## calib/

`calib_t`: servo ID per joint, direction, zero tick, accel and gyro offsets, `R_imu_cam`. For now the
values are compile-time defaults (`calib.c`). The accel offsets come from `mpu_unit.h`, which belongs
to one physical MPU board (`MPU-CALIBRATION.md`).
