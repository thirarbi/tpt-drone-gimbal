
## 1. Explicitly marked as lab
| Task                                        | Why it must be in the lab                                                                                                                                         |
| ------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **T0** (smoke tests)                        | First contact with the hardware: servo IDs and baud, the scope trace of DIR against DATA (T0.7), IMU axis mapping, Pi↔ESP32 UART.                                 |
| **T8** (XL-320 driver)                      | The first half is laptop-only: `dxl_packet.c` and its host tests. The second half needs the servo, a scope or logic analyser, and the baud/Return Delay speed-up. |
| **T16** (3-axis bring-up on Pasha's gimbal) | Needs the real gimbal. You find `zero_tick` and direction signs, check the 60° dead zone, and confirm the IMU mounting matrix.                                    |

## 2. Not labelled, but hardware-bound
These are where the real **tuning** happens:

- **T9 and T10** need the ESP32 with the IMU and a servo, then all three servos. They check loop timing, servo cycle time, Sync Write and the NVS calibration. The doc doesn't say "lab", but you can't do them without servos on a bench supply.
- **T11, T12 and T17** are the core tuning tasks. They cover tuning `K`, `ω_max` and the servo P gain, step response, backlash, dead time, thermal and load behaviour, and the pointing-error RMS under hand disturbance. T11 and T12 can run on a single servo on a bench. T17 needs the full gimbal.
- **T13 and T14** are mostly laptop and Pi work. The final "Done when" (`g.set_angles(-90,0,0)` moves the gimbal) needs the servos, and the cable-pull test needs the real link.
- **T15 and T18** need the Pi 5 and the OV9281. T18 also needs the physical sign test for T3c, where you move the marker to the drone's right and check that body y > 0. The checkerboard calibration for the real `CAM_K` is also physical.
- **T19 and T20** are integration and rehearsal on the real rig.

## 3. Laptop or home only
- **T1 and T6** are documents and conversations with Pasha and Aga.
- **T2, T3a, T3b and T3c** are code and tests. T3c's final physical check is deferred to T18.
- **T4 and T5** need only the ESP32 and the MPU6050, per the plan's "take it home" note.
- **T7** is the ctypes sim.
- The host-side halves of T8 and T13: `dxl_packet`, `proto.c` and the pytest suites.