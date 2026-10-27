Pasha's answers only affect a few constants, not the code structure. In §6 of the plan he owes you five things: axis order, servo-ID-to-joint mapping, horn zero (which tick is joint zero), joint limits, and camera balance. Everything below avoids depending on those.

## Do now

| Task | What you can do | Why Pasha isn't needed |
|---|---|---|
| **T3b leftovers** | Commit it. Delete the duplicate `test:` block in the Makefile. | Pure housekeeping. |
| **T3c** steps 1, 3, 5 | Write the determinant test (the file you just opened). Rewrite `camera_to_body_rotation` as `gk.fk(...) @ R_LINK_OPTICAL`. Add the consistency test. | The mounting lives in **one named constant**. The tests are written against that constant, not a specific value, so they stay valid when Pasha answers. |
| **T3c** step 4 (sim) | Make the sim agree with the provisional mount. | The sim only needs to be internally consistent. Real-world correctness is checked later in T18. |
| **T1** (partly) | Draft `gimbal_fw/README.md` from §2. Leave the axis order, servo-ID mapping and horn zero as `TBC (Pasha)`. | You still own the interface contract, timing and pin map (Aga's part). |
| **T4** | MPU6050 driver, 1 kHz data-ready loop, logger, the two logs. | Needs only the ESP32 and the IMU. If you have them at hand this is the biggest block of real progress. |
| **T5** | Mahony in Python, then C, with host tests. | Runs on T4's logs. Keep `R_imu_cam` as identity for now. |
| **T7** | `stab.c`, then the `ctypes` sim. | Pure maths on FK/IK, which you now have working. |
| **T8 (laptop half)** | `dxl_packet.c`: `crc16_dxl`, `dxl_build`, `dxl_parser_feed`, plus host tests. | Pure C. Only the "scope it on the real bus" half needs the lab. |
| **T13 (laptop half)** | `proto.c`, `gimbal_proto.py`, pytest round-trips. | Pure C and Python. |

## Provisional choices to make for T3c

Pick the upright-camera mount (`R_LINK_OPTICAL = [[0,0,1],[1,0,0],[0,1,0]]`) and mark it clearly:

- A `# PROVISIONAL: confirm with Pasha (T1) and physically in T18` comment next to the constant.
- A line in `NOTES.md`.

If Pasha says the camera is mounted differently, you change one constant and one sim bookkeeping line. That is the argument for the refactor in my earlier message (let the synthetic camera derive its pose from `frames.py`). With it, the sim follows automatically and there are no hand-matched axes to redo.

## Wait for Pasha

- **T16, T17:** the real gimbal.
- Real `zero_tick` and servo-ID values for `calib_t`. Use compile-time placeholders in T10 for now.
- Final joint `LIMITS`. The current ones are marked as placeholders in `gimbal_kinematics.py`.
- The physical sign test in T18, which is the final check on the mounting.

## Questions to send Pasha when you reach them

1. Camera mounting: with all joints at zero (bore forward), is the top of the image up? Which way is the image-top at nadir?
2. Chain order: yaw motor on the frame, then roll, then pitch holding the camera?
3. Servo ID to joint, and which tick is joint zero for each horn?
4. Mechanical limits per joint, and whether the camera is balanced.

## Suggested order

T3c, then commit, then T4 or T5 if you have the board, otherwise T7 or T8's laptop half. T7 is first on the plan's cut list, so don't spend long on it. T8's `dxl_packet.c` and T13's `proto.c` are not cut candidates and are the better use of waiting time.

One last thing: the table in the plan puts T3b on Sun 11, but your files are dated 26 Oct and the plan header says the demo is 27 Oct. If the demo is tomorrow rather than weeks away, tell me, because that changes what to cut. The "Must" gimbal-holds-attitude item needs T4, T5, T8–T11 and a real gimbal.