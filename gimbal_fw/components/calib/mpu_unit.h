// Corrections for ONE physical MPU6050 board. Swap the board -> re-measure -> edit ONLY this file.
// Procedure and history: gimbal_fw/MPU-CALIBRATION.md. Used by calib.c as compile-time defaults.
// Values are in the CHIP's own axes (the silkscreen X/Y arrows), before R_imu_cam is applied.
#pragma once

#define MPU_UNIT_ID   "bench board #1"

// Accel counts per g at +-4 g. Must equal ACC_LSB_PER_G in components/mpu6050/mpu6050.c.
#define MPU_ACC_NOMINAL_LSB_PER_G   8192.0f

// Accel zero-g offset, counts, X Y Z: copy the "offset=" numbers from the AXIS lines of the
// six-pose test. Z is solid. X and Y are PROVISIONAL (poses 3-5 were hand-held and moved).
#define MPU_UNIT_ACC_OFFSET_X_COUNTS    196.0f
#define MPU_UNIT_ACC_OFFSET_Y_COUNTS   -115.0f
#define MPU_UNIT_ACC_OFFSET_Z_COUNTS  -1699.0f

// Derived (do not edit): the same offsets in g, which is what calib_t.acc_bias holds.
#define MPU_UNIT_ACC_BIAS_G  { MPU_UNIT_ACC_OFFSET_X_COUNTS / MPU_ACC_NOMINAL_LSB_PER_G, \
                               MPU_UNIT_ACC_OFFSET_Y_COUNTS / MPU_ACC_NOMINAL_LSB_PER_G, \
                               MPU_UNIT_ACC_OFFSET_Z_COUNTS / MPU_ACC_NOMINAL_LSB_PER_G }

// Gyro bias, rad/s, X Y Z. Not measured yet. T9's boot calibration will replace it at runtime,
// so this is only the start-up fallback.
#define MPU_UNIT_GYRO_BIAS_RADS     { 0.0f, 0.0f, 0.0f }
