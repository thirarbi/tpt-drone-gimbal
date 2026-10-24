// Per-unit calibration data, persisted in NVS (plan §3.1).
// Compile-time defaults below are placeholders until T10 measurement/calibration.
#pragma once

#include <stdbool.h>
#include "rot.h"  // mat3_t

typedef enum { AXIS_PITCH = 0, AXIS_ROLL = 1, AXIS_YAW = 2 } axis_t;

typedef struct {
    int   pole_pairs[3];       // per axis [pitch, roll, yaw]; count rotor magnets / 2 -- TODO(measure, T0.3)
    int   dir[3];              // +1 or -1 per axis; motor direction sign -- TODO(measure, T8)
    float zero_offset_rad[3];  // electrical zero offset per axis -- TODO(calibrate, T10)
    float gyro_bias[3];        // gyro bias [x, y, z], rad/s -- TODO(measured at boot, 2s still)
    mat3_t R_imu_cam;          // IMU -> camera mounting matrix -- TODO(measure T0.5, confirm with Pasha's CAD)
} calib_t;

bool calib_load(calib_t *out);
void calib_save(const calib_t *in);
