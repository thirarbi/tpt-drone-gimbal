// Per-unit calibration data, persisted in NVS (plan §3.1).
// Compile-time defaults below are placeholders until T10 measurement/calibration.
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "kinematics.h"   // GIMBAL_YAW/ROLL/PITCH: the ONLY joint index order

typedef struct {
    uint16_t version;       // bump when the layout changes: an old NVS blob must not load into a new struct
    uint8_t  servo_id[3];   // Dynamixel ID per joint
    int8_t   dir[3];        // +1/-1: joint sign vs tick direction
    uint16_t zero_tick[3];  // Present Position at joint zero, ~512
    float    gyro_bias[3];
    mat3_t   R_imu_cam;
} calib_t;


bool calib_load(calib_t *out);
void calib_save(const calib_t *in);
