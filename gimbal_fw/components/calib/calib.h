// Per-unit calibration data (plan §3.1). Compile-time defaults for now (plan T10 allows it);
// NVS persistence is not implemented yet.
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "kinematics.h"   // GIMBAL_YAW/ROLL/PITCH: the ONLY joint index order

typedef struct {
    uint16_t version;       // bump when the layout changes: an old NVS blob must not load into a new struct
    uint8_t  servo_id[3];   // Dynamixel ID per joint; a joint whose servo does not answer at boot is held at 0
    int8_t   dir[3];        // +1/-1: joint sign vs tick direction
    uint16_t zero_tick[3];  // Present Position at joint zero, ~512
    float    acc_bias[3];   // g, chip axes, subtracted BEFORE R_imu_cam (MPU-CALIBRATION.md)
    float    gyro_bias[3];  // rad/s, chip axes; replaced by the boot calibration
    mat3_t   R_imu_cam;     // v_cam = R_imu_cam * v_imu
} calib_t;


bool calib_load(calib_t *out);
void calib_save(const calib_t *in);
