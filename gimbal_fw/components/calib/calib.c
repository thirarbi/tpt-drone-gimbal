#include <string.h>
#include "calib.h"
#include "mpu_unit.h"

// Compile-time defaults (plan T10: "or compile-time defaults for now").
// TESTING.md step 3 (bus scan) tells you the IDs; step 5 (joint monitor) gives dir and zero_tick.
static const calib_t s_default_calib = {
    .version         = 3,
    .servo_id        = {3, 1, 2},                   // yaw, roll, pitch. 2-servo rig: yaw (ID 3) absent -> held at 0
    .dir             = {1, 1, 1},                   // TODO(T16): confirm sign per joint
    .zero_tick       = {512, 512, 512},             // servo centre until T16
    .acc_bias        = MPU_UNIT_ACC_BIAS_G,
    .gyro_bias       = MPU_UNIT_GYRO_BIAS_RADS,     // fallback only, measured at boot
    .R_imu_cam       = {                             // TODO(T16): measure on the real camera mount
        .m = {
            {1.0f,  0.0f,  0.0f},
            {0.0f, -1.0f,  0.0f}, // TEMP bench: chip flat, face up, X forward = 180 deg about x (det = +1)
            {0.0f,  0.0f, -1.0f},
        }
    },
};

bool calib_load(calib_t *out) {
    // TODO: load from NVS once a calibration routine writes real data there.
    memcpy(out, &s_default_calib, sizeof(*out));
    return false;  // false = defaults used, not a saved calibration
}

void calib_save(const calib_t *in) {
    // TODO: persist to NVS (nvs_set_blob) once calibration console/routine exists.
    (void)in;
}
