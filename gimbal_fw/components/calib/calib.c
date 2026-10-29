#include <string.h>
#include "calib.h"

// Compile-time defaults (plan T10: "or compile-time defaults for now").
// Replace with real values from Pasha/measurement, then switch to calib_save()
// once a calibration routine exists.
static const calib_t s_default_calib = {
    .version         = 2,
    .servo_id        = {1, 2, 3},                   // indexed yaw, roll, pitch -- TODO(Pasha + T0.3): which ID is which joint
    .dir             = {1, 1, 1},                   // TODO(T16): confirm sign per joint
    .zero_tick       = {512, 512, 512},             // servo centre until T16
    .gyro_bias      = {0.0f, 0.0f, 0.0f},          // TODO(measured at boot)
    .R_imu_cam       = {                             // TODO(measure T0.5): identity until confirmed
        .m = {
            {1.0f,  0.0f,  0.0f},
            {0.0f, -1.0f,  0.0f}, // TEMP bench: 180 deg about x (det = +1)
            {0.0f,  0.0f, -1.0f},
        }
    },
};

bool calib_load(calib_t *out) {
    // TODO: load from NVS once a calibration routine writes real data there.
    // For now, always fall back to compile-time defaults.
    memcpy(out, &s_default_calib, sizeof(*out));
    return false;  // false = defaults used, not a saved calibration
}

void calib_save(const calib_t *in) {
    // TODO: persist to NVS (nvs_set_blob) once calibration console/routine exists.
    (void)in;
}
