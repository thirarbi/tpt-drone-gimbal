#include <string.h>
#include "calib.h"

// Compile-time defaults (plan T10: "or compile-time defaults for now").
// Replace with real values from Pasha/measurement, then switch to calib_save()
// once a calibration routine exists.
static const calib_t s_default_calib = {
    .pole_pairs      = {7, 7, 7},                  // TODO(measure): count magnets / 2 per axis
    .dir             = {1, 1, 1},                  // TODO(measure, T8): confirm sign per axis
    .zero_offset_rad = {0.0f, 0.0f, 0.0f},          // TODO(calibrate, T10)
    .gyro_bias       = {0.0f, 0.0f, 0.0f},          // TODO(measured at boot)
    .R_imu_cam       = {                             // TODO(measure T0.5): identity until confirmed
        .m = {
            {1.0f, 0.0f, 0.0f},
            {0.0f, 1.0f, 0.0f},
            {0.0f, 0.0f, 1.0f},
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
