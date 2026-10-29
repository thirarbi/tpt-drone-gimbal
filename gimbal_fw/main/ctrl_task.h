#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "calib.h"
#include "mpu6050.h"
#include "rot.h"

typedef struct {            // written by ctrl_task, read by app_main (dev only, no lock)
    uint32_t n;             // samples read
    uint32_t missed;        // INT mode: notifications beyond the first. Polled mode: late 1 ms wake-ups
    uint32_t timeouts;      // INT mode only: 5 ms with no data-ready edge (always 0 when polled)
    uint32_t read_err;
    uint32_t read_sum_us;
    uint32_t read_max_us;

    // Read + scale + rotate + mahony (+ stab_step on every 10th tick)
    uint32_t loop_sum_us;
    uint32_t loop_max_us;
    mpu6050_raw_t last;     // most recent sample
    quat_t q;

} ctrl_stats_t;

extern ctrl_stats_t g_ctrl_stats;

// §4.6 step 2: 2 s still -> cal->gyro_bias. Retries while moving; false = kept the mpu_unit.h fallback.
bool gyro_calibrate(calib_t *cal);

// cal: gyro bias already measured. run_stab = false for IMU-only modes (no stab_step, no q_cmd).
void ctrl_task_start(const calib_t *cal, bool run_stab);
