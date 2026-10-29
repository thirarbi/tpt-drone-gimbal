#pragma once
#include <stdint.h>
#include "mpu6050.h"
#include "rot.h"

typedef struct {            // written by ctrl_task, read by app_main (dev only, no lock)
    uint32_t n;             // samples read
    uint32_t missed;        // notifications beyond the first
    uint32_t timeouts;      // 5 ms with no data-ready edge
    uint32_t read_err;
    uint32_t read_sum_us;
    uint32_t read_max_us;
    
    // Read + scale + rotate + mahony
    uint32_t loop_sum_us;
    uint32_t loop_max_us;
    mpu6050_raw_t last;     // most recent sample
    quat_t q;
    
} ctrl_stats_t;

extern ctrl_stats_t g_ctrl_stats;
void ctrl_task_start(void);
