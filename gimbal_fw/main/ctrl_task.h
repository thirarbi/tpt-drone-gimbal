#pragma once
#include <stdint.h>

typedef struct {            // written by ctrl_task, read by log_task (dev only, no lock)
    uint32_t n;             // samples read
    uint32_t missed;        // notifications beyond the first
    uint32_t timeouts;      // 5 ms with no data-ready edge
    uint32_t read_err;
    uint32_t queue_drops;
    uint32_t read_sum_us;
    uint32_t read_max_us;
} ctrl_stats_t;

extern ctrl_stats_t g_ctrl_stats;
void ctrl_task_start(void);
