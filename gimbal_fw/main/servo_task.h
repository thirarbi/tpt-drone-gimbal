#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "calib.h"

typedef struct {            // written by servo_task, read by the status print (dev only, no lock)
    uint32_t cycles;
    uint32_t overruns;      // cycle took longer than 1/SERVO_RATE_HZ
    uint32_t read_timeouts;
    uint32_t read_bad;      // CRC / malformed reply
    uint32_t cycle_sum_us;
    uint32_t cycle_max_us;
} servo_stats_t;

extern servo_stats_t g_servo_stats;

// Plan §4.6 steps 4-5 (ping, voltage, errors, angle limits, goal = present), then starts the
// servo_task loop. Returns false if no servo answered; the gimbal then runs IMU-only.
bool servo_task_start(const calib_t *cal);

// Ticks <-> joint radians: the only place this conversion lives (§4.3)
uint16_t servo_rad_to_tick(const calib_t *c, int joint, float q_rad);
float    servo_tick_to_rad(const calib_t *c, int joint, uint16_t tick);
