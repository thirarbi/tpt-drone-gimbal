#pragma once
#include <stdint.h>

typedef struct {            // written by comms_task, read by the status print (dev only, no lock)
    uint32_t rx_frames;     // valid SET_TARGET frames
    uint32_t crc_errors;
    uint32_t watchdog_trips;
    uint32_t tx_frames;
} comms_stats_t;

extern comms_stats_t g_comms_stats;

// UART1 to the Pi: SET_TARGET in, TELEM out at TELEM_RATE_HZ, watchdog -> NADIR
void comms_task_start(void);
