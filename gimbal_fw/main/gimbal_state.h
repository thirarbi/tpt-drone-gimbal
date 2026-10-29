// Data passed between tasks. Each is a 1-slot mailbox (plan §4.4): the writer calls
// xQueueOverwrite, readers call xQueuePeek(..., 0). Readers always get the latest value
// and nobody blocks or needs a mutex.
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

typedef struct {            // comms_task -> ctrl_task
    uint8_t mode;           // GMODE_* (proto.h)
    float   pitch_rad, roll_rad, yaw_rel_rad;
    bool    watchdog;       // true = set by the watchdog, not by the Pi
} target_t;

typedef struct {            // ctrl_task -> servo_task
    float q_rad[3];
    bool  torque;
} qcmd_t;

typedef struct {            // servo_task -> ctrl_task, comms_task
    float   q_rad[3];       // measured joint angles
    int64_t t_us;           // when the reads finished
    uint8_t present;        // bit j = joint j has a servo that answered at bring-up
    uint8_t ok;             // bit j = joint j answered in this cycle
    uint8_t err[3];         // last servo error byte per joint
    bool    torque_allowed; // false = bring-up failed or voltage out of range
    bool    torque_on;
} qmeas_t;

typedef struct {            // ctrl_task -> comms_task, status print
    float    cam_roll_rad, cam_pitch_rad;
    float    q_cmd[3];
    uint16_t flags;         // GFLAG_* (proto.h)
    uint8_t  mode;
} ctrl_out_t;

extern QueueHandle_t g_target_mb, g_qcmd_mb, g_qmeas_mb, g_ctrl_mb;

void mailboxes_init(void);   // also posts the boot target: NADIR
