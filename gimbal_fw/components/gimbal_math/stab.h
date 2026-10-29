// stab_step(): the whole per-tick control law (plan §4.2). Pure function of its inputs,
// so the host test runs the exact same code as the ESP32.
#pragma once
#include <stdint.h>
#include "rot.h"

#define STAB_MODE_IK   0   // main mode: infer base attitude, solve IK for the target
#define STAB_MODE_PID  1   // fallback: per-axis PID on camera roll/pitch error

#define STAB_FLAG_NEAR_SINGULAR  0x01
#define STAB_FLAG_LIMIT          0x02   // at least one joint was clamped to its limit
#define STAB_FLAG_USED_QCMD      0x04   // q_meas invalid, fell back to q_cmd

typedef struct {
    int     mode;
    uint8_t joint_en;           // bit j = joint j has a servo; disabled joints are held at 0
    float   k;                  // IK mode: fraction of the error removed per tick (0..1)
    float   omega_max;          // rad/s, max joint slew
    float   q_min[3], q_max[3]; // rad, indexed GIMBAL_YAW/ROLL/PITCH
    float   pid_kp, pid_ki, pid_kd, pid_i_max;
} stab_cfg_t;

typedef struct {
    mat3_t R_wc;                // camera (link) attitude in the world, from Mahony
    float  q_meas[3];           // measured joint angles, rad
    int    q_meas_valid;
    float  pitch_rad, roll_rad; // target camera tilt, world-referenced (nadir = -pi/2, 0)
    float  yaw_rel_rad;         // target yaw relative to the drone body
    float  dt;                  // s, time since the previous call
} stab_in_t;

typedef struct {
    float q_cmd[3];
    int   flags;
} stab_out_t;

typedef struct {
    stab_cfg_t cfg;
    float q_cmd[3];
    float pid_i[3], pid_prev[3];
} stab_t;

void stab_init(stab_t *s, const stab_cfg_t *cfg, const float q0[3]);
void stab_step(stab_t *s, const stab_in_t *in, stab_out_t *out);
