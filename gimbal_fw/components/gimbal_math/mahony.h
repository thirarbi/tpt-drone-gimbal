#pragma once
#include "rot.h"

typedef struct {
    quat_t q;
    float integ[3];
    float kp, ki;
    float gate_g;
    float i_max;
} mahony_t;

void mahony_init(mahony_t *m, float kp, float ki);
void mahony_update(mahony_t *m, const float gyro_rads[3], const float acc_g[3], float dt);
void mahony_get_R(const mahony_t *m, mat3_t *R_world_cam);
