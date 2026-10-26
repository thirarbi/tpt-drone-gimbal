#pragma once
#include "rot.h"

enum { GIMBAL_YAW = 0, GIMBAL_ROLL = 1, GIMBAL_PITCH = 2 };
#define GIMBAL_IK_OK 0
#define GIMBAL_IK_NEAR_SINGULAR 1

void gimbal_fk(const float q_rad[3], mat3_t * R_body_link);
int gimbal_ik(const mat3_t * R_body_link, float q_rad_out[3]);