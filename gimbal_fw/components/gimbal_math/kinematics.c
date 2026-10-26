#include "kinematics.h"
#include <math.h>

void gimbal_fk(const float q[3], mat3_t * R) {
    mat3_t rz = rot_z(q[GIMBAL_YAW]);
    mat3_t rx = rot_x(q[GIMBAL_ROLL]);
    mat3_t ry = rot_y(q[GIMBAL_PITCH]);
    mat3_t rzx = mat3_mul(&rz, &rx);
    *R = mat3_mul(&rzx, &ry);
}

int gimbal_ik(const mat3_t * R, float q[3]) {
    float s_roll = fminf(fmax(R -> m[2][1], -1.0f), 1.0f); // Sice asinf(1.0000001f) = NaN)
    q[GIMBAL_ROLL] = asinf(s_roll);
    q[GIMBAL_YAW] = atan2f(-R -> m[0][1], R -> m[1][1]);
    q[GIMBAL_PITCH] = atan2f(-R -> m[2][0], R -> [2][2]);
    
    return fabsf(s_roll) > 0.995f ? GIMBAL_IK_NEAR_SINGULAR : GIMBAL_IK_OK;
}