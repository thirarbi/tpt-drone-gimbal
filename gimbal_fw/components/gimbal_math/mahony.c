#include "mahony.h"
#include <math.h>

void mahony_init(mahony_t *m, float kp, float ki) {
    m->q = (quat_t){1.0f, 0.0f, 0.0f, 0.0f};
    m->integ[0] = m->integ[1] = m->integ[2] = 0.0f;
    m->kp = kp;
    m->ki = ki;
    m->gate_g = 0.15f;
}

void mahony_update(mahony_t *m, const float gyro_rads[3], const float acc_g[3], float dt) {
    const float w = m->q.w, x = m->q.x, y = m->q.y, z = m->q.z;
    float e[3] = {0.0f, 0.0f, 0.0f};

    float an = sqrtf(acc_g[0]*acc_g[0] + acc_g[1]*acc_g[1] + acc_g[2]*acc_g[2]);
    if (an > 1e-6f && fabsf(an - 1.0f) <= m->gate_g) {
        float ax = acc_g[0] / an, ay = acc_g[1] / an, az = acc_g[2] / an;
        float vx = -2.0f * (x*z - w*y);
        float vy = -2.0f * (y*z + w*x);
        float vz = -(w*w - x*x - y*y + z*z);

        e[0] = ay*vz - az*vy;
        e[1] = az*vx - ax*vz;
        e[2] = ax*vy - ay*vx;
    }

    for (int i = 0; i < 3; i++) m->integ[i] += m->ki * e[i] * dt;
    float gx = gyro_rads[0] + m->kp * e[0] + m->integ[0];
    float gy = gyro_rads[1] + m->kp * e[1] + m->integ[1];
    float gz = gyro_rads[2] + m->kp * e[2] + m->integ[2];

    float h = 0.5f * dt;
    float nw = w + h * (-x*gx - y*gy - z*gz);
    float nx = x + h * ( w*gx + y*gz - z*gy);
    float ny = y + h * ( w*gy - x*gz + z*gx);
    float nz = z + h * ( w*gz + x*gy - y*gx);

    float inv = 1.0f / sqrtf(nw*nw + nx*nx + ny*ny + nz*nz);
    m->q = (quat_t){nw*inv, nx*inv, ny*inv, nz*inv};
}

void mahony_get_R(const mahony_t *m, mat3_t *R) {
    const float w = m->q.w, x = m->q.x, y = m->q.y, z = m->q.z;
    R->m[0][0] = 1.0f - 2.0f*(y*y + z*z); R->m[0][1] = 2.0f*(x*y - w*z);        R->m[0][2] = 2.0f*(x*z + w*y);
    R->m[1][0] = 2.0f*(x*y + w*z);        R->m[1][1] = 1.0f - 2.0f*(x*x + z*z); R->m[1][2] = 2.0f*(y*z - w*x);
    R->m[2][0] = 2.0f*(x*z - w*y);        R->m[2][1] = 2.0f*(y*z + w*x);        R->m[2][2] = 1.0f - 2.0f*(x*x + y*y);
}