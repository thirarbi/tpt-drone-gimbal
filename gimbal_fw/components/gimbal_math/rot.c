#include "rot.h"
#include <math.h>

#define PI_F 3.14159265f

mat3_t rot_x(float a) {
    float c = cosf(a), s = sinf(a);
    return (mat3_t){{{1, 0, 0}, {0, c, -s}, {0, s, c}}};
}

mat3_t rot_y(float a) {
    float c = cosf(a), s = sinf(a);
    return (mat3_t){{{c, 0, s}, {0, 1, 0}, {-s, 0, c}}};
}

mat3_t rot_z(float a) {
    float c = cosf(a), s = sinf(a);
    return (mat3_t){{{c, -s, 0}, {s, c, 0}, {0, 0, 1}}};
}

mat3_t mat3_mul(const mat3_t *A, const mat3_t * B) {
    mat3_t C;
    for (int i = 0; i < 3; i++){
        for (int j = 0; j < 3; j++) {
            C.m[i][j] = 0.0f;
            for (int k = 0; k < 3; k++) C.m[i][j] += A -> m[i][k] * B -> m[k][j];
        }
    }
    return C;
}

float wrap_pi(float a) {
    a = fmodf(a + PI_F, 2.0f * PI_F);
    if (a < 0.0f) { 
        a += 2.0f * PI_F;
    }
    return a - PI_F;
}

void mat3_vec(const mat3_t *A, const float v[3], float out[3]) {
    for (int i = 0; i < 3; i++)
        out[i] = A->m[i][0]*v[0] + A->m[i][1]*v[1] + A->m[i][2]*v[2];
}
