#pragma once

typedef struct {
    float m[3][3];
} mat3_t;

typedef struct {
    float w, x, y, z;
} quat_t;

mat3_t rot_x(float a);
mat3_t rot_y(float a);
mat3_t rot_z(float a);
mat3_t mat3_mul(const mat3_t *A, const mat3_t *B);
float  wrap_pi(float a);

void mat3_vec(const mat3_t *A, const float v[3], float out[3]);
