// Closed-loop checks of stab_step(): a simulated base attitude, perfect servos (q_meas = the
// previous q_cmd, so one tick of lag), and an IMU that sees R_wc = Rz(drift) * R_wb * FK(q).
// Pass = the camera bore ends up pointing at the ground.
#include <math.h>
#include <stdio.h>
#include "kinematics.h"
#include "stab.h"

static int fails;
#define CHECK(c) do { if (!(c)) { printf("FAIL line %d: %s\n", __LINE__, #c); fails++; } } while (0)

#define PI_F  3.14159265f
#define DEG   (PI_F / 180.0f)
#define DT    0.01f
#define ALL   0x7u
#define NO_YAW ((1u << GIMBAL_ROLL) | (1u << GIMBAL_PITCH))

static stab_cfg_t cfg(uint8_t en, int mode) {
    stab_cfg_t c = {
        .mode = mode, .joint_en = en, .k = 0.3f, .omega_max = PI_F,
        .q_min = {-145.0f * DEG, -40.0f * DEG, -110.0f * DEG},
        .q_max = { 145.0f * DEG,  40.0f * DEG,   20.0f * DEG},
        .pid_kp = 20.0f, .pid_ki = 2.0f, .pid_kd = 0.0f, .pid_i_max = 0.5f,
    };
    return c;
}

static mat3_t base(float yaw, float pitch, float roll) {   // drone attitude, ZYX
    mat3_t z = rot_z(yaw), y = rot_y(pitch), x = rot_x(roll);
    mat3_t zy = mat3_mul(&z, &y);
    return mat3_mul(&zy, &x);
}

// Angle between the camera bore (link x axis = column 0 of R_wc) and world down, degrees
static float bore_err_deg(const mat3_t *R_wb, const float q[3]) {
    mat3_t R_bl;
    gimbal_fk(q, &R_bl);
    mat3_t R_wc = mat3_mul(R_wb, &R_bl);
    float c = fminf(fmaxf(R_wc.m[2][0], -1.0f), 1.0f);
    return acosf(c) / DEG;
}

static int run(stab_t *s, const mat3_t *R_wb, int steps, float drift_per_step, int q_valid) {
    float q_meas[3] = {s->q_cmd[0], s->q_cmd[1], s->q_cmd[2]};
    stab_out_t out = {0};
    int flags = 0;
    for (int i = 0; i < steps; i++) {
        mat3_t R_bl;
        gimbal_fk(q_meas, &R_bl);
        mat3_t R_true = mat3_mul(R_wb, &R_bl);
        mat3_t Rd = rot_z(drift_per_step * (float)i);
        stab_in_t in = {
            .R_wc = mat3_mul(&Rd, &R_true),
            .q_meas = {q_meas[0], q_meas[1], q_meas[2]},
            .q_meas_valid = q_valid,
            .pitch_rad = -PI_F / 2.0f, .roll_rad = 0.0f, .yaw_rel_rad = 0.0f,
            .dt = DT,
        };
        stab_step(s, &in, &out);
        flags |= out.flags;
        for (int j = 0; j < 3; j++) q_meas[j] = out.q_cmd[j];
    }
    return flags;
}

int main(void) {
    const float zero[3] = {0.0f, 0.0f, 0.0f};
    const mat3_t tilted = base(0.7f, 0.15f, -0.2f);
    stab_t s;
    stab_cfg_t c;

    // 1. Three joints, tilted base: bore down, and no yaw needed for a pure tilt
    c = cfg(ALL, STAB_MODE_IK);
    stab_init(&s, &c, zero);
    run(&s, &tilted, 400, 0.0f, 1);
    CHECK(bore_err_deg(&tilted, s.q_cmd) < 0.05f);
    CHECK(fabsf(s.q_cmd[GIMBAL_YAW]) < 1e-3f);

    // 2. Two servos (yaw absent, the current circuit): roll + pitch alone still reach nadir
    c = cfg(NO_YAW, STAB_MODE_IK);
    stab_init(&s, &c, zero);
    run(&s, &tilted, 400, 0.0f, 1);
    CHECK(bore_err_deg(&tilted, s.q_cmd) < 0.05f);
    CHECK(s.q_cmd[GIMBAL_YAW] == 0.0f);

    // 3. Mahony yaw drift (exaggerated, ~6 deg/s) cancels
    c = cfg(ALL, STAB_MODE_IK);
    stab_init(&s, &c, zero);
    run(&s, &tilted, 400, 0.001f, 1);
    CHECK(bore_err_deg(&tilted, s.q_cmd) < 0.05f);
    CHECK(fabsf(s.q_cmd[GIMBAL_YAW]) < 1e-3f);

    // 4. Stale readback: falls back to q_cmd, still converges, and says so
    stab_init(&s, &c, zero);
    int f = run(&s, &tilted, 400, 0.0f, 0);
    CHECK(bore_err_deg(&tilted, s.q_cmd) < 0.05f);
    CHECK(f & STAB_FLAG_USED_QCMD);

    // 5. Slew limit: one tick never moves a joint more than omega_max * dt
    const mat3_t level = base(0.0f, 0.0f, 0.0f);
    stab_init(&s, &c, zero);
    run(&s, &level, 1, 0.0f, 1);
    CHECK(fabsf(s.q_cmd[GIMBAL_PITCH]) <= c.omega_max * DT + 1e-6f);

    // 6. Joint limit: a 60 deg base roll needs -60 deg of roll joint, limit is -40
    const mat3_t rolled = base(0.0f, 0.0f, 60.0f * DEG);
    stab_init(&s, &c, zero);
    f = run(&s, &rolled, 400, 0.0f, 1);
    CHECK(fabsf(s.q_cmd[GIMBAL_ROLL] - c.q_min[GIMBAL_ROLL]) < 1e-6f);
    CHECK(f & STAB_FLAG_LIMIT);

    // 7. PID fallback converges for a small tilt (signs are right)
    const mat3_t small = base(0.5f, 0.1f, -0.1f);
    c = cfg(NO_YAW, STAB_MODE_PID);
    stab_init(&s, &c, zero);
    run(&s, &small, 2000, 0.0f, 1);
    CHECK(bore_err_deg(&small, s.q_cmd) < 0.5f);

    if (fails) { printf("test_stab: %d failure(s)\n", fails); return 1; }
    printf("test_stab: OK\n");
    return 0;
}
