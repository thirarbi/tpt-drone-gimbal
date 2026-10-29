#include "stab.h"
#include <math.h>
#include "kinematics.h"

static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

static int enabled(const stab_t *s, int j) { return (s->cfg.joint_en >> j) & 1u; }

void stab_init(stab_t *s, const stab_cfg_t *cfg, const float q0[3]) {
    s->cfg = *cfg;
    for (int j = 0; j < 3; j++) {
        s->q_cmd[j]    = enabled(s, j) ? clampf(q0[j], cfg->q_min[j], cfg->q_max[j]) : 0.0f;
        s->pid_i[j]    = 0.0f;
        s->pid_prev[j] = NAN;   // NAN = no previous error yet, so no D kick on the first step
    }
}

// §4.2 steps 1-3: q_goal = IK of the camera target, expressed relative to the inferred base
static int goal_ik(const stab_in_t *in, const float q_fb[3], float q_goal[3]) {
    mat3_t R_bl;
    gimbal_fk(q_fb, &R_bl);
    mat3_t R_lb = mat3_transpose(&R_bl);
    mat3_t R_wb = mat3_mul(&in->R_wc, &R_lb);           // 1. base attitude, no extra sensor

    // 2. Target uses the base's own heading: Mahony yaw drift is in both R_wc and R_wb, so it cancels
    float  yaw_b = atan2f(R_wb.m[1][0], R_wb.m[0][0]);
    mat3_t Rz = rot_z(yaw_b + in->yaw_rel_rad);
    mat3_t Ry = rot_y(in->pitch_rad);
    mat3_t Rx = rot_x(in->roll_rad);
    mat3_t Rzy = mat3_mul(&Rz, &Ry);
    mat3_t R_wc_t = mat3_mul(&Rzy, &Rx);

    mat3_t R_bw = mat3_transpose(&R_wb);
    mat3_t R_bl_t = mat3_mul(&R_bw, &R_wc_t);            // 3. desired link attitude in the body
    return gimbal_ik(&R_bl_t, q_goal) == GIMBAL_IK_OK ? 0 : STAB_FLAG_NEAR_SINGULAR;
}

void stab_step(stab_t *s, const stab_in_t *in, stab_out_t *out) {
    const stab_cfg_t *c = &s->cfg;
    int flags = 0;
    float max_step = c->omega_max * in->dt;

    float q_fb[3];
    if (!in->q_meas_valid) flags |= STAB_FLAG_USED_QCMD;
    for (int j = 0; j < 3; j++)
        q_fb[j] = enabled(s, j) ? (in->q_meas_valid ? in->q_meas[j] : s->q_cmd[j]) : 0.0f;

    float step[3] = {0.0f, 0.0f, 0.0f};
    if (c->mode == STAB_MODE_IK) {
        float q_goal[3];
        flags |= goal_ik(in, q_fb, q_goal);
        for (int j = 0; j < 3; j++) step[j] = c->k * wrap_pi(q_goal[j] - s->q_cmd[j]);   // 4. low-pass
    } else {
        // Small angles: each joint mostly corrects its own camera axis. Yaw has no IMU reference,
        // so the yaw joint just goes to yaw_rel.
        const mat3_t *R = &in->R_wc;
        float cam_roll  = asinf(clampf(R->m[2][1], -1.0f, 1.0f));
        float cam_pitch = atan2f(-R->m[2][0], R->m[2][2]);
        float e[3] = { wrap_pi(in->yaw_rel_rad - s->q_cmd[GIMBAL_YAW]),
                       wrap_pi(in->roll_rad - cam_roll),
                       wrap_pi(in->pitch_rad - cam_pitch) };
        for (int j = 0; j < 3; j++) {
            float d = isnan(s->pid_prev[j]) ? 0.0f : (e[j] - s->pid_prev[j]) / in->dt;
            s->pid_prev[j] = e[j];
            float rate = c->pid_kp * e[j] + s->pid_i[j] + c->pid_kd * d;   // joint rate, rad/s
            step[j] = rate * in->dt;
            float q_next = s->q_cmd[j] + step[j];
            if (q_next > c->q_min[j] && q_next < c->q_max[j])              // anti-windup: freeze at limits
                s->pid_i[j] = clampf(s->pid_i[j] + c->pid_ki * e[j] * in->dt, -c->pid_i_max, c->pid_i_max);
        }
    }

    for (int j = 0; j < 3; j++) {
        if (!enabled(s, j)) { s->q_cmd[j] = 0.0f; continue; }
        float q = s->q_cmd[j] + clampf(step[j], -max_step, max_step);
        float qc = clampf(q, c->q_min[j], c->q_max[j]);
        if (qc != q) flags |= STAB_FLAG_LIMIT;
        s->q_cmd[j] = qc;
    }

    for (int j = 0; j < 3; j++) out->q_cmd[j] = s->q_cmd[j];
    out->flags = flags;
}
