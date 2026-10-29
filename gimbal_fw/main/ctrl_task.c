#include "ctrl_task.h"
#include <math.h>
#include "board.h"
#include "config_defaults.h"
#include "gimbal_state.h"
#include "mpu6050.h"
#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mahony.h"
#include "proto.h"
#include "stab.h"


static const char *TAG = "ctrl";
ctrl_stats_t g_ctrl_stats;
static TaskHandle_t s_ctrl;
static mahony_t s_mahony;
static uint32_t s_warm;
static calib_t  s_cal;
static bool     s_run_stab;
static stab_t   s_stab;
static bool     s_stab_ready;
static bool     s_imu_ok;

#if MPU_USE_INT
static void IRAM_ATTR imu_drdy_isr(void * arg) {
    BaseType_t woken = pdFALSE;
    vTaskNotifyGiveFromISR(s_ctrl, &woken);
    portYIELD_FROM_ISR(woken);
}
#else
// pdMS_TO_TICKS(1) is 0 below 1000 Hz, which would turn the loop below into a busy loop
_Static_assert(configTICK_RATE_HZ == 1000, "polled ctrl_task needs CONFIG_FREERTOS_HZ=1000");
#endif

static void stab_config(stab_cfg_t *c, uint8_t joint_en) {
    *c = (stab_cfg_t){
        .mode = STAB_MODE, .joint_en = joint_en, .k = STAB_GAIN_K, .omega_max = STAB_OMEGA_MAX_RAD_S,
        .q_min = {JOINT_YAW_MIN_RAD, JOINT_ROLL_MIN_RAD, JOINT_PITCH_MIN_RAD},
        .q_max = {JOINT_YAW_MAX_RAD, JOINT_ROLL_MAX_RAD, JOINT_PITCH_MAX_RAD},
        .pid_kp = STAB_PID_KP, .pid_ki = STAB_PID_KI, .pid_kd = STAB_PID_KD, .pid_i_max = STAB_PID_I_MAX,
    };
}

// 100 Hz part of the loop: target + IMU attitude + measured joints -> q_cmd mailbox
static void stab_tick(void) {
    mat3_t R_wc;
    mahony_get_R(&s_mahony, &R_wc);
    ctrl_out_t o = {
        .cam_roll_rad  = asinf(fminf(fmaxf(R_wc.m[2][1], -1.0f), 1.0f)),
        .cam_pitch_rad = atan2f(-R_wc.m[2][0], R_wc.m[2][2]),
        .flags = (uint16_t)((s_imu_ok ? GFLAG_IMU_OK : 0) | (STAB_MODE == STAB_MODE_PID ? GFLAG_PID_MODE : 0)),
    };
    target_t tg;
    xQueuePeek(g_target_mb, &tg, 0);
    o.mode = tg.mode;
    if (tg.watchdog) o.flags |= GFLAG_WATCHDOG;

    qmeas_t m;
    bool have_m = s_run_stab && xQueuePeek(g_qmeas_mb, &m, 0) == pdTRUE && m.present;
    if (have_m) {
        if (m.ok == m.present)  o.flags |= GFLAG_SERVOS_OK;
        if (!m.torque_allowed)  o.flags |= GFLAG_NO_TORQUE;
        if (m.err[0] | m.err[1] | m.err[2]) o.flags |= GFLAG_SERVO_ERR;

        stab_cfg_t cfg;
        stab_config(&cfg, m.present);
        if (!s_stab_ready || tg.mode == GMODE_PARK) {   // seed from where the servos are: no jump
            stab_init(&s_stab, &cfg, m.q_rad);
            s_stab_ready = true;
        }

        qcmd_t cmd = { .torque = tg.mode != GMODE_PARK };
        if (tg.mode == GMODE_PARK) {
            for (int j = 0; j < 3; j++) cmd.q_rad[j] = s_stab.q_cmd[j];
        } else {
            bool nadir = tg.mode == GMODE_NADIR;
            bool fresh = m.ok == m.present &&
                         esp_timer_get_time() - m.t_us < (int64_t)SERVO_QMEAS_STALE_MS * 1000;
            stab_in_t in = {
                .R_wc = R_wc, .q_meas = {m.q_rad[0], m.q_rad[1], m.q_rad[2]}, .q_meas_valid = fresh,
                .pitch_rad   = nadir ? -1.5707963f : tg.pitch_rad,
                .roll_rad    = nadir ? 0.0f : tg.roll_rad,
                .yaw_rel_rad = nadir ? 0.0f : tg.yaw_rel_rad,
                .dt = 1.0f / (float)SERVO_RATE_HZ,
            };
            stab_out_t out;
            stab_step(&s_stab, &in, &out);
            for (int j = 0; j < 3; j++) cmd.q_rad[j] = out.q_cmd[j];
            if (out.flags & STAB_FLAG_USED_QCMD)     o.flags |= GFLAG_QMEAS_STALE;
            if (out.flags & STAB_FLAG_NEAR_SINGULAR) o.flags |= GFLAG_SINGULAR;
            if (out.flags & STAB_FLAG_LIMIT)         o.flags |= GFLAG_AT_LIMIT;
        }
        xQueueOverwrite(g_qcmd_mb, &cmd);
        for (int j = 0; j < 3; j++) o.q_cmd[j] = cmd.q_rad[j];
    }
    xQueueOverwrite(g_ctrl_mb, &o);
}

static void ctrl_task(void *arg) {
    const uint32_t stab_every = IMU_SAMPLE_RATE_HZ / SERVO_RATE_HZ;
    uint32_t tick = 0;
#if !MPU_USE_INT
    TickType_t last_wake = xTaskGetTickCount();
#endif
    for (;;) {
#if MPU_USE_INT
        uint32_t cnt = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(5));
        if (cnt == 0) {
            g_ctrl_stats.timeouts++; continue;
        }
        if (cnt > 1) {
            g_ctrl_stats.missed += cnt -1;
        }
#else
        // No INT wire: 1 kHz from the tick. xTaskDelayUntil returns pdFALSE when the deadline had
        // already passed (previous iteration overran), which we count as "missed".
        if (xTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1)) == pdFALSE) {
            g_ctrl_stats.missed++;
        }
        const uint32_t cnt = 1;
#endif

        int64_t t0 = esp_timer_get_time();
        mpu6050_raw_t raw;
        s_imu_ok = mpu6050_read_raw(&raw) == ESP_OK;
        if (!s_imu_ok) {
            g_ctrl_stats.read_err++;
            continue;
        }
        uint32_t dur = (uint32_t)(esp_timer_get_time() - t0);

        mpu6050_sample_t s;
        mpu6050_scale(&raw, &s);

        // Offsets are in chip axes, so subtract them BEFORE rotating into the camera frame
        float g_imu[3], a_imu[3];
        for (int i = 0; i < 3; i++) {
            g_imu[i] = s.gyro_rads[i] - s_cal.gyro_bias[i];
            a_imu[i] = s.acc_g[i] - s_cal.acc_bias[i];
        }
        float g_cam[3], a_cam[3];
        mat3_vec(&s_cal.R_imu_cam, g_imu, g_cam);
        mat3_vec(&s_cal.R_imu_cam, a_imu, a_cam);
        s_mahony.kp = (s_warm < MAHONY_WARMUP_SAMPLES) ? MAHONY_KP_BOOT : MAHONY_KP;
        s_warm++;
        mahony_update(&s_mahony, g_cam, a_cam, (float)cnt / (float)IMU_SAMPLE_RATE_HZ);

        if (++tick >= stab_every) {
            tick = 0;
            stab_tick();
        }

        uint32_t loop = (uint32_t)(esp_timer_get_time() - t0);
        g_ctrl_stats.loop_sum_us += loop;
        if (loop > g_ctrl_stats.loop_max_us) {
            g_ctrl_stats.loop_max_us = loop;
        }

        g_ctrl_stats.q = s_mahony.q;


        g_ctrl_stats.n++;
        g_ctrl_stats.read_sum_us += dur;
        if (dur > g_ctrl_stats.read_max_us) g_ctrl_stats.read_max_us = dur;

        g_ctrl_stats.last = raw;
    }
}

bool gyro_calibrate(calib_t *cal) {
    for (int attempt = 1; attempt <= GYRO_CAL_ATTEMPTS; attempt++) {
        float sum[3] = {0.0f, 0.0f, 0.0f}, sq[3] = {0.0f, 0.0f, 0.0f};
        int n = 0;
        for (int i = 0; i < GYRO_CAL_SAMPLES; i++) {
            mpu6050_raw_t r;
            mpu6050_sample_t s;
            if (mpu6050_read_raw(&r) == ESP_OK) {
                mpu6050_scale(&r, &s);
                for (int a = 0; a < 3; a++) { sum[a] += s.gyro_rads[a]; sq[a] += s.gyro_rads[a] * s.gyro_rads[a]; }
                n++;
            }
            vTaskDelay(1);
        }
        if (n < GYRO_CAL_SAMPLES / 2) continue;
        float mean[3], worst_std = 0.0f;
        for (int a = 0; a < 3; a++) {
            mean[a] = sum[a] / (float)n;
            worst_std = fmaxf(worst_std, sqrtf(fmaxf(sq[a] / (float)n - mean[a] * mean[a], 0.0f)));
        }
        if (worst_std < GYRO_CAL_MAX_STD_RADS) {
            for (int a = 0; a < 3; a++) cal->gyro_bias[a] = mean[a];
            ESP_LOGI(TAG, "gyro bias %.5f %.5f %.5f rad/s (std %.5f)", (double)mean[0], (double)mean[1],
                     (double)mean[2], (double)worst_std);
            return true;
        }
        ESP_LOGW(TAG, "gyro cal %d/%d: moving (std %.4f rad/s), keep the gimbal still", attempt,
                 GYRO_CAL_ATTEMPTS, (double)worst_std);
    }
    ESP_LOGW(TAG, "gyro cal failed: using the mpu_unit.h fallback");
    return false;
}

void ctrl_task_start(const calib_t *cal, bool run_stab) {

    s_cal = *cal;
    s_run_stab = run_stab;
    mahony_init(&s_mahony, MAHONY_KP, MAHONY_KI);
    s_mahony.gate_g = MAHONY_ACCEL_GATE_G;
    s_mahony.i_max = MAHONY_INTEGRAL_MAX_RAD_S;

    // Task must be put first so s_ctrl is valid. Only then, interrupt
    xTaskCreatePinnedToCore(ctrl_task, "ctrl", 4096, NULL, configMAX_PRIORITIES - 2, &s_ctrl, 1);

#if MPU_USE_INT
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << MPU_INT_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_POSEDGE, // 50 us pulse: edge
    };

    ESP_ERROR_CHECK(gpio_config(&io));
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(MPU_INT_GPIO, imu_drdy_isr, NULL));
#endif  // MPU_USE_INT: with no wire, GPIO35 floats, so its interrupt must never be armed
}
