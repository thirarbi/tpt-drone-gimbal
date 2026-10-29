#include <inttypes.h>
#include <stdio.h>
#include "bench_tests.h"
#include "board.h"
#include "calib.h"
#include "comms_task.h"
#include "config_defaults.h"
#include "ctrl_task.h"
#include "gimbal_state.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mpu6050.h"
#include "servo_task.h"

#define RAD2DEG 57.29578f

static const char *TAG = "app_main";

// One human-readable summary per second: what TESTING.md tells you to look at
static void print_summary(const ctrl_stats_t *cur, const ctrl_stats_t *prev, const servo_stats_t *sv,
                          const servo_stats_t *sv_prev) {
    ctrl_out_t o = {0};
    qmeas_t m = {0};
    xQueuePeek(g_ctrl_mb, &o, 0);
    xQueuePeek(g_qmeas_mb, &m, 0);
    uint32_t dn = cur->n - prev->n;
    uint32_t dc = sv->cycles - sv_prev->cycles;

    ESP_LOGI(TAG, "imu %" PRIu32 " Hz missed=%" PRIu32 " err=%" PRIu32 " read %" PRIu32 "/%" PRIu32
                  " us loop %" PRIu32 "/%" PRIu32 " us | cam roll %+6.1f pitch %+6.1f deg",
             dn, cur->missed - prev->missed, cur->read_err - prev->read_err,
             dn ? (cur->read_sum_us - prev->read_sum_us) / dn : 0, cur->read_max_us,
             dn ? (cur->loop_sum_us - prev->loop_sum_us) / dn : 0, cur->loop_max_us,
             (double)(o.cam_roll_rad * RAD2DEG), (double)(o.cam_pitch_rad * RAD2DEG));
    if (!m.present) return;
    ESP_LOGI(TAG, "servo %" PRIu32 " Hz cyc %" PRIu32 "/%" PRIu32 " us ovr=%" PRIu32 " tmo=%" PRIu32
                  " bad=%" PRIu32 " ok=%x/%x torque=%d | mode %u flags 0x%03x"
                  " | q_cmd %+6.1f %+6.1f %+6.1f | q_meas %+6.1f %+6.1f %+6.1f deg",
             dc, dc ? (sv->cycle_sum_us - sv_prev->cycle_sum_us) / dc : 0, sv->cycle_max_us, sv->overruns,
             sv->read_timeouts, sv->read_bad, m.ok, m.present, m.torque_on, o.mode, o.flags,
             (double)(o.q_cmd[0] * RAD2DEG), (double)(o.q_cmd[1] * RAD2DEG), (double)(o.q_cmd[2] * RAD2DEG),
             (double)(m.q_rad[0] * RAD2DEG), (double)(m.q_rad[1] * RAD2DEG), (double)(m.q_rad[2] * RAD2DEG));
    ESP_LOGI(TAG, "pi rx=%" PRIu32 " crc=%" PRIu32 " watchdog=%" PRIu32 " tx=%" PRIu32,
             g_comms_stats.rx_frames, g_comms_stats.crc_errors, g_comms_stats.watchdog_trips,
             g_comms_stats.tx_frames);
}

// "D,t_ms,cam_roll,cam_pitch,qcmd_y,qcmd_r,qcmd_p,qmeas_y,qmeas_r,qmeas_p,flags" (deg) for tools/serial_log.py
static void print_csv(void) {
    ctrl_out_t o = {0};
    qmeas_t m = {0};
    xQueuePeek(g_ctrl_mb, &o, 0);
    xQueuePeek(g_qmeas_mb, &m, 0);
    printf("D,%" PRId64 ",%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%u\n", esp_timer_get_time() / 1000,
           (double)(o.cam_roll_rad * RAD2DEG), (double)(o.cam_pitch_rad * RAD2DEG),
           (double)(o.q_cmd[0] * RAD2DEG), (double)(o.q_cmd[1] * RAD2DEG), (double)(o.q_cmd[2] * RAD2DEG),
           (double)(m.q_rad[0] * RAD2DEG), (double)(m.q_rad[1] * RAD2DEG), (double)(m.q_rad[2] * RAD2DEG),
           o.flags);
}

void app_main(void)
{
    mailboxes_init();
#if APP_MODE != APP_GIMBAL && APP_MODE != APP_IMU_ONLY
    bench_run(APP_MODE);
#endif

    // §4.6 boot sequence. 1: the DIR pull-down keeps the servo bus released until xl320_init.
    calib_t cal;
    calib_load(&cal);

    // 2: IMU + gyro bias. Servo torque is still off, so the camera hangs freely: it just has to be still.
    ESP_ERROR_CHECK(mpu6050_init(MPU_SDA_GPIO, MPU_SCL_GPIO, MPU_I2C_HZ));
    ESP_LOGI(TAG, "mpu6050 ok, calibrating gyro: keep the gimbal STILL");
    gyro_calibrate(&cal);

    // 3: Mahony converges with the high boot gain for MAHONY_WARMUP_SAMPLES
    ctrl_task_start(&cal, APP_MODE == APP_GIMBAL);
    vTaskDelay(pdMS_TO_TICKS(1000));

#if APP_MODE == APP_GIMBAL
    // 4-5: servos found, checked, goal = present, torque on; stab_step then walks them to NADIR
    if (!servo_task_start(&cal)) ESP_LOGW(TAG, "running IMU-only");
    // 6: Pi link. Mode stays NADIR until the Pi says otherwise.
    comms_task_start();
#endif

    const int period_ms = LOG_CSV_HZ > 0 ? 1000 / LOG_CSV_HZ : 1000;
    ctrl_stats_t prev = g_ctrl_stats;
    servo_stats_t sv_prev = g_servo_stats;
    int64_t next_summary = esp_timer_get_time() + 1000000;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(period_ms));
        if (LOG_CSV_HZ > 0) print_csv();
        if (esp_timer_get_time() < next_summary) continue;
        next_summary += 1000000;
        ctrl_stats_t cur = g_ctrl_stats;
        servo_stats_t sv = g_servo_stats;
        print_summary(&cur, &prev, &sv, &sv_prev);
        prev = cur;
        sv_prev = sv;
    }
}
