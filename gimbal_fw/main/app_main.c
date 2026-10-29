#include <inttypes.h>
#include "board.h"
#include "config_defaults.h"
#include "ctrl_task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mpu6050.h"
#include "xl320.h"
#include <math.h>

#define RAD2DEG (180.0f / 3.14159265f)

static const char *TAG = "app_main";

void app_main(void)
{
    ESP_ERROR_CHECK(mpu6050_init(MPU_SDA_GPIO, MPU_SCL_GPIO, MPU_I2C_HZ));
    ESP_LOGI(TAG, "mpu6050 ok");
    
#if MPU_CALIB_SIXPOSE
    // Re-calibration only: set MPU_CALIB_SIXPOSE to 1 in config_defaults.h (off by default).
    // Six-pose static accel check (~45 s), see MPU-CALIBRATION.md. For each axis we need the reading at
    // +1 g and at -1 g:  offset = (p + m) / 2,  sensitivity = (p - m) / 2  (nominal 8192 counts/g).
    // Move the board DURING the 6 s countdown, then hold it perfectly still.
    static const char *pose_name[6] = {
        "+Z up: flat, chip face up",
        "-Z up: upside down",
        "+X up: stand on an edge, X arrow pointing to the ceiling",
        "-X up: X arrow pointing to the floor",
        "+Y up: stand on an edge, Y arrow pointing to the ceiling",
        "-Y up: Y arrow pointing to the floor",
    };
    static const int pose_axis[6] = {2, 2, 0, 0, 1, 1};
    float plus[3] = {0.0f, 0.0f, 0.0f}, minus[3] = {0.0f, 0.0f, 0.0f};
    for (int p = 0; p < 6; p++) {
        ESP_LOGI(TAG, "POSE %d/6: %s -- measuring in 6 s, hold STILL", p + 1, pose_name[p]);
        vTaskDelay(pdMS_TO_TICKS(6000));
        float sum[3] = {0.0f, 0.0f, 0.0f}, lo[3] = {1e9f, 1e9f, 1e9f}, hi[3] = {-1e9f, -1e9f, -1e9f};
        for (int i = 0; i < 50; i++) {
            mpu6050_raw_t r;
            mpu6050_read_raw(&r);
            for (int a = 0; a < 3; a++) {
                float v = (float)r.acc[a];
                sum[a] += v;
                if (v < lo[a]) lo[a] = v;
                if (v > hi[a]) hi[a] = v;
            }
            vTaskDelay(pdMS_TO_TICKS(20));
        }
        float m[3] = {sum[0] / 50.0f, sum[1] / 50.0f, sum[2] / 50.0f};
        ESP_LOGI(TAG, "  mean %.0f %.0f %.0f   spread %.0f %.0f %.0f   (spread > ~250 = it moved, redo)",
                 (double)m[0], (double)m[1], (double)m[2],
                 (double)(hi[0] - lo[0]), (double)(hi[1] - lo[1]), (double)(hi[2] - lo[2]));
        if (p % 2 == 0) plus[pose_axis[p]] = m[pose_axis[p]];
        else            minus[pose_axis[p]] = m[pose_axis[p]];
    }
    for (int a = 0; a < 3; a++) {
        ESP_LOGI(TAG, "AXIS %c: +1g=%.0f  -1g=%.0f  offset=%.0f counts  sens=%.0f counts/g (nominal 8192)",
                 "XYZ"[a], (double)plus[a], (double)minus[a],
                 (double)((plus[a] + minus[a]) / 2.0f), (double)((plus[a] - minus[a]) / 2.0f));
    }
#endif

#if T8_BUS_TEST
    ESP_ERROR_CHECK(xl320_init(DXL_UART_NUM, DXL_BAUD, DXL_TX_GPIO, DXL_RX_GPIO, DXL_DIR_GPIO));
    const uint8_t id = 1;

    #if T8_SPEEDUP
    xl320_torque(id, false); // EEPROM writes only work with torque off
    xl320_write_u8(id, 5, 0);
    xl320_set_srl(id, 1);
    xl320_write_u8(id, 4, 3);
    xl320_set_baud(1000000); // Move host to match
    vTaskDelay(pdMS_TO_TICKS(50));
    ESP_LOGI(TAG, "ping @1M = %d", xl320_ping(id));
    #endif

    for (;;) {
        int64_t t0 = esp_timer_get_time();
        int r = xl320_ping(id);
        int64_t t1 = esp_timer_get_time();
        uint16_t pos = 0;

        int r2 = xl320_read_u16(id, 37, &pos); // Present Position
        int64_t t2 = esp_timer_get_time();

        ESP_LOGI(TAG, "ping=%d (%" PRId64 " us)  read=%d pos=%u (%" PRId64 " us)",
                 r, t1 - t0, r2, pos, t2 - t1);
        vTaskDelay(pdMS_TO_TICKS(500));
    }

#endif

    ctrl_task_start();

    ctrl_stats_t prev = g_ctrl_stats;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        ctrl_stats_t cur = g_ctrl_stats;

        const quat_t q = cur.q;

        // asinf(1.0000001) = NaN
        float s_roll    = fminf(fmaxf(2.0f*(q.y*q.z + q.w*q.x), -1.0f), 1.0f);
        float roll_deg  = asinf(s_roll) * RAD2DEG;
        float pitch_deg = atan2f(-2.0f*(q.x*q.z - q.w*q.y), 1.0f - 2.0f*(q.x*q.x + q.y*q.y)) * RAD2DEG;

        ESP_LOGI(TAG, "roll=%.2f pitch=%.2f deg", (double)roll_deg, (double)pitch_deg);

        uint32_t dn = cur.n - prev.n;

        ESP_LOGI(TAG,   "rate=%" PRIu32 " Hz missed=%" PRIu32 " timeouts=%" PRIu32
                        " err=%" PRIu32 " read mean=%" PRIu32 " max=%" PRIu32 " us"
                        " loop mean=%" PRIu32 " max=%" PRIu32 " us"
                        " | acc raw %d %d %d",
                dn, 
                cur.missed - prev.missed, 
                cur.timeouts - prev.timeouts,
                cur.read_err - prev.read_err,

                dn ? (cur.read_sum_us - prev.read_sum_us) / dn : 0,
                
                cur.read_max_us,
                dn ? (cur.loop_sum_us - prev.loop_sum_us) / dn : 0,
                cur.loop_max_us,

                cur.last.acc[0],
                cur.last.acc[1],
                cur.last.acc[2]
        );
        prev = cur;
    }
}
