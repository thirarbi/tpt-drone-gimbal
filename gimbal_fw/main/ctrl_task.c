#include "ctrl_task.h"
#include "board.h"
#include "config_defaults.h"
#include "mpu6050.h"
#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mahony.h"
#include "calib.h"


ctrl_stats_t g_ctrl_stats;
static TaskHandle_t s_ctrl;
static mahony_t s_mahony;
static uint32_t s_warm;
static calib_t  s_cal;

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

static void ctrl_task(void *arg) {
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
        if (mpu6050_read_raw(&raw) != ESP_OK) {
            g_ctrl_stats.read_err++;
            continue;
        }
        uint32_t dur = (uint32_t)(esp_timer_get_time() - t0);

        mpu6050_sample_t s;
        mpu6050_scale(&raw, &s);

        float g_imu[3] = { s.gyro_rads[0] - s_cal.gyro_bias[0],
                           s.gyro_rads[1] - s_cal.gyro_bias[1],
                           s.gyro_rads[2] - s_cal.gyro_bias[2] };
        float g_cam[3], a_cam[3];
        mat3_vec(&s_cal.R_imu_cam, g_imu, g_cam);
        mat3_vec(&s_cal.R_imu_cam, s.acc_g, a_cam);
        s_mahony.kp = (s_warm < MAHONY_WARMUP_SAMPLES) ? MAHONY_KP_BOOT : MAHONY_KP;
        s_warm++;
        mahony_update(&s_mahony, g_cam, a_cam, (float)cnt / (float)IMU_SAMPLE_RATE_HZ);

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

void ctrl_task_start(void) {
    
    calib_load(&s_cal);
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