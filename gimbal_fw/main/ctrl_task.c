#include "ctrl_task.h"
#include "board.h"
#include "config_defaults.h"
#include "mpu6050.h"
#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

ctrl_stats_t g_ctrl_stats;
static TaskHandle_t s_ctrl;

static void IRAM_ATTR imu_drdy_isr(void * arg) {
    BaseType_t woken = pdFALSE;
    vTaskNotifyGiveFromISR(s_ctrl, &woken);
    portYIELD_FROM_ISR(woken);
}

static void ctrl_task(void *arg) {
    for (;;) {
        uint32_t cnt = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(5));
        if (cnt == 0) {
            g_ctrl_stats.timeouts++; continue;
        }
        if (cnt > 1) {
            g_ctrl_stats.missed += cnt -1;
        }

        int64_t t0 = esp_timer_get_time();
        mpu6050_raw_t raw;
        if (mpu6050_read_raw(&raw) != ESP_OK) {
            g_ctrl_stats.read_err++;
            continue;
        }
        uint32_t dur = (uint32_t)(esp_timer_get_time() - t0);

        g_ctrl_stats.n++;
        g_ctrl_stats.read_sum_us += dur;
        if (dur > g_ctrl_stats.read_max_us) g_ctrl_stats.read_max_us = dur;

        g_ctrl_stats.last = raw;
    }
}

void ctrl_task_start(void) {
    // Task must be put first so s_ctrl is valid. Only then, interrupt
    xTaskCreatePinnedToCore(ctrl_task, "ctrl", 4096, NULL, configMAX_PRIORITIES - 2, &s_ctrl, 1);

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
}