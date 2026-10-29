#include <inttypes.h>
#include "board.h"
#include "config_defaults.h"
#include "ctrl_task.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mpu6050.h"

static const char *TAG = "app_main";

void app_main(void)
{
    ESP_ERROR_CHECK(mpu6050_init(MPU_SDA_GPIO, MPU_SCL_GPIO, MPU_I2C_HZ));
    ESP_LOGI(TAG, "mpu6050 ok");
    ctrl_task_start();

    ctrl_stats_t prev = g_ctrl_stats;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        ctrl_stats_t cur = = g_ctrl_stats;
        uint32_t dn = cur.n - prev.n;

        ESP_LOGI(TAG,   "rate=%" PRIu32 " Hz missed=%" PRIu32 " timeouts=%" PRIu32
                        " err=%" PRIu32 " mean=%" PRIu32 " us max=%" PRIu32 " us"
                        " | acc raw %d %d %d",
                dn, 
                cur.missed - prev.missed, 
                cur.timeouts - prev.timeouts,
                cur.read_err - prev.read_err,

                dn ? (cur.read_sum_us - prev.read_sum_us) / dn : 0,
                
                cur.read_max_us,
                cur.last.acc[0],
                cur.last.acc[1],
                cur.last.acc[2]
        )
    }
}
