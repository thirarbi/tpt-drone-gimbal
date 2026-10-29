#include "servo_task.h"
#include <math.h>
#include "board.h"
#include "config_defaults.h"
#include "gimbal_state.h"
#include "xl320.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define XL320_RAD_PER_TICK  (300.0f / 1023.0f * 0.017453293f)   // ~0.0051183 rad

static const char *TAG = "servo";

servo_stats_t g_servo_stats;
static calib_t s_cal;
static uint8_t s_present;
static bool    s_torque_allowed;

uint16_t servo_rad_to_tick(const calib_t *c, int j, float q_rad) {
    float t = roundf((float)c->zero_tick[j] + (float)c->dir[j] * q_rad / XL320_RAD_PER_TICK);
    return (uint16_t)fminf(fmaxf(t, 0.0f), 1023.0f);
}

float servo_tick_to_rad(const calib_t *c, int j, uint16_t tick) {
    return (float)c->dir[j] * (float)((int)tick - (int)c->zero_tick[j]) * XL320_RAD_PER_TICK;
}

#if SERVO_WRITE_ANGLE_LIMITS
static const float s_qmin[3] = {JOINT_YAW_MIN_RAD, JOINT_ROLL_MIN_RAD, JOINT_PITCH_MIN_RAD};
static const float s_qmax[3] = {JOINT_YAW_MAX_RAD, JOINT_ROLL_MAX_RAD, JOINT_PITCH_MAX_RAD};

// T10: servo-side safety net. EEPROM (torque must be off) and only written when different.
static void write_angle_limits(int j) {
    uint8_t  id = s_cal.servo_id[j];
    uint16_t a = servo_rad_to_tick(&s_cal, j, s_qmin[j]);
    uint16_t b = servo_rad_to_tick(&s_cal, j, s_qmax[j]);
    uint16_t cw = a < b ? a : b, ccw = a < b ? b : a;      // dir = -1 swaps them
    uint16_t cur_cw = 0, cur_ccw = 0;
    if (xl320_read_u16(id, XL320_ADDR_CW_LIMIT, &cur_cw) < 0 ||
        xl320_read_u16(id, XL320_ADDR_CCW_LIMIT, &cur_ccw) < 0) return;
    bool changed = (cur_cw != cw) || (cur_ccw != ccw);
    if (cur_cw != cw)   xl320_write_u16(id, XL320_ADDR_CW_LIMIT, cw);
    if (cur_ccw != ccw) xl320_write_u16(id, XL320_ADDR_CCW_LIMIT, ccw);
    ESP_LOGI(TAG, "joint %d: angle limits %u..%u ticks%s", j, cw, ccw, changed ? " (written)" : "");
}
#endif

static bool bring_up(qmeas_t *m) {
    for (int j = 0; j < 3; j++) {
        uint8_t id = s_cal.servo_id[j];
        if (id == 0) continue;
        int r = xl320_ping(id);
        if (r < 0) {
            ESP_LOGW(TAG, "joint %d: ID %u does not answer (%d) -> joint held at 0", j, id, r);
            continue;
        }
        s_present |= (uint8_t)(1u << j);
    }
    if (!s_present) {
        ESP_LOGE(TAG, "no servo answered: servo power on? wiring? (TESTING.md step 3)");
        return false;
    }

    s_torque_allowed = true;
    for (int j = 0; j < 3; j++) {
        if (!(s_present & (1u << j))) continue;
        uint8_t id = s_cal.servo_id[j], volt = 0, hw = 0;

        if (xl320_read_u8(id, XL320_ADDR_HW_ERROR, &hw) >= 0 && hw) {
            ESP_LOGW(TAG, "joint %d: hardware error 0x%02X -> reboot", j, hw);
            xl320_reboot(id);
            vTaskDelay(pdMS_TO_TICKS(500));
        }
        xl320_read_u8(id, XL320_ADDR_PRESENT_VOLT, &volt);
        float v = 0.1f * (float)volt;
        if (v < SERVO_VOLTAGE_MIN_V || v > SERVO_VOLTAGE_MAX_V) {
            ESP_LOGE(TAG, "joint %d: %.1f V is outside %.1f-%.1f V -> torque refused",
                     j, (double)v, (double)SERVO_VOLTAGE_MIN_V, (double)SERVO_VOLTAGE_MAX_V);
            s_torque_allowed = false;
        }

        xl320_torque(id, false);
#if SERVO_WRITE_ANGLE_LIMITS
        write_angle_limits(j);
#endif
        uint16_t tick = 512;
        xl320_read_u16(id, XL320_ADDR_PRESENT_POS, &tick);
        m->q_rad[j] = servo_tick_to_rad(&s_cal, j, tick);
        xl320_write_u16(id, XL320_ADDR_GOAL_POSITION, tick);    // goal = where it is: torque-on moves nothing
        xl320_write_u16(id, XL320_ADDR_MOVING_SPEED, SERVO_SPEED_BOOT);
        xl320_write_u16(id, XL320_ADDR_TORQUE_LIMIT, SERVO_TORQUE_LIMIT);
        xl320_write_u8(id, XL320_ADDR_LED, 2);                   // green = found
        ESP_LOGI(TAG, "joint %d: ID %u, %.1f V, tick %u = %.1f deg", j, id, (double)v, tick,
                 (double)(m->q_rad[j] * 57.29578f));
    }
    return true;
}

static void servo_task(void *arg) {
    qmeas_t m = *(qmeas_t *)arg;
    uint8_t ids[3];
    int     jn[3], n = 0;
    for (int j = 0; j < 3; j++) {
        if (s_present & (1u << j)) { ids[n] = s_cal.servo_id[j]; jn[n] = j; n++; }
    }

    const TickType_t period = pdMS_TO_TICKS(1000 / SERVO_RATE_HZ);
    TickType_t last_wake = xTaskGetTickCount();
    int64_t torque_on_us = 0;
    bool run_speed = false;

    for (;;) {
        if (xTaskDelayUntil(&last_wake, period) == pdFALSE) g_servo_stats.overruns++;
        int64_t t0 = esp_timer_get_time();

        qcmd_t cmd;
        bool want = xQueuePeek(g_qcmd_mb, &cmd, 0) == pdTRUE && cmd.torque && s_torque_allowed;
        if (want) {                                    // goals first, so torque-on holds the right place
            uint16_t ticks[3];
            for (int i = 0; i < n; i++) ticks[i] = servo_rad_to_tick(&s_cal, jn[i], cmd.q_rad[jn[i]]);
            xl320_sync_write_goal(ids, ticks, n);
        }
        if (want != m.torque_on) {
            for (int i = 0; i < n; i++) xl320_torque(ids[i], want);
            m.torque_on = want;
            if (want && !torque_on_us) torque_on_us = t0;
        }
        if (!run_speed && torque_on_us && t0 - torque_on_us > (int64_t)SERVO_BOOT_WALK_MS * 1000) {
            for (int i = 0; i < n; i++) xl320_write_u16(ids[i], XL320_ADDR_MOVING_SPEED, SERVO_SPEED_RUN);
            run_speed = true;
        }

        m.ok = 0;
        for (int i = 0; i < n; i++) {
            uint16_t tick;
            int r = xl320_read_u16(ids[i], XL320_ADDR_PRESENT_POS, &tick);
            if (r >= 0) {
                m.q_rad[jn[i]] = servo_tick_to_rad(&s_cal, jn[i], tick);
                m.err[jn[i]] = (uint8_t)r;
                m.ok |= (uint8_t)(1u << jn[i]);
            } else if (r == XL320_ERR_TIMEOUT) {
                g_servo_stats.read_timeouts++;
            } else {
                g_servo_stats.read_bad++;
            }
        }
        m.t_us = esp_timer_get_time();
        xQueueOverwrite(g_qmeas_mb, &m);

        uint32_t dur = (uint32_t)(m.t_us - t0);
        g_servo_stats.cycles++;
        g_servo_stats.cycle_sum_us += dur;
        if (dur > g_servo_stats.cycle_max_us) g_servo_stats.cycle_max_us = dur;
    }
}

bool servo_task_start(const calib_t *cal) {
    static qmeas_t m0;   // static: the task copies it after it starts
    s_cal = *cal;
    ESP_ERROR_CHECK(xl320_init(DXL_UART_NUM, DXL_BAUD, DXL_TX_GPIO, DXL_RX_GPIO, DXL_DIR_GPIO));

    bool ok = bring_up(&m0);
    m0.present = s_present;
    m0.ok = s_present;
    m0.torque_allowed = ok && s_torque_allowed;
    m0.t_us = esp_timer_get_time();
    xQueueOverwrite(g_qmeas_mb, &m0);   // ctrl_task seeds q_cmd from this, so the first goal = present
    if (!ok) return false;

    // Core 0, below ctrl_task (core 1). Sole owner of UART2, so no mutex.
    xTaskCreatePinnedToCore(servo_task, "servo", 4096, &m0, configMAX_PRIORITIES - 3, NULL, 0);
    return true;
}
