#include "bench_tests.h"
#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include "board.h"
#include "calib.h"
#include "config_defaults.h"
#include "ctrl_task.h"
#include "mpu6050.h"
#include "servo_task.h"
#include "xl320.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define DEG2RAD 0.017453293f
#define RAD2DEG 57.29578f

static const char *TAG = "bench";

static void idle_forever(void) {
    for (;;) vTaskDelay(pdMS_TO_TICKS(1000));
}

// T8: who is on the bus, at which baud. The XL-320 only knows these four rates.
static void bus_scan(void) {
    static const int bauds[] = {115200, 1000000, 57600, 9600};
    ESP_ERROR_CHECK(xl320_init(DXL_UART_NUM, bauds[0], DXL_TX_GPIO, DXL_RX_GPIO, DXL_DIR_GPIO));
    for (int round = 1;; round++) {
        int found = 0;
        for (int b = 0; b < 4; b++) {
            xl320_set_baud(bauds[b]);
            vTaskDelay(pdMS_TO_TICKS(10));
            for (int id = 0; id <= 252; id++) {
                if (xl320_ping((uint8_t)id) < 0) continue;
                uint16_t pos = 0;
                uint8_t volt = 0;
                xl320_read_u16((uint8_t)id, XL320_ADDR_PRESENT_POS, &pos);
                xl320_read_u8((uint8_t)id, XL320_ADDR_PRESENT_VOLT, &volt);
                ESP_LOGI(TAG, "FOUND ID %d @ %d baud: tick %u (%.1f deg), %.1f V",
                         id, bauds[b], pos, (double)((float)pos * 300.0f / 1023.0f), (double)(0.1f * (float)volt));
                found++;
            }
        }
        if (found) ESP_LOGI(TAG, "scan %d: %d servo(s)", round, found);
        else       ESP_LOGW(TAG, "scan %d: 0 replies. Servo power switch ON? DATA wiring? (TESTING.md step 3)", round);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

static int present_joints(const calib_t *cal, uint8_t ids[3], int jn[3]) {
    int n = 0;
    for (int j = 0; j < 3; j++) {
        uint8_t id = cal->servo_id[j];
        if (!id) continue;
        int64_t t0 = esp_timer_get_time();
        int r = xl320_ping(id);
        int64_t t1 = esp_timer_get_time();
        uint16_t pos = 0;
        int r2 = xl320_read_u16(id, XL320_ADDR_PRESENT_POS, &pos);
        int64_t t2 = esp_timer_get_time();
        ESP_LOGI(TAG, "joint %d ID %u: ping=%d (%" PRId64 " us)  read=%d (%" PRId64 " us)  tick %u",
                 j, id, r, t1 - t0, r2, t2 - t1, pos);
        if (r < 0) continue;
        ids[n] = id;
        jn[n] = j;
        n++;
    }
    return n;
}

static void log_positions(const calib_t *cal, const uint8_t ids[3], const int jn[3], int n,
                          const uint16_t goal[3]) {
    for (int i = 0; i < n; i++) {
        uint16_t pos = 0;
        int r = xl320_read_u16(ids[i], XL320_ADDR_PRESENT_POS, &pos);
        ESP_LOGI(TAG, "  joint %d: goal %4u  present %4u (%+5.1f deg)  err %+d ticks  r=%d", jn[i], goal[i],
                 pos, (double)(servo_tick_to_rad(cal, jn[i], pos) * RAD2DEG), (int)pos - (int)goal[i], r);
    }
}

// T8/T10: each joint alone (WRITE), then all together (one SYNC WRITE)
static void servo_sweep(const calib_t *cal) {
    static const float sweep_deg[] = {-20.0f, 20.0f, 0.0f};
    ESP_ERROR_CHECK(xl320_init(DXL_UART_NUM, DXL_BAUD, DXL_TX_GPIO, DXL_RX_GPIO, DXL_DIR_GPIO));
    uint8_t ids[3];
    int jn[3];
    int n = present_joints(cal, ids, jn);
    if (!n) {
        ESP_LOGE(TAG, "no configured servo answers: run APP_BUS_SCAN first");
        idle_forever();
    }

    for (int i = 0; i < n; i++) {
        uint16_t pos = 512;
        xl320_read_u16(ids[i], XL320_ADDR_PRESENT_POS, &pos);
        xl320_write_u16(ids[i], XL320_ADDR_GOAL_POSITION, pos);
        xl320_write_u16(ids[i], XL320_ADDR_MOVING_SPEED, 100);
        xl320_write_u16(ids[i], XL320_ADDR_TORQUE_LIMIT, SERVO_TORQUE_LIMIT);
        xl320_torque(ids[i], true);
    }

    for (int i = 0; i < n; i++) {
        for (int k = 0; k < 3; k++) {
            uint16_t goal[3] = {0, 0, 0};
            goal[i] = servo_rad_to_tick(cal, jn[i], sweep_deg[k] * DEG2RAD);
            ESP_LOGI(TAG, "joint %d alone -> %+.0f deg", jn[i], (double)sweep_deg[k]);
            xl320_write_u16(ids[i], XL320_ADDR_GOAL_POSITION, goal[i]);
            vTaskDelay(pdMS_TO_TICKS(1500));
            log_positions(cal, &ids[i], &jn[i], 1, &goal[i]);
        }
    }

    for (int k = 0; k < 3; k++) {
        uint16_t goal[3];
        for (int i = 0; i < n; i++) goal[i] = servo_rad_to_tick(cal, jn[i], sweep_deg[k] * DEG2RAD);
        int64_t t0 = esp_timer_get_time();
        xl320_sync_write_goal(ids, goal, n);
        ESP_LOGI(TAG, "all joints (SYNC WRITE, %" PRId64 " us) -> %+.0f deg", esp_timer_get_time() - t0,
                 (double)sweep_deg[k]);
        vTaskDelay(pdMS_TO_TICKS(1500));
        log_positions(cal, ids, jn, n, goal);
    }

    for (int i = 0; i < n; i++) xl320_torque(ids[i], false);
    ESP_LOGI(TAG, "sweep done, torque off. err should be within a few ticks.");
    idle_forever();
}

// T16: torque off, move the joints by hand. Gives zero_tick, dir, limits and checks R_imu_cam.
static void joint_monitor(calib_t *cal) {
    ESP_ERROR_CHECK(mpu6050_init(MPU_SDA_GPIO, MPU_SCL_GPIO, MPU_I2C_HZ));
    gyro_calibrate(cal);
    ctrl_task_start(cal, false);
    ESP_ERROR_CHECK(xl320_init(DXL_UART_NUM, DXL_BAUD, DXL_TX_GPIO, DXL_RX_GPIO, DXL_DIR_GPIO));
    for (int j = 0; j < 3; j++) {
        if (cal->servo_id[j]) xl320_torque(cal->servo_id[j], false);
    }

    static const char *name[3] = {"yaw", "roll", "pitch"};
    for (;;) {
        char line[160];
        int k = 0;
        for (int j = 0; j < 3; j++) {
            uint16_t pos = 0;
            if (cal->servo_id[j] && xl320_read_u16(cal->servo_id[j], XL320_ADDR_PRESENT_POS, &pos) >= 0)
                k += snprintf(line + k, sizeof line - (size_t)k, "%s %4u = %+6.1f deg | ", name[j], pos,
                              (double)(servo_tick_to_rad(cal, j, pos) * RAD2DEG));
            else
                k += snprintf(line + k, sizeof line - (size_t)k, "%s  --- | ", name[j]);
        }
        const quat_t q = g_ctrl_stats.q;
        float roll  = asinf(fminf(fmaxf(2.0f * (q.y * q.z + q.w * q.x), -1.0f), 1.0f)) * RAD2DEG;
        float pitch = atan2f(-2.0f * (q.x * q.z - q.w * q.y), 1.0f - 2.0f * (q.x * q.x + q.y * q.y)) * RAD2DEG;
        ESP_LOGI(TAG, "%scam roll %+6.1f pitch %+6.1f deg", line, (double)roll, (double)pitch);
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

// Six-pose static accel check (~45 s), see MPU-CALIBRATION.md. For each axis we need the reading at
// +1 g and at -1 g:  offset = (p + m) / 2,  sensitivity = (p - m) / 2  (nominal 8192 counts/g).
// Move the board DURING the 6 s countdown, then hold it perfectly still.
static void mpu_sixpose(void) {
    ESP_ERROR_CHECK(mpu6050_init(MPU_SDA_GPIO, MPU_SCL_GPIO, MPU_I2C_HZ));
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
    idle_forever();
}

// §4.3 bus speed-up, ONE-SHOT. EEPROM writes need torque off. Afterwards the servos only answer
// at 1 Mbit/s: set DXL_BAUD to 1000000. Aga's sketch (with its DXL_BAUD edited) is the recovery tool.
static void servo_speedup(const calib_t *cal) {
    ESP_ERROR_CHECK(xl320_init(DXL_UART_NUM, DXL_BAUD, DXL_TX_GPIO, DXL_RX_GPIO, DXL_DIR_GPIO));
    uint8_t ids[3];
    int jn[3];
    int n = present_joints(cal, ids, jn);
    for (int i = 0; i < n; i++) {
        xl320_torque(ids[i], false);
        xl320_write_u8(ids[i], 5, 0);    // Return Delay Time = 0
        xl320_set_srl(ids[i], 1);        // reply to PING/READ only
        xl320_write_u8(ids[i], 4, 3);    // Baud Rate 3 = 1 Mbit/s
    }
    xl320_set_baud(1000000);
    vTaskDelay(pdMS_TO_TICKS(50));
    for (int i = 0; i < n; i++) ESP_LOGI(TAG, "ID %u ping @1M = %d", ids[i], xl320_ping(ids[i]));
    ESP_LOGW(TAG, "now set DXL_BAUD 1000000 in config_defaults.h and set APP_MODE back");
    idle_forever();
}

void bench_run(int app_mode) {
    calib_t cal;
    calib_load(&cal);
    switch (app_mode) {
    case APP_BUS_SCAN:      bus_scan(); break;
    case APP_SERVO_SWEEP:   servo_sweep(&cal); break;
    case APP_JOINT_MONITOR: joint_monitor(&cal); break;
    case APP_MPU_SIXPOSE:   mpu_sixpose(); break;
    case APP_SERVO_SPEEDUP: servo_speedup(&cal); break;
    default: ESP_LOGE(TAG, "unknown APP_MODE %d", app_mode); break;
    }
    idle_forever();
}
