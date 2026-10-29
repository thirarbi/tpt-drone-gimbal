#include "comms_task.h"
#include <math.h>
#include <string.h>
#include "board.h"
#include "config_defaults.h"
#include "ctrl_task.h"
#include "gimbal_state.h"
#include "proto.h"
#include "driver/uart.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

comms_stats_t g_comms_stats;

static bool valid_target(const msg_set_target_t *t) {
    return t->mode <= GMODE_NADIR && isfinite(t->pitch_rad) && isfinite(t->roll_rad) && isfinite(t->yaw_rel_rad);
}

static void send_telem(uint16_t seq, uint16_t ack_seq) {
    qmeas_t m = {0};
    ctrl_out_t o = {0};
    xQueuePeek(g_qmeas_mb, &m, 0);
    xQueuePeek(g_ctrl_mb, &o, 0);
    uint32_t loop_max = g_ctrl_stats.loop_max_us;
    msg_telem_t t = {
        .proto_version = PROTO_VERSION, .mode = o.mode, .flags = o.flags,
        .q_rad = {m.q_rad[0], m.q_rad[1], m.q_rad[2]},
        .cam_roll_rad = o.cam_roll_rad, .cam_pitch_rad = o.cam_pitch_rad,
        .loop_max_us = (uint16_t)(loop_max > 0xFFFF ? 0xFFFF : loop_max),
        .ack_seq = ack_seq, .servo_err = {m.err[0], m.err[1], m.err[2]},
    };
    uint8_t frame[PROTO_MAX_FRAME];
    size_t n = proto_encode(MSG_TELEM, seq, &t, sizeof t, frame);
    uart_write_bytes(PI_UART_NUM, frame, n);
    g_comms_stats.tx_frames++;
}

static void comms_task(void *arg) {
    proto_parser_t p = {0};
    uint16_t tx_seq = 0, ack_seq = 0;
    int64_t last_rx = 0;
    bool ever_rx = false, wd = false;
    const int64_t period_us = 1000000 / TELEM_RATE_HZ;
    int64_t next_tx = esp_timer_get_time() + period_us;

    for (;;) {
        // Block for the first byte until the next TELEM is due, then drain whatever else arrived
        int64_t wait_us = next_tx - esp_timer_get_time();
        TickType_t wait = wait_us > 1000 ? pdMS_TO_TICKS(wait_us / 1000) : 1;
        uint8_t buf[64];
        int n = uart_read_bytes(PI_UART_NUM, buf, 1, wait);
        if (n == 1) {
            size_t avail = 0;
            uart_get_buffered_data_len(PI_UART_NUM, &avail);
            if (avail > sizeof buf - 1) avail = sizeof buf - 1;
            if (avail) n += uart_read_bytes(PI_UART_NUM, buf + 1, avail, 0);
        }

        for (int i = 0; i < n; i++) {
            if (proto_parser_feed(&p, buf[i]) != 1) continue;
            if (proto_type(&p) != MSG_SET_TARGET || proto_len(&p) != sizeof(msg_set_target_t)) continue;
            msg_set_target_t st;
            memcpy(&st, proto_payload(&p), sizeof st);   // memcpy: payload is not float-aligned
            if (!valid_target(&st)) continue;
            const target_t tg = { st.mode, st.pitch_rad, st.roll_rad, st.yaw_rel_rad, false };
            xQueueOverwrite(g_target_mb, &tg);
            ack_seq = proto_seq(&p);
            last_rx = esp_timer_get_time();
            ever_rx = true;
            wd = false;
            g_comms_stats.rx_frames++;
        }
        g_comms_stats.crc_errors = p.crc_errors;

        int64_t now = esp_timer_get_time();
        // Only after the Pi has spoken once: before that we are in the boot NADIR anyway
        if (ever_rx && !wd && now - last_rx > (int64_t)COMMS_WATCHDOG_MS * 1000) {
            const target_t tg = { GMODE_NADIR, -1.5707963f, 0.0f, 0.0f, true };
            xQueueOverwrite(g_target_mb, &tg);
            wd = true;
            g_comms_stats.watchdog_trips++;
        }
        if (now >= next_tx) {
            next_tx += period_us;
            if (next_tx < now) next_tx = now + period_us;   // fell behind: don't burst
            send_telem(tx_seq++, ack_seq);
        }
    }
}

void comms_task_start(void) {
    const uart_config_t cfg = {
        .baud_rate = PI_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(PI_UART_NUM, 256, 256, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(PI_UART_NUM, &cfg));
    // UART1's default pins are flash pins: remapping is mandatory (§2.5)
    ESP_ERROR_CHECK(uart_set_pin(PI_UART_NUM, PI_UART_TX_GPIO, PI_UART_RX_GPIO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    xTaskCreatePinnedToCore(comms_task, "comms", 4096, NULL, configMAX_PRIORITIES - 4, NULL, 0);
}
