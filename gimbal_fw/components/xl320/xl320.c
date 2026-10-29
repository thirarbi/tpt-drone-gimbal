#include <assert.h>
#include "xl320.h"
#include "dxl_packet.h"
#include "driver/uart.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static int s_uart;
static int s_srl = 2;
// 0 = Never reply
// 1 = PING/READ only
// 2 = Everything (default)

esp_err_t xl320_init(int uart, int baud, int tx_gpio, int rx_gpio, int dir_gpio)
{
    s_uart = uart;
    const uart_config_t cfg = {
        .baud_rate = baud,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(uart, 256, 256, 0, NULL, 0)); // RX buffer must be > 128
    ESP_ERROR_CHECK(uart_param_config(uart, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(uart, tx_gpio, rx_gpio, dir_gpio, UART_PIN_NO_CHANGE)); // RTS = DIR
    ESP_ERROR_CHECK(uart_set_mode(uart, UART_MODE_RS485_HALF_DUPLEX));
    return ESP_OK;
}

static int txrx(uint8_t id, uint8_t inst, const uint8_t *params, uint16_t n,
                bool expect_reply, dxl_parser_t *ps)
{
    uint8_t pkt[DXL_MAX_PKT];
    size_t len = dxl_build(id, inst, params, n, pkt);
    uart_flush_input(s_uart);
    uart_write_bytes(s_uart, pkt, len);

    if (!expect_reply) {
        return 0;
    }

    dxl_parser_reset(ps);
    int64_t deadline = esp_timer_get_time() + XL320_TIMEOUT_US;

    while (esp_timer_get_time() < deadline) {
        uint8_t b;
        if (uart_read_bytes(s_uart, &b, 1, pdMS_TO_TICKS(1)) != 1) {
            continue;
        }
        int r = dxl_parser_feed(ps, b);

        if (r == 1 && dxl_status_id(ps) == id) {
            return dxl_status_err(ps);
        }
        if (r < 0) {
            return XL320_ERR_CRC;
        }
    }
    return XL320_ERR_TIMEOUT;
}

int xl320_ping(uint8_t id)
{ dxl_parser_t p; return txrx(id, DXL_INST_PING, NULL, 0, true, &p); }

int xl320_read_u16(uint8_t id, uint16_t addr, uint16_t *v)
{
    uint8_t prm[4] = { (uint8_t)(addr & 0xFF), (uint8_t)(addr >> 8), 2, 0 };  // addr L/H, length L/H
    dxl_parser_t p;
    int r = txrx(id, DXL_INST_READ, prm, 4, true, &p);
    if (r == 0) *v = (uint16_t)(dxl_status_data(&p)[0] | (dxl_status_data(&p)[1] << 8));
    return r;
}

// WRITE: params = addr L/H, then 1 or 2 data bytes (little-endian)
static int write_bytes(uint8_t id, uint16_t addr, const uint8_t *data, uint16_t n)
{
    assert(n >= 1 && n <= 2);
    uint8_t prm[4];
    prm[0] = (uint8_t)(addr & 0xFF);
    prm[1] = (uint8_t)(addr >> 8);
    for (uint16_t i = 0; i < n; i++) {
        prm[2 + i] = data[i];
    }

    dxl_parser_t p;
    bool reply = (s_srl == 2) && (id != DXL_BROADCAST_ID);

    return txrx(id, DXL_INST_WRITE, prm, (uint16_t)(2 + n), reply, &p);
}

int xl320_write_u8(uint8_t id, uint16_t addr, uint8_t v) {
    return write_bytes(id, addr, &v, 1);
}

int xl320_write_u16(uint8_t id, uint16_t addr, uint16_t v) {
    uint8_t d[2] = { (uint8_t)(v & 0xFF), (uint8_t)(v >> 8) };
    return write_bytes(id, addr, d, 2);
}

int xl320_torque(uint8_t id, bool on) {
    return xl320_write_u8(id, 24, on ? 1 : 0); // Torque Enable
}

int xl320_set_srl(uint8_t id, uint8_t level) {
    
    // the reply to this write, if any, still follows the OLD level
    int r = xl320_write_u8(id, 17, level);

    // A timeout here means the servo was already at 1 from a previous run (EEPROM persists
    // across power cycles, but s_srl restarts at 2). The write went through anyway.
    if (r == 0 || r == XL320_ERR_TIMEOUT) {
        s_srl = level;
    }
    return r;
}

// host UART only; the servo's own baud is a separate EEPROM write
int xl320_set_baud(int baud) {
    uart_wait_tx_done(s_uart, pdMS_TO_TICKS(20)); // let the last packet finish at the OLD rate
    return uart_set_baudrate(s_uart, (uint32_t)baud) == ESP_OK ? 0 : XL320_ERR_BAD;
}


int xl320_sync_write_goal(const uint8_t ids[], const uint16_t ticks[], int n) {
    
    // start addr(2), data len(2), then n x [id, lo, hi]
    uint8_t prm[2 + 2 + 3 * 3];
    assert(n >= 1 && n <= 3); // prm[] holds 3 servos
    uint16_t k = 0;

    // Goal Position, addr 30
    prm[k++] = 30; 
    prm[k++] = 0;

    // 2 bytes per servo
    prm[k++] = 2;  
    prm[k++] = 0;

    for (int i = 0; i < n; i++) {
        assert(ticks[i] <= 1023);
        prm[k++] = ids[i];
        prm[k++] = (uint8_t)(ticks[i] & 0xFF);
        prm[k++] = (uint8_t)(ticks[i] >> 8);
    }
    return txrx(DXL_BROADCAST_ID, DXL_INST_SYNC_WRITE, prm, k, false, NULL);
}
