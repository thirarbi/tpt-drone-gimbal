#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

#define XL320_ERR_TIMEOUT  (-1)
#define XL320_ERR_CRC      (-2)
#define XL320_ERR_BAD      (-3)

// Max wait for a status packet. > one read round trip (~2.85 ms @115200). TODO(measure, T8)
#ifndef XL320_TIMEOUT_US
#define XL320_TIMEOUT_US   5000
#endif

esp_err_t xl320_init(int uart_num, int baud, int tx_gpio, int rx_gpio, int dir_gpio);
int xl320_ping(uint8_t id);
int xl320_read_u8(uint8_t id, uint16_t addr, uint8_t *v);
int xl320_read_u16(uint8_t id, uint16_t addr, uint16_t *v);
int xl320_write_u8(uint8_t id, uint16_t addr, uint8_t v);
int xl320_write_u16(uint8_t id, uint16_t addr, uint16_t v);
int xl320_sync_write_goal(const uint8_t ids[], const uint16_t ticks[], int n);
int xl320_torque(uint8_t id, bool on);
int xl320_reboot(uint8_t id);
int xl320_set_baud(int baud); // host side only: uart_set_baudrate
int xl320_set_srl(uint8_t id, uint8_t level);

// XL-320 control table (decimal, from the XL-320 e-manual)
#define XL320_ADDR_CW_LIMIT       6   // 2 B, EEPROM
#define XL320_ADDR_CCW_LIMIT      8   // 2 B, EEPROM
#define XL320_ADDR_TORQUE_ENABLE 24
#define XL320_ADDR_LED           25   // bits R=1 G=2 B=4
#define XL320_ADDR_GOAL_POSITION 30   // 2 B, 0..1023 = 0..300 deg
#define XL320_ADDR_MOVING_SPEED  32   // 2 B, 0 = max
#define XL320_ADDR_TORQUE_LIMIT  35   // 2 B
#define XL320_ADDR_PRESENT_POS   37   // 2 B
#define XL320_ADDR_PRESENT_VOLT  45   // 1 B, x0.1 V
#define XL320_ADDR_PRESENT_TEMP  46   // 1 B, deg C
#define XL320_ADDR_HW_ERROR      50   // 1 B
