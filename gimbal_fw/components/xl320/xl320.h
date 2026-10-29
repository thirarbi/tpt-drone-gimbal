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
int xl320_read_u16(uint8_t id, uint16_t addr, uint16_t *v);
int xl320_write_u8(uint8_t id, uint16_t addr, uint8_t v);
int xl320_write_u16(uint8_t id, uint16_t addr, uint16_t v);
int xl320_sync_write_goal(const uint8_t ids[], const uint16_t ticks[], int n);
int xl320_torque(uint8_t id, bool on);
int xl320_set_baud(int baud); // host side only: uart_set_baudrate
int xl320_set_srl(uint8_t id, uint8_t level);