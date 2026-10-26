// Pin map. ONLY place pins live (plan §2.5/§3.1); servo IDs live in calib_t.
// DRAFT: pending Aga's pin-map sign-off (T1).
#pragma once

// --- Servo bus: 3x XL-320, Dynamixel Protocol 2.0, half-duplex via 74HC126 ---
#define DXL_UART_NUM         2
#define DXL_TX_GPIO         17  // -> 74HC126 pin 2 (1A)
#define DXL_RX_GPIO         16  // <- DATA line; hears our own echo
#define DXL_DIR_GPIO         4  // -> 74HC126 pin 1 (1OE); UART2 RTS in RS485 mode; ext. 10k pull-down

// --- UART1 to Pi (must be remapped: UART1 default pins are flash pins) ---
#define PI_UART_NUM          1
#define PI_UART_RX_GPIO     26  // Pi GPIO14 TX -> here
#define PI_UART_TX_GPIO     25  // Pi GPIO15 RX <- here


// --- MPU6050 IMU (I2C + data-ready interrupt) ---
#define MPU_SDA_GPIO         21
#define MPU_SCL_GPIO         22
#define MPU_INT_GPIO         35  // input-only

// --- Misc ---
#define STATUS_LED_GPIO      2

// Never use: GPIO 6-11 (flash).
// Avoid for outputs: 0, 2, 5, 12, 15 (strapping; GPIO12 high at boot breaks boot), 1/3 (USB console).
// If module is WROVER, GPIO16/17 are taken by PSRAM -> move the servo bus.
