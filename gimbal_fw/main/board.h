// Pin map + motor-to-MCPWM mapping. ONLY place pins live (plan §2.5/§3.1).
// DRAFT: pending Aga's pin-map sign-off (T1).
#pragma once

// --- Motor phase outputs (MCPWM) ---
// Pitch: MCPWM unit 0
#define PITCH_IN1_GPIO      25
#define PITCH_IN2_GPIO      26
#define PITCH_IN3_GPIO      27

// Roll: MCPWM unit 0
#define ROLL_IN1_GPIO       32
#define ROLL_IN2_GPIO       33
#define ROLL_IN3_GPIO       4

// Yaw: MCPWM unit 1
#define YAW_IN1_GPIO         18
#define YAW_IN2_GPIO         19
#define YAW_IN3_GPIO         23

// --- DRV8313 driver control ---
#define DRV_EN_GPIO          13  // shared enable; external 10k pull-down
#define DRV_NFAULT_GPIO      34  // input-only, wired-OR fault; external 10k pull-up to 3V3

// --- MPU6050 IMU (I2C + data-ready interrupt) ---
#define MPU_SDA_GPIO         21
#define MPU_SCL_GPIO         22
#define MPU_INT_GPIO         35  // input-only

// --- UART2 to Pi ---
#define UART2_RX_GPIO        16  // Pi GPIO14 TX -> here
#define UART2_TX_GPIO        17  // Pi GPIO15 RX <- here

// --- Misc ---
#define STATUS_LED_GPIO      2

// Never use: GPIO 6-11 (flash).
// Avoid for outputs: 0, 2, 5, 12, 15 (strapping; GPIO12 high at boot breaks boot), 1/3 (USB console).
// If module is WROVER, GPIO16/17 are taken by PSRAM -> move UART2.
