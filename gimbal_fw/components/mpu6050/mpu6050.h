#pragma once
#include <stdint.h>
#include "esp_err.h"

// Register order
typedef struct { 
    int16_t acc[3];
    int16_t temp;
    int16_t gyro[3];
} mpu6050_raw_t;

// SI
typedef struct {
    float acc_g[3];
    float gyro_rads[3];
    float temp_c;
} mpu6050_sample_t;

esp_err_t mpu6050_init(int sda_gpio, int scl_gpio, uint32_t scl_hz);
esp_err_t mpu6050_read_raw(mpu6050_raw_t *out); // One 14-byte burst
void mpu6050_scale(const mpu6050_raw_t *r, mpu6050_sample_t *s);