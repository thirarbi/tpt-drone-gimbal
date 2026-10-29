#include "mpu6050.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_check.h"

#define MPU_ADDR        0x68
#define I2C_TIMEOUT_MS  10

// Registers
#define REG_SMPLRT_DIV    0x19
#define REG_CONFIG        0x1A
#define REG_GYRO_CONFIG   0x1B
#define REG_ACCEL_CONFIG  0x1C
#define REG_INT_PIN_CFG   0x37
#define REG_INT_ENABLE    0x38
#define REG_ACCEL_XOUT_H  0x3B
#define REG_PWR_MGMT_1    0x6B
#define REG_WHO_AM_I      0x75

// Values
#define VAL_SMPLRT_DIV    0x00   // 1 kHz / (1 + 0)
#define VAL_CONFIG        0x02   // DLPF_CFG = 2, ~94 Hz
#define VAL_GYRO_CONFIG   0x10   // +-1000 dps
#define VAL_ACCEL_CONFIG  0x08   // +-4 g
#define VAL_INT_PIN_CFG   0x00   // active high, push-pull, 50 us pulse
#define VAL_INT_ENABLE    0x01   // DATA_RDY_EN
#define VAL_PWR_RESET     0x80
#define VAL_PWR_WAKE_PLL  0x01   // CLKSEL = 1 (PLL, X gyro)
#define WHO_AM_I_EXPECTED 0x68

// Scales - must match tools/serial_log.py
#define ACC_LSB_PER_G     8192.0f
#define GYRO_LSB_PER_DPS  32.8f
#define DEG2RAD           0.017453293f

static const char *TAG = "mpu6050";

static i2c_master_bus_handle_t s_bus;
static i2c_master_dev_handle_t s_dev;

static esp_err_t write_reg(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = { reg, val };
    return i2c_master_transmit(s_dev, buf, sizeof buf, I2C_TIMEOUT_MS);
}

static esp_err_t read_regs(uint8_t reg, uint8_t *dst, size_t len) {
    return i2c_master_transmit_receive(s_dev, &reg, 1, dst, len, I2C_TIMEOUT_MS);
}

esp_err_t mpu6050_init(int sda_gpio, int scl_gpio, uint32_t scl_hz) {
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = sda_gpio,
        .scl_io_num = scl_gpio,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_cfg, &s_bus), TAG, "bus");

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MPU_ADDR,
        .scl_speed_hz = scl_hz,
    };

    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(s_bus, &dev_cfg, &s_dev), TAG, "add dev");

    ESP_RETURN_ON_ERROR(write_reg(REG_PWR_MGMT_1, VAL_PWR_RESET), TAG, "reset");
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_RETURN_ON_ERROR(write_reg(REG_PWR_MGMT_1, VAL_PWR_WAKE_PLL), TAG, "wake");

    uint8_t id = 0;
    ESP_RETURN_ON_ERROR(read_regs(REG_WHO_AM_I, &id, 1), TAG, "who_am_i read");
    if (id != WHO_AM_I_EXPECTED) {
        ESP_LOGE(TAG, "WHO_AM_I = 0x%02X, expected 0x%02X", id, WHO_AM_I_EXPECTED);
        return ESP_ERR_NOT_FOUND;
    }

    ESP_RETURN_ON_ERROR(write_reg(REG_CONFIG,       VAL_CONFIG),        TAG, "config");
    ESP_RETURN_ON_ERROR(write_reg(REG_SMPLRT_DIV,   VAL_SMPLRT_DIV),    TAG, "smplrt");
    ESP_RETURN_ON_ERROR(write_reg(REG_ACCEL_CONFIG, VAL_ACCEL_CONFIG),  TAG, "accel cfg");
    ESP_RETURN_ON_ERROR(write_reg(REG_GYRO_CONFIG,  VAL_GYRO_CONFIG),   TAG, "gyro cfg");
    ESP_RETURN_ON_ERROR(write_reg(REG_INT_PIN_CFG,  VAL_INT_PIN_CFG),   TAG, "int pin cfg");
    
    // Do not interrupt when sensor is not configured
    ESP_RETURN_ON_ERROR(write_reg(REG_INT_ENABLE,   VAL_INT_ENABLE),    TAG, "int enable");

    return ESP_OK;
}

esp_err_t mpu6050_read_raw(mpu6050_raw_t *out) {
    uint8_t b[14];
    ESP_RETURN_ON_ERROR(read_regs(REG_ACCEL_XOUT_H, b, sizeof b), TAG, "burst");

    // Big-endian pairs, in register order: acc xyz, temp, gyro xyz
    int16_t w[7];
    for (int i = 0; i < 7; i++) {
        w[i] = (int16_t)(((uint16_t)b[2 * i] << 8) | b[2 * i + 1]);
    }
    out -> acc[0] = w[0]; 
    out -> acc[1] = w[1]; 
    out->acc[2] = w[2];
    
    out -> temp = w[3];
    
    out -> gyro[0] = w[4];
    out -> gyro[1] = w[5];
    out -> gyro[2] = w[6];
    return ESP_OK;
}

void mpu6050_scale(const mpu6050_raw_t *r, mpu6050_sample_t * s) {
    for (int i = 0; i < 3; i++) {
        s->acc_g[i] = r->acc[i] / ACC_LSB_PER_G;
        s->gyro_rads[i] = (r->gyro[i] / GYRO_LSB_PER_DPS) * DEG2RAD;
    }
    s->temp_c = r->temp / 340.0f + 36.53f;
}