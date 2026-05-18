#pragma once
#include <stdint.h>
#include "esp_err.h"

#define I2C_SCL_IO          10
#define I2C_SDA_IO          11
#define I2C_MASTER_NUM      0
#define I2C_MASTER_FREQ_HZ  400000
#define I2C_TIMEOUT_MS      1000

void     i2c_drv_init(void);
esp_err_t i2c_drv_write(uint8_t dev_addr, uint8_t reg, const uint8_t *data, uint32_t len);
esp_err_t i2c_drv_read(uint8_t dev_addr, uint8_t reg, uint8_t *data, uint32_t len);
