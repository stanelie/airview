#include "i2c_driver.h"
#include <string.h>
#include "driver/i2c.h"
#include "esp_log.h"

void i2c_drv_init(void)
{
    i2c_config_t conf = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = I2C_SDA_IO,
        .scl_io_num       = I2C_SCL_IO,
        .sda_pullup_en    = GPIO_PULLUP_ENABLE,
        .scl_pullup_en    = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0));
}

esp_err_t i2c_drv_write(uint8_t dev_addr, uint8_t reg, const uint8_t *data, uint32_t len)
{
    uint8_t buf[len + 1];
    buf[0] = reg;
    memcpy(&buf[1], data, len);
    return i2c_master_write_to_device(I2C_MASTER_NUM, dev_addr, buf, len + 1,
                                      I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
}

esp_err_t i2c_drv_read(uint8_t dev_addr, uint8_t reg, uint8_t *data, uint32_t len)
{
    return i2c_master_write_read_device(I2C_MASTER_NUM, dev_addr, &reg, 1, data, len,
                                        I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
}
