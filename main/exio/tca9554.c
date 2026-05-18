#include "tca9554.h"
#include "i2c_driver.h"

#define REG_OUTPUT  0x01
#define REG_CONFIG  0x03

void tca9554_init(void)
{
    uint8_t val = 0x00;  /* all pins = output */
    i2c_drv_write(TCA9554_ADDR, REG_CONFIG, &val, 1);
}

void tca9554_set(uint8_t pin, bool state)
{
    uint8_t cur = 0;
    i2c_drv_read(TCA9554_ADDR, REG_OUTPUT, &cur, 1);
    if (state)
        cur |=  (1u << (pin - 1));
    else
        cur &= ~(1u << (pin - 1));
    i2c_drv_write(TCA9554_ADDR, REG_OUTPUT, &cur, 1);
}
