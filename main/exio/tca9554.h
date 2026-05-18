#pragma once
#include <stdbool.h>
#include <stdint.h>

#define TCA9554_ADDR    0x20
#define TCA9554_EXIO1   1
#define TCA9554_EXIO2   2
#define TCA9554_EXIO3   3
#define TCA9554_EXIO4   4
#define TCA9554_EXIO5   5
#define TCA9554_EXIO6   6
#define TCA9554_EXIO7   7
#define TCA9554_EXIO8   8

void tca9554_init(void);
void tca9554_set(uint8_t pin, bool state);
