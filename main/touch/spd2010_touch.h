#pragma once
#include <stdbool.h>
#include <stdint.h>

void touch_init(void);
/* Returns true on tap-up. If non-NULL, *out_x / *out_y receive the lift-off position (display pixels). */
bool touch_poll_tap(uint16_t *out_x, uint16_t *out_y);
/* Returns true if finger is currently down. Reads cached state — no I2C. */
bool touch_is_down(uint16_t *out_x, uint16_t *out_y);
