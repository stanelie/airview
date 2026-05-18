#include "spd2010_touch.h"
#include "tca9554.h"
#include "i2c_driver.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TOUCH_ADDR  0x53

/* Register addresses (16-bit, big-endian on wire) */
#define REG_STATUS      0x2000
#define REG_HDP_DATA    0x0003
#define REG_HDP_STATUS  0xFC02
#define REG_CLR_INT     0x0200
#define REG_CPU_START   0x0400
#define REG_PT_MODE     0x5000
#define REG_TP_START    0x4600

static bool     s_was_down = false;
static uint16_t s_last_x   = 0;
static uint16_t s_last_y   = 0;

static esp_err_t tp_read(uint16_t reg, uint8_t *data, size_t len)
{
    uint8_t addr[2] = {reg >> 8, reg & 0xFF};
    return i2c_master_write_read_device(I2C_MASTER_NUM, TOUCH_ADDR,
                                        addr, 2, data, len,
                                        I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
}

static esp_err_t tp_write(uint16_t reg, uint16_t val)
{
    /* Wire format: 2-byte reg addr, then 2-byte value little-endian */
    uint8_t buf[4] = {reg >> 8, reg & 0xFF, val & 0xFF, val >> 8};
    return i2c_master_write_to_device(I2C_MASTER_NUM, TOUCH_ADDR, buf, 4,
                                      I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
}

void touch_init(void)
{
    tca9554_set(TCA9554_EXIO1, false);
    vTaskDelay(pdMS_TO_TICKS(50));
    tca9554_set(TCA9554_EXIO1, true);
    vTaskDelay(pdMS_TO_TICKS(50));
}

bool touch_poll_tap(uint16_t *out_x, uint16_t *out_y)
{
    uint8_t st[4] = {0};
    if (tp_read(REG_STATUS, st, 4) != ESP_OK) return false;

    uint8_t  sl       = st[0];
    uint8_t  sh       = st[1];
    uint16_t read_len = (uint16_t)((st[3] << 8) | st[2]);

    bool tic_in_bios = (sh >> 6) & 1;
    bool tic_in_cpu  = (sh >> 5) & 1;
    bool cpu_run     = (sh >> 3) & 1;
    bool pt_exist    = sl & 1;
    bool aux         = (sl >> 3) & 1;
    bool tapped      = false;

    if (tic_in_bios) {
        tp_write(REG_CLR_INT,   0x0001);
        tp_write(REG_CPU_START, 0x0001);
    } else if (tic_in_cpu) {
        tp_write(REG_PT_MODE,  0x0000);
        tp_write(REG_TP_START, 0x0000);
        tp_write(REG_CLR_INT,  0x0001);
    } else if (cpu_run && read_len == 0) {
        tp_write(REG_CLR_INT, 0x0001);
    } else if (pt_exist) {
        /* HDP: 4-byte header + 6 bytes per touch point */
        uint8_t hdp[10] = {0};
        size_t to_read = (read_len < sizeof(hdp)) ? read_len : sizeof(hdp);
        tp_read(REG_HDP_DATA, hdp, to_read);

        /* Coordinates: x = {hdp[7][7:4], hdp[5][7:0]}, y = {hdp[7][3:0], hdp[6][7:0]} */
        uint8_t  weight = (to_read >= 9) ? hdp[8] : 0;
        uint16_t x      = (to_read >= 8) ? (((uint16_t)(hdp[7] & 0xF0) << 4) | hdp[5]) : 0;
        uint16_t y      = (to_read >= 8) ? (((uint16_t)(hdp[7] & 0x0F) << 8) | hdp[6]) : 0;

        if (weight > 0) {
            s_was_down = true;
            s_last_x   = x;
            s_last_y   = y;
        } else if (s_was_down) {
            s_was_down = false;
            if (out_x) *out_x = s_last_x;
            if (out_y) *out_y = s_last_y;
            tapped = true;
        }

        /* Drain remaining HDP data if any */
        uint8_t hdp_st[8] = {0};
        tp_read(REG_HDP_STATUS, hdp_st, 8);
        tp_write(REG_CLR_INT, 0x0001);
    } else if (cpu_run && aux) {
        tp_write(REG_CLR_INT, 0x0001);
    }

    return tapped;
}
