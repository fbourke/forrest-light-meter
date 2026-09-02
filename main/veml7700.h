#pragma once

#include "driver/i2c_master.h"
#include "esp_err.h"

#define VEML7700_I2C_ADDR 0x10

// ALS_CONF gain field values (register 0x00, bits 12:11)
typedef enum {
    VEML7700_GAIN_1   = 0,
    VEML7700_GAIN_2   = 1,
    VEML7700_GAIN_1_8 = 2,
    VEML7700_GAIN_1_4 = 3,
} veml7700_gain_t;

// ALS_CONF integration time field values (register 0x00, bits 9:6)
typedef enum {
    VEML7700_IT_100MS = 0x0,
    VEML7700_IT_200MS = 0x1,
    VEML7700_IT_400MS = 0x2,
    VEML7700_IT_800MS = 0x3,
    VEML7700_IT_50MS  = 0x8,
    VEML7700_IT_25MS  = 0xC,
} veml7700_it_t;

typedef struct {
    i2c_master_dev_handle_t dev;
    veml7700_gain_t gain;
    veml7700_it_t it;
} veml7700_t;

esp_err_t veml7700_init(veml7700_t *s, i2c_master_bus_handle_t bus);
esp_err_t veml7700_set_range(veml7700_t *s, veml7700_gain_t gain, veml7700_it_t it);
esp_err_t veml7700_read_als_raw(veml7700_t *s, uint16_t *counts);
esp_err_t veml7700_read_white_raw(veml7700_t *s, uint16_t *counts);

// Reads ALS, adjusting gain/IT until the count sits in a usable window.
esp_err_t veml7700_read_lux_auto(veml7700_t *s, float *lux, uint16_t *counts_out);

float veml7700_resolution(veml7700_gain_t gain, veml7700_it_t it);
const char *veml7700_gain_str(veml7700_gain_t gain);
int veml7700_it_ms(veml7700_it_t it);
