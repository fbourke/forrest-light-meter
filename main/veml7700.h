#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

#define VEML7700_I2C_ADDR 0x10

typedef struct {
    i2c_master_dev_handle_t dev_handle;
} veml7700_t;

// Adds the VEML7700 as a device on an already-initialized I2C master bus
// and powers it on with default gain (x1) / integration time (100 ms).
esp_err_t veml7700_init(veml7700_t *dev, i2c_master_bus_handle_t bus_handle);

// Reads the ambient light sensor and converts the raw count to lux.
esp_err_t veml7700_read_lux(veml7700_t *dev, float *lux_out);
