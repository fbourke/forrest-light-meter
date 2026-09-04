#pragma once

#include "esp_err.h"
#include "driver/i2c_master.h"

#define ADS1115_I2C_ADDR 0x48 // ADDR pin tied to GND

typedef struct {
    i2c_master_dev_handle_t dev_handle;
} ads1115_t;

// Adds the ADS1115 as a device on an already-initialized I2C master bus.
esp_err_t ads1115_init(ads1115_t *dev, i2c_master_bus_handle_t bus_handle);

// Single-shot conversion on A0 (single-ended vs GND), +-4.096V range. Blocks
// until the conversion completes (~1.2ms at 860 SPS) and returns mV.
esp_err_t ads1115_read_mv(ads1115_t *dev, int *mv_out);
