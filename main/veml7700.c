#include "veml7700.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define VEML7700_REG_ALS_CONF 0x00
#define VEML7700_REG_ALS      0x04

// Lux-per-count at the default config (gain x1, integration time 100 ms),
// per the Vishay VEML7700 application note.
#define VEML7700_LUX_RESOLUTION 0.0576f

#define I2C_TIMEOUT_MS 100

esp_err_t veml7700_init(veml7700_t *dev, i2c_master_bus_handle_t bus_handle)
{
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = VEML7700_I2C_ADDR,
        .scl_speed_hz = 100000,
    };
    esp_err_t err = i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev->dev_handle);
    if (err != ESP_OK) {
        return err;
    }

    // ALS_CONF_0 = 0x0000: ALS powered on, gain x1, integration time 100 ms,
    // persistence 1. All reserved bits are 0, so this is simply {0x00, 0x00}.
    uint8_t config_write[3] = {VEML7700_REG_ALS_CONF, 0x00, 0x00};
    err = i2c_master_transmit(dev->dev_handle, config_write, sizeof(config_write), I2C_TIMEOUT_MS);
    if (err != ESP_OK) {
        return err;
    }

    // Datasheet-recommended settle time after power-on before the first read.
    vTaskDelay(pdMS_TO_TICKS(5));
    return ESP_OK;
}

esp_err_t veml7700_read_lux(veml7700_t *dev, float *lux_out)
{
    uint8_t reg = VEML7700_REG_ALS;
    uint8_t read_buf[2];
    esp_err_t err = i2c_master_transmit_receive(dev->dev_handle, &reg, 1, read_buf, sizeof(read_buf), I2C_TIMEOUT_MS);
    if (err != ESP_OK) {
        return err;
    }

    uint16_t raw = (uint16_t)read_buf[0] | ((uint16_t)read_buf[1] << 8);
    *lux_out = raw * VEML7700_LUX_RESOLUTION;
    return ESP_OK;
}
