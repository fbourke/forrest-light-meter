#include "ads1115.h"

#include "esp_timer.h"

#define REG_CONVERSION 0x00
#define REG_CONFIG     0x01

// OS=1 (start) | MUX=100 (AIN0 vs GND) | PGA=001 (+-4.096V) | MODE=1
// (single-shot) | DR=111 (860 SPS) | COMP_QUE=11 (comparator disabled).
#define CONFIG_START_CONVERSION 0xC3E3
#define CONFIG_OS_READY_MASK    0x8000

#define FULL_SCALE_MV 4096.0f // matches the PGA setting above

#define I2C_TIMEOUT_MS 100
#define CONVERSION_POLL_TIMEOUT_US 5000 // ~4x the 860 SPS conversion time

static esp_err_t write_reg16(ads1115_t *dev, uint8_t reg, uint16_t val)
{
    uint8_t buf[3] = {reg, (uint8_t)(val >> 8), (uint8_t)(val & 0xFF)};
    return i2c_master_transmit(dev->dev_handle, buf, sizeof(buf), I2C_TIMEOUT_MS);
}

static esp_err_t read_reg16(ads1115_t *dev, uint8_t reg, uint16_t *val)
{
    uint8_t rx[2];
    esp_err_t err = i2c_master_transmit_receive(dev->dev_handle, &reg, 1, rx, sizeof(rx), I2C_TIMEOUT_MS);
    if (err != ESP_OK) {
        return err;
    }
    *val = ((uint16_t)rx[0] << 8) | rx[1];
    return ESP_OK;
}

esp_err_t ads1115_init(ads1115_t *dev, i2c_master_bus_handle_t bus_handle)
{
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = ADS1115_I2C_ADDR,
        .scl_speed_hz = 400000,
    };
    return i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev->dev_handle);
}

esp_err_t ads1115_read_mv(ads1115_t *dev, int *mv_out)
{
    esp_err_t err = write_reg16(dev, REG_CONFIG, CONFIG_START_CONVERSION);
    if (err != ESP_OK) {
        return err;
    }

    // Poll the OS (ready) bit rather than trust a fixed delay - a short wait
    // would read a still-in-progress conversion.
    int64_t start = esp_timer_get_time();
    uint16_t config;
    do {
        err = read_reg16(dev, REG_CONFIG, &config);
        if (err != ESP_OK) {
            return err;
        }
    } while (!(config & CONFIG_OS_READY_MASK) && esp_timer_get_time() - start < CONVERSION_POLL_TIMEOUT_US);

    uint16_t raw;
    err = read_reg16(dev, REG_CONVERSION, &raw);
    if (err != ESP_OK) {
        return err;
    }

    *mv_out = (int)(((int16_t)raw) * FULL_SCALE_MV / 32768.0f);
    return ESP_OK;
}
