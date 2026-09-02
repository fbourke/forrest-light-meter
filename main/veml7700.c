#include "veml7700.h"

#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define REG_ALS_CONF  0x00
#define REG_ALS       0x04
#define REG_WHITE     0x05

#define I2C_TIMEOUT_MS 100

// Sensitivity ladder, least to most sensitive. Auto-ranging walks this.
static const struct {
    veml7700_gain_t gain;
    veml7700_it_t it;
} k_ladder[] = {
    {VEML7700_GAIN_1_8, VEML7700_IT_25MS},
    {VEML7700_GAIN_1_8, VEML7700_IT_50MS},
    {VEML7700_GAIN_1_8, VEML7700_IT_100MS},
    {VEML7700_GAIN_1_4, VEML7700_IT_100MS},
    {VEML7700_GAIN_1,   VEML7700_IT_100MS},
    {VEML7700_GAIN_2,   VEML7700_IT_100MS},
    {VEML7700_GAIN_2,   VEML7700_IT_200MS},
    {VEML7700_GAIN_2,   VEML7700_IT_400MS},
    {VEML7700_GAIN_2,   VEML7700_IT_800MS},
};
#define LADDER_LEN (sizeof(k_ladder) / sizeof(k_ladder[0]))

// Target count window. Kept high so the ladder settles on a sensitive rung:
// at gain x1/8 the LSB is 0.46 lx, at x1 it is 0.058 lx. The window is 15x
// wide and the largest ladder step is 4x, so it cannot oscillate.
#define COUNT_LOW  2000   // below this, step up sensitivity
#define COUNT_HIGH 30000  // above this, step down

int veml7700_it_ms(veml7700_it_t it)
{
    switch (it) {
        case VEML7700_IT_25MS:  return 25;
        case VEML7700_IT_50MS:  return 50;
        case VEML7700_IT_100MS: return 100;
        case VEML7700_IT_200MS: return 200;
        case VEML7700_IT_400MS: return 400;
        case VEML7700_IT_800MS: return 800;
    }
    return 100;
}

const char *veml7700_gain_str(veml7700_gain_t gain)
{
    switch (gain) {
        case VEML7700_GAIN_1:   return "x1";
        case VEML7700_GAIN_2:   return "x2";
        case VEML7700_GAIN_1_8: return "x1/8";
        case VEML7700_GAIN_1_4: return "x1/4";
    }
    return "?";
}

static float gain_factor(veml7700_gain_t gain)
{
    switch (gain) {
        case VEML7700_GAIN_1:   return 1.0f;
        case VEML7700_GAIN_2:   return 2.0f;
        case VEML7700_GAIN_1_8: return 0.125f;
        case VEML7700_GAIN_1_4: return 0.25f;
    }
    return 1.0f;
}

// Datasheet anchor: 0.0036 lx/count at gain x2, IT 800 ms; scales inversely with both.
float veml7700_resolution(veml7700_gain_t gain, veml7700_it_t it)
{
    return 0.0036f * (2.0f / gain_factor(gain)) * (800.0f / (float)veml7700_it_ms(it));
}

static esp_err_t write_reg(veml7700_t *s, uint8_t reg, uint16_t val)
{
    uint8_t buf[3] = {reg, (uint8_t)(val & 0xFF), (uint8_t)(val >> 8)};
    return i2c_master_transmit(s->dev, buf, sizeof(buf), I2C_TIMEOUT_MS);
}

static esp_err_t read_reg(veml7700_t *s, uint8_t reg, uint16_t *val)
{
    uint8_t rx[2];
    esp_err_t err = i2c_master_transmit_receive(s->dev, &reg, 1, rx, sizeof(rx), I2C_TIMEOUT_MS);
    if (err == ESP_OK) {
        *val = (uint16_t)rx[0] | ((uint16_t)rx[1] << 8);
    }
    return err;
}

esp_err_t veml7700_set_range(veml7700_t *s, veml7700_gain_t gain, veml7700_it_t it)
{
    uint16_t conf = ((uint16_t)gain << 11) | ((uint16_t)it << 6);  // ALS_SD=0 -> powered on
    esp_err_t err = write_reg(s, REG_ALS_CONF, conf);
    if (err != ESP_OK) {
        return err;
    }
    s->gain = gain;
    s->it = it;
    // One integration plus margin before the new setting's data is valid.
    vTaskDelay(pdMS_TO_TICKS(veml7700_it_ms(it) * 5 / 2 + 10));
    return ESP_OK;
}

esp_err_t veml7700_init(veml7700_t *s, i2c_master_bus_handle_t bus)
{
    i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = VEML7700_I2C_ADDR,
        .scl_speed_hz = 100000,
    };
    esp_err_t err = i2c_master_bus_add_device(bus, &cfg, &s->dev);
    if (err != ESP_OK) {
        return err;
    }
    // Shutdown first: config only latches reliably after a power-on transition.
    err = write_reg(s, REG_ALS_CONF, 0x0001);
    if (err != ESP_OK) {
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(5));
    return veml7700_set_range(s, VEML7700_GAIN_1_8, VEML7700_IT_100MS);
}

esp_err_t veml7700_read_als_raw(veml7700_t *s, uint16_t *counts)
{
    return read_reg(s, REG_ALS, counts);
}

esp_err_t veml7700_read_white_raw(veml7700_t *s, uint16_t *counts)
{
    return read_reg(s, REG_WHITE, counts);
}

// Datasheet correction for the sensor's compression above ~1000 lx.
static float correct_lux(float lux)
{
    if (lux <= 1000.0f) {
        return lux;
    }
    return 6.0135e-13f * powf(lux, 4) - 9.3924e-9f * powf(lux, 3)
           + 8.1488e-5f * powf(lux, 2) + 1.0023f * lux;
}

static int ladder_index(veml7700_t *s)
{
    for (int i = 0; i < (int)LADDER_LEN; i++) {
        if (k_ladder[i].gain == s->gain && k_ladder[i].it == s->it) {
            return i;
        }
    }
    return 2;  // default startup rung
}

esp_err_t veml7700_read_lux_auto(veml7700_t *s, float *lux, uint16_t *counts_out)
{
    int idx = ladder_index(s);
    uint16_t counts = 0;

    for (int attempt = 0; attempt < (int)LADDER_LEN; attempt++) {
        esp_err_t err = veml7700_read_als_raw(s, &counts);
        if (err != ESP_OK) {
            return err;
        }
        int next = idx;
        if (counts < COUNT_LOW && idx < (int)LADDER_LEN - 1) {
            next = idx + 1;
        } else if (counts > COUNT_HIGH && idx > 0) {
            next = idx - 1;
        }
        if (next == idx) {
            break;
        }
        err = veml7700_set_range(s, k_ladder[next].gain, k_ladder[next].it);
        if (err != ESP_OK) {
            return err;
        }
        idx = next;
    }

    *lux = correct_lux((float)counts * veml7700_resolution(s->gain, s->it));
    if (counts_out) {
        *counts_out = counts;
    }
    return ESP_OK;
}
