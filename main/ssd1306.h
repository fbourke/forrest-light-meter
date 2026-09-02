#pragma once

#include <stdint.h>
#include "driver/i2c_master.h"
#include "esp_err.h"

#define SSD1306_I2C_ADDR 0x3C
#define SSD1306_WIDTH    128
#define SSD1306_HEIGHT   32
#define SSD1306_PAGES    (SSD1306_HEIGHT / 8)

typedef struct {
    i2c_master_dev_handle_t dev;
    uint8_t fb[SSD1306_WIDTH * SSD1306_PAGES];
} ssd1306_t;

esp_err_t ssd1306_init(ssd1306_t *d, i2c_master_bus_handle_t bus);
void ssd1306_clear(ssd1306_t *d);
// Draws into the framebuffer. scale 1 or 2; y is in pixels, snapped to a page.
void ssd1306_text(ssd1306_t *d, int x, int y, const char *str, int scale);
esp_err_t ssd1306_flush(ssd1306_t *d);
