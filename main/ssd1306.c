#include "ssd1306.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "font5x7.h"

#define I2C_TIMEOUT_MS 100
#define CTRL_CMD  0x00
#define CTRL_DATA 0x40

static esp_err_t send_cmds(ssd1306_t *d, const uint8_t *cmds, size_t len)
{
    uint8_t buf[32];
    buf[0] = CTRL_CMD;
    memcpy(&buf[1], cmds, len);
    return i2c_master_transmit(d->dev, buf, len + 1, I2C_TIMEOUT_MS);
}

esp_err_t ssd1306_init(ssd1306_t *d, i2c_master_bus_handle_t bus)
{
    i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = SSD1306_I2C_ADDR,
        .scl_speed_hz = 400000,
    };
    esp_err_t err = i2c_master_bus_add_device(bus, &cfg, &d->dev);
    if (err != ESP_OK) {
        return err;
    }

    static const uint8_t init_seq[] = {
        0xAE,             // display off
        0xD5, 0x80,       // clock divide
        0xA8, 0x1F,       // multiplex ratio: 32 rows
        0xD3, 0x00,       // display offset
        0x40,             // start line 0
        0x8D, 0x14,       // charge pump on
        0x20, 0x00,       // horizontal addressing mode
        0xA1,             // segment remap
        0xC8,             // COM scan direction: remapped
        0xDA, 0x02,       // COM pin config for 128x32
        0x81, 0x8F,       // contrast
        0xD9, 0xF1,       // pre-charge
        0xDB, 0x40,       // VCOMH deselect
        0xA4,             // resume from RAM
        0xA6,             // normal (non-inverted)
        0x2E,             // scroll off
        0xAF,             // display on
    };
    vTaskDelay(pdMS_TO_TICKS(100));
    err = send_cmds(d, init_seq, sizeof(init_seq));
    if (err != ESP_OK) {
        return err;
    }
    ssd1306_clear(d);
    return ssd1306_flush(d);
}

void ssd1306_clear(ssd1306_t *d)
{
    memset(d->fb, 0, sizeof(d->fb));
}

static void put_column(ssd1306_t *d, int x, int page, uint8_t bits)
{
    if (x < 0 || x >= SSD1306_WIDTH || page < 0 || page >= SSD1306_PAGES) {
        return;
    }
    d->fb[page * SSD1306_WIDTH + x] |= bits;
}

void ssd1306_text(ssd1306_t *d, int x, int y, const char *str, int scale)
{
    if (scale < 1) {
        scale = 1;
    }
    int page = y / 8;
    for (const char *p = str; *p; p++) {
        unsigned char c = (unsigned char)*p;
        if (c < FONT_FIRST_CHAR || c > FONT_LAST_CHAR) {
            c = '?';
        }
        const uint8_t *glyph = font5x7[c - FONT_FIRST_CHAR];
        for (int col = 0; col < FONT_WIDTH; col++) {
            uint8_t bits = glyph[col];
            if (scale == 1) {
                put_column(d, x + col, page, bits);
            } else {
                // Stretch each glyph column into `scale` columns and rows.
                for (int sy = 0; sy < scale; sy++) {
                    uint8_t out = 0;
                    for (int bit = 0; bit < 8 / scale; bit++) {
                        if (bits & (1 << (sy * (8 / scale) + bit))) {
                            out |= (uint8_t)(((1 << scale) - 1) << (bit * scale));
                        }
                    }
                    for (int sx = 0; sx < scale; sx++) {
                        put_column(d, x + col * scale + sx, page + sy, out);
                    }
                }
            }
        }
        x += (FONT_WIDTH + 1) * scale;  // 1px inter-character gap
        if (x >= SSD1306_WIDTH) {
            break;
        }
    }
}

esp_err_t ssd1306_flush(ssd1306_t *d)
{
    static const uint8_t window[] = {
        0x21, 0, SSD1306_WIDTH - 1,      // column range
        0x22, 0, SSD1306_PAGES - 1,      // page range
    };
    esp_err_t err = send_cmds(d, window, sizeof(window));
    if (err != ESP_OK) {
        return err;
    }
    uint8_t buf[1 + sizeof(d->fb)];
    buf[0] = CTRL_DATA;
    memcpy(&buf[1], d->fb, sizeof(d->fb));
    return i2c_master_transmit(d->dev, buf, sizeof(buf), I2C_TIMEOUT_MS);
}
