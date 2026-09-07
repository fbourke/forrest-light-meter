#include "display.h"

#include <math.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_heap_caps.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_log.h"

#include "font5x7.h"

static const char *TAG = "display";

// Waveshare ESP32-C6-LCD-1.47 wiring. GPIO4/5 are the microSD's CS/MISO and
// are used by the sensor I2C bus instead, so SD and the sensors are mutually
// exclusive on this board.
#define PIN_MOSI 6
#define PIN_SCLK 7
#define PIN_CS   14
#define PIN_DC   15
#define PIN_RST  21
#define PIN_BL   22

#define LCD_HOST      SPI2_HOST
#define LCD_CLOCK_HZ  (40 * 1000 * 1000)

// The ST7789's RAM is 240 columns wide; a 172-wide panel is centred in it,
// so every column address needs shifting by (240 - 172) / 2.
#define LCD_X_GAP 34

// Flushed in horizontal bands so no single SPI transfer has to be huge.
#define BAND_ROWS 40

static esp_lcd_panel_handle_t s_panel;
static uint16_t *s_fb;

esp_err_t display_init(void)
{
    gpio_config_t bl = {
        .pin_bit_mask = 1ULL << PIN_BL,
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&bl));
    gpio_set_level(PIN_BL, 0);  // stay dark until there is something to show

    s_fb = heap_caps_malloc(DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t),
                            MALLOC_CAP_DMA);
    if (s_fb == NULL) {
        ESP_LOGE(TAG, "no DMA memory for a %dx%d framebuffer",
                 DISPLAY_WIDTH, DISPLAY_HEIGHT);
        return ESP_ERR_NO_MEM;
    }

    spi_bus_config_t bus = {
        .mosi_io_num = PIN_MOSI,
        .sclk_io_num = PIN_SCLK,
        .miso_io_num = -1,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = DISPLAY_WIDTH * BAND_ROWS * (int)sizeof(uint16_t) + 64,
    };
    esp_err_t err = spi_bus_initialize(LCD_HOST, &bus, SPI_DMA_CH_AUTO);
    if (err != ESP_OK) {
        return err;
    }

    esp_lcd_panel_io_handle_t io = NULL;
    esp_lcd_panel_io_spi_config_t io_cfg = {
        .dc_gpio_num = PIN_DC,
        .cs_gpio_num = PIN_CS,
        .pclk_hz = LCD_CLOCK_HZ,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .spi_mode = 0,
        .trans_queue_depth = 10,
    };
    err = esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_cfg, &io);
    if (err != ESP_OK) {
        return err;
    }

    esp_lcd_panel_dev_config_t panel_cfg = {
        .reset_gpio_num = PIN_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 16,
    };
    err = esp_lcd_new_panel_st7789(io, &panel_cfg, &s_panel);
    if (err != ESP_OK) {
        return err;
    }

    ESP_ERROR_CHECK(esp_lcd_panel_reset(s_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_init(s_panel));
    // These IPS panels ship with inverted polarity; without this, black and
    // white come out swapped.
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(s_panel, true));
    ESP_ERROR_CHECK(esp_lcd_panel_set_gap(s_panel, LCD_X_GAP, 0));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(s_panel, true));

    display_clear(DISPLAY_BLACK);
    err = display_flush();
    gpio_set_level(PIN_BL, 1);
    ESP_LOGI(TAG, "ST7789 %dx%d up on SPI%d", DISPLAY_WIDTH, DISPLAY_HEIGHT,
             LCD_HOST + 1);
    return err;
}

void display_clear(uint16_t color)
{
    for (int i = 0; i < DISPLAY_WIDTH * DISPLAY_HEIGHT; i++) {
        s_fb[i] = color;
    }
}

void display_fill_rect(int x, int y, int w, int h, uint16_t color)
{
    if (x < 0) { w += x; x = 0; }
    if (y < 0) { h += y; y = 0; }
    if (x + w > DISPLAY_WIDTH)  { w = DISPLAY_WIDTH - x; }
    if (y + h > DISPLAY_HEIGHT) { h = DISPLAY_HEIGHT - y; }
    for (int row = y; row < y + h; row++) {
        uint16_t *p = &s_fb[row * DISPLAY_WIDTH + x];
        for (int col = 0; col < w; col++) {
            p[col] = color;
        }
    }
}

void display_fill_circle(int cx, int cy, int r, uint16_t color)
{
    for (int dy = -r; dy <= r; dy++) {
        int dx = (int)(sqrt((double)(r * r - dy * dy)) + 0.5);
        display_fill_rect(cx - dx, cy + dy, 2 * dx + 1, 1, color);
    }
}

int display_text_width(const char *str, int scale)
{
    return (int)strlen(str) * DISPLAY_CHAR_W * scale;
}

void display_text(int x, int y, const char *str, int scale, uint16_t color)
{
    if (scale < 1) {
        scale = 1;
    }
    for (const char *p = str; *p; p++) {
        unsigned char c = (unsigned char)*p;
        if (c < FONT_FIRST_CHAR || c > FONT_LAST_CHAR) {
            c = '?';
        }
        const uint8_t *glyph = font5x7[c - FONT_FIRST_CHAR];
        for (int col = 0; col < FONT_WIDTH; col++) {
            uint8_t bits = glyph[col];
            for (int bit = 0; bit < 8; bit++) {
                if (bits & (1 << bit)) {
                    display_fill_rect(x + col * scale, y + bit * scale,
                                      scale, scale, color);
                }
            }
        }
        x += DISPLAY_CHAR_W * scale;
        if (x >= DISPLAY_WIDTH) {
            break;
        }
    }
}

esp_err_t display_flush(void)
{
    for (int y = 0; y < DISPLAY_HEIGHT; y += BAND_ROWS) {
        int rows = DISPLAY_HEIGHT - y;
        if (rows > BAND_ROWS) {
            rows = BAND_ROWS;
        }
        esp_err_t err = esp_lcd_panel_draw_bitmap(s_panel, 0, y, DISPLAY_WIDTH,
                                                  y + rows,
                                                  &s_fb[y * DISPLAY_WIDTH]);
        if (err != ESP_OK) {
            return err;
        }
    }
    return ESP_OK;
}
