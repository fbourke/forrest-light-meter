#pragma once

#include <stdint.h>
#include "esp_err.h"

// Waveshare ESP32-C6-LCD-1.47: 172x320 ST7789 on SPI, portrait.
#define DISPLAY_WIDTH  172
#define DISPLAY_HEIGHT 320

// Character cell at scale 1, including the one-pixel inter-character gap.
#define DISPLAY_CHAR_W 6
#define DISPLAY_CHAR_H 8

// The panel wants each RGB565 pixel big-endian, so colors are stored
// byte-swapped in the framebuffer. Build every color through this.
static inline uint16_t display_rgb(uint8_t r, uint8_t g, uint8_t b)
{
    uint16_t c = (uint16_t)(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
    return (uint16_t)((c >> 8) | (c << 8));
}

#define DISPLAY_BLACK  display_rgb(0, 0, 0)
#define DISPLAY_WHITE  display_rgb(255, 255, 255)
#define DISPLAY_GREY   display_rgb(128, 128, 128)
#define DISPLAY_AMBER  display_rgb(255, 176, 0)
#define DISPLAY_RED    display_rgb(255, 48, 48)
#define DISPLAY_GREEN  display_rgb(48, 220, 96)
#define DISPLAY_YELLOW display_rgb(255, 230, 60)

// Brings up the SPI bus, the panel and the backlight, and clears to black.
esp_err_t display_init(void);

// Backlight brightness, 0-100. PWM rather than on/off - the panel runs hot
// at full brightness, and there's no real reason to drive it that hard for
// a bench setup.
void display_set_backlight(uint8_t percent);

// All drawing goes to an offscreen framebuffer; display_flush() sends it.
void display_clear(uint16_t color);
void display_fill_rect(int x, int y, int w, int h, uint16_t color);
void display_text(int x, int y, const char *str, int scale, uint16_t color);
void display_fill_circle(int cx, int cy, int r, uint16_t color);
esp_err_t display_flush(void);

// Pixel width a string occupies at this scale, for right-aligning.
int display_text_width(const char *str, int scale);
