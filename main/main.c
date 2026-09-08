#include <stdio.h>
#include <math.h>
#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"

#include "veml7700.h"
#include "integrator.h"
#include "metering.h"
#include "console.h"
#include "display.h"
#include "exposure.h"

// Waveshare ESP32-C6-LCD-1.47. The onboard display takes the SPI pins, so the
// sensor bus runs on the header.
#define I2C_SDA_GPIO 5
#define I2C_SCL_GPIO 4
#define I2C_PORT     I2C_NUM_0

static const char *TAG = "light_meter";

// Exposure settings the physical UI will eventually own. 1/60 is the dialed
// speed; the aperture is what the meter solves for.
static float g_iso = 100.0f;
static float g_shutter = 1.0f / 60;
static ev_display_t g_display = EV_DISPLAY_FULL_TENTHS;

// Right-aligns a value against the panel edge so digits do not jitter
// horizontally as the number of characters changes.
static void draw_right(int y, const char *str, int scale, uint16_t color)
{
    display_text(DISPLAY_WIDTH - 8 - display_text_width(str, scale), y,
                 str, scale, color);
}

// Bench figure from the VEML7700 cross-check, with a ~9% spread across
// readings. A placeholder so the LED path can be watched on screen; it is not
// a calibration and will move once the op-amp swap settles the analog side.
#define LED_LUX_PER_MV_PER_S 1.35f

// Backlight follows ambient brightness on a log scale (lux spans orders of
// magnitude, a linear map would be all-bright or all-dark) - dim enough to
// stay legible in a dark room, capped well under 100% since full brightness
// is most of why the panel runs hot.
#define BACKLIGHT_MIN_PERCENT 1
#define BACKLIGHT_MAX_PERCENT 70
#define BACKLIGHT_LUX_MIN 50.0f
#define BACKLIGHT_LUX_MAX 10000.0f

static uint8_t backlight_percent_for_lux(float lux)
{
    if (lux < BACKLIGHT_LUX_MIN) {
        lux = BACKLIGHT_LUX_MIN;
    } else if (lux > BACKLIGHT_LUX_MAX) {
        lux = BACKLIGHT_LUX_MAX;
    }
    float frac = log10f(lux / BACKLIGHT_LUX_MIN) / log10f(BACKLIGHT_LUX_MAX / BACKLIGHT_LUX_MIN);
    return (uint8_t)(BACKLIGHT_MIN_PERCENT + frac * (BACKLIGHT_MAX_PERCENT - BACKLIGHT_MIN_PERCENT));
}

static void draw_meter(float lux, const integrator_sample_t *led,
                       bool reset_blink, uint32_t reset_count)
{
    float ev = ev_from_lux(lux, g_iso, EV_CAL_C_FLAT);
    ev_readout_t r = ev_readout(ev, g_shutter, g_display);
    char buf[24];

    display_clear(DISPLAY_BLACK);

    display_text(8, 6, "AMBIENT", 2, DISPLAY_GREY);
    // One frame per reset. At the 1s refresh below that reads as a blink.
    if (reset_blink) {
        display_fill_circle(DISPLAY_WIDTH - 16, 13, 5, DISPLAY_YELLOW);
    }
    display_fill_rect(8, 26, DISPLAY_WIDTH - 16, 1, DISPLAY_GREY);

    display_text(8, 46, "EV", 2, DISPLAY_GREY);
    snprintf(buf, sizeof(buf), "%.1f", ev);
    draw_right(32, buf, 6, DISPLAY_WHITE);

    snprintf(buf, sizeof(buf), "%.0f lx", lux);
    draw_right(88, buf, 3, DISPLAY_AMBER);

    display_fill_rect(8, 120, DISPLAY_WIDTH - 16, 1, DISPLAY_GREY);

    display_text(8, 128, "T", 3, DISPLAY_GREY);
    draw_right(128, r.shutter, 3, DISPLAY_WHITE);

    display_text(8, 160, "F", 3, DISPLAY_GREY);
    if (r.out_of_range) {
        draw_right(160, "--", 3, DISPLAY_RED);
    } else if (r.tenths >= 0) {
        snprintf(buf, sizeof(buf), "%s %d", r.aperture, r.tenths);
        draw_right(160, buf, 3, DISPLAY_GREEN);
    } else {
        draw_right(160, r.aperture, 3, DISPLAY_GREEN);
    }

    snprintf(buf, sizeof(buf), "ISO %.0f", g_iso);
    display_text(8, 192, buf, 2, DISPLAY_GREY);
    display_text(8, 212, ev_display_str(g_display), 1, DISPLAY_GREY);

    // Debug block: the LED integrator, which drives nothing above this line.
    display_fill_rect(8, 230, DISPLAY_WIDTH - 16, 1, DISPLAY_GREY);
    snprintf(buf, sizeof(buf), "LED  R%lu", (unsigned long)reset_count);
    display_text(8, 238, buf, 1, DISPLAY_GREY);

    snprintf(buf, sizeof(buf), "%.0f mV/s", led->slope_mv_per_s);
    draw_right(250, buf, 2, DISPLAY_WHITE);

    // Derived lux next to how far it sits from the VEML7700, which is the
    // number actually worth watching while the analog side is in flux.
    float led_lux = led->slope_mv_per_s * LED_LUX_PER_MV_PER_S;
    if (led->slope_mv_per_s > 5.0f && lux > 1.0f) {
        snprintf(buf, sizeof(buf), "%.0f lx %+.0f%%", led_lux,
                 100.0f * (led_lux - lux) / lux);
    } else {
        snprintf(buf, sizeof(buf), "%.0f lx", led_lux);
    }
    draw_right(270, buf, 2, DISPLAY_AMBER);

    snprintf(buf, sizeof(buf), "%.0f mV", led->voltage_mv);
    draw_right(290, buf, 2, DISPLAY_GREY);

    if (r.out_of_range) {
        display_text(8, 310, "F out of range", 1, DISPLAY_RED);
    }

    display_flush();
}

// Prints which addresses ACK on the bus, so a wiring/address mismatch (e.g.
// the ADS1115's ADDR pin not actually tied to GND) shows up at a glance
// instead of as a wall of NACK errors from whichever driver tries it first.
static void i2c_bus_scan(i2c_master_bus_handle_t bus_handle)
{
    printf("I2C scan (SDA=%d SCL=%d):\n     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\n",
           I2C_SDA_GPIO, I2C_SCL_GPIO);
    for (int hi = 0; hi < 8; hi++) {
        printf("%02x: ", hi * 16);
        for (int lo = 0; lo < 16; lo++) {
            uint8_t addr = hi * 16 + lo;
            if (addr < 0x08 || addr > 0x77) {
                printf("   ");
            } else if (i2c_master_probe(bus_handle, addr, 50) == ESP_OK) {
                printf("%02x ", addr);
            } else {
                printf("-- ");
            }
        }
        printf("\n");
    }
}

void app_main(void)
{
    i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_PORT,
        .scl_io_num = I2C_SCL_GPIO,
        .sda_io_num = I2C_SDA_GPIO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus_handle));

    i2c_bus_scan(bus_handle);

    veml7700_t sensor;
    ESP_ERROR_CHECK(veml7700_init(&sensor, bus_handle));

    ESP_ERROR_CHECK(display_init());

    metering_init();
    console_init();

    uint32_t last_reset_count = integrator_reset_count();

    while (1) {
        float lux;
        esp_err_t err = veml7700_read_lux(&sensor, &lux);
        if (err == ESP_OK) {
            integrator_sample_t led;
            integrator_get_latest(&led);
            uint32_t resets = integrator_reset_count();
            bool blinked = resets != last_reset_count;
            last_reset_count = resets;
            draw_meter(lux, &led, blinked, resets);
            display_set_backlight(backlight_percent_for_lux(lux));

            // Paired with metering.c's live-log flag ("verbose"/"quiet" on
            // the console) so this doesn't scroll the console independently.
            if (metering_live_log_enabled()) {
                ESP_LOGI(TAG, "Ambient light: %.2f lux", lux);

                // Rough LED-as-photodiode calibration: cross-reference the
                // VEML7700's lux reading against the integrator's slope. Vary
                // the light level and average the ratio across readings; skip
                // near-zero slopes (e.g. right after a reset) since the ratio
                // blows up.
                integrator_sample_t sample;
                integrator_get_latest(&sample);
                if (sample.slope_mv_per_s > 5.0f) {
                    ESP_LOGI(TAG, "Calibration: %.3f lux per mV/s (lux=%.2f, slope=%.1f mV/s)",
                             lux / sample.slope_mv_per_s, lux, sample.slope_mv_per_s);
                }
            }
        } else {
            ESP_LOGE(TAG, "Failed to read VEML7700: %s", esp_err_to_name(err));
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
