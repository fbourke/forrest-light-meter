#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "exposure.h"
#include "ssd1306.h"
#include "veml7700.h"

// SparkFun Thing Plus ESP32-C6 Qwiic bus
#define I2C_SDA_GPIO 6
#define I2C_SCL_GPIO 7

static const char *TAG = "lightmeter";

// Metering settings. All will become user-settable.
static float g_iso = 100.0f;
static float g_shutter = 1.0f / 125;     // the speed you dial; F is the answer
static ev_display_t g_display = EV_DISPLAY_FULL_TENTHS;

static i2c_master_bus_handle_t bus_init(void)
{
    i2c_master_bus_config_t cfg = {
        .i2c_port = -1,  // let the driver pick a free port
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus;
    ESP_ERROR_CHECK(i2c_new_master_bus(&cfg, &bus));
    return bus;
}

static void bus_scan(i2c_master_bus_handle_t bus)
{
    ESP_LOGI(TAG, "I2C scan on SDA=%d SCL=%d", I2C_SDA_GPIO, I2C_SCL_GPIO);
    printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\n");
    for (int hi = 0; hi < 8; hi++) {
        printf("%02x: ", hi * 16);
        for (int lo = 0; lo < 16; lo++) {
            uint8_t addr = hi * 16 + lo;
            if (addr < 0x08 || addr > 0x77) {
                printf("   ");
            } else if (i2c_master_probe(bus, addr, 50) == ESP_OK) {
                printf("%02x ", addr);
            } else {
                printf("-- ");
            }
        }
        printf("\n");
    }
}

static void log_pair_table(float ev)
{
    ev_pair_t pairs[48];
    int n = ev_pair_table(ev, ev_display_step(g_display), pairs, 48);
    ESP_LOGI(TAG, "exposure scale at EV %.2f, ISO %.0f:", ev, g_iso);
    for (int i = 0; i < n; i++) {
        ESP_LOGI(TAG, "  f/%-4s %-7s (%+.2f stop)", pairs[i].aperture->label,
                 pairs[i].shutter->label, pairs[i].err_stops);
    }
}

static void format_lux(char *out, size_t n, float lux)
{
    if (lux < 10.0f) {
        snprintf(out, n, "%.2f", lux);
    } else if (lux < 1000.0f) {
        snprintf(out, n, "%.1f", lux);
    } else {
        snprintf(out, n, "%.0f", lux);
    }
}

void app_main(void)
{
    i2c_master_bus_handle_t bus = bus_init();
    bus_scan(bus);

    ssd1306_t oled;
    bool have_oled = ssd1306_init(&oled, bus) == ESP_OK;
    ESP_LOGI(TAG, "OLED @ 0x%02x: %s", SSD1306_I2C_ADDR, have_oled ? "ok" : "FAILED");

    veml7700_t als;
    bool have_als = veml7700_init(&als, bus) == ESP_OK;
    ESP_LOGI(TAG, "VEML7700 @ 0x%02x: %s", VEML7700_I2C_ADDR, have_als ? "ok" : "FAILED");

    if (have_oled && !have_als) {
        ssd1306_clear(&oled);
        ssd1306_text(&oled, 0, 0, "VEML7700", 1);
        ssd1306_text(&oled, 0, 12, "not responding", 1);
        ssd1306_flush(&oled);
    }
    if (!have_als) {
        return;
    }

    bool dumped_table = false;

    while (true) {
        float lux = 0.0f;
        uint16_t counts = 0, white = 0;
        esp_err_t err = veml7700_read_lux_auto(&als, &lux, &counts);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "ALS read failed: %s", esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }
        veml7700_read_white_raw(&als, &white);

        float ev = ev_from_lux(lux, g_iso, EV_CAL_C_FLAT);

        ev_readout_t r = ev_readout(ev, g_shutter, g_display);
        char readout[32];
        ev_format_readout(readout, sizeof(readout), &r);

        char luxstr[16];
        format_lux(luxstr, sizeof(luxstr), lux);
        ESP_LOGI(TAG, "%s lux  EV %.2f @ISO%.0f  %s  (exact f/%.2f, %+.2f stop;"
                      " raw=%u gain=%s it=%dms)",
                 luxstr, ev, g_iso, readout, r.exact_aperture, r.err_stops,
                 counts, veml7700_gain_str(als.gain), veml7700_it_ms(als.it));

        if (!dumped_table) {
            // Same measurement rendered in each display mode, then the scale.
            for (int m = 0; m <= EV_DISPLAY_THIRD; m++) {
                ev_readout_t alt = ev_readout(ev, g_shutter, (ev_display_t)m);
                char buf[32];
                ev_format_readout(buf, sizeof(buf), &alt);
                ESP_LOGI(TAG, "  [%-15s] %-16s %+.2f stop",
                         ev_display_str((ev_display_t)m), buf, alt.err_stops);
            }
            log_pair_table(ev);
            dumped_table = true;
        }

        if (have_oled) {
            char line[32];
            ssd1306_clear(&oled);
            snprintf(line, sizeof(line), "EV %.1f", ev);
            ssd1306_text(&oled, 0, 0, line, 2);
            ssd1306_text(&oled, 62, 4, readout, 1);
            snprintf(line, sizeof(line), "%s lx  ISO %.0f", luxstr, g_iso);
            ssd1306_text(&oled, 0, 24, line, 1);
            ssd1306_flush(&oled);
        }
        vTaskDelay(pdMS_TO_TICKS(400));
    }
}
