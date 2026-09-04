#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"

#include "veml7700.h"
#include "integrator.h"
#include "metering.h"
#include "console.h"

// SparkFun ESP32-C6 boards route their Qwiic connector to these pins.
// Adjust if your specific board's pinout differs.
#define I2C_SDA_GPIO 6
#define I2C_SCL_GPIO 7
#define I2C_PORT     I2C_NUM_0

static const char *TAG = "light_meter";

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

    veml7700_t sensor;
    ESP_ERROR_CHECK(veml7700_init(&sensor, bus_handle));

    metering_init();
    console_init();

    while (1) {
        float lux;
        esp_err_t err = veml7700_read_lux(&sensor, &lux);
        if (err == ESP_OK) {
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
