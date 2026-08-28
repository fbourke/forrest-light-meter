#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"

#include "veml7700.h"

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

    while (1) {
        float lux;
        esp_err_t err = veml7700_read_lux(&sensor, &lux);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "Ambient light: %.2f lux", lux);
        } else {
            ESP_LOGE(TAG, "Failed to read VEML7700: %s", esp_err_to_name(err));
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
