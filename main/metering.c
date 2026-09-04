#include <stdbool.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "metering.h"
#include "integrator.h"

static const char *TAG = "metering";

// Background poll period for the live ambient monitor and un-corded watch.
#define POLL_PERIOD_MS 20

// Beyond this the op-amp/ADC chain is no longer linear. Any capture that
// lands at or above it is reported as an error, not a number.
#define RAIL_MV 2200

// Auto-reset ceiling while armed for an un-corded flash: kept low so there's
// headroom below RAIL_MV to catch a big flash jump without clipping.
#define UNCORDED_IDLE_RESET_MV 700

// How far a sample's actual rise must exceed its ambient-predicted rise to
// be called a flash rather than ambient drift/noise.
#define UNCORDED_JUMP_MARGIN_MV 150.0f

// Averaged reads for the start/end of a timed capture, to resolve small
// deltas (short exposures) that single-sample ADC noise would swamp.
#define CAPTURE_OVERSAMPLE 8

// The FreeRTOS tick (often 10ms, see CONFIG_FREERTOS_HZ) is too coarse for
// photographic exposure times - pdMS_TO_TICKS() truncates, so e.g. an 8ms
// request would round to 0 ticks. Wait via esp_timer instead: coarse
// vTaskDelay(1) ticks to avoid hogging the CPU while comfortably more than
// one tick remains, then a precise busy-wait for the last stretch. The
// margin has to clear a full tick period, or vTaskDelay(1) can itself
// overshoot the target by almost a tick.
#define COARSE_WAIT_MARGIN_US 15000
static void precise_wait_ms(uint32_t ms)
{
    int64_t target = esp_timer_get_time() + (int64_t)ms * 1000;
    while (esp_timer_get_time() < target - COARSE_WAIT_MARGIN_US) {
        vTaskDelay(1);
    }
    while (esp_timer_get_time() < target) {
        esp_rom_delay_us(50);
    }
}

// TODO: drive the real sync-cable trigger GPIO once that connector is wired.
static void fire_trigger_stub(void)
{
    ESP_LOGW(TAG, "Trigger: no sync output wired yet - simulating fire");
}

typedef enum {
    MODE_AMBIENT_LIVE,
    MODE_UNCORDED_ARMED,
} background_mode_t;

static QueueHandle_t s_cmd_queue;
static background_mode_t s_mode = MODE_AMBIENT_LIVE;
// Off by default: the rolling readout fights with typing console commands.
static volatile bool s_live_log_enabled = false;

// integrator_reset() only guarantees landing at or below RESET_TARGET_MV
// (see integrator.c) - it can stop anywhere in that range, so the actual
// post-reset baseline has to be measured, not assumed to be 0.
static esp_err_t reset_and_get_baseline(int *start_mv)
{
    integrator_reset();
    return integrator_read_mv_oversampled(start_mv, CAPTURE_OVERSAMPLE);
}

static void run_ambient_capture(uint32_t exposure_ms)
{
    ESP_LOGI(TAG, "Ambient capture: exposing %u ms...", (unsigned)exposure_ms);
    int start_mv;
    if (reset_and_get_baseline(&start_mv) != ESP_OK) {
        ESP_LOGE(TAG, "Ambient capture failed: could not read baseline");
        return;
    }
    int64_t t0 = esp_timer_get_time();
    precise_wait_ms(exposure_ms);
    int64_t t1 = esp_timer_get_time(); // mark end-of-exposure before the read adds its own delay

    int end_mv;
    esp_err_t err = integrator_read_mv_oversampled(&end_mv, CAPTURE_OVERSAMPLE);
    int64_t elapsed_us = t1 - t0;
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Ambient capture failed: %s", esp_err_to_name(err));
        return;
    }
    if (end_mv >= RAIL_MV) {
        ESP_LOGE(TAG, "Ambient capture OVEREXPOSED (hit %d mV) - shorten exposure and retry", end_mv);
        return;
    }
    int delta_mv = end_mv - start_mv;
    float rate = (float)delta_mv / ((float)elapsed_us / 1e6f);
    ESP_LOGI(TAG, "Ambient result: %d mV delta (start %d, end %d) over %d ms -> %.1f mV/s",
             delta_mv, start_mv, end_mv, (int)(elapsed_us / 1000), rate);
}

static void run_corded_capture(uint32_t exposure_ms)
{
    ESP_LOGI(TAG, "Corded capture: exposure %u ms...", (unsigned)exposure_ms);
    int start_mv;
    if (reset_and_get_baseline(&start_mv) != ESP_OK) {
        ESP_LOGE(TAG, "Corded capture failed: could not read baseline");
        return;
    }
    fire_trigger_stub();
    int64_t t0 = esp_timer_get_time();
    precise_wait_ms(exposure_ms);
    int64_t t1 = esp_timer_get_time();

    int end_mv;
    esp_err_t err = integrator_read_mv_oversampled(&end_mv, CAPTURE_OVERSAMPLE);
    int64_t elapsed_us = t1 - t0;
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Corded capture failed: %s", esp_err_to_name(err));
        return;
    }
    if (end_mv >= RAIL_MV) {
        ESP_LOGE(TAG, "Corded capture OVEREXPOSED (hit %d mV) - shorten exposure and retry", end_mv);
        return;
    }
    int delta_mv = end_mv - start_mv;
    ESP_LOGI(TAG, "Corded result: %d mV delta (start %d, end %d) over %d ms",
             delta_mv, start_mv, end_mv, (int)(elapsed_us / 1000));
}

// Returns the skip_delta value the caller should carry into the next sample.
static bool handle_command(const meter_cmd_t *cmd)
{
    switch (cmd->type) {
    case METER_CMD_AMBIENT:
        if (cmd->arg_ms > 0) {
            run_ambient_capture(cmd->arg_ms);
        } else {
            ESP_LOGI(TAG, "Mode: ambient (live)");
        }
        s_mode = MODE_AMBIENT_LIVE;
        integrator_set_saturation_threshold_mv(RAIL_MV);
        integrator_reset();
        return true;
    case METER_CMD_ARM_UNCORDED:
        s_mode = MODE_UNCORDED_ARMED;
        integrator_set_saturation_threshold_mv(UNCORDED_IDLE_RESET_MV);
        integrator_reset();
        ESP_LOGI(TAG, "Mode: un-corded flash detection armed (idle reset @ %d mV, jump margin %.0f mV)",
                 UNCORDED_IDLE_RESET_MV, UNCORDED_JUMP_MARGIN_MV);
        return true;
    case METER_CMD_CORDED:
        run_corded_capture(cmd->arg_ms);
        s_mode = MODE_AMBIENT_LIVE;
        integrator_set_saturation_threshold_mv(RAIL_MV);
        integrator_reset();
        return true;
    case METER_CMD_SET_LIVE_LOG:
        s_live_log_enabled = cmd->arg_ms != 0;
        ESP_LOGI(TAG, "Rolling ambient log: %s", s_live_log_enabled ? "on" : "off");
        return true;
    }
    return true;
}

static void metering_task(void *arg)
{
    (void)arg;
    integrator_set_saturation_threshold_mv(RAIL_MV);

    TickType_t last_log = xTaskGetTickCount();
    float prev_v = 0.0f;
    float prev_rate = 0.0f;
    bool skip_delta = true; // true right after any reset/mode change

    while (1) {
        meter_cmd_t cmd;
        if (xQueueReceive(s_cmd_queue, &cmd, 0) == pdTRUE) {
            skip_delta = handle_command(&cmd);
            last_log = xTaskGetTickCount();
            continue;
        }

        integrator_sample_t sample;
        esp_err_t err = integrator_poll(&sample);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "ADC read failed: %s", esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(POLL_PERIOD_MS));
            continue;
        }

        bool did_reset = sample.reset_triggered;

        if (s_mode == MODE_AMBIENT_LIVE) {
            if (sample.reset_triggered && s_live_log_enabled) {
                ESP_LOGI(TAG, "Ambient: auto-reset at %.0f mV", sample.voltage_mv);
            }
            if (s_live_log_enabled && xTaskGetTickCount() - last_log >= pdMS_TO_TICKS(1000)) {
                ESP_LOGI(TAG, "Ambient: %.0f mV, %.1f mV/s", sample.voltage_mv, sample.slope_mv_per_s);
                last_log = xTaskGetTickCount();
            }
        } else { // MODE_UNCORDED_ARMED
            if (!skip_delta) {
                float raw_delta = sample.voltage_mv - prev_v;
                float expected = prev_rate * (POLL_PERIOD_MS / 1000.0f);
                float jump = raw_delta - expected;
                if (jump > UNCORDED_JUMP_MARGIN_MV) {
                    if (sample.voltage_mv >= RAIL_MV) {
                        ESP_LOGE(TAG, "Flash detected but OVEREXPOSED (hit %.0f mV) - reading discarded",
                                 sample.voltage_mv);
                    } else {
                        ESP_LOGI(TAG, "Flash detected: %.0f mV jump (ambient-corrected)", jump);
                    }
                    integrator_reset();
                    did_reset = true;
                }
            }
        }

        prev_v = sample.voltage_mv;
        prev_rate = sample.slope_mv_per_s;
        skip_delta = did_reset;

        vTaskDelay(pdMS_TO_TICKS(POLL_PERIOD_MS));
    }
}

void metering_init(void)
{
    ESP_ERROR_CHECK(integrator_init());
    s_cmd_queue = xQueueCreate(4, sizeof(meter_cmd_t));
    xTaskCreate(metering_task, "metering", 4096, NULL, 5, NULL);
}

void metering_submit_command(meter_cmd_t cmd)
{
    xQueueSend(s_cmd_queue, &cmd, portMAX_DELAY);
}

bool metering_live_log_enabled(void)
{
    return s_live_log_enabled;
}
