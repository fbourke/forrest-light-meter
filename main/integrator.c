#include <string.h>
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"

#include "integrator.h"

static const char *TAG = "integrator";

// Guards s_latest, written by integrator_poll() (integrator task) and read
// by integrator_get_latest() (any other task, e.g. the VEML7700 loop).
static portMUX_TYPE s_latest_lock = portMUX_INITIALIZER_UNLOCKED;
static integrator_sample_t s_latest;

// Reset FET gate drive. Active high: shorts the integrator feedback cap
// (C4) to discharge it and pull ADC_OUT back near CHARGE NODE. Was a BJT
// (Q1) with a base resistor; swapped for a FET so this switches fast enough
// for ~1ms exposures - see RESET_SETTLE_US.
#define RESET_GPIO 2

// Rather than trust a fixed pulse width, hold the switch on and poll the
// ADC until the voltage actually drops - a blind pulse can't tell a real
// discharge from a drive/switching issue that leaves it stuck. Each poll is
// a near-instant internal-ADC read, so pace attempts a little rather than
// hammer the ADC pointlessly.
#define RESET_TARGET_MV 500
#define RESET_POLL_US 20
#define RESET_TIMEOUT_US 50000

// Releasing the reset switch leaves a brief settling transient on ADC_OUT
// (charge injection / op-amp recovery) before it reaches its true resting
// baseline. Callers that treat "just after reset" as the zero point
// (metering.c) would otherwise see that transient misread as signal.
// Measured: with the FET this settles on roughly the same timescale the BJT
// needed (~15-20ms) - dropping this to 200us produced a huge, exponential-
// looking "delta" on short exposures (up to 240mV, plateauing over ~20ms)
// that tracked nothing about the real ambient rate. That rules out the
// switch itself as the bottleneck; something else (op-amp recovery, most
// likely) dominates the settling time.
// Also retested at 200us after raising the reset baseline to 300mV (away
// from the LM358's near-ground crossover region) - identical artifact, same
// ~270mV plateau over the same ~15-20ms. So it isn't the crossover region
// either; this looks like a more basic bandwidth/slew-rate recovery limit
// of the LM358 itself. Back to a safe value; the real fix is the op-amp.
#define RESET_SETTLE_US 20000

// D1's cathode is at CHARGE_NODE and its anode is grounded, so light current
// only ever sinks charge out of the summing node - the feedback cap can only
// drive ADC_OUT up from its post-reset baseline, never down. So there's only
// one saturation direction to guard against (stay off the rail to keep the
// slope estimate linear).
//
// Runtime-adjustable: metering.c lowers this while armed for an un-corded
// flash, so idle ambient drift gets reset well before RAIL_MV, leaving
// headroom to capture a big flash jump without clipping.
#define SATURATION_HIGH_MV_DEFAULT 2200
static int s_saturation_high_mv = SATURATION_HIGH_MV_DEFAULT;

// Number of samples used for the linear-regression slope estimate. At the
// caller's ~20ms poll period this is roughly a 320ms window.
#define SLOPE_WINDOW 16

// ADC_OUT, downstream of U2A/R4/C5, back on the ESP32-C6's internal ADC
// (GPIO3) - fast enough (microsecond-scale reads) to chase ~1ms exposures,
// which the ADS1115's ~1.2ms-per-conversion couldn't keep up with.
#define ADC_UNIT ADC_UNIT_1
#define ADC_CHANNEL ADC_CHANNEL_3 // GPIO3
#define ADC_ATTEN ADC_ATTEN_DB_12 // full 0-3.3V range
#define ADC_BITWIDTH ADC_BITWIDTH_DEFAULT

static adc_oneshot_unit_handle_t s_adc_handle;
static adc_cali_handle_t s_cali_handle;

static int64_t s_sample_time_us[SLOPE_WINDOW];
static float s_sample_mv[SLOPE_WINDOW];
static int s_sample_count;
static int s_sample_next;

esp_err_t integrator_init(void)
{
    gpio_config_t reset_cfg = {
        .pin_bit_mask = 1ULL << RESET_GPIO,
        .mode = GPIO_MODE_OUTPUT,
    };
    esp_err_t err = gpio_config(&reset_cfg);
    if (err != ESP_OK) {
        return err;
    }
    gpio_set_level(RESET_GPIO, 0);

    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = ADC_UNIT,
    };
    err = adc_oneshot_new_unit(&unit_cfg, &s_adc_handle);
    if (err != ESP_OK) {
        return err;
    }

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH,
    };
    err = adc_oneshot_config_channel(s_adc_handle, ADC_CHANNEL, &chan_cfg);
    if (err != ESP_OK) {
        return err;
    }

    adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id = ADC_UNIT,
        .chan = ADC_CHANNEL,
        .atten = ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH,
    };
    err = adc_cali_create_scheme_curve_fitting(&cali_cfg, &s_cali_handle);
    if (err != ESP_OK) {
        return err;
    }

    integrator_reset();
    return ESP_OK;
}

static esp_err_t read_voltage_mv(int *mv)
{
    int raw;
    esp_err_t err = adc_oneshot_read(s_adc_handle, ADC_CHANNEL, &raw);
    if (err != ESP_OK) {
        return err;
    }
    return adc_cali_raw_to_voltage(s_cali_handle, raw, mv);
}

esp_err_t integrator_read_mv(int *mv)
{
    return read_voltage_mv(mv);
}

esp_err_t integrator_read_mv_oversampled(int *mv, int samples)
{
    int64_t sum = 0;
    for (int i = 0; i < samples; i++) {
        int sample;
        esp_err_t err = read_voltage_mv(&sample);
        if (err != ESP_OK) {
            return err;
        }
        sum += sample;
    }
    *mv = (int)(sum / samples);
    return ESP_OK;
}

void integrator_set_saturation_threshold_mv(int mv)
{
    s_saturation_high_mv = mv;
}

void integrator_reset(void)
{
    gpio_set_level(RESET_GPIO, 1);

    int64_t start = esp_timer_get_time();
    int mv = RESET_TARGET_MV + 1; // assume undischarged until proven otherwise
    while (esp_timer_get_time() - start < RESET_TIMEOUT_US) {
        esp_rom_delay_us(RESET_POLL_US);
        if (read_voltage_mv(&mv) == ESP_OK && mv <= RESET_TARGET_MV) {
            break;
        }
    }
    if (mv > RESET_TARGET_MV) {
        ESP_LOGW(TAG, "Reset did not bring ADC_OUT below %d mV (stuck at %d mV) - check the reset FET drive",
                 RESET_TARGET_MV, mv);
    }

    gpio_set_level(RESET_GPIO, 0);
    esp_rom_delay_us(RESET_SETTLE_US);

    s_sample_count = 0;
    s_sample_next = 0;
}

// Least-squares slope (mV/s) over the samples currently in the ring buffer.
static float compute_slope_mv_per_s(void)
{
    if (s_sample_count < 2) {
        return 0.0f;
    }

    double t_mean = 0.0, v_mean = 0.0;
    for (int i = 0; i < s_sample_count; i++) {
        t_mean += (double)s_sample_time_us[i];
        v_mean += (double)s_sample_mv[i];
    }
    t_mean /= s_sample_count;
    v_mean /= s_sample_count;

    double num = 0.0, den = 0.0;
    for (int i = 0; i < s_sample_count; i++) {
        double dt = (double)s_sample_time_us[i] - t_mean;
        double dv = (double)s_sample_mv[i] - v_mean;
        num += dt * dv;
        den += dt * dt;
    }
    if (den == 0.0) {
        return 0.0f;
    }

    double slope_mv_per_us = num / den;
    return (float)(slope_mv_per_us * 1e6);
}

esp_err_t integrator_poll(integrator_sample_t *out)
{
    int mv;
    esp_err_t err = read_voltage_mv(&mv);
    if (err != ESP_OK) {
        return err;
    }

    s_sample_time_us[s_sample_next] = esp_timer_get_time();
    s_sample_mv[s_sample_next] = (float)mv;
    s_sample_next = (s_sample_next + 1) % SLOPE_WINDOW;
    if (s_sample_count < SLOPE_WINDOW) {
        s_sample_count++;
    }

    out->voltage_mv = (float)mv;
    out->slope_mv_per_s = compute_slope_mv_per_s();
    out->reset_triggered = false;

    if (mv >= s_saturation_high_mv) {
        integrator_reset();
        out->reset_triggered = true;
    }

    portENTER_CRITICAL(&s_latest_lock);
    s_latest = *out;
    portEXIT_CRITICAL(&s_latest_lock);

    return ESP_OK;
}

void integrator_get_latest(integrator_sample_t *out)
{
    portENTER_CRITICAL(&s_latest_lock);
    *out = s_latest;
    portEXIT_CRITICAL(&s_latest_lock);
}
