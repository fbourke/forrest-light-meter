#include <string.h>

#include "driver/gpio.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_continuous.h"
#include "esp_adc/adc_monitor.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "soc/soc_caps.h"

#include "integrator.h"

static const char *TAG = "integrator";

// Reset FET gate drive. Active high: shorts the integrator feedback cap
// (C4) to discharge it and pull ADC_OUT back near CHARGE NODE. Was a BJT
// (Q1) with a base resistor; swapped for a FET so this switches fast enough
// for ~1ms exposures - see RESET_SETTLE_US.
#define RESET_GPIO 0

// Rather than trust a fixed pulse width, hold the switch on and watch the
// sample stream until the voltage actually drops - a blind pulse can't tell
// a real discharge from a drive/switching issue that leaves it stuck.
#define RESET_TARGET_MV 500
#define RESET_TIMEOUT_US 50000

// Releasing the reset switch leaves a brief settling transient on ADC_OUT
// (charge injection / op-amp recovery) before it reaches its true resting
// baseline. Callers that treat "just after reset" as the zero point
// (metering.c) would otherwise see that transient misread as signal.
// Measured: with the FET this settles on roughly the same timescale the BJT
// needed (~15-20ms) - dropping this to 200us produced a huge, exponential-
// looking "delta" on short exposures (up to 240mV, plateauing over ~20ms)
// that tracked nothing about the real ambient rate. That rules out the
// switch itself as the bottleneck. The LM358's slew rate is 0.5 V/us, which
// is 1000x too fast to explain a 7ms fall, so the remaining suspects are the
// R4/C5 output filter and the LM358's ~50uA sink limit near ground. Both
// disappear with a rail-to-rail CMOS part; re-measure this after that swap.
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

// Hardware backstop, checked by the ADC's threshold monitor rather than by
// this driver. It only fires if the software check above somehow misses one,
// so it sits above every threshold metering.c sets. The monitor cannot be
// reconfigured while the converter runs, hence a fixed value here and the
// adjustable one in software. Raise this once the LM358 (which tops out
// around 2.3V on a 3.3V rail) is replaced by a rail-to-rail part.
#define HARD_CEILING_MV 2400

// ADC_OUT, downstream of U2A/R4/C5, on the ESP32-C6's internal ADC (GPIO1) -
// fast enough (microsecond-scale reads) to chase ~1ms exposures, which the
// ADS1115's ~1.2ms-per-conversion couldn't keep up with.
#define ADC_UNIT_USED ADC_UNIT_1
#define ADC_CHANNEL_USED ADC_CHANNEL_1 // GPIO1
#define ADC_ATTEN_USED ADC_ATTEN_DB_12 // full 0-3.3V range
#define ADC_RAW_MAX ((1 << SOC_ADC_DIGI_MAX_BITWIDTH) - 1)

// Continuous (DMA) mode instead of one-shot reads. The chip's ceiling is
// SOC_ADC_SAMPLE_FREQ_THRES_HIGH (83333 Hz); 50 kHz leaves margin and puts
// one conversion every 20us. A frame is the driver's wakeup granularity, so
// FRAME_SAMPLES sets how quickly saturation is noticed: 16 * 20us = 320us,
// versus the 20ms software poll this replaced.
#define SAMPLE_RATE_HZ 50000
#define FRAME_SAMPLES 16
#define FRAME_BYTES (FRAME_SAMPLES * SOC_ADC_DIGI_RESULT_BYTES)
#define POOL_FRAMES 8

// Number of decimated points used for the linear-regression slope estimate.
// One point per SLOPE_POINT_PERIOD_US keeps the ~320ms window the 20ms
// software poll used to give, but each point is now the mean of ~1000
// hardware-paced conversions rather than a single one.
#define SLOPE_WINDOW 16
#define SLOPE_POINT_PERIOD_US 20000

// RESET_SETTLE_US covers the bulk of the post-reset transient, but its tail
// runs past it and biases the ramp low - measured as a slope of 74 mV/s
// against a true 126 mV/s one point after a reset. A contaminated point stays
// in the regression for the whole SLOPE_WINDOW, so drop the first points of a
// fresh window rather than letting them in. Revisit (probably to 0) once the
// LM358 is replaced: this tail is an artifact of that part and the R4/C5
// filter, not of the integrator itself.
#define SKIP_POINTS_AFTER_RESET 2

// Raw-sample ring backing integrator_read_mv_oversampled().
#define RECENT_SAMPLES 256

static adc_continuous_handle_t s_adc;
static adc_cali_handle_t s_cali;
static adc_monitor_handle_t s_monitor;
static TaskHandle_t s_drain_task;
static SemaphoreHandle_t s_reset_done;

// Guards s_latest, written by the drain task and read by integrator_poll() /
// integrator_get_latest() from any other task.
static portMUX_TYPE s_latest_lock = portMUX_INITIALIZER_UNLOCKED;
static integrator_sample_t s_latest;
static bool s_reset_since_poll;
// The voltage that tripped the reset. reset_triggered survives until the next
// integrator_poll(), so the voltage reported with it has to be latched too -
// otherwise post-reset frames overwrite it within a frame or two and the
// caller logs the baseline instead of the trip point.
static float s_reset_voltage_mv;
static uint32_t s_reset_count;

// Thresholds are mirrored into the raw domain so the per-sample comparison
// is an integer compare; converting 50k samples/s through the calibration
// curve would be pure waste.
static int s_saturation_high_mv = SATURATION_HIGH_MV_DEFAULT;
static int s_saturation_high_raw = ADC_RAW_MAX;
static int s_reset_target_raw;

static portMUX_TYPE s_recent_lock = portMUX_INITIALIZER_UNLOCKED;
static uint16_t s_recent_raw[RECENT_SAMPLES];
static uint32_t s_recent_seq;  // total conversions ever stored

static int64_t s_point_time_us[SLOPE_WINDOW];
static float s_point_mv[SLOPE_WINDOW];
static int s_point_count;
static int s_point_next;

static int64_t s_acc_sum;
static int s_acc_n;
static int64_t s_acc_start_us;
static int s_skip_points;

static volatile bool s_reset_requested;
static volatile bool s_hw_ceiling_hit;

// ISR context: the monitor fires the instant a conversion crosses the
// ceiling (~12us at SAMPLE_RATE_HZ). The reset itself needs to poll the
// sample stream, so hand it to the drain task rather than doing it here.
static bool on_ceiling(adc_monitor_handle_t handle,
                       const adc_monitor_evt_data_t *data, void *ctx)
{
    s_hw_ceiling_hit = true;
    return false;
}

// The calibration curve only maps raw->mV, so invert it by bisection. It is
// monotonic, and 12 iterations is cheap next to doing this per sample.
static int raw_for_mv(int target_mv)
{
    int lo = 0, hi = ADC_RAW_MAX;
    while (lo < hi) {
        int mid = lo + (hi - lo) / 2;
        int mv = 0;
        if (adc_cali_raw_to_voltage(s_cali, mid, &mv) != ESP_OK) {
            return ADC_RAW_MAX;
        }
        if (mv < target_mv) {
            lo = mid + 1;
        } else {
            hi = mid;
        }
    }
    return lo;
}

static void push_recent(uint16_t raw)
{
    portENTER_CRITICAL(&s_recent_lock);
    s_recent_raw[s_recent_seq % RECENT_SAMPLES] = raw;
    s_recent_seq++;
    portEXIT_CRITICAL(&s_recent_lock);
}

// Least-squares slope (mV/s) over the decimated points currently held.
// Order does not matter to the fit, so the ring can be walked as a flat array.
static float compute_slope_mv_per_s(void)
{
    if (s_point_count < 2) {
        return 0.0f;
    }

    double t_mean = 0.0, v_mean = 0.0;
    for (int i = 0; i < s_point_count; i++) {
        t_mean += (double)s_point_time_us[i];
        v_mean += (double)s_point_mv[i];
    }
    t_mean /= s_point_count;
    v_mean /= s_point_count;

    double num = 0.0, den = 0.0;
    for (int i = 0; i < s_point_count; i++) {
        double dt = (double)s_point_time_us[i] - t_mean;
        double dv = (double)s_point_mv[i] - v_mean;
        num += dt * dv;
        den += dt * dt;
    }
    if (den == 0.0) {
        return 0.0f;
    }
    return (float)((num / den) * 1e6);
}

static void clear_history(void)
{
    s_point_count = 0;
    s_point_next = 0;
    s_acc_sum = 0;
    s_acc_n = 0;
    s_acc_start_us = esp_timer_get_time();
    s_skip_points = SKIP_POINTS_AFTER_RESET;
}

static void publish(float voltage_mv, bool recompute_slope)
{
    portENTER_CRITICAL(&s_latest_lock);
    s_latest.voltage_mv = voltage_mv;
    portEXIT_CRITICAL(&s_latest_lock);
    if (recompute_slope) {
        float slope = compute_slope_mv_per_s();
        portENTER_CRITICAL(&s_latest_lock);
        s_latest.slope_mv_per_s = slope;
        portEXIT_CRITICAL(&s_latest_lock);
    }
}

// adc_continuous_flush_pool() is only legal while the converter is stopped,
// so drop the backlog by reading it out and discarding it instead.
static void discard_backlog(void)
{
    adc_continuous_data_t frame[FRAME_SAMPLES];
    for (int i = 0; i < POOL_FRAMES * 2; i++) {
        uint32_t got = 0;
        if (adc_continuous_read_parse(s_adc, frame, FRAME_SAMPLES, &got, 0) != ESP_OK
                || got == 0) {
            return;
        }
    }
}

// Only ever called on the drain task, which owns the ADC and the RESET GPIO.
// Centralising it there means a saturation-triggered reset and a caller's
// explicit reset can never interleave.
static void do_reset(void)
{
    gpio_set_level(RESET_GPIO, 1);

    adc_continuous_data_t frame[FRAME_SAMPLES];
    int64_t start = esp_timer_get_time();
    int raw = ADC_RAW_MAX;  // assume undischarged until proven otherwise
    while (esp_timer_get_time() - start < RESET_TIMEOUT_US) {
        uint32_t got = 0;
        if (adc_continuous_read_parse(s_adc, frame, FRAME_SAMPLES, &got, 10) != ESP_OK) {
            continue;
        }
        for (uint32_t i = 0; i < got; i++) {
            if (frame[i].valid) {
                raw = (int)frame[i].raw_data;
            }
        }
        if (raw <= s_reset_target_raw) {
            break;
        }
    }
    if (raw > s_reset_target_raw) {
        int mv = 0;
        adc_cali_raw_to_voltage(s_cali, raw, &mv);
        ESP_LOGW(TAG, "Reset did not bring ADC_OUT below %d mV (stuck at %d mV) - check the reset FET drive",
                 RESET_TARGET_MV, mv);
    }

    gpio_set_level(RESET_GPIO, 0);
    esp_rom_delay_us(RESET_SETTLE_US);

    // Conversions taken during the discharge and the settling transient would
    // poison the next slope fit, so drop whatever queued up behind us.
    discard_backlog();
    clear_history();
    s_hw_ceiling_hit = false;
}

static void mark_reset(float trip_mv)
{
    portENTER_CRITICAL(&s_latest_lock);
    s_reset_since_poll = true;
    s_reset_voltage_mv = trip_mv;
    s_reset_count++;
    portEXIT_CRITICAL(&s_latest_lock);
}

static void drain_task(void *arg)
{
    adc_continuous_data_t frame[FRAME_SAMPLES * 2];

    while (true) {
        if (s_reset_requested) {
            s_reset_requested = false;
            float before = s_latest.voltage_mv;
            do_reset();
            mark_reset(before);
            xSemaphoreGive(s_reset_done);
            continue;
        }

        uint32_t got = 0;
        esp_err_t err = adc_continuous_read_parse(s_adc, frame, FRAME_SAMPLES * 2,
                                                  &got, 100);
        if (err != ESP_OK || got == 0) {
            continue;
        }

        int64_t frame_sum = 0;
        int frame_n = 0;
        int peak_raw = -1;
        for (uint32_t i = 0; i < got; i++) {
            if (!frame[i].valid) {
                continue;
            }
            int raw = (int)frame[i].raw_data;
            push_recent((uint16_t)raw);
            frame_sum += raw;
            frame_n++;
            if (raw > peak_raw) {
                peak_raw = raw;
            }
        }
        if (frame_n == 0) {
            continue;
        }

        int frame_mv = 0;
        adc_cali_raw_to_voltage(s_cali, (int)(frame_sum / frame_n), &frame_mv);

        s_acc_sum += frame_sum;
        s_acc_n += frame_n;

        int64_t now = esp_timer_get_time();
        bool new_point = false;
        if (now - s_acc_start_us >= SLOPE_POINT_PERIOD_US) {
            int point_mv = 0;
            adc_cali_raw_to_voltage(s_cali, (int)(s_acc_sum / s_acc_n), &point_mv);
            if (s_skip_points > 0) {
                s_skip_points--;  // still riding the post-reset tail
            } else {
                s_point_time_us[s_point_next] = now;
                s_point_mv[s_point_next] = (float)point_mv;
                s_point_next = (s_point_next + 1) % SLOPE_WINDOW;
                if (s_point_count < SLOPE_WINDOW) {
                    s_point_count++;
                }
                new_point = true;
            }
            s_acc_sum = 0;
            s_acc_n = 0;
            s_acc_start_us = now;
        }
        publish((float)frame_mv, new_point);

        // Saturation is now judged on the frame peak, so a spike that clips
        // between two slope points still gets caught.
        if (peak_raw >= s_saturation_high_raw || s_hw_ceiling_hit) {
            int mv = 0;
            adc_cali_raw_to_voltage(s_cali, peak_raw, &mv);
            do_reset();
            mark_reset((float)mv);  // report the voltage that tripped it
        }
    }
}

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

    adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id = ADC_UNIT_USED,
        .chan = ADC_CHANNEL_USED,
        .atten = ADC_ATTEN_USED,
        .bitwidth = SOC_ADC_DIGI_MAX_BITWIDTH,
    };
    err = adc_cali_create_scheme_curve_fitting(&cali_cfg, &s_cali);
    if (err != ESP_OK) {
        return err;
    }
    s_reset_target_raw = raw_for_mv(RESET_TARGET_MV);
    s_saturation_high_raw = raw_for_mv(s_saturation_high_mv);

    adc_continuous_handle_cfg_t hdl_cfg = {
        .max_store_buf_size = FRAME_BYTES * POOL_FRAMES,
        .conv_frame_size = FRAME_BYTES,
        // Prefer dropping stale conversions over stalling if the drain task
        // is held off (the 20ms reset settle does exactly that).
        .flags.flush_pool = true,
    };
    err = adc_continuous_new_handle(&hdl_cfg, &s_adc);
    if (err != ESP_OK) {
        return err;
    }

    adc_digi_pattern_config_t pattern = {
        .atten = ADC_ATTEN_USED,
        .channel = ADC_CHANNEL_USED,
        .unit = ADC_UNIT_USED,
        .bit_width = SOC_ADC_DIGI_MAX_BITWIDTH,
    };
    adc_continuous_config_t cont_cfg = {
        .pattern_num = 1,
        .adc_pattern = &pattern,
        .sample_freq_hz = SAMPLE_RATE_HZ,
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE2,
    };
    err = adc_continuous_config(s_adc, &cont_cfg);
    if (err != ESP_OK) {
        return err;
    }

    // The monitor has to be installed before the converter starts.
    adc_monitor_config_t mon_cfg = {
        .adc_unit = ADC_UNIT_USED,
        .channel = ADC_CHANNEL_USED,
        .h_threshold = raw_for_mv(HARD_CEILING_MV),
        .l_threshold = -1,
    };
    err = adc_new_continuous_monitor(s_adc, &mon_cfg, &s_monitor);
    if (err == ESP_OK) {
        adc_monitor_evt_cbs_t cbs = { .on_over_high_thresh = on_ceiling };
        err = adc_continuous_monitor_register_event_callbacks(s_monitor, &cbs, NULL);
        if (err == ESP_OK) {
            err = adc_continuous_monitor_enable(s_monitor);
        }
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "hardware ceiling monitor unavailable (%s); relying on the software check",
                     esp_err_to_name(err));
        }
    } else {
        ESP_LOGW(TAG, "could not install ceiling monitor (%s); relying on the software check",
                 esp_err_to_name(err));
    }

    s_reset_done = xSemaphoreCreateBinary();
    if (s_reset_done == NULL) {
        return ESP_ERR_NO_MEM;
    }

    err = adc_continuous_start(s_adc);
    if (err != ESP_OK) {
        return err;
    }

    clear_history();
    do_reset();  // safe here: the drain task does not exist yet

    // Above the metering and console tasks (both priority 5): if this one is
    // starved the DMA pool overflows and conversions are lost.
    if (xTaskCreate(drain_task, "adc_drain", 4096, NULL, 6, &s_drain_task) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "ADC continuous: %d Hz, %d-sample frames (%dus), slope over %d x %dms",
             SAMPLE_RATE_HZ, FRAME_SAMPLES,
             FRAME_SAMPLES * 1000000 / SAMPLE_RATE_HZ,
             SLOPE_WINDOW, SLOPE_POINT_PERIOD_US / 1000);
    return ESP_OK;
}

esp_err_t integrator_read_mv(int *mv)
{
    uint32_t seq;
    uint16_t raw;
    portENTER_CRITICAL(&s_recent_lock);
    seq = s_recent_seq;
    raw = s_recent_raw[(s_recent_seq - 1) % RECENT_SAMPLES];
    portEXIT_CRITICAL(&s_recent_lock);
    if (seq == 0) {
        return ESP_ERR_INVALID_STATE;
    }
    return adc_cali_raw_to_voltage(s_cali, raw, mv);
}

esp_err_t integrator_read_mv_oversampled(int *mv, int samples)
{
    if (samples <= 0 || samples > RECENT_SAMPLES) {
        return ESP_ERR_INVALID_ARG;
    }

    // Wait for `samples` conversions strictly newer than this call, so the
    // answer reflects the voltage now rather than a frame captured before the
    // caller's exposure ended. At SAMPLE_RATE_HZ that is samples * 20us plus
    // at most one frame of latency.
    uint32_t start_seq;
    portENTER_CRITICAL(&s_recent_lock);
    start_seq = s_recent_seq;
    portEXIT_CRITICAL(&s_recent_lock);

    int64_t deadline = esp_timer_get_time() + 100000;
    uint32_t seq;
    for (;;) {
        portENTER_CRITICAL(&s_recent_lock);
        seq = s_recent_seq;
        portEXIT_CRITICAL(&s_recent_lock);
        if (seq - start_seq >= (uint32_t)samples) {
            break;
        }
        if (esp_timer_get_time() >= deadline) {
            return ESP_ERR_TIMEOUT;
        }
        esp_rom_delay_us(FRAME_SAMPLES * 1000000 / SAMPLE_RATE_HZ);
    }

    // Averaged in the raw domain, then converted once: the calibration curve
    // is close enough to linear over a few LSBs that this is equivalent, and
    // it avoids `samples` curve evaluations.
    int64_t sum = 0;
    portENTER_CRITICAL(&s_recent_lock);
    for (int i = 0; i < samples; i++) {
        sum += s_recent_raw[(s_recent_seq - 1 - (uint32_t)i) % RECENT_SAMPLES];
    }
    portEXIT_CRITICAL(&s_recent_lock);

    return adc_cali_raw_to_voltage(s_cali, (int)(sum / samples), mv);
}

void integrator_set_saturation_threshold_mv(int mv)
{
    s_saturation_high_mv = mv;
    s_saturation_high_raw = raw_for_mv(mv);
}

void integrator_reset(void)
{
    if (s_drain_task == NULL || xTaskGetCurrentTaskHandle() == s_drain_task) {
        do_reset();
        return;
    }
    s_reset_requested = true;
    if (xSemaphoreTake(s_reset_done, pdMS_TO_TICKS(500)) != pdTRUE) {
        ESP_LOGW(TAG, "reset request was not serviced within 500ms");
    }
}

esp_err_t integrator_poll(integrator_sample_t *out)
{
    portENTER_CRITICAL(&s_latest_lock);
    *out = s_latest;
    out->reset_triggered = s_reset_since_poll;
    if (s_reset_since_poll) {
        out->voltage_mv = s_reset_voltage_mv;
    }
    s_reset_since_poll = false;
    portEXIT_CRITICAL(&s_latest_lock);
    return ESP_OK;
}

uint32_t integrator_reset_count(void)
{
    portENTER_CRITICAL(&s_latest_lock);
    uint32_t count = s_reset_count;
    portEXIT_CRITICAL(&s_latest_lock);
    return count;
}

void integrator_get_latest(integrator_sample_t *out)
{
    portENTER_CRITICAL(&s_latest_lock);
    *out = s_latest;
    portEXIT_CRITICAL(&s_latest_lock);
}
