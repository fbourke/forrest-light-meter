#pragma once

#include <stdbool.h>
#include "esp_err.h"

// One integrator reading: the filtered ADC_OUT voltage plus the current
// slope estimate (rate of charge on the integrator cap, i.e. proportional
// to instantaneous light intensity).
typedef struct {
    float voltage_mv;
    float slope_mv_per_s;
    bool reset_triggered; // true if this poll caused an auto-reset (saturation)
} integrator_sample_t;

// Configures the ADC_OUT input and the integrator RESET output.
esp_err_t integrator_init(void);

// Discharges the integrator cap (pulses RESET) and clears the slope history.
void integrator_reset(void);

// Raw ADC_OUT reading in mV, with no side effects (no slope bookkeeping, no
// saturation check) - for callers doing their own timed capture.
esp_err_t integrator_read_mv(int *mv);

// Same, but averages `samples` back-to-back reads to cut noise - for callers
// resolving small deltas (e.g. short exposures) where single-sample noise
// would swamp the signal.
esp_err_t integrator_read_mv_oversampled(int *mv, int samples);

// Sets the voltage integrator_poll() auto-resets at. Callers should restore
// this to a sane default when leaving a mode that lowered it.
void integrator_set_saturation_threshold_mv(int mv);

// Takes one ADC reading, updates the slope estimate over the recent
// sample window, and auto-resets if the output has saturated near a rail.
// Call this on a steady period (see INTEGRATOR_SAMPLE_PERIOD_MS).
esp_err_t integrator_poll(integrator_sample_t *out);

// Returns the most recent sample computed by integrator_poll(), for callers
// (e.g. the VEML7700 loop) that want to cross-reference it against another
// sensor without driving the ADC themselves.
void integrator_get_latest(integrator_sample_t *out);
