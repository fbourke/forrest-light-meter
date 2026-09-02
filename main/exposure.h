#pragma once

#include <stdbool.h>
#include <stddef.h>

// ISO 2720 incident-light calibration constants (the C in N^2/t = E*S/C).
// Sekonic uses 250 for the flat disc and 340 for the dome; a bare sensor with
// no diffuser is closest to the flat case.
#define EV_CAL_C_FLAT 250.0f
#define EV_CAL_C_DOME 340.0f

// Granularity of the marked settings a camera actually offers.
typedef enum {
    EV_STEP_FULL,
    EV_STEP_HALF,
    EV_STEP_THIRD,
} ev_step_t;

// A marked shutter speed: exact duration plus the dial engraving.
typedef struct {
    float seconds;
    const char *label;
} ev_shutter_t;

// A marked f-number. `n` is the exact value the label stands for
// (f/11 is engraved on a 2^(7/2) = 11.31 stop).
typedef struct {
    float n;
    const char *label;
} ev_aperture_t;

// The three shutter/aperture display combinations a handheld meter offers.
// They are the legal DIP-switch states, not a free product of two settings:
// the tenths mode always pairs with full-stop shutter speeds.
typedef enum {
    EV_DISPLAY_FULL_TENTHS,  // T on full stops, F as a full stop + tenths digit
    EV_DISPLAY_HALF,         // T and F both on half stops
    EV_DISPLAY_THIRD,        // T and F both on third stops
} ev_display_t;

// What the screen shows for one measurement.
typedef struct {
    const char *shutter;   // engraving of the dialed speed, e.g. "1/125"
    const char *aperture;  // engraving of the answer, e.g. "5.6"
    int tenths;            // 0-9 residual in tenths mode, else -1
    float exact_aperture;  // unrounded solution, before display quantizing
    float err_stops;       // error of what is shown, + = overexposed
    bool out_of_range;     // answer falls outside f/1.0 - f/90
} ev_readout_t;

// One rung of the exposure scale for a given EV.
typedef struct {
    const ev_aperture_t *aperture;
    const ev_shutter_t *shutter;  // nearest marked speed
    float exact_seconds;          // before snapping
    float err_stops;              // exposure error of the marked pair, + = over
} ev_pair_t;

// EV for an incident illuminance, at the given film speed.
// Returns NAN for non-positive lux.
float ev_from_lux(float lux, float iso, float cal_c);

// Solve either side of the reciprocity relation N^2/t = 2^EV.
float ev_shutter_seconds(float ev, float aperture);
float ev_aperture(float ev, float seconds);

// Snap to the nearest marked setting. `err_stops` (optional) reports the
// exposure error the snap introduces, positive meaning overexposed.
const ev_shutter_t *ev_snap_shutter(float seconds, ev_step_t step, float *err_stops);
const ev_aperture_t *ev_snap_aperture(float aperture, ev_step_t step, float *err_stops);

// Every aperture whose partner shutter speed is within the marked scale.
// Writes up to `max` entries, returns how many.
int ev_pair_table(float ev, ev_step_t step, ev_pair_t *out, int max);

// "1/125" / "0.6s" for an arbitrary duration, for showing unsnapped values.
void ev_format_seconds(char *out, size_t n, float seconds);

// A handheld meter is shutter priority: you dial T, it answers with F.
// `seconds` is snapped to the mode's shutter scale first, so the answer
// belongs to the speed actually shown.
ev_readout_t ev_readout(float ev, float seconds, ev_display_t mode);

// "1/125  f/5.6 5" -- the readout as the screen would print it.
void ev_format_readout(char *out, size_t n, const ev_readout_t *r);

// Granularity of the marked scales this display mode dials on.
ev_step_t ev_display_step(ev_display_t mode);
const char *ev_display_str(ev_display_t mode);
