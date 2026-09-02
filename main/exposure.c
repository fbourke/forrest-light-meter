#include "exposure.h"

#include <math.h>
#include <stdio.h>

// Marked scales, ordered fastest/widest first. Each entry pairs the EXACT stop
// value used for all arithmetic with the dial engraving used for display: the
// speed engraved "1/125" is really 2^-7 = 1/128 s, and f/11 is 2^(7/2) = 11.31.
// Doing the math on the engraved numbers instead would bias exposure by up to
// 0.15 stop, so keep these two things separate.

static const ev_shutter_t k_shutter_full[] = {
    {0.00012207f, "1/8000"}, {0.000244141f, "1/4000"}, {0.000488281f, "1/2000"},
    {0.000976562f, "1/1000"}, {0.00195312f, "1/500"}, {0.00390625f, "1/250"},
    {0.0078125f, "1/125"}, {0.015625f, "1/60"}, {0.03125f, "1/30"},
    {0.0625f, "1/15"}, {0.125f, "1/8"}, {0.25f, "1/4"},
    {0.5f, "1/2"}, {1.0f, "1s"}, {2.0f, "2s"},
    {4.0f, "4s"}, {8.0f, "8s"}, {16.0f, "15s"},
    {32.0f, "30s"},
};

static const ev_shutter_t k_shutter_half[] = {
    {0.00012207f, "1/8000"}, {0.000172633f, "1/6000"}, {0.000244141f, "1/4000"},
    {0.000345267f, "1/3000"}, {0.000488281f, "1/2000"}, {0.000690534f, "1/1500"},
    {0.000976562f, "1/1000"}, {0.00138107f, "1/750"}, {0.00195312f, "1/500"},
    {0.00276214f, "1/350"}, {0.00390625f, "1/250"}, {0.00552427f, "1/180"},
    {0.0078125f, "1/125"}, {0.0110485f, "1/90"}, {0.015625f, "1/60"},
    {0.0220971f, "1/45"}, {0.03125f, "1/30"}, {0.0441942f, "1/20"},
    {0.0625f, "1/15"}, {0.0883883f, "1/10"}, {0.125f, "1/8"},
    {0.176777f, "1/6"}, {0.25f, "1/4"}, {0.353553f, "1/3"},
    {0.5f, "1/2"}, {0.707107f, "1/1.5"}, {1.0f, "1s"},
    {1.41421f, "1.5s"}, {2.0f, "2s"}, {2.82843f, "3s"},
    {4.0f, "4s"}, {5.65685f, "6s"}, {8.0f, "8s"},
    {11.3137f, "12s"}, {16.0f, "15s"}, {22.6274f, "20s"},
    {32.0f, "30s"},
};

static const ev_shutter_t k_shutter_third[] = {
    {0.00012207f, "1/8000"}, {0.000153799f, "1/6400"}, {0.000193775f, "1/5000"},
    {0.000244141f, "1/4000"}, {0.000307598f, "1/3200"}, {0.000387549f, "1/2500"},
    {0.000488281f, "1/2000"}, {0.000615196f, "1/1600"}, {0.000775098f, "1/1250"},
    {0.000976562f, "1/1000"}, {0.00123039f, "1/800"}, {0.0015502f, "1/640"},
    {0.00195312f, "1/500"}, {0.00246078f, "1/400"}, {0.00310039f, "1/320"},
    {0.00390625f, "1/250"}, {0.00492157f, "1/200"}, {0.00620079f, "1/160"},
    {0.0078125f, "1/125"}, {0.00984313f, "1/100"}, {0.0124016f, "1/80"},
    {0.015625f, "1/60"}, {0.0196863f, "1/50"}, {0.0248031f, "1/40"},
    {0.03125f, "1/30"}, {0.0393725f, "1/25"}, {0.0496063f, "1/20"},
    {0.0625f, "1/15"}, {0.0787451f, "1/13"}, {0.0992126f, "1/10"},
    {0.125f, "1/8"}, {0.15749f, "1/6"}, {0.198425f, "1/5"},
    {0.25f, "1/4"}, {0.31498f, "1/3"}, {0.39685f, "1/2.5"},
    {0.5f, "1/2"}, {0.629961f, "1/1.6"}, {0.793701f, "1/1.3"},
    {1.0f, "1s"}, {1.25992f, "1.3s"}, {1.5874f, "1.6s"},
    {2.0f, "2s"}, {2.51984f, "2.5s"}, {3.1748f, "3.2s"},
    {4.0f, "4s"}, {5.03968f, "5s"}, {6.3496f, "6s"},
    {8.0f, "8s"}, {10.0794f, "10s"}, {12.6992f, "13s"},
    {16.0f, "15s"}, {20.1587f, "20s"}, {25.3984f, "25s"},
    {32.0f, "30s"},
};

// Apertures run to f/90, the end of the analog scale on a handheld meter.

static const ev_aperture_t k_aperture_full[] = {
    {1.0f, "1.0"}, {1.41421f, "1.4"}, {2.0f, "2"}, {2.82843f, "2.8"},
    {4.0f, "4"}, {5.65685f, "5.6"}, {8.0f, "8"}, {11.3137f, "11"},
    {16.0f, "16"}, {22.6274f, "22"}, {32.0f, "32"}, {45.2548f, "45"},
    {64.0f, "64"}, {90.5097f, "90"},
};

// Half-stop engravings are NOT a subset of the third-stop ones:
// f/1.7, f/2.4, f/3.3 versus f/1.6, f/2.5, f/3.2.

static const ev_aperture_t k_aperture_half[] = {
    {1.0f, "1.0"}, {1.18921f, "1.2"}, {1.41421f, "1.4"}, {1.68179f, "1.7"},
    {2.0f, "2"}, {2.37841f, "2.4"}, {2.82843f, "2.8"}, {3.36359f, "3.3"},
    {4.0f, "4"}, {4.75683f, "4.8"}, {5.65685f, "5.6"}, {6.72717f, "6.7"},
    {8.0f, "8"}, {9.51366f, "9.5"}, {11.3137f, "11"}, {13.4543f, "13"},
    {16.0f, "16"}, {19.0273f, "19"}, {22.6274f, "22"}, {26.9087f, "27"},
    {32.0f, "32"}, {38.0546f, "38"}, {45.2548f, "45"}, {53.8174f, "54"},
    {64.0f, "64"}, {76.1093f, "76"}, {90.5097f, "90"},
};

static const ev_aperture_t k_aperture_third[] = {
    {1.0f, "1.0"}, {1.12246f, "1.1"}, {1.25992f, "1.2"}, {1.41421f, "1.4"},
    {1.5874f, "1.6"}, {1.7818f, "1.8"}, {2.0f, "2"}, {2.24492f, "2.2"},
    {2.51984f, "2.5"}, {2.82843f, "2.8"}, {3.1748f, "3.2"}, {3.56359f, "3.5"},
    {4.0f, "4"}, {4.48985f, "4.5"}, {5.03968f, "5"}, {5.65685f, "5.6"},
    {6.3496f, "6.3"}, {7.12719f, "7.1"}, {8.0f, "8"}, {8.9797f, "9"},
    {10.0794f, "10"}, {11.3137f, "11"}, {12.6992f, "13"}, {14.2544f, "14"},
    {16.0f, "16"}, {17.9594f, "18"}, {20.1587f, "20"}, {22.6274f, "22"},
    {25.3984f, "25"}, {28.5088f, "29"}, {32.0f, "32"}, {35.9188f, "36"},
    {40.3175f, "40"}, {45.2548f, "45"}, {50.7968f, "51"}, {57.0175f, "57"},
    {64.0f, "64"}, {71.8376f, "72"}, {80.6349f, "80"}, {90.5097f, "90"},
};

#define ARRAY_LEN(a) ((int)(sizeof(a) / sizeof((a)[0])))

static void shutter_scale(ev_step_t step, const ev_shutter_t **tab, int *n)
{
    switch (step) {
        case EV_STEP_FULL:
            *tab = k_shutter_full;
            *n = ARRAY_LEN(k_shutter_full);
            return;
        case EV_STEP_HALF:
            *tab = k_shutter_half;
            *n = ARRAY_LEN(k_shutter_half);
            return;
        case EV_STEP_THIRD:
        default:
            *tab = k_shutter_third;
            *n = ARRAY_LEN(k_shutter_third);
            return;
    }
}

static void aperture_scale(ev_step_t step, const ev_aperture_t **tab, int *n)
{
    switch (step) {
        case EV_STEP_FULL:
            *tab = k_aperture_full;
            *n = ARRAY_LEN(k_aperture_full);
            return;
        case EV_STEP_HALF:
            *tab = k_aperture_half;
            *n = ARRAY_LEN(k_aperture_half);
            return;
        case EV_STEP_THIRD:
        default:
            *tab = k_aperture_third;
            *n = ARRAY_LEN(k_aperture_third);
            return;
    }
}

float ev_from_lux(float lux, float iso, float cal_c)
{
    if (lux <= 0.0f) {
        return NAN;
    }
    return log2f(lux * iso / cal_c);
}

float ev_shutter_seconds(float ev, float aperture)
{
    // EV = log2(N^2 / t)  ->  t = N^2 / 2^EV
    return (aperture * aperture) / powf(2.0f, ev);
}

float ev_aperture(float ev, float seconds)
{
    // EV = log2(N^2 / t)  ->  N = sqrt(t * 2^EV)
    return sqrtf(seconds * powf(2.0f, ev));
}

const ev_shutter_t *ev_snap_shutter(float seconds, ev_step_t step, float *err_stops)
{
    const ev_shutter_t *tab;
    int n;
    shutter_scale(step, &tab, &n);

    int best = 0;
    float best_err = INFINITY;
    for (int i = 0; i < n; i++) {
        // A longer marked time overexposes, so the error is signed this way.
        float err = log2f(tab[i].seconds / seconds);
        if (fabsf(err) < fabsf(best_err)) {
            best_err = err;
            best = i;
        }
    }
    if (err_stops) {
        *err_stops = best_err;
    }
    return &tab[best];
}

const ev_aperture_t *ev_snap_aperture(float aperture, ev_step_t step, float *err_stops)
{
    const ev_aperture_t *tab;
    int n;
    aperture_scale(step, &tab, &n);

    int best = 0;
    float best_err = INFINITY;
    for (int i = 0; i < n; i++) {
        // One stop is a sqrt(2) change in f-number, and a larger f-number
        // underexposes -- hence the factor of two and the sign.
        float err = 2.0f * log2f(aperture / tab[i].n);
        if (fabsf(err) < fabsf(best_err)) {
            best_err = err;
            best = i;
        }
    }
    if (err_stops) {
        *err_stops = best_err;
    }
    return &tab[best];
}

int ev_pair_table(float ev, ev_step_t step, ev_pair_t *out, int max)
{
    if (isnan(ev) || max <= 0) {
        return 0;
    }
    const ev_aperture_t *ap;
    int n_ap;
    aperture_scale(step, &ap, &n_ap);
    const ev_shutter_t *sh;
    int n_sh;
    shutter_scale(step, &sh, &n_sh);

    float fastest = sh[0].seconds;
    float slowest = sh[n_sh - 1].seconds;

    int count = 0;
    for (int i = 0; i < n_ap && count < max; i++) {
        float t = ev_shutter_seconds(ev, ap[i].n);
        if (t < fastest || t > slowest) {
            continue;  // no marked speed can pair with this aperture
        }
        float err;
        out[count].aperture = &ap[i];
        out[count].shutter = ev_snap_shutter(t, step, &err);
        out[count].exact_seconds = t;
        out[count].err_stops = err;
        count++;
    }
    return count;
}

void ev_format_seconds(char *out, size_t n, float seconds)
{
    if (seconds >= 1.0f) {
        snprintf(out, n, "%.1fs", (double)seconds);
    } else {
        snprintf(out, n, "1/%.0f", (double)(1.0f / seconds));
    }
}

ev_step_t ev_display_step(ev_display_t mode)
{
    switch (mode) {
        case EV_DISPLAY_FULL_TENTHS: return EV_STEP_FULL;
        case EV_DISPLAY_HALF:        return EV_STEP_HALF;
        case EV_DISPLAY_THIRD:
        default:                     return EV_STEP_THIRD;
    }
}

const char *ev_display_str(ev_display_t mode)
{
    switch (mode) {
        case EV_DISPLAY_FULL_TENTHS: return "T full / F 1/10";
        case EV_DISPLAY_HALF:        return "T+F 1/2";
        case EV_DISPLAY_THIRD:
        default:                     return "T+F 1/3";
    }
}

ev_readout_t ev_readout(float ev, float seconds, ev_display_t mode)
{
    ev_readout_t r = {
        .shutter = "--",
        .aperture = "--",
        .tenths = -1,
        .exact_aperture = NAN,
        .err_stops = 0.0f,
        .out_of_range = false,
    };
    if (isnan(ev) || seconds <= 0.0f) {
        r.out_of_range = true;
        return r;
    }

    // The answer has to belong to the speed on the dial, so snap T first.
    const ev_shutter_t *t = ev_snap_shutter(seconds, ev_display_step(mode), NULL);
    r.shutter = t->label;

    float n = ev_aperture(ev, t->seconds);
    r.exact_aperture = n;

    // Every scale spans the same range, so range-check against the full one.
    const ev_aperture_t *full;
    int n_full;
    aperture_scale(EV_STEP_FULL, &full, &n_full);
    if (n < full[0].n || n > full[n_full - 1].n) {
        r.out_of_range = true;
        return r;
    }

    if (mode == EV_DISPLAY_FULL_TENTHS) {
        // A full-stop engraving plus a tenths-of-a-stop digit, as the meter
        // shows it: "5.6 5" is five tenths of a stop past f/5.6.
        float stops = 2.0f * log2f(n);  // stops above f/1.0
        int idx = (int)floorf(stops);
        int tenths = (int)lroundf((stops - (float)idx) * 10.0f);
        if (tenths >= 10) {  // rounded up onto the next whole stop
            idx++;
            tenths = 0;
        }
        if (idx >= n_full) {
            r.out_of_range = true;
            return r;
        }
        r.aperture = full[idx].label;
        r.tenths = tenths;
        r.err_stops = stops - ((float)idx + (float)tenths / 10.0f);
    } else {
        ev_step_t step = (mode == EV_DISPLAY_HALF) ? EV_STEP_HALF : EV_STEP_THIRD;
        float err;
        r.aperture = ev_snap_aperture(n, step, &err)->label;
        r.err_stops = err;
    }
    return r;
}

void ev_format_readout(char *out, size_t n, const ev_readout_t *r)
{
    if (r->out_of_range) {
        snprintf(out, n, "%s  f/-- over", r->shutter);
    } else if (r->tenths >= 0) {
        snprintf(out, n, "%s  f/%s %d", r->shutter, r->aperture, r->tenths);
    } else {
        snprintf(out, n, "%s  f/%s", r->shutter, r->aperture);
    }
}
