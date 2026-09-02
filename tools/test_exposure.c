// Host-side tests for the exposure math. No hardware needed:
//   cc -I main -o /tmp/test_exposure tools/test_exposure.c main/exposure.c -lm && /tmp/test_exposure
#include "exposure.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int failures = 0;

static void fail(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    printf("  FAIL ");
    vprintf(fmt, ap);
    printf("\n");
    va_end(ap);
    failures++;
}

static float engraved(const char *label)
{
    if (strchr(label, '/')) {
        return 1.0f / (float)atof(label + 2);
    }
    size_t n = strlen(label);
    if (n && label[n - 1] == 's') {
        return (float)atof(label);
    }
    return (float)atof(label);
}

static float granularity(ev_display_t mode)
{
    switch (mode) {
        case EV_DISPLAY_FULL_TENTHS: return 0.1f;
        case EV_DISPLAY_HALF:        return 0.5f;
        default:                     return 1.0f / 3.0f;
    }
}

// A readout must be internally consistent: the aperture it solved has to
// reproduce the EV it was given, and the value it displays has to sit on the
// mode's grid within half a step.
static void check_readout(float ev, float secs, ev_display_t mode)
{
    ev_readout_t r = ev_readout(ev, secs, mode);
    if (r.out_of_range) {
        return;
    }
    const ev_shutter_t *t = ev_snap_shutter(secs, ev_display_step(mode), NULL);

    float ev_rt = 2.0f * log2f(r.exact_aperture) - log2f(t->seconds);
    if (fabsf(ev_rt - ev) > 2e-3f) {
        fail("EV %.3f %s: round trip gave %.4f", ev, ev_display_str(mode), ev_rt);
    }

    float step = granularity(mode);
    float shown = 2.0f * log2f(r.exact_aperture) - r.err_stops;
    float off_grid = fabsf(shown / step - roundf(shown / step)) * step;
    if (off_grid > 2e-3f) {
        fail("EV %.3f %s: shown %.4f stop is %.4f off the %.3f grid",
             ev, ev_display_str(mode), shown, off_grid, step);
    }
    if (fabsf(r.err_stops) > step / 2.0f + 2e-3f) {
        fail("EV %.3f %s: err %.4f exceeds half of %.3f",
             ev, ev_display_str(mode), r.err_stops, step);
    }

    // The engraving shown must match the stop it stands for.
    float shown_from_label = 2.0f * log2f(engraved(r.aperture))
                             + (r.tenths >= 0 ? (float)r.tenths / 10.0f : 0.0f);
    if (fabsf(shown_from_label - shown) > 0.16f) {
        fail("EV %.3f %s: label f/%s%s implies %.3f stop, shown %.3f",
             ev, ev_display_str(mode), r.aperture,
             r.tenths >= 0 ? " +tenths" : "", shown_from_label, shown);
    }
}

static void expect_readout(float ev, float secs, ev_display_t mode,
                           const char *want)
{
    ev_readout_t r = ev_readout(ev, secs, mode);
    char got[32];
    ev_format_readout(got, sizeof(got), &r);
    if (strcmp(got, want) != 0) {
        fail("EV %.2f %s: got \"%s\", want \"%s\"", ev, ev_display_str(mode), got, want);
    } else {
        printf("  ok   EV %-5.2f %-15s -> %s\n", ev, ev_display_str(mode), got);
    }
}

int main(void)
{
    printf("spot checks\n");
    // Sunny 16: EV 15 at ISO 100 is f/16 at 1/125, exactly on a full stop.
    expect_readout(15.0f, 1.0f / 125, EV_DISPLAY_FULL_TENTHS, "1/125  f/16 0");
    expect_readout(15.0f, 1.0f / 125, EV_DISPLAY_HALF,        "1/125  f/16");
    expect_readout(15.0f, 1.0f / 125, EV_DISPLAY_THIRD,       "1/125  f/16");
    // EV 10 at 1/60 is f/4.
    expect_readout(10.0f, 1.0f / 60, EV_DISPLAY_THIRD, "1/60  f/4");
    // The 131 lux we measured on the bench, at a usable speed.
    expect_readout(5.71f, 1.0f, EV_DISPLAY_FULL_TENTHS, "1s  f/5.6 7");
    // Dim light at a fast speed has no answer on the f/1.0-f/90 scale.
    expect_readout(5.71f, 1.0f / 125, EV_DISPLAY_THIRD, "1/125  f/-- over");

    printf("\nsweep\n");
    int samples = 0;
    for (int i = 0; i <= 1900; i++) {
        float ev = 1.0f + i * 0.01f;
        for (int k = -13; k <= 5; k++) {
            float secs = powf(2.0f, (float)k);
            for (int m = 0; m <= EV_DISPLAY_THIRD; m++) {
                check_readout(ev, secs, (ev_display_t)m);
                samples++;
            }
        }
    }
    printf("  %d readouts checked\n", samples);

    printf("\nev_from_lux\n");
    // EV 0 is lux*ISO/C == 1, so 2.5 lux at ISO 100 with the flat constant.
    int before = failures;
    float ev0 = ev_from_lux(2.5f, 100.0f, EV_CAL_C_FLAT);
    if (fabsf(ev0) > 1e-4f) fail("2.5 lux should be EV 0, got %.4f", ev0);
    // 250 lux is then a hundredfold more light: log2(100) = 6.644 stops.
    float ev250 = ev_from_lux(250.0f, 100.0f, EV_CAL_C_FLAT);
    if (fabsf(ev250 - 6.6439f) > 1e-3f) fail("250 lux gave EV %.4f", ev250);
    // Doubling illuminance is one stop; quadrupling ISO is two.
    float a = ev_from_lux(100.0f, 100.0f, EV_CAL_C_FLAT);
    float b = ev_from_lux(200.0f, 100.0f, EV_CAL_C_FLAT);
    float c = ev_from_lux(100.0f, 400.0f, EV_CAL_C_FLAT);
    if (fabsf((b - a) - 1.0f) > 1e-4f) fail("doubling lux gave %.4f stop", b - a);
    if (fabsf((c - a) - 2.0f) > 1e-4f) fail("4x ISO gave %.4f stop", c - a);
    if (!isnan(ev_from_lux(0.0f, 100.0f, EV_CAL_C_FLAT))) fail("0 lux should be NAN");
    if (failures == before) {
        printf("  ok   2.5 lx = EV 0, reciprocity in lux and ISO holds\n");
    }

    printf("\n%s (%d failure%s)\n", failures ? "FAILED" : "PASSED",
           failures, failures == 1 ? "" : "s");
    return failures ? 1 : 0;
}
