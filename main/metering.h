#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    // arg_ms == 0: switch to the continuous live ambient monitor.
    // arg_ms  > 0: one-shot timed exposure, report mV/s, then return to live.
    METER_CMD_AMBIENT,
    // Arm un-corded flash detection: watch the free-running integrator for a
    // slope discontinuity and report the ambient-corrected jump.
    METER_CMD_ARM_UNCORDED,
    // Reset, fire the sync trigger, integrate for arg_ms, report the result,
    // then return to live ambient monitoring.
    METER_CMD_CORDED,
    // arg_ms != 0: enable the rolling ambient log line; 0: silence it. Off
    // by default since it otherwise scrolls the console while typing.
    METER_CMD_SET_LIVE_LOG,
} meter_cmd_type_t;

typedef struct {
    meter_cmd_type_t type;
    uint32_t arg_ms;
} meter_cmd_t;

// Starts the integrator and the metering task (owns all integrator access).
void metering_init(void);

// Thread-safe: queues a command for the metering task to act on next.
void metering_submit_command(meter_cmd_t cmd);

// True if the rolling ambient log (and the VEML7700 calibration cross-check
// in main.c) should print. See METER_CMD_SET_LIVE_LOG.
bool metering_live_log_enabled(void);
