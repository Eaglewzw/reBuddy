// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#define RB_ARM_JOINTS 6u
#define RB_ARM_CHANNELS 7u
#define RB_ARM_FEEDBACK_MAX_AGE_US 100000LL

typedef enum {
    RB_ARM_UNINITIALIZED, RB_ARM_STANDBY, RB_ARM_MONITORING, RB_ARM_FAULT
} rb_arm_state_t;

typedef struct {
    uint64_t timestamp_us;
    rb_arm_state_t state;
    esp_err_t error;
    bool bus_started, bus_healthy, planner_ready;
    /* Availability does not imply enable, calibrated coordinates or readiness
       to move. No motion commands are exposed until action commissioning. */
    uint8_t feedback_valid_mask;
    uint32_t fault[RB_ARM_CHANNELS];
    float position_rad[RB_ARM_CHANNELS];
    float velocity_rad_s[RB_ARM_CHANNELS];
    float torque_nm[RB_ARM_CHANNELS];
} rb_arm_status_t;

/* One-shot, before application tasks use this adapter; board must be ready.
 * Initializes the local RS model, disabled motor slots and idle planner.
 * monitor=true starts hardware LISTEN_ONLY, never active motor control.
 * A partial failure is latched; retry requires a reboot (upstream singletons).
 * No trajectory, calibration, motor enable, reset or zero write is performed.
 */
esp_err_t rb_arm_init(bool monitor);
/* Concurrent read-only snapshots after init returns. Invalid feedback is NaN.
 * Seven latest measurements are not an atomic physical joint sample.
 */
esp_err_t rb_arm_get_status(rb_arm_status_t *status);
