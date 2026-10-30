/*
 * SPDX-FileCopyrightText: 2026 XIAO-C5-CAN
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stddef.h>
#include <stdint.h>

#include "can_console_motor_types.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Return the highest configured motor ID. ID zero remains the legacy slot. */
uint8_t can_console_motor_max_id(void);

/** Copy one motor's public state without exposing manager or vendor internals. */
esp_err_t can_console_motor_get_state(uint8_t motor_id,
                                      can_console_motor_state_t *state);

/** Bind a motor slot while it is disabled. model may be NULL. */
esp_err_t can_console_motor_bind(uint8_t motor_id, const char *vendor,
                                const char *model, int channel);

/** Reconfigure disabled slots on the service owner, after older queued work.
 * Requested motors must be disabled. Validates the entire group first;
 * resets their vendor sessions and invalidates their old feedback/commands.
 * Other motor slots retain their state and sessions. No CAN enable
 * is sent. Caller must exclude competing commands until this returns.
 * Queue-full returns TIMEOUT without changes; once queued, waits for completion
 * (or cancellation by ESTOP). Do not call from a motor-service callback. */
esp_err_t can_console_motor_rebind_disabled(
    const can_console_motor_binding_t *bindings, size_t count);

/** Remove a disabled slot from all CAN channels until it is bound again. */
esp_err_t can_console_motor_park(uint8_t motor_id);

/** Update MIT gains after validating them against the selected model limits. */
esp_err_t can_console_motor_set_gains(uint8_t motor_id, float kp, float kd);

/** Validate and queue one target through the normal vendor mode path. */
esp_err_t can_console_motor_set_target(
    const can_console_motor_target_t *target);

/** Validate and queue up to eight targets as one service event. */
esp_err_t can_console_motor_set_targets(
    const can_console_motor_target_t *targets, size_t count);

typedef enum { CAN_CONSOLE_MOTOR_REFERENCE_PENDING, CAN_CONSOLE_MOTOR_REFERENCE_DELIVERED,
    CAN_CONSOLE_MOTOR_REFERENCE_FAILED } can_console_motor_reference_state_t;
typedef struct {
    uint32_t sequence;
    bool owned;
    can_console_motor_reference_state_t state;
    int64_t queued_us;
    int64_t confirmed_us;
    esp_err_t error;
} can_console_motor_reference_status_t;

/** One exclusive MIT group, one in-flight batch. acquire=true first reserves
 * the IDs, drains older service/CAN work, then delivers the initial hold.
 * Subsequent submissions require the same IDs and prior delivery confirmation.
 * Regular control ticks are suppressed for this group until release/admin.
 * Caller coordinates competing targets/admin/gains; this is not a general
 * multi-client arbiter. Confirmation is TX completion, not physical execution.
 * TIMEOUT means nothing queued, retry without advancing the point. */
esp_err_t can_console_motor_submit_reference(const can_console_motor_target_t *targets,
    size_t count, bool acquire, uint32_t *sequence);
/** Ordered release after final confirmation; targets retain their hold. */
esp_err_t can_console_motor_release_reference(uint32_t *sequence);
esp_err_t can_console_motor_get_reference_status(uint32_t sequence,
    can_console_motor_reference_status_t *status);

/* Ordered service + CAN TX fence, one in flight. Caller excludes competing
 * command sources. Completion confirms prior sends, not device state. */
typedef struct { bool complete; int64_t confirmed_us; esp_err_t error; } can_console_motor_fence_status_t;
esp_err_t can_console_motor_fence_begin(int channel,uint32_t *token);
esp_err_t can_console_motor_fence_poll(uint32_t token,can_console_motor_fence_status_t *status);

/**
 * Validate and accept an administrative command without constructing a host
 * CAN frame. ESP_OK means accepted; device completion is reflected in state.
 */
esp_err_t can_console_motor_submit_admin(
    const can_console_motor_admin_command_t *command);

/** Submit one administrative action for every motor in an inclusive range. */
esp_err_t can_console_motor_submit_admin_range(
    const can_console_motor_admin_range_t *command);

/**
 * Accept an asynchronous scan request on one configured CAN channel.
 * ESP_OK does not mean the scan has completed.
 */
esp_err_t can_console_motor_scan(
    const can_console_motor_scan_request_t *request);

/** Stop an active asynchronous scan after its caller has a verdict. */
esp_err_t can_console_motor_stop_scan(int channel);

/** Copy read-only CAN channel counters for diagnostic frontends. */
esp_err_t can_console_motor_get_bus_state(
    int channel, int32_t recent_window_ms,
    can_console_motor_bus_state_t *state);

#ifdef __cplusplus
}
#endif
