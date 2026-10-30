/*
 * SPDX-FileCopyrightText: 2026 XIAO-C5-CAN
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CAN_CONSOLE_MOTOR_MAX_TARGET_GROUP 8U
#define CAN_CONSOLE_MOTOR_VENDOR_NAME_SIZE 16U
#define CAN_CONSOLE_MOTOR_FW_VERSION_SIZE   4U

/** Control modes understood by the motor service. */
typedef enum {
    MOTOR_MODE_DISABLED = 0,
    MOTOR_MODE_MIT = 1,
    MOTOR_MODE_POS_VEL = 2,
    MOTOR_MODE_VEL = 3,
    MOTOR_MODE_FORCE_POS = 4,
    MOTOR_MODE_POS_CSP = 5,
    MOTOR_MODE_POS_PP = 6,

    /* Public names; legacy constants above remain valid during migration. */
    CAN_CONSOLE_MOTOR_MODE_DISABLED = MOTOR_MODE_DISABLED,
    CAN_CONSOLE_MOTOR_MODE_MIT = MOTOR_MODE_MIT,
    CAN_CONSOLE_MOTOR_MODE_POS_VEL = MOTOR_MODE_POS_VEL,
    CAN_CONSOLE_MOTOR_MODE_VEL = MOTOR_MODE_VEL,
    CAN_CONSOLE_MOTOR_MODE_FORCE_POS = MOTOR_MODE_FORCE_POS,
    CAN_CONSOLE_MOTOR_MODE_POS_CSP = MOTOR_MODE_POS_CSP,
    CAN_CONSOLE_MOTOR_MODE_POS_PP = MOTOR_MODE_POS_PP,
} motor_mode_t;

typedef motor_mode_t can_console_motor_mode_t;

/** One disabled slot's vendor/model, channel and normal MIT gains. */
typedef struct {
    uint8_t motor_id;
    const char *vendor;
    const char *model;
    int channel;
    float kp;
    float kd;
} can_console_motor_binding_t;

typedef struct {
    float position;
    float velocity;
    float torque;
    float velocity_limit;
    float force_ratio;
    float acceleration;
} can_console_motor_target_value_t;

/** One typed target submitted by a control client. */
typedef struct {
    uint8_t motor_id;
    can_console_motor_mode_t mode;
    can_console_motor_target_value_t value;
} can_console_motor_target_t;

typedef enum {
    CAN_CONSOLE_MOTOR_ADMIN_ENABLE = 0,
    CAN_CONSOLE_MOTOR_ADMIN_DISABLE,
    CAN_CONSOLE_MOTOR_ADMIN_STOP,
    CAN_CONSOLE_MOTOR_ADMIN_ESTOP,
    CAN_CONSOLE_MOTOR_ADMIN_SET_ZERO,
    CAN_CONSOLE_MOTOR_ADMIN_CLEAR_ERROR,
} can_console_motor_admin_action_t;

/** motor_id == 0 addresses every configured motor. */
typedef struct {
    uint8_t motor_id;
    can_console_motor_admin_action_t action;
} can_console_motor_admin_command_t;

/** Inclusive motor range for arm-level administrative operations. */
typedef struct {
    uint8_t first_motor_id;
    uint8_t last_motor_id;
    can_console_motor_admin_action_t action;
} can_console_motor_admin_range_t;

typedef struct {
    int channel;
    uint8_t first_motor_id;
    uint8_t last_motor_id;
} can_console_motor_scan_request_t;

typedef enum {
    CAN_CONSOLE_MOTOR_BUS_STOPPED = 0,
    CAN_CONSOLE_MOTOR_BUS_ACTIVE,
    CAN_CONSOLE_MOTOR_BUS_WARNING,
    CAN_CONSOLE_MOTOR_BUS_PASSIVE,
    CAN_CONSOLE_MOTOR_BUS_OFF,
} can_console_motor_bus_status_t;

/** Read-only transport diagnostics copied without exposing the CAN handle. */
typedef struct {
    bool started;
    bool recent_rx;
    can_console_motor_bus_status_t status;
    uint16_t tx_error_count;
    uint16_t rx_error_count;
    uint32_t bus_error_count;
    uint32_t tx_queued_count;
    uint32_t tx_completed_count;
    uint32_t tx_failed_count;
    uint32_t rx_received_count;
    uint32_t rx_dropped_count;
} can_console_motor_bus_state_t;

/** Immutable snapshot returned to applications and debug frontends. */
typedef struct {
    uint8_t motor_id;
    bool online;
    bool enabled;
    bool mode_ready;
    can_console_motor_mode_t mode;
    int channel;
    char vendor_name[CAN_CONSOLE_MOTOR_VENDOR_NAME_SIZE];

    float position;
    float velocity;
    float torque;
    float mos_temperature;
    float rotor_temperature;
    uint16_t status;
    int64_t last_seen_us;
    int64_t last_feedback_us;

    can_console_motor_target_value_t target;
    float kp;
    float kd;
    float position_min;
    float position_max;
    float velocity_min;
    float velocity_max;
    float torque_min;
    float torque_max;

    uint8_t host_id;
    bool host_id_valid;
    uint32_t fault_raw;
    uint32_t warning_raw;
    int64_t fault_seen_us;
    uint8_t firmware_version[CAN_CONSOLE_MOTOR_FW_VERSION_SIZE];
    int64_t firmware_seen_us;
} can_console_motor_state_t;

#ifdef __cplusplus
}
#endif
