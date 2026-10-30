/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include "can_console_motor_runtime.h"
/* One writer per physical bus. NONE permits only RS telemetry requests.
 * UI clients retain their validated trajectory/calibration state machines;
 * all physical CAN traffic and write ownership are managed by this library. */
typedef enum { CAN_MOTOR_CLIENT_NONE, CAN_MOTOR_CLIENT_UI, CAN_MOTOR_CLIENT_USB,
    CAN_MOTOR_CLIENT_UI_NATIVE, /* UI using the typed motor service. */
    CAN_MOTOR_CLIENT_SERVICE, /* Internal transport identity, never an owner. */
    CAN_MOTOR_CLIENT_RAW, /* Exclusive USB bridge: no local polling/targets. */
    CAN_MOTOR_CLIENT_RAW_ESTOP, /* Latched bridge halt; only explicit disable traffic. */
    CAN_MOTOR_CLIENT_EMERGENCY, /* Internal disable identity, never an owner. */
    CAN_MOTOR_CLIENT_WEB /* Browser frontend using the typed service. */
} can_motor_client_t;
esp_err_t can_console_motor_sessions_init(void);
bool can_console_motor_session_acquire(int channel,can_motor_client_t client);
void can_console_motor_session_release(int channel,can_motor_client_t client);
can_motor_client_t can_console_motor_session_owner(int channel);
esp_err_t can_console_motor_session_send(int channel,can_motor_client_t client,
    const can_console_motor_frame_t *frame,uint32_t timeout_ms);

/* RAW ownership survives a link failure. Only explicit verified handoff releases it. */
bool can_console_motor_session_raw_active(int channel);
void can_console_motor_session_halt_raw(int channel);
/* While RAW owns the bus and hardware disable is freshly confirmed, discard
 * cached vendor mode/targets without changing bindings or transmitting frames. */
esp_err_t can_console_motor_reset_disabled_bus(int channel);
