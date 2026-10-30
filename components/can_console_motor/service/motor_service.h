/*
 * SPDX-FileCopyrightText: 2026 XIAO-C5-CAN
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#include "can_console_motor_runtime.h"
#include "can_console_motor_client.h"
#include "core/motor_manager.h"
#include "protocol/host_protocol.h"
#include "transport/can_manager.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Start the single owner task for router and vendor transaction state. */
esp_err_t motor_service_start(void);

esp_err_t motor_service_rebind_disabled(
    const can_console_motor_binding_t *bindings, size_t count);

/** Install a startup-time observer, invoked on the corresponding CAN RX task. */
esp_err_t motor_service_set_observer(int channel,
                                     can_console_motor_bus_observer_t observer,
                                     void *context);

/** CAN callback ingress. The frame is copied into the service queue. */
void motor_service_handle_can_rx(can_channel_t channel,
                                 const twai_message_t *message);

/** Queue one administrative command for the service owner. */
esp_err_t motor_service_submit_admin(const host_admin_cmd_t *command);

/** Queue an inclusive range as one service event. */
esp_err_t motor_service_submit_admin_range(host_admin_opcode_t opcode,
                                           uint8_t first, uint8_t last);

typedef struct {
    uint8_t motor_id;
    motor_mode_t mode;
    damiao_cmd_t command;
} motor_service_mode_command_t;

/** Queue one mode/target command for the service owner. */
esp_err_t motor_service_submit_mode(uint8_t motor_id, motor_mode_t mode,
                                    const damiao_cmd_t *command);

/** Queue all targets together or none of them. */
esp_err_t motor_service_submit_modes(const motor_service_mode_command_t *commands,
                                     size_t count);
esp_err_t motor_service_submit_reference(const motor_service_mode_command_t *commands,
    size_t count, bool acquire, bool release, uint32_t *sequence);
esp_err_t motor_service_get_reference(uint32_t sequence, can_console_motor_reference_status_t *status);

/** Queue one control, vendor-maintenance and safety cycle. */
esp_err_t motor_service_submit_tick(float dt_s);

esp_err_t motor_service_fence_begin(int channel,uint32_t *token);
esp_err_t motor_service_fence_poll(uint32_t token,can_console_motor_fence_status_t *status);

#ifdef __cplusplus
}
#endif
