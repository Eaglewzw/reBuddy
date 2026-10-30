#pragma once

/*
 * frame_router.h — RX 帧分发 + host 指令派发（芯片无关）。
 *
 * These functions run only on the motor-service owner task. Applications and
 * debug frontends submit work through can_console_motor_client.h.
 */

#include <stdint.h>

#include "core/motor_manager.h"
#include "protocol/host_protocol.h"
#include "transport/can_manager.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Classify and process one received frame on the service owner task. */
void frame_router_process_can_rx(can_channel_t ch, const twai_message_t *msg);

/* Process one already-decoded administrative command on the owner task. */
void frame_router_process_admin_cmd(const host_admin_cmd_t *cmd);

/* Process one mode command through the vendor prepare/switch path. */
bool frame_router_process_mode_cmd(uint8_t id, motor_mode_t mode,
                                   const damiao_cmd_t *cmd);

/* Run vendor transaction maintenance on the service owner task. */
void frame_router_process_reg_verify_tick(void);

/* Refresh the manager-owned readiness snapshots after an owner event. */
void frame_router_refresh_mode_readiness(void);

#ifdef __cplusplus
}
#endif
