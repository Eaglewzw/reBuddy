#pragma once

/*
 * host_protocol.h — 上位机 CAN 协议解析（芯片无关）。
 * 可选 ingress：若上层通过主机 CAN 帧下发指令则用之；否则可直接调 motor_manager C API。
 */

#include <stdbool.h>
#include <stdint.h>

#include "driver/twai.h"

#include "config/app_config.h"
#include "core/motor_manager.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    HOST_OP_SET_MODE = 1,
    HOST_OP_ESTOP = 2,
    HOST_OP_SCAN = 3,
    HOST_OP_ENABLE = 4,
    HOST_OP_DISABLE = 5,
    HOST_OP_SET_ZERO = 6,
    HOST_OP_CLEAR_ERROR = 7,
    HOST_OP_SET_GAINS = 8,
    HOST_OP_SET_IDS = 9,
    HOST_OP_STOP = 10, /* 受控停机(减速/保持),与 DISABLE 区分 */
} host_admin_opcode_t;

typedef struct {
    host_admin_opcode_t op;
    uint8_t motor_id;
    motor_mode_t mode;
    float kp;
    float kd;
    uint8_t new_motor_id;
    uint8_t new_feedback_id;
    bool store_after_set;
    bool verify_after_set;
} host_admin_cmd_t;

bool host_protocol_parse_admin_cmd(const twai_message_t *msg, host_admin_cmd_t *out_cmd);
bool host_protocol_parse_mode_frame(const twai_message_t *msg, uint8_t *id, motor_mode_t *mode, damiao_cmd_t *out_cmd);

#ifdef __cplusplus
}
#endif


