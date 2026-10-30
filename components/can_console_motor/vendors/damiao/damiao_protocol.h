#pragma once

/*
 * damiao_protocol.h — Damiao 协议编解码（芯片无关）。
 */

#include <stdbool.h>
#include <stdint.h>

#include "driver/twai.h"

#include "core/motor_manager.h"
#include "protocol/host_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

void damiao_build_scan_request(uint8_t id, twai_message_t *out);
void damiao_build_admin_frame(uint8_t id, host_admin_opcode_t op,
                              uint8_t host_id, twai_message_t *out);
bool damiao_build_control_frame(uint8_t id,
                                motor_mode_t mode,
                                const damiao_cmd_t *cmd,
                                const motor_params_t *params,
                                twai_message_t *out);
bool damiao_parse_feedback(const twai_message_t *msg,
                           const motor_params_t *params,
                           uint8_t *id,
                           float *pos,
                           float *vel,
                           float *torque,
                           float *t_mos,
                           float *t_rotor,
                           uint16_t *status);
/* 按 Damiao 型号查 PvT 量程,与 motorbridge 的 DAMIAO_MODELS 对齐。 */
bool damiao_lookup_model_limits(const char *model,
                                float *pmax, float *vmax, float *tmax);
/* 按 Damiao 型号查 MIT 增益上限。镜像 motorbridge damiao/src/protocol.rs 的
 * pub const KP_MAX/KD_MAX:全型号固定 500/5。返回 true 表示型号已知。
 * 未来若某型号需不同增益,在此按型号分流即可。 */
bool damiao_lookup_model_gains(const char *model,
                               float *kp_max, float *kd_max);
void damiao_build_register_write_cmd(uint8_t motor_id, uint8_t rid, uint32_t value, twai_message_t *out);
void damiao_build_register_read_cmd(uint8_t motor_id, uint8_t rid, twai_message_t *out);
void damiao_build_store_params_cmd(uint8_t motor_id, twai_message_t *out);
bool damiao_parse_register_reply(const twai_message_t *msg, uint8_t *id, uint8_t *rid, uint32_t *value);
/* 解析 0x55 寄存器写应答帧:成功写回执 data[2]==0x55、rid 在 data[3]。
 * 不解析则该 ack 被静默丢弃。与 main 一致。 */
bool damiao_parse_register_write_ack(const twai_message_t *msg, uint8_t *rid);

#ifdef __cplusplus
}
#endif
