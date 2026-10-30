#pragma once

/*
 * robstride_protocol.h — Robstride 协议编解码(单目标 ESP32-C5,直调 twai)。
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "driver/twai.h"

#include "protocol/host_protocol.h"
#include "core/motor_manager.h"

#ifdef __cplusplus
extern "C" {
#endif

void robstride_build_scan_request(uint8_t id, twai_message_t *out);
bool robstride_parse_scan_reply(const twai_message_t *msg, uint8_t *id);
void robstride_build_admin_frame(uint8_t id, host_admin_opcode_t op,
                                 uint8_t host_id, twai_message_t *out);
bool robstride_build_control_frame(uint8_t id, motor_mode_t mode, const damiao_cmd_t *cmd, const motor_params_t *params, twai_message_t *out);
bool robstride_parse_feedback(const twai_message_t *msg, const motor_params_t *params, uint8_t *id, float *pos, float *vel, float *torque, float *t_mos, float *t_rotor, uint16_t *status);
bool robstride_parse_host_id(const twai_message_t *msg, uint8_t *host_id);
size_t robstride_get_host_id_candidates(uint8_t preferred_host_id,
                                        uint8_t *out_ids, size_t capacity);

/* 按型号 Robstride PvT 量程查表,移植自 motorbridge motor.rs。返回 true 表示
 * 已知型号并填出 position/velocity/torque 量程,供 MIT 编解码 1:1 缩放。 */
bool robstride_lookup_model_limits(const char *model, float *pmax, float *vmax, float *tmax);

/* 按型号查 MIT 增益上限:rs-03/04/06 -> (5000, 100),其余 -> (500, 5)。
 * 移植自 motorbridge motor.rs:167-171。返回 true 表示已知型号。 */
bool robstride_lookup_model_gains(const char *model, float *kp_max, float *kd_max);

/* ---------------------------------------------------------------------------
 * Robstride 运行参数读写(comm 17/18)。从 motorbridge 移植
 * (motor_vendors/robstride/src/protocol.rs):参数帧 data[0..2] 放 param id(LE),
 * data[2..4] 置零,data[4..8] 放 raw value(LE);写用 comm 18,读用 comm 17。
 * extra_data 放 host_id(电机应答哪个主机),出厂默认 0xFF。
 * ------------------------------------------------------------------------- */

/* Robstride run_mode (param 0x7005, Int8) 枚举。 */
#define RS_RUNMODE_MIT 0   /* MOTOR_MODE_MIT */
#define RS_RUNMODE_PP  1   /* POS_VEL/POS_PP/FORCE_POS (PP) */
#define RS_RUNMODE_VEL 2   /* MOTOR_MODE_VEL */
#define RS_RUNMODE_CSP 5   /* MOTOR_MODE_POS_CSP (native CSP) */

/* esp32 路径实际写入的参数 id。完整表在 host(registers.rs),此处只列用到的。 */
#define RS_PARAM_MODE          0x7005 /* Int8,  run_mode */
#define RS_PARAM_VEL_TARGET    0x700A /* F32,   velocity-mode 目标 (rad/s) */
#define RS_PARAM_POS_TARGET    0x7016 /* F32,   位置目标 (rad) */
#define RS_PARAM_VEL_LIMIT     0x7017 /* F32,   CSP 速度上限 (rad/s) */
#define RS_PARAM_PP_VEL_MAX    0x7024 /* F32,   PP 模式速度 (rad/s) */
#define RS_PARAM_ZERO_STATE    0x7029 /* UInt8, 0: 0..2pi, 1: -pi..pi */

/* 出厂默认 host id。收到 RobStride ping/status/fault/version 应答后，
 * frame_router 会保存每个电机实际 responder/host id，后续参数帧复用它。 */
#define RS_DEFAULT_HOST_ID 0xFF

/* 写参数(comm 18)。raw_value 为 4 字节 LE(见 encode helpers)。host_id 放 extra_data;
 * 电机 node id 为 id。 */
void robstride_build_param_write_frame(uint8_t id,
                                        uint16_t param_id,
                                        const uint8_t raw_value[4],
                                        uint8_t host_id,
                                        twai_message_t *out);

/* 读参数(comm 17),fire-and-forget 读回。 */
void robstride_build_param_read_frame(uint8_t id,
                                      uint16_t param_id,
                                      uint8_t host_id,
                                      twai_message_t *out);

/* Enable/disable periodic operation-status reporting (comm 24). */
void robstride_build_active_report_frame(uint8_t id, uint8_t host_id,
                                         bool enabled,
                                         twai_message_t *out);

/* 解析读参数应答(comm 17)。param_id 取自 data[0..2],value 取自 data[4..8]。 */
bool robstride_parse_param_reply(const twai_message_t *msg, uint8_t *id,
                                 uint16_t *param_id,
                                 uint8_t value[4]);

/* 保存参数(comm 22)。payload 与 host save_parameters() 一致。 */
void robstride_build_save_frame(uint8_t id, uint8_t host_id, twai_message_t *out);

/* motor_mode_t -> Robstride run_mode 字节;无对应返回 0xFF(如 DISABLED)。 */
uint8_t robstride_run_mode_for(motor_mode_t mode);

/* Native PP acceleration parameter. */
#define RS_PARAM_PP_ACCEL      0x7025 /* PP acceleration target (F32). */

/* 值编码:填 4 字节 LE、零填充,匹配参数类型(Int8/UInt8/Float32)。 */
void robstride_encode_param_i8(int8_t v, uint8_t out[4]);
void robstride_encode_param_u8(uint8_t v, uint8_t out[4]);
void robstride_encode_param_f32(float v, uint8_t out[4]);

/* ---------------------------------------------------------------------------
 * Robstride fault 上报(comm 21)。移植自 motorbridge protocol.rs
 * decode_fault_report (9908d0f)。8 字节载荷为两个 LE u32:fault_raw 在
 * [0..4],warning_raw 在 [4..8];电机 id 在 29 位 CAN id 的低字节。
 * ------------------------------------------------------------------------- */

#define RS_COMM_FAULT_REPORT 21

/* 解析 FAULT_REPORT(comm 21)帧。返回 true 并填 id(仲裁 id 低字节)、
 * fault_raw、warning_raw。 */
bool robstride_parse_fault_report(const twai_message_t *msg,
                                  uint8_t *id,
                                  uint32_t *fault_raw,
                                  uint32_t *warning_raw);
/* fault_raw 某位(0..31)的人类可读名称;无定义返回 NULL。用于日志。 */
const char *robstride_fault_bit_name(int bit);

/* ---------------------------------------------------------------------------
 * Robstride 固件版本请求/应答。移植自 motorbridge protocol.rs (5e54fe1)。
 * 请求是 comm 4(DISABLE)帧,data[1] 带 marker 0xC4;应答以 comm 2
 * (OPERATION_STATUS)帧到达,data[0..3] == {0x00,0xC4,0x56},故被动监听者
 * 必须在 decode_status_frame 前检查 robstride_is_version_reply(),否则版本
 * 字节被当垃圾遥测解码。
 * ------------------------------------------------------------------------- */

#define RS_VERSION_REQUEST_MARKER   0xC4
#define RS_VERSION_REPLY_SIG_BYTE0  0x00
#define RS_VERSION_REPLY_SIG_BYTE1  0xC4
#define RS_VERSION_REPLY_SIG_BYTE2  0x56

void robstride_build_version_request_frame(uint8_t id, uint8_t host_id,
                                           twai_message_t *out);
bool robstride_is_version_reply(const twai_message_t *msg);
bool robstride_decode_version_reply(const twai_message_t *msg,
                                    uint8_t *id,
                                    uint8_t version[4]);

/* ---------------------------------------------------------------------------
 * Robstride 受控停机。移植自 motorbridge motor.rs controlled_stop (4690e3f)。
 * host 读当前 run_mode + 机械位置(param 0x7019)用阻塞读;esp32 单 RX 任务
 * 模型不能阻塞,故本 builder 用电机缓存的 mode + 位置/增益。发一帧零/保持
 * 目标使电机在自己的环里减速;调用方随后解除控制环(set_mode DISABLED)。
 * ------------------------------------------------------------------------- */

/* 按给定 mode + 缓存电机状态构造一帧停机帧。mode 无对应停机
 * (DISABLED/未知)时返回 false。 */
bool robstride_build_stop_frame(uint8_t id,
                                motor_mode_t mode,
                                const motor_state_t *m,
                                twai_message_t *out);

#ifdef __cplusplus
}
#endif
