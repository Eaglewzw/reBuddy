#pragma once

/*
 * motor_vendor.h — 厂商抽象接口（芯片无关）。
 *
 * 各厂商实现本 ops 表；core 经此间接调用，不认识具体厂商。
 * 新增厂商：实现 ops + 在 motor_vendor.c 注册表登记。
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "driver/twai.h"

#include "core/motor_manager.h"
#include "protocol/host_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 厂商发帧回调:经 context 指针注入携带通道,厂商实现只调 send(ctx, &frame),
 * 不直接依赖 can_manager。frame_router 提供 send_on_ctx 实现。 */
typedef void (*motor_send_fn_t)(void *send_ctx, const twai_message_t *frame);

typedef struct motor_vendor_ops_s {
    const char *name;
    /** Service-owner-only reset of one disabled motor's protocol session. */
    void (*reset_session)(uint8_t id);

    void (*build_scan_request)(uint8_t id, twai_message_t *out);
    /* Optional discovery reply decoder.  Discovery proves presence/address
     * only; it must not be treated as telemetry because it carries no PvT. */
    bool (*parse_scan_reply)(const twai_message_t *msg, uint8_t *id);
    void (*build_admin_frame)(uint8_t id, host_admin_opcode_t op,
                              uint8_t host_id, twai_message_t *out);
    bool (*build_control_frame)(uint8_t id,
                                motor_mode_t mode,
                                const damiao_cmd_t *cmd,
                                const motor_params_t *params,
                                twai_message_t *out);
    /* 多帧参数控制。返回 true 表示厂商已发帧，false 表示
     * 交给 build_control_frame 单帧路径。单帧厂商可置 NULL。 */
    bool (*send_control)(uint8_t id,
                         uint8_t host_id,
                         bool host_id_valid,
                         motor_mode_t mode,
                         const damiao_cmd_t *cmd,
                         const motor_params_t *params,
                         motor_send_fn_t send,
                         void *send_ctx);
    bool (*parse_feedback)(const twai_message_t *msg,
                           const motor_params_t *params,
                           uint8_t *id,
                           float *pos,
                           float *vel,
                           float *torque,
                           float *t_mos,
                           float *t_rotor,
                           uint16_t *status);

    /* Optional response address decoder. For RobStride this returns the
     * responder/host ID from the low byte of the extended CAN ID. */
    bool (*parse_host_id)(const twai_message_t *msg, uint8_t *host_id);
    /* Optional host-ID probing order. Used before a vendor response has
     * taught the manager the motor's actual responder/host ID. */
    size_t (*get_host_id_candidates)(uint8_t preferred_host_id,
                                     uint8_t *out_ids, size_t capacity);

    /* Consume register replies/acks; update transactions only for matching
     * motor/bus, including replies to a newly assigned address. */
    bool (*process_register_reply)(can_channel_t channel, const twai_message_t *msg);

    /* 运行参数应答(robstride comm 17):可观测。可选。 */
    bool (*parse_param_reply)(const twai_message_t *msg, uint8_t *id,
                             uint16_t *param_id, uint8_t value[4]);
    /* Optional RobStride parameter transaction callback. */
    void (*on_param_reply)(const twai_message_t *msg, uint16_t param_id,
                           const uint8_t value[4]);

    /* 可选:fault 上报(robstride comm 21)解析 + 记录钩子。RX 侧,无 send——
     * parse 出 id/fault_raw/warning_raw,on_fault_report 记入 motor_manager。
     * 镜像 motorbridge 9908d0f。 */
    bool (*parse_fault_report)(const twai_message_t *msg,
                               uint8_t *id,
                               uint32_t *fault_raw,
                               uint32_t *warning_raw);
    void (*on_fault_report)(uint8_t id, uint32_t fault_raw, uint32_t warning_raw);

    /* 可选:固件版本应答(robstride comm 2,signature 0x00C456)解析 + 记录钩子。
     * RX 侧,无 send。parse 出 device_id/version[4],on_version_reply 记入 motor_manager。
     * 镜像 motorbridge 5e54fe1。build_version_request 用于 scan 命中后主动请求版本。 */
    bool (*parse_version_reply)(const twai_message_t *msg, uint8_t *id, uint8_t version[4]);
    void (*on_version_reply)(uint8_t id, const uint8_t version[4]);
    void (*build_version_request)(uint8_t id, uint8_t host_id,
                                  twai_message_t *out);

    /* 可选:受控停机(减速/保持),与 DISABLE 区分。TX 侧,经注入 send 发一帧零/保持
     * 目标;返回 true 表示已发停机帧。调用方随后 set_mode DISABLED 解除控制环。
     * robstride→per-mode stop frame;damiao→best-effort MIT hold frame。
     * 镜像 motorbridge 4690e3f (controlled_stop)。 */
    bool (*stop)(uint8_t id, const motor_state_t *m, motor_send_fn_t send, void *send_ctx);

    /* 可选:厂商默认参数应用。motor_manager_set_vendor 绑定后调用,使厂商写
     * 适配的 PvT 量程。 */
    void (*apply_vendor_defaults)(motor_params_t *params);
    /* 可选:按型号名查量程(robstride RS_MODELS)。motor_manager_set_model 调用,
     * 返回 true 并填出量程。无型号表的厂商置 NULL。 */
    bool (*resolve_model)(const char *model, float *pmax, float *vmax, float *tmax);
    /* 可选:按型号名查 MIT 增益上限(robstride:rs-03/04/06 -> 5000/100,其余 500/5)。
     * motor_manager_set_model 与 apply_vendor_defaults 调用。无此映射的厂商置 NULL,
     * 调用方回退 500/5。镜像 motorbridge motor.rs:167-171。 */
    bool (*resolve_model_gains)(const char *model, float *kp_max, float *kd_max);

    /* Optional mode capability check.  If present and false, the router
     * rejects the command before changing the motor's local mode. */
    bool (*supports_mode)(motor_mode_t mode);

    /* 可选:通用寄存器写 + 异步读回校验(damiao,任意 rid)。TX 侧经注入 send。
     * reg_verify_tick 由集成方周期调(超时重读)。镜像 motorbridge 40194bc。 */
    void (*reg_write_verified)(uint8_t id, uint8_t rid, uint32_t value,
                               motor_send_fn_t send, void *send_ctx);
    void (*reg_verify_tick)(void);

    /* 可选:模式切换前的厂商预处理(如 damiao 写 rid=10 控制模式寄存器 + 读回 verify)。
     * frame_router 在 set_*_cmd 前调用;robstride 等可置 NULL。
     * send/send_ctx:经 context 指针注入携带通道(frame_router 提供 send_on_ctx,
     * ctx 为 motor 的 channel),厂商只调 send(send_ctx, &frame) 发帧,不依赖 can_manager。 */
    void (*prepare_mode_change)(uint8_t id, motor_mode_t mode,
                                uint8_t host_id,
                                motor_send_fn_t send, void *send_ctx);
    /* Optional asynchronous mode transaction state. */
    bool (*mode_transition_pending)(uint8_t id);
    bool (*mode_ready)(uint8_t id, motor_mode_t mode);

    /* Optional asynchronous enable transaction.  Return true when the
     * vendor has accepted responsibility for sending ENABLE and waiting for
     * its acknowledgement before control targets may be emitted. */
    bool (*prepare_enable)(uint8_t id, motor_mode_t mode,
                           uint8_t host_id,
                           motor_send_fn_t send, void *send_ctx);

    /* 可选:厂商特有的管理命令执行(如 damiao set-ids 写寄存器 + verify)。
     * 由 frame_router 在收到 HOST_OP_SET_IDS 时调用,把厂商细节封在厂商模块内。 */
    void (*apply_admin_cmd)(uint8_t motor_id, const host_admin_cmd_t *cmd,
                            motor_send_fn_t send, void *send_ctx);

    /* 可选:SET_ZERO 厂商扩展(robstride: zero_sta 二段写)。
     * frame_router 在发完通用 SET_ZERO admin 帧后调用。 */
    void (*on_set_zero)(uint8_t id, uint8_t host_id,
                        motor_send_fn_t send, void *send_ctx);
} motor_vendor_ops_t;

/* 按名取厂商 ops */
const motor_vendor_ops_t *motor_vendor_get(const char *name);

/* 厂商枚举：供 frame_router 做 O(厂商数) 反馈分发 */
int motor_vendor_count(void);
const motor_vendor_ops_t *motor_vendor_get_by_index(int index);

#ifdef __cplusplus
}
#endif
