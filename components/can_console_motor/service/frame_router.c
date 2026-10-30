#include "service/frame_router.h"

#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "mb_port.h"

#include "config/app_config.h"
#include "core/motor_manager.h"
#include "protocol/host_protocol.h"
#include "transport/can_manager.h"
#include "vendors/motor_vendor.h"

static const char *TAG = "frame_router";

/* 厂商发帧回调:ctx 为该电机 channel(intptr_t 编码),由调用方注入。
 * 厂商只调 send(ctx, &frame),不依赖 can_manager——通道归属由 frame_router 决定。 */
static void send_on_ctx(void *send_ctx, const twai_message_t *frame)
{
    (void)can_manager_send((can_channel_t)(intptr_t)send_ctx, frame, 10);
}

static size_t host_ids_for_motor(const motor_state_t *motor,
                                 uint8_t *out_ids, size_t capacity)
{
    if (motor == NULL || motor->vendor == NULL || out_ids == NULL ||
        capacity == 0U) {
        return 0U;
    }
    if (motor->host_id_valid || motor->vendor->get_host_id_candidates == NULL) {
        out_ids[0] = motor->host_id;
        return 1U;
    }
    size_t count = motor->vendor->get_host_id_candidates(
        motor->host_id, out_ids, capacity);
    if (count == 0U) {
        out_ids[0] = motor->host_id;
        count = 1U;
    }
    return count;
}

/* ---- admin 派发 ---- */

static void send_admin_to_motor(uint8_t id, host_admin_opcode_t op)
{
    motor_state_t m;
    if (!motor_manager_get_state(id, &m) || m.vendor == NULL || m.vendor->build_admin_frame == NULL) {
        return;
    }
    twai_message_t tx;
    uint8_t host_ids[4] = {0};
    const size_t host_count = host_ids_for_motor(&m, host_ids,
                                                 sizeof(host_ids));
    for (size_t i = 0U; i < host_count; ++i) {
        m.vendor->build_admin_frame(id, op, host_ids[i], &tx);
        /* 按该电机绑定的通道发送 */
        if(op==HOST_OP_DISABLE && can_console_motor_session_owner(m.channel)==CAN_MOTOR_CLIENT_RAW_ESTOP)
            (void)can_manager_send_for_client(m.channel,&tx,10,CAN_MOTOR_CLIENT_EMERGENCY);
        else (void)can_manager_send(m.channel, &tx, 10);
    }
}

void frame_router_process_admin_cmd(const host_admin_cmd_t *cmd)
{
    if (cmd == NULL) {
        return;
    }
    const app_config_t *cfg = app_config_get();

    if (cmd->op == HOST_OP_SCAN) {
        /* 触发所有已登记通道扫描;未登记通道 trigger_scan 内部 no-op。 */
        for (can_channel_t ch = 0; ch < CAN_MAX_CHANNELS; ++ch) {
            can_manager_trigger_scan(ch, MOTORBRIDGE_MIN_MOTOR_ID, cfg->max_motors);
        }
        return;
    }

    if (cmd->op == HOST_OP_ESTOP) {
        for(int ch=0;ch<CAN_MAX_CHANNELS;++ch)can_console_motor_session_halt_raw(ch);
        motor_manager_estop_all();
        for (int i = MOTORBRIDGE_MIN_MOTOR_ID; i <= motor_manager_count(); ++i) {
            send_admin_to_motor((uint8_t)i, HOST_OP_DISABLE);
        }
        return;
    }

    if (cmd->op == HOST_OP_SET_IDS) {
        uint8_t old_id = cmd->motor_id == 0 ? cfg->default_motor_id : cmd->motor_id;
        uint8_t new_motor = cmd->new_motor_id == 0 ? cfg->default_motor_id : cmd->new_motor_id;
        uint8_t new_fb = cmd->new_feedback_id == 0 ? cfg->default_feedback_id : cmd->new_feedback_id;

        host_admin_cmd_t resolved = *cmd;
        resolved.motor_id = old_id;
        resolved.new_motor_id = new_motor;
        resolved.new_feedback_id = new_fb;

        motor_state_t m;
        if (motor_manager_get_state(old_id, &m) && m.vendor != NULL && m.vendor->apply_admin_cmd != NULL) {
            m.vendor->apply_admin_cmd(old_id, &resolved, send_on_ctx, (void *)(intptr_t)m.channel);
        }
        return;
    }

    /* motor_id 0 fans out per motor here with no spacing and no per-motor
     * emergency check; paced bulk operations belong to motor_service's
     * ADMIN_RANGE loop (see motor_service_submit_admin_range). */
    int start = (cmd->motor_id == 0) ? MOTORBRIDGE_MIN_MOTOR_ID : cmd->motor_id;
    int end   = (cmd->motor_id == 0) ? motor_manager_count()   : cmd->motor_id;

    for (int i = start; i <= end; ++i) {
        uint8_t id = (uint8_t)i;
        switch (cmd->op) {
        case HOST_OP_SET_MODE: {
            /* Reuse the same vendor-specific mode transaction as the
             * position/velocity host frames.  The admin frame carries no
             * target, so preserve the motor's current command values. */
            motor_state_t m;
            if (motor_manager_get_state(id, &m)) {
                damiao_cmd_t current_cmd = m.cmd;
                (void)frame_router_process_mode_cmd(id, cmd->mode, &current_cmd);
            }
            break;
        }
        case HOST_OP_SET_GAINS:
            (void)motor_manager_set_gains(id, cmd->kp, cmd->kd);
            break;
        case HOST_OP_ENABLE:
            {
                motor_state_t m;
                bool vendor_handles_enable = false;
                if (motor_manager_get_state(id, &m) && m.vendor != NULL &&
                    m.vendor->prepare_enable != NULL) {
                    uint8_t host_ids[4] = {0};
                    const size_t host_count = host_ids_for_motor(
                        &m, host_ids, sizeof(host_ids));
                    for (size_t i = 0U; i < host_count; ++i) {
                        vendor_handles_enable |= m.vendor->prepare_enable(
                            id, m.mode, host_ids[i], send_on_ctx,
                            (void *)(intptr_t)m.channel);
                    }
                }
                (void)motor_manager_set_enabled(id, true);
                if (vendor_handles_enable) {
                    break;
                }
            }
            send_admin_to_motor(id, HOST_OP_ENABLE);
            break;
        case HOST_OP_DISABLE:
            (void)motor_manager_set_enabled(id, false);
            send_admin_to_motor(id, HOST_OP_DISABLE);
            break;
        case HOST_OP_SET_ZERO:
            send_admin_to_motor(id, HOST_OP_SET_ZERO);
            /* 厂商二段写(robstride: zero_sta);damiao 无,ops->on_set_zero 为 NULL。 */
            {
                motor_state_t m;
                if (motor_manager_get_state(id, &m) && m.vendor != NULL &&
                    m.vendor->on_set_zero != NULL) {
                    uint8_t host_ids[4] = {0};
                    const size_t host_count = host_ids_for_motor(
                        &m, host_ids, sizeof(host_ids));
                    for (size_t i = 0U; i < host_count; ++i) {
                        m.vendor->on_set_zero(
                            id, host_ids[i], send_on_ctx,
                            (void *)(intptr_t)m.channel);
                    }
                }
            }
            break;
        case HOST_OP_CLEAR_ERROR:
            send_admin_to_motor(id, HOST_OP_CLEAR_ERROR);
            break;
        case HOST_OP_STOP:
            /* 受控停机:先发厂商停机帧(robstride: 各 run_mode 的保持/归零帧;
             * damiao: MIT hold 帧),再 disarm 控制环(set_mode DISABLED,enabled 保持
             * 以便反馈仍解析)。镜像 motorbridge 4690e3f。 */
            {
                motor_state_t m;
                if (motor_manager_get_state(id, &m) && m.vendor != NULL &&
                    m.vendor->stop != NULL) {
                    (void)m.vendor->stop(id, &m, send_on_ctx, (void *)(intptr_t)m.channel);
                    (void)motor_manager_set_mode(id, MOTOR_MODE_DISABLED);
                }
            }
            break;
        default:
            break;
        }
    }
}

/* ---- mode 派发 ---- */

bool frame_router_process_mode_cmd(uint8_t id, motor_mode_t mode,
                                   const damiao_cmd_t *cmd)
{
    if (cmd == NULL || id > motor_manager_count()) {
        return false;
    }

    motor_state_t m;
    if (!motor_manager_get_state(id, &m)) {
        return false;
    }
    if (m.vendor != NULL && m.vendor->supports_mode != NULL &&
        !m.vendor->supports_mode(mode)) {
        return false;
    }
    if (m.vendor != NULL && m.vendor->prepare_mode_change != NULL) {
        bool need_prepare = m.mode != mode;
        if (!need_prepare && m.vendor->mode_ready != NULL) {
            need_prepare = !m.vendor->mode_ready(id, mode);
        }
        if (need_prepare) {
            uint8_t host_ids[4] = {0};
            const size_t host_count = host_ids_for_motor(&m, host_ids,
                                                         sizeof(host_ids));
            for (size_t i = 0U; i < host_count; ++i) {
                m.vendor->prepare_mode_change(
                    id, mode, host_ids[i], send_on_ctx,
                    (void *)(intptr_t)m.channel);
            }
        }
    }

    switch (mode) {
    case MOTOR_MODE_MIT:
        return motor_manager_set_mit_cmd(id, cmd->pos, cmd->vel, cmd->tau);
    case MOTOR_MODE_POS_VEL:
        return motor_manager_set_pos_vel_cmd(id, cmd->pos, cmd->vlim);
    case MOTOR_MODE_POS_CSP:
        return motor_manager_set_pos_csp_cmd(id, cmd->pos, cmd->vlim);
    case MOTOR_MODE_POS_PP:
        return motor_manager_set_pos_pp_cmd(id, cmd->pos, cmd->vlim, cmd->acc);
    case MOTOR_MODE_VEL:
        return motor_manager_set_vel_cmd(id, cmd->vel);
    case MOTOR_MODE_FORCE_POS:
        return motor_manager_set_force_pos_cmd(id, cmd->pos, cmd->vlim, cmd->ratio);
    default:
        return false;
    }
}

/* ---- RX 主分发(优先级镜像 main:write-ack → reg-reply → param-reply
 *      → host-admin → host-mode → 电机反馈)---- */

static bool reply_matches_binding(uint8_t id, can_channel_t ch, const motor_vendor_ops_t *ops)
{
    motor_state_t state;
    return motor_manager_get_state(id, &state) && state.channel == ch && state.vendor == ops;
}

void frame_router_process_can_rx(can_channel_t ch, const twai_message_t *msg)
{
    if (msg == NULL) {
        return;
    }

    /* Discovery replies carry an address but no position/velocity/torque.
     * Consume them before the telemetry path so they cannot overwrite the
     * last real joint state with zeroes. */
    for (int v = 0; v < motor_vendor_count(); ++v) {
        const motor_vendor_ops_t *ops = motor_vendor_get_by_index(v);
        if (ops != NULL && ops->parse_scan_reply != NULL) {
            uint8_t id = 0U;
            if (ops->parse_scan_reply(msg, &id)) {
                if (!reply_matches_binding(id, ch, ops)) {
                    return;
                }
                uint8_t host_id = 0U;
                if (ops->parse_host_id != NULL &&
                    ops->parse_host_id(msg, &host_id)) {
                    motor_manager_update_host_id(id, host_id);
                }
                motor_manager_mark_seen(id);
                return;
            }
        }
    }

    /* Register transactions are vendor-owned; their handler validates the
     * sender and bus before updating mode/set-ids verification state. */
    for (int v = 0; v < motor_vendor_count(); ++v) {
        const motor_vendor_ops_t *ops = motor_vendor_get_by_index(v);
        if (!can_console_motor_session_raw_active(ch) && ops->process_register_reply != NULL && ops->process_register_reply(ch, msg)) {
            return;
        }
    }

    /* 3. 运行参数应答(robstride comm 17):可观测。 */
    for (int v = 0; v < motor_vendor_count(); ++v) {
        const motor_vendor_ops_t *ops = motor_vendor_get_by_index(v);
        if (ops != NULL && ops->parse_param_reply != NULL) {
            uint8_t id = 0;
            uint16_t pid = 0;
            uint8_t pval[4] = {0};
            if (ops->parse_param_reply(msg, &id, &pid, pval)) {
                if (!can_console_motor_session_raw_active(ch) && reply_matches_binding(id, ch, ops) && ops->on_param_reply != NULL) {
                    ops->on_param_reply(msg, pid, pval);
                }
                return;
            }
        }
    }

    /* 4. RobStride 故障上报(robstride comm 21):自识别,按厂商数 O(vendor) 分派,
     * 先于 host 管理帧。镜像 motorbridge 9908d0f。 */
    for (int v = 0; v < motor_vendor_count(); ++v) {
        const motor_vendor_ops_t *ops = motor_vendor_get_by_index(v);
        if (ops != NULL && ops->parse_fault_report != NULL) {
            uint8_t fid = 0;
            uint32_t fault_raw = 0, warning_raw = 0;
            if (ops->parse_fault_report(msg, &fid, &fault_raw, &warning_raw)) {
                if (!reply_matches_binding(fid, ch, ops)) {
                    return;
                }
                uint8_t host_id = 0U;
                if (ops->parse_host_id != NULL &&
                    ops->parse_host_id(msg, &host_id)) {
                    motor_manager_update_host_id(fid, host_id);
                }
                if (ops->on_fault_report != NULL) {
                    ops->on_fault_report(fid, fault_raw, warning_raw);
                } else {
                    MB_LOGW(TAG, "fault report (%s) id=%u fault=0x%08" PRIX32 " warn=0x%08" PRIX32,
                            ops->name, fid, fault_raw, warning_raw);
                }
                return;
            }
        }
    }

    /* 5. RobStride 固件版本应答(robstride comm 2,signature 0x00 0xC4 0x56):
     * 自识别,先于 host 管理帧/模式帧/反馈环按厂商数 O(vendor) 分派。
     * 版本应答与遥测同是 comm 2,必须在反馈环之前拦截,否则版本字节被当遥测
     * 误解码;若放在反馈环内,第一个 robstride 槽位会吞掉所有发往更高 id 电机的
     * 版本应答并丢弃它(parsed_id!=i 不回写 -> return),fw_seen_us 永不推进 ->
     * 扫描永久重发版本请求。镜像 motorbridge 5e54fe1 + main 的 bug 修复。
     * (host 管理帧/模式帧用 11 位标准 id,版本应答用 29 位 ext id,故此处先于
     *  admin/mode 与放在其后功能等价;此处对齐 main 的"先版本后 admin"顺序。) */
    for (int v = 0; v < motor_vendor_count(); ++v) {
        const motor_vendor_ops_t *ops = motor_vendor_get_by_index(v);
        if (ops != NULL && ops->parse_version_reply != NULL) {
            uint8_t vid = 0;
            uint8_t ver[4] = {0};
            if (ops->parse_version_reply(msg, &vid, ver)) {
                if (!reply_matches_binding(vid, ch, ops)) {
                    return;
                }
                uint8_t host_id = 0U;
                if (ops->parse_host_id != NULL &&
                    ops->parse_host_id(msg, &host_id)) {
                    motor_manager_update_host_id(vid, host_id);
                }
                if (ops->on_version_reply != NULL) {
                    ops->on_version_reply(vid, ver);
                }
                return;
            }
        }
    }

    /* 6. host 管理帧 */
    host_admin_cmd_t admin;
    if (!can_console_motor_session_raw_active(ch) && host_protocol_parse_admin_cmd(msg, &admin)) {
        frame_router_process_admin_cmd(&admin);
        return;
    }

    /* 7. host 模式帧 */
    uint8_t id = 0;
    motor_mode_t mode = MOTOR_MODE_DISABLED;
    damiao_cmd_t cmd = {0};
    if (!can_console_motor_session_raw_active(ch) && host_protocol_parse_mode_frame(msg, &id, &mode, &cmd)) {
        if (id <= motor_manager_count()) {
            (void)frame_router_process_mode_cmd(id, mode, &cmd);
        }
        return;
    }

    /* 8. 电机反馈:O(电机数),逐 id 用该电机 params 解码,parsed_id==i 命中才回写。
     * (robstride 解码需 per-motor 量程,故无法 O(厂商数);n<=15 可忽略。) */
    float pos = 0, vel = 0, torque = 0, t_mos = 0, t_rotor = 0;
    uint16_t status = 0;
    uint8_t parsed_id = 0;

    for (int i = MOTORBRIDGE_MIN_MOTOR_ID; i <= motor_manager_count(); ++i) {
        motor_state_t m;
        if (motor_manager_get_state((uint8_t)i, &m) && m.channel == ch && m.vendor != NULL &&
            m.vendor->parse_feedback != NULL) {
            if (m.vendor->parse_feedback(msg, &m.params, &parsed_id,
                                         &pos, &vel, &torque, &t_mos, &t_rotor, &status)) {
                if (parsed_id == i) {
                    uint8_t host_id = 0U;
                    if (m.vendor->parse_host_id != NULL &&
                        m.vendor->parse_host_id(msg, &host_id)) {
                        motor_manager_update_host_id(parsed_id, host_id);
                        m.host_id = host_id;
                    }
                    motor_manager_update_feedback(parsed_id, pos, vel, torque, t_mos, t_rotor, status);
                    /* 扫描命中后顺带探一次固件版本(robstride only)。
                     * fw_seen_us==0 表示从未收到版本应答。镜像 motorbridge 5e54fe1。 */
                    if (!can_console_motor_session_raw_active(ch) && m.vendor->build_version_request != NULL && m.fw_seen_us == 0) {
                        twai_message_t vreq;
                        m.vendor->build_version_request((uint8_t)i, m.host_id, &vreq);
                        (void)can_manager_send(m.channel, &vreq, 5);
                    }
                    return;
                }
            }
        }
    }
}

void frame_router_process_reg_verify_tick(void)
{
    /* 厂商级钩子:每厂商一次,内部遍历自己的 pending 状态。 */
    for (int v = 0; v < motor_vendor_count(); ++v) {
        const motor_vendor_ops_t *ops = motor_vendor_get_by_index(v);
        if (ops != NULL && ops->reg_verify_tick != NULL) {
            ops->reg_verify_tick();
        }
    }
}

static void refresh_mode_readiness(motor_state_t *motor, void *context)
{
    (void)context;
    bool ready = false;
    if (motor->vendor != NULL && motor->mode != MOTOR_MODE_DISABLED) {
        if (motor->vendor->mode_ready != NULL) {
            ready = motor->vendor->mode_ready(motor->id, motor->mode);
        } else if (motor->vendor->mode_transition_pending != NULL) {
            ready = !motor->vendor->mode_transition_pending(motor->id);
        } else {
            ready = true;
        }
    }
    motor->mode_ready = ready;
}

void frame_router_refresh_mode_readiness(void)
{
    motor_manager_for_each(refresh_mode_readiness, NULL);
}
