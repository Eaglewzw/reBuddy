#include "vendors/motor_vendor.h"
#include "vendors/robstride/robstride_protocol.h"

#include <string.h>

#include "config/app_config.h"
#include "core/motor_manager.h"
#include "mb_port.h"

static const char *TAG = "robstride_vendor";

/* The Rust implementation treats mode selection as a transaction rather
 * than a fire-and-forget write. The single motor-service owner advances this
 * state machine on ticks and applies parameter replies in queue order. */
#define RS_MODE_DISABLE_SETTLE_US  60000LL
#define RS_MODE_WRITE_SETTLE_US    30000LL
#define RS_MODE_REPLY_TIMEOUT_US   180000LL
#define RS_ENABLE_REPLY_TIMEOUT_US 240000LL
#define RS_MODE_MAX_ATTEMPTS       3U

typedef enum {
    RS_MODE_IDLE = 0,
    RS_MODE_WAIT_DISABLE,
    RS_MODE_WAIT_MODE_SETTLE,
    RS_MODE_WAIT_MODE_REPLY,
    RS_MODE_NEED_ENABLE,
    RS_MODE_WAIT_ENABLE_REPLY,
    RS_MODE_READY,
    RS_MODE_FAILED,
} rs_mode_phase_t;

typedef struct {
    rs_mode_phase_t phase;
    uint8_t target_mode;
    uint8_t host_id;
    uint8_t host_ids[4];
    uint8_t host_count;
    uint8_t host_index;
    uint8_t attempts;
    int64_t deadline_us;
    bool command_sent;
    uint32_t sent_command_seq;
    int64_t enable_sent_us;
    motor_send_fn_t send;
    void *send_ctx;
} rs_mode_transaction_t;

static rs_mode_transaction_t s_mode_transactions[MOTORBRIDGE_MAX_MOTOR_ID + 1];

static void robstride_reset_session(uint8_t id)
{
    memset(&s_mode_transactions[id], 0, sizeof(s_mode_transactions[id]));
}

static rs_mode_transaction_t *rs_mode_transaction(uint8_t id)
{
    if (id > MOTORBRIDGE_MAX_MOTOR_ID) {
        return NULL;
    }
    return &s_mode_transactions[id];
}

static void rs_add_host_candidate(rs_mode_transaction_t *transaction,
                                  uint8_t host_id)
{
    if (transaction == NULL) {
        return;
    }
    for (uint8_t i = 0U; i < transaction->host_count; ++i) {
        if (transaction->host_ids[i] == host_id) {
            return;
        }
    }
    if (transaction->host_count < (uint8_t)(sizeof(transaction->host_ids) /
                                             sizeof(transaction->host_ids[0]))) {
        transaction->host_ids[transaction->host_count++] = host_id;
    }
}

static void rs_send_admin(uint8_t id, host_admin_opcode_t op,
                          uint8_t host_id, motor_send_fn_t send,
                          void *send_ctx)
{
    if (send == NULL) {
        return;
    }
    twai_message_t tx;
    robstride_build_admin_frame(id, op, host_id, &tx);
    send(send_ctx, &tx);
}

static void rs_send_mode_write(uint8_t id, uint8_t run_mode,
                               uint8_t host_id, motor_send_fn_t send,
                               void *send_ctx)
{
    if (send == NULL) {
        return;
    }
    uint8_t raw[4];
    robstride_encode_param_i8((int8_t)run_mode, raw);
    twai_message_t tx;
    robstride_build_param_write_frame(id, RS_PARAM_MODE, raw, host_id, &tx);
    send(send_ctx, &tx);
}

static void rs_send_mode_read(uint8_t id, uint8_t host_id,
                              motor_send_fn_t send, void *send_ctx)
{
    if (send == NULL) {
        return;
    }
    twai_message_t tx;
    robstride_build_param_read_frame(id, RS_PARAM_MODE, host_id, &tx);
    send(send_ctx, &tx);
}

/* RobStride uses parameter replies for mode verification, without DM register
 * transactions or set-ids handling. */

/* robstride 非 MIT 控制(pos_vel/vel/force_pos)由运行参数写(comm 18)驱动,不走
 * MIT 控制帧。对照 motorbridge Rust 层 send_cmd_pos_vel_csp / send_cmd_pos_vel_pp
 / set_velocity_target(motor_vendors/robstride/src/motor.rs):每次发目标前写
 * run_mode(0x7005),再按模式写目标参数。逐 tick 重写 run_mode 使丢帧自愈
 *(镜像 host 每次 send_cmd_* 都 set_mode;esp32 适配为 fire-and-forget,无 ack)。
 * MIT 模式返回 false,交回 build_control_frame 单帧路径。
 * 对照 host:POS_VEL→CSP(0x7017 速度上限 + 0x7016 位置目标);
 *           VEL→0x700A 速度目标;FORCE_POS→PP(0x7024 速度上限 + 0x7016 位置目标)。 */
static bool robstride_send_control(uint8_t id,
                                   uint8_t host_id,
                                   bool host_id_valid,
                                   motor_mode_t mode,
                                   const damiao_cmd_t *cmd,
                                   const motor_params_t *params,
                                   motor_send_fn_t send,
                                   void *send_ctx)
{
    (void)params;
    if (send == NULL || cmd == NULL) {
        return false;
    }
    /* MIT 走 build_control_frame(comm 1 单帧)。 */
    const uint8_t run_mode = robstride_run_mode_for(mode);
    if (run_mode == 0xFF) {
        return false;
    }

    rs_mode_transaction_t *transaction = rs_mode_transaction(id);
    if (transaction == NULL || transaction->phase != RS_MODE_READY ||
        transaction->target_mode != run_mode) {
        /* Do not send a target until run_mode has been read back. */
        return true;
    }

    if (mode == MOTOR_MODE_MIT) {
        return false;
    }
    if (transaction->command_sent &&
        transaction->sent_command_seq == cmd->seq) {
        return true;
    }

    uint8_t host_ids[4] = {0};
    size_t host_count = 0U;
    if (host_id_valid) {
        host_ids[0] = host_id;
        host_count = 1U;
    } else {
        host_count = robstride_get_host_id_candidates(
            host_id, host_ids, sizeof(host_ids));
    }
    if (host_count == 0U) {
        host_ids[0] = host_id;
        host_count = 1U;
    }

    const float vlim = cmd->vlim < 0.0f ? -cmd->vlim : cmd->vlim;
    const float acc = cmd->acc < 0.0f ? -cmd->acc : cmd->acc;
    for (size_t h = 0U; h < host_count; ++h) {
        const uint8_t target_host = host_ids[h];
        twai_message_t tx;
        uint8_t raw[4];

        switch (mode) {
        case MOTOR_MODE_POS_VEL:
            /* motorbridge generic position path uses PP run_mode=1. */
            if (vlim > 0.0f) {
                robstride_encode_param_f32(vlim, raw);
                robstride_build_param_write_frame(id, RS_PARAM_PP_VEL_MAX, raw, target_host, &tx);
                send(send_ctx, &tx);
            }
            robstride_encode_param_f32(cmd->pos, raw);
            robstride_build_param_write_frame(id, RS_PARAM_POS_TARGET, raw, target_host, &tx);
            send(send_ctx, &tx);
            break;
        case MOTOR_MODE_POS_CSP:
            /* Native CSP: limit_spd(0x7017) + loc_ref(0x7016). */
            if (vlim > 0.0f) {
                robstride_encode_param_f32(vlim, raw);
                robstride_build_param_write_frame(id, RS_PARAM_VEL_LIMIT, raw, target_host, &tx);
                send(send_ctx, &tx);
            }
            robstride_encode_param_f32(cmd->pos, raw);
            robstride_build_param_write_frame(id, RS_PARAM_POS_TARGET, raw, target_host, &tx);
            send(send_ctx, &tx);
            break;
        case MOTOR_MODE_POS_PP:
            /* Native PP: vel_max(0x7024) + acc_set(0x7025) + loc_ref(0x7016). */
            if (vlim > 0.0f) {
                robstride_encode_param_f32(vlim, raw);
                robstride_build_param_write_frame(id, RS_PARAM_PP_VEL_MAX, raw, target_host, &tx);
                send(send_ctx, &tx);
            }
            if (acc > 0.0f) {
                robstride_encode_param_f32(acc, raw);
                robstride_build_param_write_frame(id, RS_PARAM_PP_ACCEL, raw, target_host, &tx);
                send(send_ctx, &tx);
            }
            robstride_encode_param_f32(cmd->pos, raw);
            robstride_build_param_write_frame(id, RS_PARAM_POS_TARGET, raw, target_host, &tx);
            send(send_ctx, &tx);
            break;
        case MOTOR_MODE_VEL:
            robstride_encode_param_f32(cmd->vel, raw);
            robstride_build_param_write_frame(id, RS_PARAM_VEL_TARGET, raw, target_host, &tx);
            send(send_ctx, &tx);
            break;
        case MOTOR_MODE_FORCE_POS:
            if (vlim > 0.0f) {
                robstride_encode_param_f32(vlim, raw);
                robstride_build_param_write_frame(id, RS_PARAM_PP_VEL_MAX, raw, target_host, &tx);
                send(send_ctx, &tx);
            }
            robstride_encode_param_f32(cmd->pos, raw);
            robstride_build_param_write_frame(id, RS_PARAM_POS_TARGET, raw, target_host, &tx);
            send(send_ctx, &tx);
            break;
        default:
            break;
        }
    }
    transaction->sent_command_seq = cmd->seq;
    transaction->command_sent = true;
    return true;
}

/* SET_ZERO 二段写:通用 SET_ZERO admin 帧(comm 6)设机械零点后,再写
 * zero_sta(0x7029)=1 选启动坐标范围 -pi..pi(host 默认)。镜像 motorbridge
 * set_zero_position (motor_vendors/robstride/src/motor.rs)。 */
static void robstride_on_set_zero(uint8_t id, uint8_t host_id,
                                  motor_send_fn_t send, void *send_ctx)
{
    if (send == NULL) {
        return;
    }
    uint8_t raw[4];
    robstride_encode_param_u8(1, raw);
    twai_message_t tx;
    robstride_build_param_write_frame(id, RS_PARAM_ZERO_STATE, raw, host_id, &tx);
    send(send_ctx, &tx);
}

static void robstride_prepare_mode_change(uint8_t id, motor_mode_t mode,
                                           uint8_t host_id,
                                           motor_send_fn_t send, void *send_ctx)
{
    rs_mode_transaction_t *transaction = rs_mode_transaction(id);
    if (transaction == NULL || send == NULL) {
        return;
    }
    const uint8_t run_mode = robstride_run_mode_for(mode);
    if (run_mode == 0xFFU) {
        return;
    }

    /* frame_router may offer several host-ID candidates before the first
     * response.  Collect them into one transaction instead of replacing the
     * transaction on each callback. */
    if (transaction->phase != RS_MODE_IDLE &&
        transaction->phase != RS_MODE_READY &&
        transaction->phase != RS_MODE_FAILED &&
        transaction->target_mode == run_mode) {
        rs_add_host_candidate(transaction, host_id);
        return;
    }

    memset(transaction, 0, sizeof(*transaction));
    transaction->phase = RS_MODE_WAIT_DISABLE;
    transaction->target_mode = run_mode;
    transaction->host_id = host_id;
    transaction->host_count = 0U;
    transaction->host_index = 0U;
    rs_add_host_candidate(transaction, host_id);
    transaction->attempts = 0U;
    transaction->deadline_us = mb_now_us() + RS_MODE_DISABLE_SETTLE_US;
    transaction->send = send;
    transaction->send_ctx = send_ctx;

    /* Mode writes are only reliable while torque is disabled. */
    rs_send_admin(id, HOST_OP_DISABLE, host_id, send, send_ctx);
}

static bool robstride_mode_transition_pending(uint8_t id)
{
    const rs_mode_transaction_t *transaction = rs_mode_transaction(id);
    if (transaction == NULL) {
        return false;
    }
    switch (transaction->phase) {
    case RS_MODE_WAIT_DISABLE:
    case RS_MODE_WAIT_MODE_SETTLE:
    case RS_MODE_WAIT_MODE_REPLY:
    case RS_MODE_NEED_ENABLE:
    case RS_MODE_WAIT_ENABLE_REPLY:
        return true;
    default:
        return false;
    }
}

static bool robstride_mode_ready(uint8_t id, motor_mode_t mode)
{
    const rs_mode_transaction_t *transaction = rs_mode_transaction(id);
    const uint8_t run_mode = robstride_run_mode_for(mode);
    return transaction != NULL && run_mode != 0xFFU &&
           transaction->phase == RS_MODE_READY &&
           transaction->target_mode == run_mode;
}

static bool robstride_prepare_enable(uint8_t id, motor_mode_t mode,
                                     uint8_t host_id,
                                     motor_send_fn_t send, void *send_ctx)
{
    rs_mode_transaction_t *transaction = rs_mode_transaction(id);
    const uint8_t run_mode = robstride_run_mode_for(mode);
    if (transaction == NULL || send == NULL || run_mode == 0xFFU) {
        return false;
    }

    transaction->send = send;
    transaction->send_ctx = send_ctx;
    rs_add_host_candidate(transaction, host_id);

    if (transaction->phase == RS_MODE_READY &&
        transaction->target_mode == run_mode) {
        /* Match motorbridge enable(): do not release the target writer until
         * a fresh operation-status frame acknowledges ENABLE. */
        transaction->host_id = host_id;
        transaction->command_sent = false;
        transaction->phase = RS_MODE_NEED_ENABLE;
        transaction->deadline_us = mb_now_us();
        return true;
    }

    if (robstride_mode_transition_pending(id)) {
        /* The mode readback handler will advance to NEED_ENABLE because the
         * manager's desired enabled flag is set immediately after this hook. */
        return true;
    }

    /* No verified transaction exists (for example after bind/recovery).
     * Re-run disable -> mode write -> readback before enabling. */
    robstride_prepare_mode_change(id, mode, host_id, send, send_ctx);
    return true;
}

static bool rs_retry_mode_transition(uint8_t id,
                                      rs_mode_transaction_t *transaction)
{
    if (transaction == NULL || transaction->send == NULL ||
        transaction->host_count == 0U) {
        return false;
    }

    if ((uint8_t)(transaction->host_index + 1U) < transaction->host_count) {
        transaction->host_index++;
    } else if ((uint8_t)(transaction->attempts + 1U) < RS_MODE_MAX_ATTEMPTS) {
        transaction->attempts++;
        transaction->host_index = 0U;
    } else {
        transaction->phase = RS_MODE_FAILED;
        return false;
    }

    transaction->host_id = transaction->host_ids[transaction->host_index];
    transaction->phase = RS_MODE_WAIT_DISABLE;
    transaction->deadline_us = mb_now_us() + RS_MODE_DISABLE_SETTLE_US;
    rs_send_admin(id, HOST_OP_DISABLE, transaction->host_id,
                  transaction->send, transaction->send_ctx);
    return true;
}

static void robstride_on_param_reply(const twai_message_t *msg,
                                     uint16_t param_id,
                                     const uint8_t value[4])
{
    if (msg == NULL || value == NULL || param_id != RS_PARAM_MODE ||
        !msg->extd || msg->data_length_code < 8U) {
        return;
    }

    const uint8_t id = (uint8_t)((msg->identifier >> 8) & 0xFFU);
    const uint8_t host_id = (uint8_t)(msg->identifier & 0xFFU);
    rs_mode_transaction_t *transaction = rs_mode_transaction(id);
    if (transaction == NULL || transaction->phase != RS_MODE_WAIT_MODE_REPLY ||
        transaction->host_id != host_id) {
        return;
    }

    const int8_t actual_mode = (int8_t)value[0];
    if ((uint8_t)actual_mode == transaction->target_mode) {
        /* A successful parameter readback proves which responder accepted
         * the transaction.  Persist it so steady-state control uses one host
         * ID instead of probing all fallback IDs every cycle. */
        motor_manager_update_host_id(id, host_id);
        motor_state_t m;
        if (motor_manager_get_state(id, &m) && m.enabled) {
            transaction->phase = RS_MODE_NEED_ENABLE;
            transaction->deadline_us = mb_now_us();
        } else {
            transaction->phase = RS_MODE_READY;
        }
        return;
    }

    if (rs_retry_mode_transition(id, transaction)) {
        MB_LOGW(TAG, "robstride id=%u mode mismatch expected=%u got=%d, retry %u/%u",
                id, transaction->target_mode, (int)actual_mode,
                transaction->attempts, RS_MODE_MAX_ATTEMPTS);
    } else {
        transaction->phase = RS_MODE_FAILED;
        MB_LOGE(TAG, "robstride id=%u mode verify failed expected=%u got=%d",
                id, transaction->target_mode, (int)actual_mode);
    }
}

static void robstride_reg_verify_tick(void)
{
    const int64_t now_us = mb_now_us();
    for (uint8_t id = MOTORBRIDGE_MIN_MOTOR_ID;
         id <= MOTORBRIDGE_MAX_MOTOR_ID; ++id) {
        rs_mode_transaction_t *transaction = rs_mode_transaction(id);
        if (transaction == NULL || transaction->send == NULL) {
            continue;
        }

        /* A controlled stop changes the local mode to disabled.  Do not let
         * an older pending mode transaction resurrect torque afterwards. */
        motor_state_t m;
        if (motor_manager_get_state(id, &m) &&
            m.mode == MOTOR_MODE_DISABLED &&
            transaction->phase != RS_MODE_READY) {
            transaction->phase = RS_MODE_IDLE;
            continue;
        }

        if (transaction->phase == RS_MODE_WAIT_ENABLE_REPLY) {
            if (!motor_manager_get_state(id, &m) || !m.enabled) {
                transaction->phase = RS_MODE_READY;
                continue;
            }
            if (m.last_seen_us > transaction->enable_sent_us) {
                twai_message_t active_report;
                robstride_build_active_report_frame(
                    id, transaction->host_id, true, &active_report);
                transaction->send(transaction->send_ctx, &active_report);
                transaction->phase = RS_MODE_READY;
                MB_LOGI(TAG,
                        "robstride id=%u enable acknowledged run_mode=%u, active report enabled",
                        id, transaction->target_mode);
                continue;
            }
            if (now_us >= transaction->deadline_us) {
                transaction->phase = RS_MODE_FAILED;
                (void)motor_manager_set_enabled(id, false);
                rs_send_admin(id, HOST_OP_DISABLE, transaction->host_id,
                              transaction->send, transaction->send_ctx);
                MB_LOGE(TAG, "robstride id=%u enable status timeout", id);
                continue;
            }
        }

        /* A queued enable preparation must not re-arm a motor after DISABLE.
         * A later explicit enable starts a fresh acknowledgement transaction. */
        if (transaction->phase == RS_MODE_NEED_ENABLE &&
            motor_manager_get_state(id, &m) && !m.enabled) {
            transaction->phase = RS_MODE_READY;
            continue;
        }

        if (now_us < transaction->deadline_us) {
            continue;
        }

        switch (transaction->phase) {
        case RS_MODE_WAIT_DISABLE:
            rs_send_mode_write(id, transaction->target_mode,
                               transaction->host_id, transaction->send,
                               transaction->send_ctx);
            transaction->phase = RS_MODE_WAIT_MODE_SETTLE;
            transaction->deadline_us = now_us + RS_MODE_WRITE_SETTLE_US;
            break;
        case RS_MODE_WAIT_MODE_SETTLE:
            rs_send_mode_read(id, transaction->host_id, transaction->send,
                              transaction->send_ctx);
            transaction->phase = RS_MODE_WAIT_MODE_REPLY;
            transaction->deadline_us = now_us + RS_MODE_REPLY_TIMEOUT_US;
            break;
        case RS_MODE_WAIT_MODE_REPLY:
            if (rs_retry_mode_transition(id, transaction)) {
                MB_LOGW(TAG, "robstride id=%u mode readback timeout, retry %u/%u",
                        id, transaction->attempts, RS_MODE_MAX_ATTEMPTS);
            } else {
                transaction->phase = RS_MODE_FAILED;
                MB_LOGE(TAG, "robstride id=%u mode readback timeout expected=%u",
                        id, transaction->target_mode);
            }
            break;
        case RS_MODE_NEED_ENABLE:
            rs_send_admin(id, HOST_OP_ENABLE, transaction->host_id,
                          transaction->send, transaction->send_ctx);
            transaction->enable_sent_us = now_us;
            transaction->phase = RS_MODE_WAIT_ENABLE_REPLY;
            transaction->deadline_us = now_us + RS_ENABLE_REPLY_TIMEOUT_US;
            break;
        default:
            break;
        }
    }
}

/* fault 上报(comm 21)记入 motor_manager。镜像 motorbridge 9908d0f。 */
static void robstride_on_fault_report(uint8_t id, uint32_t fault_raw, uint32_t warning_raw)
{
    motor_manager_update_fault(id, fault_raw, warning_raw);
    /* 解码置位 bit 供日志,对应主机 FaultFlags。 */
    for (int b = 0; b < 32; ++b) {
        if (fault_raw & (1u << b)) {
            const char *nm = robstride_fault_bit_name(b);
            MB_LOGW(TAG, "robstride id=%u fault bit%d%s%s", id, b,
                    nm ? " " : "", nm ? nm : "");
        }
    }
}

/* 固件版本应答记入 motor_manager。镜像 motorbridge 5e54fe1。 */
static void robstride_on_version_reply(uint8_t id, const uint8_t version[4])
{
    motor_manager_update_fw_version(id, version);
    MB_LOGI(TAG, "robstride id=%u firmware v%u.%u.%u.%u",
            id, version[0], version[1], version[2], version[3]);
}

/* 受控停机:按缓存 mode + 状态构造停机帧并发出。镜像 motorbridge 4690e3f。
 * 调用方(frame_router HOST_OP_STOP)随后 set_mode DISABLED 解除控制环。 */
static bool robstride_stop(uint8_t id, const motor_state_t *m,
                           motor_send_fn_t send, void *send_ctx)
{
    if (send == NULL || m == NULL) {
        return false;
    }
    uint8_t host_ids[4] = {0};
    size_t host_count = 0U;
    if (m->host_id_valid) {
        host_ids[0] = m->host_id;
        host_count = 1U;
    } else {
        host_count = robstride_get_host_id_candidates(
            m->host_id, host_ids, sizeof(host_ids));
    }
    if (host_count == 0U) {
        return false;
    }
    motor_state_t host_state = *m;
    for (size_t i = 0U; i < host_count; ++i) {
        host_state.host_id = host_ids[i];
        twai_message_t tx;
        if (robstride_build_stop_frame(id, m->mode, &host_state, &tx)) {
            send(send_ctx, &tx);
        }
    }
    return true;
}

/* 厂商默认参数:motor_manager_set_vendor 绑定时调用,按配置默认型号查表写 PvT 量程。
 * 镜像 motorbridge faba15f / 082cbb8。未知型号退回安全 span(4π/50/120)。 */
static void robstride_apply_vendor_defaults(motor_params_t *params)
{
    if (params == NULL) {
        return;
    }
    const app_config_t *cfg = app_config_get();
    const char *model = (cfg != NULL && cfg->robstride_default_model != NULL)
                            ? cfg->robstride_default_model : "rs-00";
    float pmax, vmax, tmax;
    if (robstride_lookup_model_limits(model, &pmax, &vmax, &tmax)) {
        params->p_min = -pmax;
        params->p_max = pmax;
        params->v_min = -vmax;
        params->v_max = vmax;
        params->t_min = -tmax;
        params->t_max = tmax;
        /* 按型号填 MIT 增益上限(rs-03/04/06 -> 5000/100)。镜像 motorbridge
         * motor.rs:167-171。未知型号查不到时保留 init_default_params 的 500/5。 */
        float kp_max = 500.0f;
        float kd_max = 5.0f;
        (void)robstride_lookup_model_gains(model, &kp_max, &kd_max);
        params->kp_max = kp_max;
        params->kd_max = kd_max;
    } else {
        /* 安全兜底:4π / 50 / 120,覆盖大多数 RS 型号上界。 */
        params->p_min = -(4.0f * 3.14159265f);
        params->p_max =  (4.0f * 3.14159265f);
        params->v_min = -50.0f;
        params->v_max =  50.0f;
        params->t_min = -120.0f;
        params->t_max =  120.0f;
        params->kp_max = 500.0f;
        params->kd_max = 5.0f;
    }
}

static const motor_vendor_ops_t s_robstride_ops = {
    .name = "robstride",
    .reset_session = robstride_reset_session,
    .build_scan_request = robstride_build_scan_request,
    .parse_scan_reply = robstride_parse_scan_reply,
    .build_admin_frame = robstride_build_admin_frame,
    .build_control_frame = robstride_build_control_frame,
    .send_control = robstride_send_control,
    .parse_feedback = robstride_parse_feedback,
    .parse_host_id = robstride_parse_host_id,
    .get_host_id_candidates = robstride_get_host_id_candidates,
    .parse_param_reply = robstride_parse_param_reply,
    .on_param_reply = robstride_on_param_reply,
    .parse_fault_report = robstride_parse_fault_report,
    .on_fault_report = robstride_on_fault_report,
    .parse_version_reply = robstride_decode_version_reply,
    .on_version_reply = robstride_on_version_reply,
    .build_version_request = robstride_build_version_request_frame,
    .stop = robstride_stop,
    .apply_vendor_defaults = robstride_apply_vendor_defaults,
    .resolve_model = robstride_lookup_model_limits,
    .resolve_model_gains = robstride_lookup_model_gains,
    .reg_write_verified = NULL,
    .reg_verify_tick = robstride_reg_verify_tick,
    .prepare_mode_change = robstride_prepare_mode_change,
    .mode_transition_pending = robstride_mode_transition_pending,
    .mode_ready = robstride_mode_ready,
    .prepare_enable = robstride_prepare_enable,
    .apply_admin_cmd = NULL,
    .on_set_zero = robstride_on_set_zero,
};

const motor_vendor_ops_t *motor_vendor_get_robstride(void)
{
    return &s_robstride_ops;
}
