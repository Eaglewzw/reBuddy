#include "vendors/motor_vendor.h"
#include "vendors/damiao/damiao_protocol.h"

#include <inttypes.h>
#include <string.h>

#include "mb_port.h"

#include "config/app_config.h"
#include "core/motor_manager.h"

static const char *TAG = "damiao_vendor";
static const motor_vendor_ops_t s_damiao_ops;

static void damiao_apply_vendor_defaults(motor_params_t *params)
{
    if (params == NULL) {
        return;
    }
    const app_config_t *cfg = app_config_get();
    const char *model = cfg != NULL ? cfg->damiao_default_model : NULL;
    float pmax, vmax, tmax;
    if (model != NULL &&
        damiao_lookup_model_limits(model, &pmax, &vmax, &tmax)) {
        params->p_min = -pmax;
        params->p_max = pmax;
        params->v_min = -vmax;
        params->v_max = vmax;
        params->t_min = -tmax;
        params->t_max = tmax;
        /* MIT 增益上限按型号填(damiao 全型号固定 500/5,镜像 motorbridge
         * protocol.rs pub const KP_MAX/KD_MAX)。与 robstride 路径对称。 */
        float kp_max = 500.0f;
        float kd_max = 5.0f;
        (void)damiao_lookup_model_gains(model, &kp_max, &kd_max);
        params->kp_max = kp_max;
        params->kd_max = kd_max;
    } else {
        /* 安全兜底:沿用 init_default_params 的 500/5 增益上限。 */
        params->kp_max = 500.0f;
        params->kd_max = 5.0f;
    }
}

static bool damiao_supports_mode(motor_mode_t mode)
{
    return mode >= MOTOR_MODE_MIT && mode <= MOTOR_MODE_FORCE_POS;
}

/* set-ids / store 前后的 settle 延迟(ms),bounded 使 set-ids 不长时间阻塞 RX。
 * 镜像 motorbridge bbcc2da / a265142。 */
#define DAMIAO_SETTLE_MS 30

/* One set-ids transaction, owned by its original bound slot. */
static struct {
    bool pending;
    uint8_t motor_id;
    uint8_t next_motor_id;
    uint8_t feedback_id;
    uint8_t received;
    uint32_t values[2];
} s_id_change;

/* 模式切换异步读回校验状态(每电机)。写 rid=10 后发读回,reply 在
 * on_register_reply 里比对,失配作废 s_mode_written 使下次重写。
 * 镜像 motorbridge bbcc2da / f4adda0,由单一 motor-service owner 推进。 */
static uint8_t s_mode_written[MOTORBRIDGE_MAX_MOTOR_ID + 1];
static int64_t s_mode_written_us[MOTORBRIDGE_MAX_MOTOR_ID + 1];
static uint8_t s_mode_expect[MOTORBRIDGE_MAX_MOTOR_ID + 1];
static bool s_mode_verify_pending[MOTORBRIDGE_MAX_MOTOR_ID + 1];
static int64_t s_mode_verify_issue_us[MOTORBRIDGE_MAX_MOTOR_ID + 1];
static int64_t s_mode_verified_us[MOTORBRIDGE_MAX_MOTOR_ID + 1];

/* 通用寄存器写 + 异步读回校验状态(任意 rid),推广自 rid=10 模式切换路径。
 * 镜像 motorbridge 40194bc (verify_register_u32: 2 attempts, 20ms gap,
 * 50ms ack timeout)。单一 motor-service owner 发送 fire-and-forget 读写、处理
 * reply,并在每次 tick 做超时重读。
 * 按 rid 索引(同一 rid 同时只允许一个待校验;set-ids 逐电机串行,满足)。
 * send/send_ctx 由 reg_write_verified 注入并缓存,供 reg_verify_tick 重读。 */
typedef struct {
    bool pending;
    uint8_t motor_id;
    uint32_t expect;
    int64_t issue_us;
    uint8_t attempts;
    motor_send_fn_t send;
    void *send_ctx;
} reg_verify_t;

#define DAMIAO_REG_VERIFY_TIMEOUT_US   50000LL
#define DAMIAO_REG_VERIFY_MAX_ATTEMPTS 2
static reg_verify_t s_reg_verify[256];

static void damiao_reset_session(uint8_t id)
{
    s_mode_written[id] = s_mode_expect[id] = 0;
    s_mode_written_us[id] = s_mode_verify_issue_us[id] = 0;
    s_mode_verified_us[id] = 0;
    s_mode_verify_pending[id] = false;
    for (size_t rid = 0; rid < 256; ++rid) {
        if (s_reg_verify[rid].motor_id == id) {
            s_reg_verify[rid].pending = false;
        }
    }
    if (s_id_change.motor_id == id || s_id_change.next_motor_id == id) {
        s_id_change.pending = false;
    }
}

/* ---- 经注入 send 发帧的 helper(不直调 can_manager)---- */

static void send_reg_write(uint8_t id, uint8_t rid, uint32_t value,
                           motor_send_fn_t send, void *send_ctx)
{
    twai_message_t tx;
    damiao_build_register_write_cmd(id, rid, value, &tx);
    send(send_ctx, &tx);
}

static void send_reg_read(uint8_t id, uint8_t rid,
                          motor_send_fn_t send, void *send_ctx)
{
    twai_message_t tx;
    damiao_build_register_read_cmd(id, rid, &tx);
    send(send_ctx, &tx);
}

static void send_store_params(uint8_t id, motor_send_fn_t send, void *send_ctx)
{
    twai_message_t tx;
    damiao_build_store_params_cmd(id, &tx);
    send(send_ctx, &tx);
}

static void send_admin(uint8_t id, host_admin_opcode_t op,
                       motor_send_fn_t send, void *send_ctx)
{
    twai_message_t tx;
    damiao_build_admin_frame(id, op, 0U, &tx);
    send(send_ctx, &tx);
}

/* ---- 模式切换预处理(写 rid=10 + 读回 + 异步 verify)---- */

static void damiao_prepare_mode_change(uint8_t id, motor_mode_t mode,
                                       uint8_t host_id,
                                       motor_send_fn_t send, void *send_ctx)
{
    (void)host_id;
    if (send == NULL) {
        return;
    }
    if (mode < MOTOR_MODE_MIT || mode > MOTOR_MODE_FORCE_POS) {
        return;
    }

    const int64_t now = mb_now_us();
    const uint8_t desired = (uint8_t)mode;

    /* 读回 verify 超过 200ms 无应答,视上次写未确认,强制重写。 */
    if (s_mode_verify_pending[id] &&
        (now - s_mode_verify_issue_us[id]) > 200000) {
        s_mode_verify_pending[id] = false;
        s_mode_written[id] = 0; /* fall through to re-write */
    }

    const bool same_mode = s_mode_written[id] == desired;
    const bool too_soon = (now - s_mode_written_us[id]) < 300000; /* 300ms */
    if (same_mode && (!s_mode_verify_pending[id] || too_soon)) {
        return;
    }

    /* rid=10 控制模式寄存器:1=MIT,2=POS_VEL,3=VEL,4=FORCE_POS。
     * 写后发 fire-and-forget 读回,reply 在 on_register_reply 比对。 */
    send_reg_write(id, 10, (uint32_t)desired, send, send_ctx);
    s_mode_written[id] = desired;
    s_mode_written_us[id] = now;

    s_mode_expect[id] = desired;
    s_mode_verify_pending[id] = true;
    s_mode_verify_issue_us[id] = now;
    s_mode_verified_us[id] = 0;
    send_reg_read(id, 10, send, send_ctx);
}

/* A queued register write is not a completed mode transition. Give the drive
 * one control period after its verified readback before ENABLE/control. */
static bool damiao_mode_ready(uint8_t id, motor_mode_t mode)
{
    return mode >= MOTOR_MODE_MIT && mode <= MOTOR_MODE_FORCE_POS &&
        s_mode_written[id] == (uint8_t)mode && !s_mode_verify_pending[id] &&
        s_mode_verified_us[id] > 0 && mb_now_us() - s_mode_verified_us[id] >= 20000;
}

/* ---- set-ids:disable/settle/写 rid 7,8/store/verify ---- */

static void damiao_apply_admin_cmd(uint8_t motor_id, const host_admin_cmd_t *cmd,
                                   motor_send_fn_t send, void *send_ctx)
{
    if (cmd == NULL || send == NULL || cmd->op != HOST_OP_SET_IDS) {
        return;
    }

    uint8_t old_id = motor_id;
    uint8_t new_motor = cmd->new_motor_id;
    uint8_t new_fb = cmd->new_feedback_id;

    MB_LOGI(TAG, "set-ids old=0x%02X -> motor=0x%02X feedback=0x%02X store=%d verify=%d",
            old_id, new_motor, new_fb, cmd->store_after_set, cmd->verify_after_set);

    /* damiao 写 MST_ID/ESC_ID 与 store 前必须 disable,否则新 id 不生效。 */
    send_admin(old_id, HOST_OP_DISABLE, send, send_ctx);
    vTaskDelay(pdMS_TO_TICKS(DAMIAO_SETTLE_MS));

    /* rid=7 MST_ID(feedback), rid=8 ESC_ID(motor) */
    send_reg_write(old_id, 7, (uint32_t)new_fb, send, send_ctx);
    send_reg_write(old_id, 8, (uint32_t)new_motor, send, send_ctx);

    if (cmd->store_after_set) {
        /* 写 ESC_ID 后电机切到监听 new_motor_id;store 必须在 new id 上发,
         * 否则 store 帧被丢、id 不跨重启持久。镜像 a265142。 */
        vTaskDelay(pdMS_TO_TICKS(DAMIAO_SETTLE_MS));
        send_store_params(new_motor, send, send_ctx);
        vTaskDelay(pdMS_TO_TICKS(DAMIAO_SETTLE_MS));
    }

    if (cmd->verify_after_set) {
        memset(&s_id_change, 0, sizeof(s_id_change));
        s_id_change.pending = true;
        s_id_change.motor_id = old_id;
        s_id_change.next_motor_id = new_motor;
        s_id_change.feedback_id = new_fb;
        send_reg_read(new_motor, 7, send, send_ctx);
        send_reg_read(new_motor, 8, send, send_ctx);
    }
}

/* ---- RX 侧 verify 比对(无 send,只读/记状态)---- */

static void damiao_on_register_reply(uint8_t id, uint8_t rid, uint32_t value)
{
    reg_verify_t *verify = &s_reg_verify[rid];
    if (verify->pending && verify->motor_id == id) {
        if (value != verify->expect) {
            MB_LOGW(TAG, "reg verify mismatch rid=%u motor=%u expected=0x%08" PRIX32
                    " got=0x%08" PRIX32, rid, id, verify->expect, value);
        }
        verify->pending = false;
        return;
    }
    if (s_id_change.pending && id == s_id_change.next_motor_id && (rid == 7 || rid == 8)) {
        const unsigned index = rid - 7U;
        s_id_change.values[index] = value;
        s_id_change.received |= (uint8_t)(1U << index);
        if (s_id_change.received == 3U) {
            if (s_id_change.values[0] != s_id_change.feedback_id ||
                s_id_change.values[1] != s_id_change.next_motor_id) {
                MB_LOGW(TAG, "set-ids verify mismatch expected motor=%u feedback=%u"
                        " got motor=%u feedback=%u", s_id_change.next_motor_id,
                        s_id_change.feedback_id, (unsigned)s_id_change.values[1],
                        (unsigned)s_id_change.values[0]);
            }
            s_id_change.pending = false;
        }
    } else if (rid == 10 && s_mode_verify_pending[id]) {
        if ((uint8_t)value != s_mode_expect[id]) {
            MB_LOGW(TAG, "damiao id=%u mode verify mismatch expected=%u got=%u, will retry",
                    id, s_mode_expect[id], (uint8_t)value);
            s_mode_written[id] = 0;
            s_mode_verified_us[id] = 0;
        } else {
            s_mode_verified_us[id] = mb_now_us();
        }
        s_mode_verify_pending[id] = false;
    }
}

static bool damiao_process_register_reply(can_channel_t channel, const twai_message_t *msg)
{
    uint8_t id, rid;
    uint32_t value;
    if (damiao_parse_register_reply(msg, &id, &rid, &value)) {
        /* set-ids replies use the new address while the slot is still bound
         * under the original ID. Other replies belong to their sender slot. */
        const uint8_t owner = s_id_change.pending && id == s_id_change.next_motor_id &&
            (rid == 7 || rid == 8) ? s_id_change.motor_id : id;
        motor_state_t motor;
        if (motor_manager_get_state(owner, &motor) && motor.vendor == &s_damiao_ops && motor.channel == channel) {
            damiao_on_register_reply(id, rid, value);
        }
        return true;
    }
    /* Write ACKs carry no control-state update. */
    return damiao_parse_register_write_ack(msg, NULL);
}

/* 受控停机:damiao 无原生 controlled_stop,用 MIT 保持帧(pos=缓存位置,vel=0,
 * kp/kd=cached,tau=0)作 best-effort hold。镜像 motorbridge 4690e3f。
 * 调用方(frame_router HOST_OP_STOP)随后 set_mode DISABLED 解除控制环。 */
static bool damiao_stop(uint8_t id, const motor_state_t *m,
                        motor_send_fn_t send, void *send_ctx)
{
    if (send == NULL || m == NULL) {
        return false;
    }
    damiao_cmd_t hold = {
        .pos = m->position,
        .vel = 0.0f,
        .tau = 0.0f,
    };
    twai_message_t tx;
    if (damiao_build_control_frame(id, MOTOR_MODE_MIT, &hold, &m->params, &tx)) {
        send(send_ctx, &tx);
        return true;
    }
    return false;
}

/* 通用寄存器写 + 异步读回校验:写寄存器,记 pending(expect/attempts/注入 send),
 * 立即发 fire-and-forget 读。镜像 motorbridge 40194bc。 */
static void damiao_reg_write_verified(uint8_t id, uint8_t rid, uint32_t value,
                                      motor_send_fn_t send, void *send_ctx)
{
    if (send == NULL) {
        return;
    }
    send_reg_write(id, rid, value, send, send_ctx);

    reg_verify_t *rv = &s_reg_verify[rid];
    rv->pending = true;
    rv->motor_id = id;
    rv->expect = value;
    rv->issue_us = mb_now_us();
    rv->attempts = 0;
    rv->send = send;
    rv->send_ctx = send_ctx;
    /* fire-and-forget 读;reply 在 on_register_reply 比对。 */
    send_reg_read(id, rid, send, send_ctx);
}

/* 寄存器校验超时重读 sweep:由 motor service owner 周期调用。
 * 50ms ack 超时 → 若 attempts<2 重发读,否则放弃。下一次 sweep 自然提供
 * 重试间隔,这里不能阻塞 owner,否则 RX reply 也无法及时消费。 */
static void damiao_reg_verify_tick(void)
{
    const int64_t now = mb_now_us();
    for (int rid = 0; rid < 256; ++rid) {
        reg_verify_t *rv = &s_reg_verify[rid];
        if (!rv->pending) {
            continue;
        }
        if ((now - rv->issue_us) <= DAMIAO_REG_VERIFY_TIMEOUT_US) {
            continue; /* 未超时,等应答。 */
        }
        if (rv->attempts < DAMIAO_REG_VERIFY_MAX_ATTEMPTS) {
            rv->attempts++;
            rv->issue_us = mb_now_us();
            send_reg_read(rv->motor_id, (uint8_t)rid, rv->send, rv->send_ctx);
            MB_LOGW(TAG, "reg verify timeout rid=%u motor=%u, retry %u/%u",
                    rid, rv->motor_id, rv->attempts, DAMIAO_REG_VERIFY_MAX_ATTEMPTS);
        } else {
            MB_LOGE(TAG, "reg verify give up rid=%u motor=%u expected=0x%08" PRIX32,
                    rid, rv->motor_id, rv->expect);
            rv->pending = false;
        }
    }
}

static const motor_vendor_ops_t s_damiao_ops = {
    .name = "damiao",
    .reset_session = damiao_reset_session,
    .build_scan_request = damiao_build_scan_request,
    .build_admin_frame = damiao_build_admin_frame,
    .build_control_frame = damiao_build_control_frame,
    .parse_feedback = damiao_parse_feedback,
    .process_register_reply = damiao_process_register_reply,
    .stop = damiao_stop,
    .apply_vendor_defaults = damiao_apply_vendor_defaults,
    .resolve_model = damiao_lookup_model_limits,
    .resolve_model_gains = damiao_lookup_model_gains,
    .supports_mode = damiao_supports_mode,
    .reg_write_verified = damiao_reg_write_verified,
    .reg_verify_tick = damiao_reg_verify_tick,
    .prepare_mode_change = damiao_prepare_mode_change,
    .mode_ready = damiao_mode_ready,
    .apply_admin_cmd = damiao_apply_admin_cmd,
    .on_set_zero = NULL,
};

const motor_vendor_ops_t *motor_vendor_get_damiao(void)
{
    return &s_damiao_ops;
}
