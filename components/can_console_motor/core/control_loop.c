#include "control_loop.h"

#include "mb_port.h"
#include "can_console_motor_types.h"

#include "config/app_config.h"
#include "core/motor_manager.h"
#include "transport/can_manager.h"
#include "vendors/motor_vendor.h"

static int64_t s_last_tx_us[MOTORBRIDGE_MAX_MOTOR_ID + 1];
static uint8_t s_reference_ids[CAN_CONSOLE_MOTOR_MAX_TARGET_GROUP];
static size_t s_reference_count;

void control_loop_reference_ids(const uint8_t *ids, size_t count)
{
    s_reference_count = count;
    for (size_t i = 0; i < count; ++i) s_reference_ids[i] = ids[i];
}

/* 厂商发帧回调:ctx 为该电机 channel(intptr_t 编码)。厂商 send_control /
 * prepare_mode_change 等只调 send(ctx,&frame),不直依赖 can_manager。 */
static void control_send_on_ctx(void *send_ctx, const twai_message_t *frame)
{
    (void)can_manager_send((can_channel_t)(intptr_t)send_ctx, frame, 8);
}

static void control_one(motor_state_t *m, void *ctx)
{
    (void)ctx;
    for (size_t i = 0; i < s_reference_count; ++i) if (m->id == s_reference_ids[i]) return;
    const app_config_t *cfg = app_config_get();

    /* Feedback is the liveness contract for closed-loop traffic.  Once a
     * motor stops answering, continuing to queue targets only drives the CAN
     * controller toward error-passive/bus-off and can replay a stale target
     * when the bus returns. */
    if (!m->online || !m->enabled || m->mode == MOTOR_MODE_DISABLED ||
        m->vendor == NULL) {
        return;
    }

    const int64_t now_us = mb_now_us();
    const int64_t min_interval_us = (int64_t)cfg->min_tx_interval_ms * 1000;
    if (min_interval_us > 0 && (now_us - s_last_tx_us[m->id]) < min_interval_us) {
        return;
    }

    /* 多帧参数控制(robstride 非 MIT:模式切换时写 run_mode,逐 tick 写目标参数)。
     * 返回 true 表示厂商已自行发帧;false 交给 build_control_frame 单帧路径。 */
    if (m->vendor->send_control != NULL &&
        m->vendor->send_control(m->id, m->host_id, m->host_id_valid, m->mode,
                                &m->cmd, &m->params,
                                control_send_on_ctx, (void *)(intptr_t)m->channel)) {
        s_last_tx_us[m->id] = now_us;
        return;
    }

    if (m->vendor->build_control_frame != NULL) {
        twai_message_t tx;
        if (m->vendor->build_control_frame(m->id, m->mode, &m->cmd, &m->params, &tx)) {
            (void)can_manager_send(m->channel, &tx, 8);
            s_last_tx_us[m->id] = now_us;
        }
    }
}

void control_loop_process_tick(float dt_s)
{
    motor_manager_for_each(control_one, NULL);
    (void)dt_s;
}
