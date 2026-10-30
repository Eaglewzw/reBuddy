#include "motor_manager.h"

#include <inttypes.h>
#include <string.h>

#include "mb_port.h"
#include "vendors/motor_vendor.h"

static motor_state_t s_motors[MOTORBRIDGE_MAX_MOTOR_ID + 1];
static int s_max_motors = 0;
static SemaphoreHandle_t s_lock;
static const char *TAG = "motor_manager";

static bool valid_id(uint8_t id)
{
    return id <= s_max_motors;
}

static void init_default_params(motor_params_t *p)
{
    const app_config_t *cfg = app_config_get();
    p->kp = cfg->default_kp;
    p->kd = cfg->default_kd;

    /* 4340/4340P defaults; tune per model if needed. */
    p->p_min = -12.5f;
    p->p_max = 12.5f;
    p->v_min = -10.0f;
    p->v_max = 10.0f;
    p->t_min = -28.0f;
    p->t_max = 28.0f;
    /* MIT 增益上限默认值;robstride 按型号覆盖,damiao 保持此值。 */
    p->kp_max = 500.0f;
    p->kd_max = 5.0f;
}

void motor_manager_init(int max_motors)
{
    if (max_motors > MOTORBRIDGE_MAX_MOTOR_ID) {
        max_motors = MOTORBRIDGE_MAX_MOTOR_ID;
    }
    if (max_motors < MOTORBRIDGE_MIN_MOTOR_ID) {
        max_motors = MOTORBRIDGE_MIN_MOTOR_ID;
    }

    s_lock = xSemaphoreCreateMutex();
    s_max_motors = max_motors;
    memset(s_motors, 0, sizeof(s_motors));

    for (int i = MOTORBRIDGE_MIN_MOTOR_ID; i <= s_max_motors; ++i) {
        motor_state_t *m = &s_motors[i];
        m->id = (uint8_t)i;
        m->mode = MOTOR_MODE_DISABLED;
        m->enabled = false;
        m->channel = 0;          /* 默认通道 0;集成方按需 set_channel */
        m->vendor = NULL;
        m->host_id = MOTOR_DEFAULT_HOST_ID;
        m->host_id_valid = false;
        m->cmd.ratio = 0.1f;
        init_default_params(&m->params);
    }
}

int motor_manager_count(void)
{
    return s_max_motors;
}

bool motor_manager_set_vendor(uint8_t id, const char *vendor_name)
{
    if (!valid_id(id)) {
        return false;
    }
    const motor_vendor_ops_t *vendor = motor_vendor_get(vendor_name);
    if (vendor == NULL) {
        MB_LOGE(TAG, "Unknown motor vendor: %s", vendor_name);
        return false;
    }

    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_motors[id].vendor = vendor;
    /* Reset local control state while retaining telemetry/fault history.
     * Explicit service rebind also clears history and protocol transactions. */
    s_motors[id].mode = MOTOR_MODE_DISABLED;
    s_motors[id].mode_ready = false;
    s_motors[id].enabled = false;
    memset(&s_motors[id].cmd, 0, sizeof(s_motors[id].cmd));
    s_motors[id].cmd.ratio = 0.1f;
    s_motors[id].host_id = MOTOR_DEFAULT_HOST_ID;
    s_motors[id].host_id_valid = false;
    /* Apply the vendor's default model limits before an explicit model override. */
    if (vendor->apply_vendor_defaults != NULL) {
        vendor->apply_vendor_defaults(&s_motors[id].params);
    }
    xSemaphoreGive(s_lock);
    return true;
}

/* Service-owner-only: preserve binding configuration, discard the old session. */
void motor_manager_reset_session(uint8_t id)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    motor_state_t *motor = &s_motors[id];
    const can_channel_t channel = motor->channel;
    const motor_params_t params = motor->params;
    const motor_vendor_ops_t *vendor = motor->vendor;
    memset(motor, 0, sizeof(*motor));
    motor->id = id;
    motor->channel = channel;
    motor->params = params;
    motor->vendor = vendor;
    motor->host_id = MOTOR_DEFAULT_HOST_ID;
    motor->cmd.ratio = 0.1f;
    xSemaphoreGive(s_lock);
}

bool motor_manager_set_channel(uint8_t id, can_channel_t channel)
{
    if (!valid_id(id)) {
        return false;
    }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_motors[id].channel = channel;
    xSemaphoreGive(s_lock);
    return true;
}

bool motor_manager_set_mode(uint8_t id, motor_mode_t mode)
{
    if (!valid_id(id)) {
        return false;
    }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    if (mode == MOTOR_MODE_DISABLED || s_motors[id].mode != mode) {
        s_motors[id].mode_ready = false;
    }
    s_motors[id].mode = mode;
    xSemaphoreGive(s_lock);
    return true;
}

bool motor_manager_set_enabled(uint8_t id, bool enabled)
{
    if (!valid_id(id)) {
        return false;
    }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_motors[id].enabled = enabled;
    xSemaphoreGive(s_lock);
    return true;
}

bool motor_manager_set_mit_cmd(uint8_t id, float pos, float vel, float tau)
{
    if (!valid_id(id)) {
        return false;
    }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    if (s_motors[id].mode != MOTOR_MODE_MIT) {
        s_motors[id].mode_ready = false;
    }
    s_motors[id].mode = MOTOR_MODE_MIT;
    s_motors[id].cmd.pos = pos;
    s_motors[id].cmd.vel = vel;
    s_motors[id].cmd.tau = tau;
    s_motors[id].cmd.seq++;
    xSemaphoreGive(s_lock);
    return true;
}

bool motor_manager_set_pos_vel_cmd(uint8_t id, float pos, float vlim)
{
    if (!valid_id(id)) {
        return false;
    }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    if (s_motors[id].mode != MOTOR_MODE_POS_VEL) {
        s_motors[id].mode_ready = false;
    }
    s_motors[id].mode = MOTOR_MODE_POS_VEL;
    s_motors[id].cmd.pos = pos;
    s_motors[id].cmd.vlim = vlim;
    s_motors[id].cmd.seq++;
    xSemaphoreGive(s_lock);
    return true;
}

bool motor_manager_set_pos_csp_cmd(uint8_t id, float pos, float vlim)
{
    if (!valid_id(id)) {
        return false;
    }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    if (s_motors[id].mode != MOTOR_MODE_POS_CSP) {
        s_motors[id].mode_ready = false;
    }
    s_motors[id].mode = MOTOR_MODE_POS_CSP;
    s_motors[id].cmd.pos = pos;
    s_motors[id].cmd.vlim = vlim;
    s_motors[id].cmd.seq++;
    xSemaphoreGive(s_lock);
    return true;
}

bool motor_manager_set_pos_pp_cmd(uint8_t id, float pos, float vlim, float acc)
{
    if (!valid_id(id)) {
        return false;
    }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    if (s_motors[id].mode != MOTOR_MODE_POS_PP) {
        s_motors[id].mode_ready = false;
    }
    s_motors[id].mode = MOTOR_MODE_POS_PP;
    s_motors[id].cmd.pos = pos;
    s_motors[id].cmd.vlim = vlim;
    s_motors[id].cmd.acc = acc;
    s_motors[id].cmd.seq++;
    xSemaphoreGive(s_lock);
    return true;
}

bool motor_manager_set_vel_cmd(uint8_t id, float vel)
{
    if (!valid_id(id)) {
        return false;
    }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    if (s_motors[id].mode != MOTOR_MODE_VEL) {
        s_motors[id].mode_ready = false;
    }
    s_motors[id].mode = MOTOR_MODE_VEL;
    s_motors[id].cmd.vel = vel;
    s_motors[id].cmd.seq++;
    xSemaphoreGive(s_lock);
    return true;
}

bool motor_manager_set_force_pos_cmd(uint8_t id, float pos, float vlim, float ratio)
{
    if (!valid_id(id)) {
        return false;
    }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    if (s_motors[id].mode != MOTOR_MODE_FORCE_POS) {
        s_motors[id].mode_ready = false;
    }
    s_motors[id].mode = MOTOR_MODE_FORCE_POS;
    s_motors[id].cmd.pos = pos;
    s_motors[id].cmd.vlim = vlim;
    s_motors[id].cmd.ratio = ratio;
    s_motors[id].cmd.seq++;
    xSemaphoreGive(s_lock);
    return true;
}

bool motor_manager_set_gains(uint8_t id, float kp, float kd)
{
    if (!valid_id(id)) {
        return false;
    }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_motors[id].params.kp = kp;
    s_motors[id].params.kd = kd;
    xSemaphoreGive(s_lock);
    return true;
}

bool motor_manager_set_model(uint8_t id, const char *model)
{
    if (!valid_id(id) || model == NULL) {
        return false;
    }
    const motor_vendor_ops_t *v = s_motors[id].vendor;
    if (v == NULL || v->resolve_model == NULL) {
        return false;
    }
    float pmax, vmax, tmax;
    if (!v->resolve_model(model, &pmax, &vmax, &tmax)) {
        return false;
    }
    float kp_max = 500.0f;
    float kd_max = 5.0f;
    if (v->resolve_model_gains != NULL) {
        (void)v->resolve_model_gains(model, &kp_max, &kd_max);
    }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    motor_params_t *p = &s_motors[id].params;
    p->p_min = -pmax;
    p->p_max = pmax;
    p->v_min = -vmax;
    p->v_max = vmax;
    p->t_min = -tmax;
    p->t_max = tmax;
    p->kp_max = kp_max;
    p->kd_max = kd_max;
    xSemaphoreGive(s_lock);
    return true;
}

bool motor_manager_get_state(uint8_t id, motor_state_t *out)
{
    if (!valid_id(id) || out == NULL) {
        return false;
    }

    xSemaphoreTake(s_lock, portMAX_DELAY);
    *out = s_motors[id];
    xSemaphoreGive(s_lock);
    return true;
}

void motor_manager_update_feedback(uint8_t id, float pos, float speed, float torque,
                                   float t_mos, float t_rotor, uint16_t status)
{
    if (!valid_id(id)) {
        return;
    }

    xSemaphoreTake(s_lock, portMAX_DELAY);
    motor_state_t *m = &s_motors[id];
    m->online = true;
    m->position = pos;
    m->speed = speed;
    m->torque = torque;
    m->t_mos = t_mos;
    m->t_rotor = t_rotor;
    m->status = (uint16_t)((m->status &
                            (MOTOR_STATUS_FAULT | MOTOR_STATUS_WARNING)) |
                           status);
    const int64_t now_us = mb_now_us();
    m->last_seen_us = now_us;
    m->last_feedback_us = now_us;
    /* While disarmed, feedback defines the next safe hold target. This keeps
     * a later enable from replaying a position commanded before the arm was
     * moved by hand or disabled by a liveness fault. */
    if (!m->enabled) {
        m->cmd.pos = pos;
        m->cmd.vel = 0.0f;
        m->cmd.tau = 0.0f;
    }
    xSemaphoreGive(s_lock);
}

void motor_manager_mark_seen(uint8_t id)
{
    if (!valid_id(id)) {
        return;
    }

    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_motors[id].online = true;
    s_motors[id].last_seen_us = mb_now_us();
    xSemaphoreGive(s_lock);
}

void motor_manager_update_host_id(uint8_t id, uint8_t host_id)
{
    if (!valid_id(id)) {
        return;
    }

    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_motors[id].host_id = host_id;
    s_motors[id].host_id_valid = true;
    xSemaphoreGive(s_lock);
}

void motor_manager_mark_offline_by_timeout(int64_t now_us, int32_t timeout_ms)
{
    int64_t timeout_us = (int64_t)timeout_ms * 1000;

    xSemaphoreTake(s_lock, portMAX_DELAY);
    for (int i = MOTORBRIDGE_MIN_MOTOR_ID; i <= s_max_motors; ++i) {
        motor_state_t *m = &s_motors[i];
        /* Disabled motors are intentionally silent and may remain in the
         * configured/disarmed state while a person types the next command.
         * Feedback liveness is only a requirement while control is armed. */
        if (m->enabled &&
            (m->last_feedback_us <= 0 ||
             now_us - m->last_feedback_us > timeout_us)) {
            MB_LOGW(TAG,
                    "id=%u armed feedback timeout (%" PRId64 "ms > %dms), disarm",
                    m->id,
                    m->last_feedback_us > 0
                        ? (now_us - m->last_feedback_us) / 1000
                        : -1,
                    timeout_ms);
            m->online = false;
            m->enabled = false;
            m->cmd.pos = m->position;
            m->cmd.vel = 0.0f;
            m->cmd.tau = 0.0f;
        }
    }
    xSemaphoreGive(s_lock);
}

void motor_manager_estop_all(void)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    for (int i = MOTORBRIDGE_MIN_MOTOR_ID; i <= s_max_motors; ++i) {
        motor_state_t *m = &s_motors[i];
        m->mode = MOTOR_MODE_DISABLED;
        m->mode_ready = false;
        m->enabled = false;
        memset(&m->cmd, 0, sizeof(m->cmd));
    }
    xSemaphoreGive(s_lock);
}

void motor_manager_clear_online_all(void)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    for (int i = MOTORBRIDGE_MIN_MOTOR_ID; i <= s_max_motors; ++i) {
        motor_state_t *m = &s_motors[i];
        m->online = false;
        m->last_seen_us = 0;
    }
    xSemaphoreGive(s_lock);
}

void motor_manager_update_fault(uint8_t id, uint32_t fault_raw, uint32_t warning_raw)
{
    if (!valid_id(id)) {
        return;
    }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    motor_state_t *m = &s_motors[id];
    m->fault_raw = fault_raw;
    m->warning_raw = warning_raw;
    m->fault_seen_us = mb_now_us();
    m->status &= (uint16_t)~(MOTOR_STATUS_FAULT | MOTOR_STATUS_WARNING);
    if (fault_raw != 0) {
        m->status |= MOTOR_STATUS_FAULT;
    }
    if (warning_raw != 0U) {
        m->status |= MOTOR_STATUS_WARNING;
    }
    xSemaphoreGive(s_lock);
}

void motor_manager_update_fw_version(uint8_t id, const uint8_t version[4])
{
    if (!valid_id(id) || version == NULL) {
        return;
    }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    motor_state_t *m = &s_motors[id];
    m->fw_version[0] = version[0];
    m->fw_version[1] = version[1];
    m->fw_version[2] = version[2];
    m->fw_version[3] = version[3];
    m->fw_seen_us = mb_now_us();
    xSemaphoreGive(s_lock);
}

void motor_manager_for_each(void (*fn)(motor_state_t *m, void *ctx), void *ctx)
{
    if (fn == NULL) {
        return;
    }

    xSemaphoreTake(s_lock, portMAX_DELAY);
    for (int i = MOTORBRIDGE_MIN_MOTOR_ID; i <= s_max_motors; ++i) {
        fn(&s_motors[i], ctx);
    }
    xSemaphoreGive(s_lock);
}
