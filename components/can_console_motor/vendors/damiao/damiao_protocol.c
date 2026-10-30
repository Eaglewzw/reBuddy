#include "vendors/damiao/damiao_protocol.h"

#include <string.h>

#include "config/app_config.h"

typedef struct {
    const char *model;
    float pmax;
    float vmax;
    float tmax;
    /* MIT 增益上限,型号固有参数。damiao 全型号固定 500/5,镜像 motorbridge
     * damiao/src/protocol.rs 的 pub const KP_MAX/KD_MAX。作为列存进表后,
     * 查表无分支;未来某型号需不同增益,改该行即可。 */
    float kp_max;
    float kd_max;
} damiao_model_spec_t;

static const damiao_model_spec_t DAMIAO_MODELS[] = {
    { "4310",      12.5f,    30.0f,  10.0f, 500.0f,   5.0f },
    { "4340",      12.5f,    10.0f,  28.0f, 500.0f,   5.0f },
};

/* 按 model 名查表,返回 spec 指针;未知返回 NULL。两个 lookup 共用,
 * 消除重复遍历与型号分支。 */
static const damiao_model_spec_t *damiao_find_model(const char *model)
{
    if (model == NULL) {
        return NULL;
    }
    for (size_t i = 0U; i < sizeof(DAMIAO_MODELS) / sizeof(DAMIAO_MODELS[0]); ++i) {
        if (strcmp(DAMIAO_MODELS[i].model, model) == 0) {
            return &DAMIAO_MODELS[i];
        }
    }
    return NULL;
}

bool damiao_lookup_model_limits(const char *model,
                                float *pmax, float *vmax, float *tmax)
{
    const damiao_model_spec_t *s = damiao_find_model(model);
    if (s == NULL) {
        return false;
    }
    if (pmax != NULL) {
        *pmax = s->pmax;
    }
    if (vmax != NULL) {
        *vmax = s->vmax;
    }
    if (tmax != NULL) {
        *tmax = s->tmax;
    }
    return true;
}

/* 按 Damiao 型号查 MIT 增益上限:纯表查找,无分支。镜像 motorbridge
 * damiao/src/protocol.rs 的 pub const KP_MAX/KD_MAX(全型号固定 500/5)。
 * 未知型号回退 500/5 并返回 false。 */
bool damiao_lookup_model_gains(const char *model,
                               float *kp_max, float *kd_max)
{
    const damiao_model_spec_t *s = damiao_find_model(model);
    if (s == NULL) {
        if (kp_max) *kp_max = 500.0f;
        if (kd_max) *kd_max = 5.0f;
        return false;
    }
    if (kp_max) *kp_max = s->kp_max;
    if (kd_max) *kd_max = s->kd_max;
    return true;
}

static uint32_t float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    float clipped = x;
    if (clipped < x_min) {
        clipped = x_min;
    }
    if (clipped > x_max) {
        clipped = x_max;
    }
    return (uint32_t)(((clipped - x_min) * ((float)((1u << bits) - 1u))) / span);
}

static float uint_to_float(uint32_t x, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    return ((float)x) * span / ((float)((1u << bits) - 1u)) + x_min;
}

void damiao_build_scan_request(uint8_t id, twai_message_t *out)
{
    if (out == NULL) {
        return;
    }
    memset(out, 0, sizeof(*out));
    out->identifier = 0x7FF;
    out->extd = false;
    out->data_length_code = 8;
    out->data[0] = id;
    out->data[1] = 0;
    out->data[2] = 0xCC;
}

void damiao_build_admin_frame(uint8_t id, host_admin_opcode_t op,
                              uint8_t host_id, twai_message_t *out)
{
    (void)host_id;
    if (out == NULL) {
        return;
    }
    memset(out, 0, sizeof(*out));
    out->identifier = id;
    out->extd = false;
    out->data_length_code = 8;
    memset(out->data, 0xFF, sizeof(out->data));

    switch (op) {
    case HOST_OP_ENABLE:
        out->data[7] = 0xFC;
        break;
    case HOST_OP_DISABLE:
        out->data[7] = 0xFD;
        break;
    case HOST_OP_SET_ZERO:
        out->data[7] = 0xFE;
        break;
    case HOST_OP_CLEAR_ERROR:
        out->data[7] = 0xFB;
        break;
    default:
        out->data[7] = 0xFD;
        break;
    }
}

bool damiao_build_control_frame(uint8_t id,
                                motor_mode_t mode,
                                const damiao_cmd_t *cmd,
                                const motor_params_t *params,
                                twai_message_t *out)
{
    if (out == NULL || cmd == NULL || params == NULL) {
        return false;
    }

    memset(out, 0, sizeof(*out));
    out->extd = false;
    out->data_length_code = 8;

    switch (mode) {
    case MOTOR_MODE_MIT: {
        out->identifier = id;
        /* kp/kd 上限走 params(由 apply_vendor_defaults/set_model 按型号填),
         * <=0 回退 500/5,镜像 motorbridge damiao protocol.rs KP_MAX/KD_MAX。 */
        float kp_max = (params->kp_max > 0.0f) ? params->kp_max : 500.0f;
        float kd_max = (params->kd_max > 0.0f) ? params->kd_max : 5.0f;
        uint32_t pos_u = float_to_uint(cmd->pos, params->p_min, params->p_max, 16);
        uint32_t vel_u = float_to_uint(cmd->vel, params->v_min, params->v_max, 12);
        uint32_t kp_u = float_to_uint(params->kp, 0.0f, kp_max, 12);
        uint32_t kd_u = float_to_uint(params->kd, 0.0f, kd_max, 12);
        uint32_t tau_u = float_to_uint(cmd->tau, params->t_min, params->t_max, 12);

        out->data[0] = (uint8_t)((pos_u >> 8) & 0xFF);
        out->data[1] = (uint8_t)(pos_u & 0xFF);
        out->data[2] = (uint8_t)((vel_u >> 4) & 0xFF);
        out->data[3] = (uint8_t)(((vel_u & 0xF) << 4) | ((kp_u >> 8) & 0xF));
        out->data[4] = (uint8_t)(kp_u & 0xFF);
        out->data[5] = (uint8_t)((kd_u >> 4) & 0xFF);
        out->data[6] = (uint8_t)(((kd_u & 0xF) << 4) | ((tau_u >> 8) & 0xF));
        out->data[7] = (uint8_t)(tau_u & 0xFF);
        break;
    }
    case MOTOR_MODE_POS_VEL:
        out->identifier = 0x100 + id;
        memcpy(&out->data[0], &cmd->pos, sizeof(float));
        memcpy(&out->data[4], &cmd->vlim, sizeof(float));
        break;
    case MOTOR_MODE_VEL:
        out->identifier = 0x200 + id;
        memcpy(&out->data[0], &cmd->vel, sizeof(float));
        break;
    case MOTOR_MODE_FORCE_POS: {
        out->identifier = 0x300 + id;
        memcpy(&out->data[0], &cmd->pos, sizeof(float));
        uint16_t v_des = (uint16_t)(cmd->vlim < 0.0f ? 0.0f : (cmd->vlim > 100.0f ? 10000.0f : cmd->vlim * 100.0f));
        float ratio = cmd->ratio;
        if (ratio < 0.0f) {
            ratio = 0.0f;
        }
        if (ratio > 1.0f) {
            ratio = 1.0f;
        }
        uint16_t i_des = (uint16_t)(ratio * 10000.0f);
        memcpy(&out->data[4], &v_des, sizeof(uint16_t));
        memcpy(&out->data[6], &i_des, sizeof(uint16_t));
        break;
    }
    case MOTOR_MODE_DISABLED:
    default:
        return false;
    }

    return true;
}

bool damiao_parse_feedback(const twai_message_t *msg,
                           const motor_params_t *params,
                           uint8_t *id,
                           float *pos,
                           float *vel,
                           float *torque,
                           float *t_mos,
                           float *t_rotor,
                           uint16_t *status)
{
    if (msg == NULL || params == NULL || msg->data_length_code != 8 ||
        msg->extd || msg->rtr || msg->identifier == 0x7FF) {
        return false;
    }
    if (msg->identifier == MOTORBRIDGE_HOST_ADMIN_ID) {
        return false;
    }
    if (damiao_parse_register_reply(msg, NULL, NULL, NULL) ||
        damiao_parse_register_write_ack(msg, NULL)) {
        return false;
    }

    uint8_t can_id = msg->data[0] & 0x0F;
    uint16_t st = msg->data[0] >> 4;

    uint32_t pos_i = ((uint32_t)msg->data[1] << 8) | (uint32_t)msg->data[2];
    uint32_t vel_i = ((uint32_t)msg->data[3] << 4) | ((uint32_t)msg->data[4] >> 4);
    uint32_t torq_i = (((uint32_t)msg->data[4] & 0x0F) << 8) | (uint32_t)msg->data[5];

    if (id != NULL) {
        *id = can_id;
    }
    if (status != NULL) {
        *status = st;
    }
    if (pos != NULL) {
        *pos = uint_to_float(pos_i, params->p_min, params->p_max, 16);
    }
    if (vel != NULL) {
        *vel = uint_to_float(vel_i, params->v_min, params->v_max, 12);
    }
    if (torque != NULL) {
        *torque = uint_to_float(torq_i, params->t_min, params->t_max, 12);
    }
    if (t_mos != NULL) {
        *t_mos = (float)msg->data[6];
    }
    if (t_rotor != NULL) {
        *t_rotor = (float)msg->data[7];
    }

    return can_id > 0;
}

void damiao_build_register_write_cmd(uint8_t motor_id, uint8_t rid, uint32_t value, twai_message_t *out)
{
    if (out == NULL) {
        return;
    }
    memset(out, 0, sizeof(*out));
    out->identifier = 0x7FF;
    out->extd = false;
    out->data_length_code = 8;
    out->data[0] = motor_id;
    out->data[1] = 0;
    out->data[2] = 0x55;
    out->data[3] = rid;
    memcpy(&out->data[4], &value, sizeof(uint32_t));
}

void damiao_build_register_read_cmd(uint8_t motor_id, uint8_t rid, twai_message_t *out)
{
    if (out == NULL) {
        return;
    }
    memset(out, 0, sizeof(*out));
    out->identifier = 0x7FF;
    out->extd = false;
    out->data_length_code = 8;
    out->data[0] = motor_id;
    out->data[1] = 0;
    out->data[2] = 0x33;
    out->data[3] = rid;
}

void damiao_build_store_params_cmd(uint8_t motor_id, twai_message_t *out)
{
    if (out == NULL) {
        return;
    }
    memset(out, 0, sizeof(*out));
    out->identifier = 0x7FF;
    out->extd = false;
    out->data_length_code = 8;
    out->data[0] = motor_id;
    out->data[1] = 0;
    out->data[2] = 0xAA;
    out->data[3] = 0x01;
}

bool damiao_parse_register_reply(const twai_message_t *msg, uint8_t *id, uint8_t *rid, uint32_t *value)
{
    /* Replies start with a little-endian motor address. Supported IDs fit
     * in byte 0, so byte 1 must be zero. In telemetry, bytes 1..2 instead
     * encode position; byte 2 alone can legitimately be 0x33 or 0x55.
     * Standard ID 0x7FF carries requests, including cross-controller RX. */
    if (msg == NULL || msg->data_length_code != 8 || msg->extd || msg->rtr ||
        msg->identifier == 0x7FF || msg->data[1] != 0) {
        return false;
    }
    if (msg->data[2] != 0x33) {
        return false;
    }
    if (id != NULL) {
        *id = msg->data[0];
    }
    if (rid != NULL) {
        *rid = msg->data[3];
    }
    if (value != NULL) {
        uint32_t v = 0;
        memcpy(&v, &msg->data[4], sizeof(uint32_t));
        *value = v;
    }
    return true;
}

bool damiao_parse_register_write_ack(const twai_message_t *msg, uint8_t *rid)
{
    /* Same address/header validation as the register-read response. */
    if (msg == NULL || msg->data_length_code != 8 || msg->extd || msg->rtr ||
        msg->identifier == 0x7FF || msg->data[1] != 0) {
        return false;
    }
    if (msg->data[2] != 0x55) {
        return false;
    }
    if (rid != NULL) {
        *rid = msg->data[3];
    }
    return true;
}
