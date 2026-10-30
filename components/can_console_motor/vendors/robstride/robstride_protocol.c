#include "vendors/robstride/robstride_protocol.h"
#include <string.h>

#define RS_COMM_GET_DEVID  0
#define RS_COMM_CONTROL    1
#define RS_COMM_STATUS     2
#define RS_COMM_ENABLE     3
#define RS_COMM_DISABLE    4
#define RS_COMM_SET_ZERO   6
#define RS_COMM_CLEAR_ERR  4 /* Clear error uses DISABLE with data[0]=1. */
#define RS_COMM_READ_PARAM  17
#define RS_COMM_WRITE_PARAM 18
#define RS_COMM_SAVE        22
#define RS_COMM_ACTIVE_REPORT 24

/* 位置量程对所有 Robstride 型号都是 4*pi(对齐 motorbridge motor.rs)。 */
#define RS_PMAX (4.0f * 3.14159265f)

/* 未知型号时的安全回退量程:取族内最大 vmax/tmax(rs-06 vmax=50,rs-04 tmax=120),
 * 使命令编码不会过驱任何型号——100% scale 映射到电机自身额定最大,更小的命令值
 * 是欠送达(安全的失败方向)。早期默认(10/28,抄自 Damiao)对多个型号过速/过矩。
 * 正确做法是按真实型号查表;esp32 路径用 per-motor params 解析量程。 */
#define RS_SAFE_VMAX 50.0f
#define RS_SAFE_TMAX 120.0f

/* 按型号规格表,移植自 motorbridge motor_vendors/robstride/src/motor.rs
 * (post bb5134e / 082cbb8 修正)。pmax 全 4*pi;vmax/tmax 按厂商手册。 */
typedef struct {
    const char *model;
    float pmax;
    float vmax;
    float tmax;
    /* MIT 增益上限,型号固有参数。高力矩型号(rs-03/04/06)用 5000/100,
     * 其余 500/5。移植自 motorbridge motor.rs:167-171。作为列存进表后,
     * 查表函数无需任何分支判断,加新型号只改一行。 */
    float kp_max;
    float kd_max;
} rs_model_spec_t;

static const rs_model_spec_t RS_MODELS[] = {
    { "rs-00", RS_PMAX, 33.0f, 14.0f,   500.0f,   5.0f },
    { "rs-01", RS_PMAX, 44.0f, 17.0f,   500.0f,   5.0f },
    { "rs-02", RS_PMAX, 44.0f, 17.0f,   500.0f,   5.0f },
    { "rs-03", RS_PMAX, 20.0f, 60.0f,  5000.0f, 100.0f },
    { "rs-04", RS_PMAX, 15.0f, 120.0f, 5000.0f, 100.0f },
    { "rs-05", RS_PMAX, 50.0f,  5.5f,   500.0f,   5.0f },
    { "rs-06", RS_PMAX, 50.0f, 36.0f,  5000.0f, 100.0f },
};

/* 按 model 名查表,返回 spec 指针;未知返回 NULL。两个 lookup 共用它,
 * 消除重复遍历与型号分支。 */
static const rs_model_spec_t *robstride_find_model(const char *model)
{
    if (model == NULL) {
        return NULL;
    }
    for (size_t i = 0; i < sizeof(RS_MODELS) / sizeof(RS_MODELS[0]); ++i) {
        if (strcmp(RS_MODELS[i].model, model) == 0) {
            return &RS_MODELS[i];
        }
    }
    return NULL;
}

bool robstride_lookup_model_limits(const char *model, float *pmax, float *vmax, float *tmax)
{
    const rs_model_spec_t *s = robstride_find_model(model);
    if (s == NULL) {
        return false;
    }
    if (pmax) *pmax = s->pmax;
    if (vmax) *vmax = s->vmax;
    if (tmax) *tmax = s->tmax;
    return true;
}

/* 按型号查 MIT 增益上限:纯表查找,无分支。未知型号回退 500/5 并返回
 * false,便于调用方回退。镜像 motorbridge motor.rs:167-171。 */
bool robstride_lookup_model_gains(const char *model, float *kp_max, float *kd_max)
{
    const rs_model_spec_t *s = robstride_find_model(model);
    if (s == NULL) {
        if (kp_max) *kp_max = 500.0f;
        if (kd_max) *kd_max = 5.0f;
        return false;
    }
    if (kp_max) *kp_max = s->kp_max;
    if (kd_max) *kd_max = s->kd_max;
    return true;
}

static uint32_t build_ext_id(uint32_t comm, uint16_t extra, uint8_t id)
{
    return (comm << 24) | (((uint32_t)extra) << 8) | ((uint32_t)id);
}

/* Robstride 电机只应答其配置 host id 的 device-id 请求。出厂默认 0xFF,
 * 但配到 0xFD / 0xFE 的单元会被单 host 扫描漏掉。在常见 host id 间轮询,
 * 使多趟扫描(扫描任务循环 while active)探到全部。镜像 motorbridge 99ecc65。 */
static uint16_t robstride_next_scan_host_id(void)
{
    static const uint16_t host_ids[] = { 0x00FF, 0x00FD, 0x00FE };
    static size_t s_idx = 0;
    const uint16_t host_id = host_ids[s_idx];
    s_idx = (s_idx + 1U) % (sizeof(host_ids) / sizeof(host_ids[0]));
    return host_id;
}

void robstride_build_scan_request(uint8_t id, twai_message_t *out)
{
    if (!out) return;
    memset(out, 0, sizeof(*out));
    out->identifier = build_ext_id(RS_COMM_GET_DEVID, robstride_next_scan_host_id(), id);
    out->extd = true;
    out->data_length_code = 8;
}

bool robstride_parse_scan_reply(const twai_message_t *msg, uint8_t *id)
{
    if (msg == NULL || id == NULL || !msg->extd ||
        msg->data_length_code < 8U) {
        return false;
    }
    const uint32_t comm_type = (msg->identifier >> 24) & 0x1FU;
    if (comm_type != RS_COMM_GET_DEVID) {
        return false;
    }
    *id = (uint8_t)((msg->identifier >> 8) & 0xFFU);
    return true;
}

void robstride_build_admin_frame(uint8_t id, host_admin_opcode_t op,
                                 uint8_t host_id, twai_message_t *out)
{
    if (!out) return;
    memset(out, 0, sizeof(*out));
    out->extd = true;

    uint32_t comm = RS_COMM_DISABLE;
    switch (op) {
    case HOST_OP_ENABLE: comm = RS_COMM_ENABLE; break;
    case HOST_OP_DISABLE: comm = RS_COMM_DISABLE; break;
    case HOST_OP_SET_ZERO: comm = RS_COMM_SET_ZERO; break;
    case HOST_OP_CLEAR_ERROR: comm = RS_COMM_CLEAR_ERR; break;
    default: break;
    }
    out->identifier = build_ext_id(comm, host_id, id);

    /* SET_ZERO 需 0x01 触发字节(dlc=1);dlc=0 空帧被电机忽略。
     * 镜像 motorbridge e058c0b(set_zero payload)。 */
    if (op == HOST_OP_SET_ZERO) {
        out->data_length_code = 8;
        out->data[0] = 0x01;
    } else {
        out->data_length_code = 8;
        if (op == HOST_OP_CLEAR_ERROR) {
            out->data[0] = 0x01;
        }
    }
}

static float clampf(float v, float min, float max)
{
    if (v < min) return min;
    if (v > max) return max;
    return v;
}

/* 按该电机 params 解析量程;params 缺省或 <=0 时用安全回退。 */
static void robstride_resolve_limits(const motor_params_t *params,
                                     float *pmax, float *vmax, float *tmax)
{
    float p = (params && params->p_max > 0.0f) ? params->p_max : RS_PMAX;
    float v = (params && params->v_max > 0.0f) ? params->v_max : RS_SAFE_VMAX;
    float t = (params && params->t_max > 0.0f) ? params->t_max : RS_SAFE_TMAX;
    if (pmax) *pmax = p;
    if (vmax) *vmax = v;
    if (tmax) *tmax = t;
}

bool robstride_build_control_frame(uint8_t id, motor_mode_t mode, const damiao_cmd_t *cmd, const motor_params_t *params, twai_message_t *out)
{
    if (!out || !cmd || !params) return false;
    /* The comm=1 frame is only the MIT path.  Parameter-based position and
     * velocity modes are emitted by the RobStride vendor send_control hook. */
    if (mode != MOTOR_MODE_MIT) return false;

    /* The caller passes a stack message. Clear RTR and the other TWAI flags:
     * an accidental remote frame discards the eight control data bytes. */
    memset(out, 0, sizeof(*out));

    float pmax, vmax, tmax;
    robstride_resolve_limits(params, &pmax, &vmax, &tmax);
    /* 按型号查表的 MIT 增益上限(rs-03/04/06 -> 5000/100,其余 500/5),
     * 由 apply_vendor_defaults / set_model 烤进 params。<=0 时回退 500/5,
     * 镜像 motorbridge motor.rs:167-171。 */
    float kp_max = (params->kp_max > 0.0f) ? params->kp_max : 500.0f;
    float kd_max = (params->kd_max > 0.0f) ? params->kd_max : 5.0f;

    uint16_t pos_u16  = (uint16_t)(((clampf(cmd->pos, -pmax, pmax) / pmax) + 1.0f) * 32767.0f);
    uint16_t vel_u16  = (uint16_t)(((clampf(cmd->vel, -vmax, vmax) / vmax) + 1.0f) * 32767.0f);
    uint16_t kp_u16   = (uint16_t)((clampf(params->kp, 0.0f, kp_max) / kp_max) * 65535.0f);
    uint16_t kd_u16   = (uint16_t)((clampf(params->kd, 0.0f, kd_max) / kd_max) * 65535.0f);
    uint16_t torq_u16 = (uint16_t)(((clampf(cmd->tau, -tmax, tmax) / tmax) + 1.0f) * 32767.0f);

    out->identifier = build_ext_id(RS_COMM_CONTROL, torq_u16, id);
    out->extd = true;
    out->data_length_code = 8;
    out->data[0] = pos_u16 >> 8;
    out->data[1] = pos_u16 & 0xFF;
    out->data[2] = vel_u16 >> 8;
    out->data[3] = vel_u16 & 0xFF;
    out->data[4] = kp_u16 >> 8;
    out->data[5] = kp_u16 & 0xFF;
    out->data[6] = kd_u16 >> 8;
    out->data[7] = kd_u16 & 0xFF;
    return true;
}

bool robstride_parse_feedback(const twai_message_t *msg, const motor_params_t *params, uint8_t *id, float *pos, float *vel, float *torque, float *t_mos, float *t_rotor, uint16_t *status)
{
    if (!msg || !msg->extd || msg->data_length_code < 8) return false;

    uint32_t comm_type = (msg->identifier >> 24) & 0x1F;
    uint16_t extra_data = (msg->identifier >> 8) & 0xFFFF;

    /* Firmware >= RS00 0.0.3.3 uses comm 24 for periodic active-report
     * feedback, while command acknowledgements remain comm 2.  Both payloads
     * share the same status/PvT layout. */
    if (comm_type != RS_COMM_STATUS &&
        comm_type != RS_COMM_ACTIVE_REPORT) {
        return false;
    }

    uint8_t node_id = (uint8_t)(extra_data & 0xFF);

    uint16_t pos_u16  = (msg->data[0] << 8) | msg->data[1];
    uint16_t vel_u16  = (msg->data[2] << 8) | msg->data[3];
    uint16_t torq_u16 = (msg->data[4] << 8) | msg->data[5];
    uint16_t temp_u16 = (msg->data[6] << 8) | msg->data[7];

    /* 用该电机自身量程解码,使反馈与命令编码 1:1。此前写死 Damiao 默认
     * (10/28),所有 Robstride 的速度/力矩反馈都错。镜像 motorbridge 082cbb8。 */
    float pmax, vmax, tmax;
    robstride_resolve_limits(params, &pmax, &vmax, &tmax);

    if (id) *id = node_id;
    if (pos) *pos = ((float)pos_u16 / 32767.0f - 1.0f) * pmax;
    if (vel) *vel = ((float)vel_u16 / 32767.0f - 1.0f) * vmax;
    if (torque) *torque = ((float)torq_u16 / 32767.0f - 1.0f) * tmax;
    if (t_mos) *t_mos = (float)temp_u16 * 0.1f;
    if (t_rotor) *t_rotor = 0.0f;

    if (status) {
        uint16_t st = (uint16_t)((extra_data >> 14) & 0x03U);
        if (extra_data & (1U << 13)) st |= MOTOR_STATUS_UNCALIBRATED;
        if (extra_data & (1U << 12)) st |= MOTOR_STATUS_STALL;
        if (extra_data & (1U << 11)) st |= MOTOR_STATUS_MAG_ENCODER_FAULT;
        if (extra_data & (1U << 10)) st |= MOTOR_STATUS_OVERTEMPERATURE;
        if (extra_data & (1U << 9)) st |= MOTOR_STATUS_OVERCURRENT;
        if (extra_data & (1U << 8)) st |= MOTOR_STATUS_UNDERVOLTAGE;
        *status = st;
    }
    return true;
}

bool robstride_parse_host_id(const twai_message_t *msg, uint8_t *host_id)
{
    if (msg == NULL || host_id == NULL || !msg->extd ||
        msg->data_length_code < 8U) {
        return false;
    }
    uint32_t comm_type = (msg->identifier >> 24) & 0x1FU;
    if (comm_type != RS_COMM_GET_DEVID && comm_type != RS_COMM_STATUS &&
        comm_type != RS_COMM_ACTIVE_REPORT &&
        comm_type != RS_COMM_FAULT_REPORT) {
        return false;
    }
    *host_id = (uint8_t)(msg->identifier & 0xFFU);
    return true;
}

size_t robstride_get_host_id_candidates(uint8_t preferred_host_id,
                                        uint8_t *out_ids, size_t capacity)
{
    static const uint8_t fallback_ids[] = {0xFD, 0xFF, 0xFE};
    const size_t max_candidates = 1U + (sizeof(fallback_ids) / sizeof(fallback_ids[0]));
    if (out_ids == NULL || capacity == 0U) {
        return 0U;
    }

    size_t count = 0U;
    const uint8_t candidates[4] = {
        preferred_host_id, fallback_ids[0], fallback_ids[1], fallback_ids[2],
    };
    for (size_t i = 0U; i < max_candidates && count < capacity; ++i) {
        bool duplicate = false;
        for (size_t j = 0U; j < count; ++j) {
            if (out_ids[j] == candidates[i]) {
                duplicate = true;
                break;
            }
        }
        if (!duplicate) {
            out_ids[count++] = candidates[i];
        }
    }
    return count;
}

/* ---- 运行参数读写(comm 17/18),从 motorbridge 移植 ---- */

void robstride_encode_param_i8(int8_t v, uint8_t out[4])
{
    if (!out) return;
    out[0] = (uint8_t)v;
    out[1] = 0;
    out[2] = 0;
    out[3] = 0;
}

void robstride_encode_param_u8(uint8_t v, uint8_t out[4])
{
    if (!out) return;
    out[0] = v;
    out[1] = 0;
    out[2] = 0;
    out[3] = 0;
}

void robstride_encode_param_f32(float v, uint8_t out[4])
{
    if (!out) return;
    memcpy(out, &v, sizeof(float));
}

void robstride_build_param_write_frame(uint8_t id,
                                        uint16_t param_id,
                                        const uint8_t raw_value[4],
                                        uint8_t host_id,
                                        twai_message_t *out)
{
    if (!out || !raw_value) return;
    memset(out, 0, sizeof(*out));
    out->identifier = build_ext_id(RS_COMM_WRITE_PARAM, host_id, id);
    out->extd = true;
    out->data_length_code = 8;
    /* param id LE in [0..2], [2..4] zero, raw value LE in [4..8]. */
    out->data[0] = (uint8_t)(param_id & 0xFF);
    out->data[1] = (uint8_t)((param_id >> 8) & 0xFF);
    out->data[2] = 0;
    out->data[3] = 0;
    out->data[4] = raw_value[0];
    out->data[5] = raw_value[1];
    out->data[6] = raw_value[2];
    out->data[7] = raw_value[3];
}

void robstride_build_param_read_frame(uint8_t id,
                                      uint16_t param_id,
                                      uint8_t host_id,
                                      twai_message_t *out)
{
    if (!out) return;
    memset(out, 0, sizeof(*out));
    out->identifier = build_ext_id(RS_COMM_READ_PARAM, host_id, id);
    out->extd = true;
    out->data_length_code = 8;
    out->data[0] = (uint8_t)(param_id & 0xFF);
    out->data[1] = (uint8_t)((param_id >> 8) & 0xFF);
    /* data[2..8] zero for a read request. */
}

void robstride_build_active_report_frame(uint8_t id, uint8_t host_id,
                                         bool enabled,
                                         twai_message_t *out)
{
    if (out == NULL) {
        return;
    }
    memset(out, 0, sizeof(*out));
    out->identifier = build_ext_id(RS_COMM_ACTIVE_REPORT, host_id, id);
    out->extd = true;
    out->data_length_code = 8;
    out->data[0] = 1U;
    out->data[1] = 2U;
    out->data[2] = 3U;
    out->data[3] = 4U;
    out->data[4] = 5U;
    out->data[5] = 6U;
    out->data[6] = enabled ? 1U : 0U;
}

bool robstride_parse_param_reply(const twai_message_t *msg, uint8_t *id,
                                 uint16_t *param_id,
                                 uint8_t value[4])
{
    if (!msg || !msg->extd || msg->data_length_code < 8) {
        return false;
    }
    uint32_t comm_type = (msg->identifier >> 24) & 0x1F;
    if (comm_type != RS_COMM_READ_PARAM) {
        return false;
    }
    if (id) {
        *id = (uint8_t)((msg->identifier >> 8) & 0xFFU);
    }
    if (param_id) {
        *param_id = (uint16_t)(((uint16_t)msg->data[1] << 8) | (uint16_t)msg->data[0]);
    }
    if (value) {
        value[0] = msg->data[4];
        value[1] = msg->data[5];
        value[2] = msg->data[6];
        value[3] = msg->data[7];
    }
    return true;
}

void robstride_build_save_frame(uint8_t id, uint8_t host_id, twai_message_t *out)
{
    if (!out) return;
    memset(out, 0, sizeof(*out));
    out->identifier = build_ext_id(RS_COMM_SAVE, host_id, id);
    out->extd = true;
    out->data_length_code = 8;
    /* Payload 匹配 motorbridge save_parameters(): [1,2,3,4,5,6,7,8]。 */
    out->data[0] = 1;
    out->data[1] = 2;
    out->data[2] = 3;
    out->data[3] = 4;
    out->data[4] = 5;
    out->data[5] = 6;
    out->data[6] = 7;
    out->data[7] = 8;
}

uint8_t robstride_run_mode_for(motor_mode_t mode)
{
    switch (mode) {
    case MOTOR_MODE_MIT:       return RS_RUNMODE_MIT;
    /* Keep the generic motorbridge POS_VEL compatibility path on run_mode=1;
     * native CSP is exposed explicitly as MOTOR_MODE_POS_CSP. */
    case MOTOR_MODE_POS_VEL:   return RS_RUNMODE_PP;
    case MOTOR_MODE_VEL:       return RS_RUNMODE_VEL;
    case MOTOR_MODE_FORCE_POS: return RS_RUNMODE_PP;
    case MOTOR_MODE_POS_CSP:   return RS_RUNMODE_CSP;
    case MOTOR_MODE_POS_PP:    return RS_RUNMODE_PP;
    default:                   return 0xFF; /* DISABLED / unknown */
    }
}

/* ---- fault 上报(comm 21),移植自 motorbridge protocol.rs decode_fault_report ---- */

static uint32_t rd_le_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

bool robstride_parse_fault_report(const twai_message_t *msg,
                                  uint8_t *id,
                                  uint32_t *fault_raw,
                                  uint32_t *warning_raw)
{
    if (msg == NULL || !msg->extd || msg->data_length_code < 8) {
        return false;
    }
    uint32_t comm_type = (msg->identifier >> 24) & 0x1F;
    if (comm_type != RS_COMM_FAULT_REPORT) {
        return false;
    }
    /* For comm=21 the responder/host occupies the CAN-ID low byte; the
     * motor/device ID is extra_data low byte (bits 8..15 of the CAN ID). */
    uint16_t extra_data = (uint16_t)((msg->identifier >> 8) & 0xFFFFU);
    if (id) *id = (uint8_t)(extra_data & 0xFFU);
    if (fault_raw) *fault_raw = rd_le_u32(&msg->data[0]);
    if (warning_raw) *warning_raw = rd_le_u32(&msg->data[4]);
    return true;
}

const char *robstride_fault_bit_name(int bit)
{
    switch (bit) {
    case 0:  return "overtemperature";
    case 1:  return "driver_fault";
    case 2:  return "undervoltage";
    case 3:  return "overvoltage";
    case 4:  return "phase_b_overcurrent";
    case 5:  return "phase_c_overcurrent";
    case 7:  return "encoder_uncalibrated";
    case 8:  return "hardware_id_fault";
    case 9:  return "position_init_fault";
    case 14: return "stall_overload";
    case 16: return "phase_a_overcurrent";
    default: return NULL;
    }
}

/* ---- 固件版本请求/应答,移植自 motorbridge protocol.rs (5e54fe1) ---- */

void robstride_build_version_request_frame(uint8_t id, uint8_t host_id,
                                           twai_message_t *out)
{
    if (!out) return;
    memset(out, 0, sizeof(*out));
    /* 请求是 comm-4 (DISABLE) 帧,data[1] 带 marker 0xC4;电机回复版本帧而非禁用。 */
    out->identifier = build_ext_id(RS_COMM_DISABLE, host_id, id);
    out->extd = true;
    out->data_length_code = 8;
    out->data[0] = 0x00;
    out->data[1] = RS_VERSION_REQUEST_MARKER;
}

bool robstride_is_version_reply(const twai_message_t *msg)
{
    if (msg == NULL || !msg->extd || msg->data_length_code < 8) {
        return false;
    }
    uint32_t comm_type = (msg->identifier >> 24) & 0x1F;
    if (comm_type != RS_COMM_STATUS) {
        return false;
    }
    return msg->data[0] == RS_VERSION_REPLY_SIG_BYTE0 &&
           msg->data[1] == RS_VERSION_REPLY_SIG_BYTE1 &&
           msg->data[2] == RS_VERSION_REPLY_SIG_BYTE2;
}

bool robstride_decode_version_reply(const twai_message_t *msg,
                                    uint8_t *id,
                                    uint8_t version[4])
{
    if (!robstride_is_version_reply(msg)) {
        return false;
    }
    /* device_id 在 29 位 id 的 extra 字段低字节。 */
    uint16_t extra_data = (msg->identifier >> 8) & 0xFFFF;
    if (id) *id = (uint8_t)(extra_data & 0xFF);
    if (version) {
        version[0] = msg->data[3];
        version[1] = msg->data[4];
        version[2] = msg->data[5];
        version[3] = msg->data[6];
    }
    return true;
}

/* ---- 受控停机,移植自 motorbridge motor.rs controlled_stop (4690e3f)。
 * 用电机缓存的 mode + 位置/增益代替阻塞参数读(单 RX 任务模型)。发一帧零/保持。 ---- */

bool robstride_build_stop_frame(uint8_t id,
                                motor_mode_t mode,
                                const motor_state_t *m,
                                twai_message_t *out)
{
    if (!out || !m) {
        return false;
    }
    const uint8_t host_id = m->host_id;
    uint8_t raw[4];

    switch (mode) {
    case MOTOR_MODE_VEL:
        /* 速度模式:spd_ref (0x700A) = 0 -> 减速到零。 */
        robstride_encode_param_f32(0.0f, raw);
        robstride_build_param_write_frame(id, RS_PARAM_VEL_TARGET, raw, host_id, out);
        return true;
    case MOTOR_MODE_FORCE_POS:
        /* PP 模式:vel_max (0x7024) = 0 -> 零速度预算保持。 */
        robstride_encode_param_f32(0.0f, raw);
        robstride_build_param_write_frame(id, RS_PARAM_PP_VEL_MAX, raw, host_id, out);
        return true;
    case MOTOR_MODE_POS_VEL:
        /* CSP 模式:保持当前机械位置 (loc_ref 0x7016)。host 从 0x7019 读机械位置;
         * 此处用缓存反馈位置避免阻塞往返。 */
        robstride_encode_param_f32(m->position, raw);
        robstride_build_param_write_frame(id, RS_PARAM_POS_TARGET, raw, host_id, out);
        return true;
    case MOTOR_MODE_POS_CSP:
        /* Native CSP uses the same hold-position frame. */
        robstride_encode_param_f32(m->position, raw);
        robstride_build_param_write_frame(id, RS_PARAM_POS_TARGET, raw, host_id, out);
        return true;
    case MOTOR_MODE_POS_PP:
        /* Native PP stop is implemented by setting vel_max to zero. */
        robstride_encode_param_f32(0.0f, raw);
        robstride_build_param_write_frame(id, RS_PARAM_PP_VEL_MAX, raw, host_id, out);
        return true;
    case MOTOR_MODE_MIT: {
        /* MIT 模式:一帧 OPERATION_CONTROL,用缓存位置 + 缓存 kp/kd,零速度/力矩。
         * host 用 last_mit_gains;此处用电机 params.kp/kd。 */
        damiao_cmd_t hold = m->cmd;
        hold.pos = m->position;
        hold.vel = 0.0f;
        hold.tau = 0.0f;
        return robstride_build_control_frame(id, MOTOR_MODE_MIT, &hold, &m->params, out);
    }
    default:
        return false;
    }
}
