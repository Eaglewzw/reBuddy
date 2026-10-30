#include "protocol/host_protocol.h"

#include <string.h>

bool host_protocol_parse_admin_cmd(const twai_message_t *msg, host_admin_cmd_t *out_cmd)
{
    if (msg == NULL || out_cmd == NULL || msg->identifier != MOTORBRIDGE_HOST_ADMIN_ID || msg->data_length_code < 2) {
        return false;
    }

    memset(out_cmd, 0, sizeof(*out_cmd));
    out_cmd->op = (host_admin_opcode_t)msg->data[0];
    out_cmd->motor_id = msg->data[1];

    switch (out_cmd->op) {
    case HOST_OP_SET_MODE:
        if (msg->data_length_code < 3) {
            return false;
        }
        out_cmd->mode = (motor_mode_t)msg->data[2];
        return true;
    case HOST_OP_SET_GAINS: {
        if (msg->data_length_code < 6) {
            return false;
        }
        uint16_t kp_u = (uint16_t)msg->data[2] | ((uint16_t)msg->data[3] << 8);
        uint16_t kd_u = (uint16_t)msg->data[4] | ((uint16_t)msg->data[5] << 8);
        out_cmd->kp = (float)kp_u / 100.0f;
        out_cmd->kd = (float)kd_u / 1000.0f;
        return true;
    }
    case HOST_OP_SET_IDS:
        if (msg->data_length_code < 5) {
            return false;
        }
        out_cmd->new_motor_id = msg->data[2];
        out_cmd->new_feedback_id = msg->data[3];
        out_cmd->store_after_set = (msg->data[4] & 0x01) != 0;
        out_cmd->verify_after_set = (msg->data[4] & 0x02) != 0;
        return true;
    case HOST_OP_ESTOP:
    case HOST_OP_SCAN:
    case HOST_OP_ENABLE:
    case HOST_OP_DISABLE:
    case HOST_OP_SET_ZERO:
    case HOST_OP_CLEAR_ERROR:
    case HOST_OP_STOP:
        return true;
    default:
        return false;
    }
}

bool host_protocol_parse_mode_frame(const twai_message_t *msg, uint8_t *id, motor_mode_t *mode, damiao_cmd_t *out_cmd)
{
    if (msg == NULL || id == NULL || mode == NULL || out_cmd == NULL || msg->data_length_code < 4) {
        return false;
    }

    memset(out_cmd, 0, sizeof(*out_cmd));

    if (msg->identifier >= MOTORBRIDGE_HOST_MIT_BASE_ID && msg->identifier < MOTORBRIDGE_HOST_MIT_BASE_ID + 0x40) {
        *id = (uint8_t)(msg->identifier - MOTORBRIDGE_HOST_MIT_BASE_ID);
        *mode = MOTOR_MODE_MIT;
        int16_t pos = (int16_t)((uint16_t)msg->data[0] | ((uint16_t)msg->data[1] << 8));
        int16_t vel = (int16_t)((uint16_t)msg->data[2] | ((uint16_t)msg->data[3] << 8));
        int16_t tau = (int16_t)((uint16_t)msg->data[4] | ((uint16_t)msg->data[5] << 8));
        out_cmd->pos = (float)pos / 1000.0f;
        out_cmd->vel = (float)vel / 100.0f;
        out_cmd->tau = (float)tau / 100.0f;
        return true;
    }

    if (msg->identifier >= MOTORBRIDGE_HOST_POS_VEL_BASE_ID && msg->identifier < MOTORBRIDGE_HOST_POS_VEL_BASE_ID + 0x40) {
        *id = (uint8_t)(msg->identifier - MOTORBRIDGE_HOST_POS_VEL_BASE_ID);
        *mode = MOTOR_MODE_POS_VEL;
        int16_t pos = (int16_t)((uint16_t)msg->data[0] | ((uint16_t)msg->data[1] << 8));
        int16_t vlim = (int16_t)((uint16_t)msg->data[2] | ((uint16_t)msg->data[3] << 8));
        out_cmd->pos = (float)pos / 1000.0f;
        out_cmd->vlim = (float)vlim / 100.0f;
        return true;
    }

    if (msg->identifier >= MOTORBRIDGE_HOST_VEL_BASE_ID && msg->identifier < MOTORBRIDGE_HOST_VEL_BASE_ID + 0x40) {
        *id = (uint8_t)(msg->identifier - MOTORBRIDGE_HOST_VEL_BASE_ID);
        *mode = MOTOR_MODE_VEL;
        int16_t vel = (int16_t)((uint16_t)msg->data[0] | ((uint16_t)msg->data[1] << 8));
        out_cmd->vel = (float)vel / 100.0f;
        return true;
    }

    if (msg->identifier >= MOTORBRIDGE_HOST_FORCE_POS_BASE_ID && msg->identifier < MOTORBRIDGE_HOST_FORCE_POS_BASE_ID + 0x40) {
        *id = (uint8_t)(msg->identifier - MOTORBRIDGE_HOST_FORCE_POS_BASE_ID);
        *mode = MOTOR_MODE_FORCE_POS;
        int16_t pos = (int16_t)((uint16_t)msg->data[0] | ((uint16_t)msg->data[1] << 8));
        int16_t vlim = (int16_t)((uint16_t)msg->data[2] | ((uint16_t)msg->data[3] << 8));
        uint16_t ratio_u = (uint16_t)msg->data[4] | ((uint16_t)msg->data[5] << 8);
        out_cmd->pos = (float)pos / 1000.0f;
        out_cmd->vlim = (float)vlim / 100.0f;
        out_cmd->ratio = (float)ratio_u / 10000.0f;
        return true;
    }

    return false;
}


