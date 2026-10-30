/*
 * SPDX-FileCopyrightText: 2026 XIAO-C5-CAN
 * SPDX-License-Identifier: Apache-2.0
 */

#include "can_console_motor_client.h"

#include <math.h>
#include <string.h>

#include "core/motor_manager.h"
#include "protocol/host_protocol.h"
#include "service/motor_service.h"
#include "transport/can_manager.h"
#include "vendors/motor_vendor.h"

static bool valid_motor_id(uint8_t motor_id)
{
    return motor_id <= (uint8_t)motor_manager_count();
}

static bool finite_target(const can_console_motor_target_t *target)
{
    if (target == NULL) {
        return false;
    }
    switch (target->mode) {
    case MOTOR_MODE_MIT:
        return isfinite(target->value.position) &&
               isfinite(target->value.velocity) &&
               isfinite(target->value.torque);
    case MOTOR_MODE_POS_VEL:
    case MOTOR_MODE_POS_CSP:
        return isfinite(target->value.position) &&
               isfinite(target->value.velocity_limit);
    case MOTOR_MODE_POS_PP:
        return isfinite(target->value.position) &&
               isfinite(target->value.velocity_limit) &&
               isfinite(target->value.acceleration);
    case MOTOR_MODE_VEL:
        return isfinite(target->value.velocity);
    case MOTOR_MODE_FORCE_POS:
        return isfinite(target->value.position) &&
               isfinite(target->value.velocity_limit) &&
               isfinite(target->value.force_ratio);
    default:
        return false;
    }
}

uint8_t can_console_motor_max_id(void)
{
    return (uint8_t)motor_manager_count();
}

esp_err_t can_console_motor_get_state(uint8_t motor_id,
                                      can_console_motor_state_t *state)
{
    if (state == NULL || !valid_motor_id(motor_id)) {
        return ESP_ERR_INVALID_ARG;
    }

    motor_state_t internal;
    if (!motor_manager_get_state(motor_id, &internal)) {
        return ESP_ERR_NOT_FOUND;
    }

    memset(state, 0, sizeof(*state));
    state->motor_id = internal.id;
    state->online = internal.online;
    state->enabled = internal.enabled;
    state->mode = internal.mode;
    state->mode_ready = internal.mode_ready;
    state->channel = internal.channel;
    if (internal.vendor != NULL && internal.vendor->name != NULL) {
        (void)strncpy(state->vendor_name, internal.vendor->name,
                      sizeof(state->vendor_name) - 1U);
    }

    state->position = internal.position;
    state->velocity = internal.speed;
    state->torque = internal.torque;
    state->mos_temperature = internal.t_mos;
    state->rotor_temperature = internal.t_rotor;
    state->status = internal.status;
    state->last_seen_us = internal.last_seen_us;
    state->last_feedback_us = internal.last_feedback_us;

    state->target.position = internal.cmd.pos;
    state->target.velocity = internal.cmd.vel;
    state->target.torque = internal.cmd.tau;
    state->target.velocity_limit = internal.cmd.vlim;
    state->target.force_ratio = internal.cmd.ratio;
    state->target.acceleration = internal.cmd.acc;
    state->kp = internal.params.kp;
    state->kd = internal.params.kd;
    state->position_min = internal.params.p_min;
    state->position_max = internal.params.p_max;
    state->velocity_min = internal.params.v_min;
    state->velocity_max = internal.params.v_max;
    state->torque_min = internal.params.t_min;
    state->torque_max = internal.params.t_max;

    state->host_id = internal.host_id;
    state->host_id_valid = internal.host_id_valid;
    state->fault_raw = internal.fault_raw;
    state->warning_raw = internal.warning_raw;
    state->fault_seen_us = internal.fault_seen_us;
    memcpy(state->firmware_version, internal.fw_version,
           sizeof(state->firmware_version));
    state->firmware_seen_us = internal.fw_seen_us;
    return ESP_OK;
}

esp_err_t can_console_motor_bind(uint8_t motor_id, const char *vendor,
                                const char *model, int channel)
{
    if (!valid_motor_id(motor_id) || vendor == NULL || vendor[0] == '\0' ||
        channel < 0 || channel >= CAN_MAX_CHANNELS) {
        return ESP_ERR_INVALID_ARG;
    }

    motor_state_t current;
    if (!motor_manager_get_state(motor_id, &current)) {
        return ESP_ERR_NOT_FOUND;
    }
    if (current.enabled) {
        return ESP_ERR_INVALID_STATE;
    }

    const motor_vendor_ops_t *ops = motor_vendor_get(vendor);
    if (ops == NULL) {
        return ESP_ERR_NOT_FOUND;
    }
    if (model != NULL) {
        float pmax = 0.0f;
        float vmax = 0.0f;
        float tmax = 0.0f;
        if (ops->resolve_model == NULL ||
            !ops->resolve_model(model, &pmax, &vmax, &tmax)) {
            return ESP_ERR_NOT_FOUND;
        }
    }

    if (!motor_manager_set_vendor(motor_id, vendor) ||
        (model != NULL && !motor_manager_set_model(motor_id, model)) ||
        !motor_manager_set_channel(motor_id, channel)) {
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t can_console_motor_park(uint8_t motor_id)
{
    if (!valid_motor_id(motor_id)) {
        return ESP_ERR_INVALID_ARG;
    }
    motor_state_t state;
    if (!motor_manager_get_state(motor_id, &state)) {
        return ESP_ERR_NOT_FOUND;
    }
    if (state.enabled) {
        return ESP_ERR_INVALID_STATE;
    }
    return motor_manager_set_channel(motor_id, -1) ? ESP_OK : ESP_FAIL;
}

esp_err_t can_console_motor_rebind_disabled(
    const can_console_motor_binding_t *bindings, size_t count)
{
    if (bindings == NULL || count == 0U || count > CAN_CONSOLE_MOTOR_MAX_TARGET_GROUP) {
        return ESP_ERR_INVALID_ARG;
    }
    for (size_t i = 0; i < count; ++i) {
        const can_console_motor_binding_t *b = &bindings[i];
        if (!valid_motor_id(b->motor_id) || b->vendor == NULL || b->model == NULL ||
            b->channel < 0 || b->channel >= CAN_MAX_CHANNELS ||
            !isfinite(b->kp) || !isfinite(b->kd) || b->kp < 0 || b->kd < 0) {
            return ESP_ERR_INVALID_ARG;
        }
        for (size_t j = 0; j < i; ++j) {
            if (bindings[j].motor_id == b->motor_id) {
                return ESP_ERR_INVALID_ARG;
            }
        }
        const motor_vendor_ops_t *ops = motor_vendor_get(b->vendor);
        float pmax, vmax, tmax, kp_max = 500.0f, kd_max = 5.0f;
        if (ops == NULL || ops->resolve_model == NULL ||
            !ops->resolve_model(b->model, &pmax, &vmax, &tmax)) {
            return ESP_ERR_NOT_FOUND;
        }
        if (ops->resolve_model_gains != NULL) {
            (void)ops->resolve_model_gains(b->model, &kp_max, &kd_max);
        }
        if (b->kp > kp_max || b->kd > kd_max) {
            return ESP_ERR_INVALID_ARG;
        }
    }
    return motor_service_rebind_disabled(bindings, count);
}

esp_err_t can_console_motor_set_gains(uint8_t motor_id, float kp, float kd)
{
    if (!valid_motor_id(motor_id) || !isfinite(kp) || !isfinite(kd) ||
        kp < 0.0f || kd < 0.0f) {
        return ESP_ERR_INVALID_ARG;
    }

    motor_state_t state;
    if (!motor_manager_get_state(motor_id, &state)) {
        return ESP_ERR_NOT_FOUND;
    }
    if (kp > state.params.kp_max || kd > state.params.kd_max) {
        return ESP_ERR_INVALID_ARG;
    }
    return motor_manager_set_gains(motor_id, kp, kd) ? ESP_OK : ESP_FAIL;
}

static esp_err_t prepare_target(const can_console_motor_target_t *target,
                                motor_service_mode_command_t *command, motor_state_t *snapshot)
{
    if (!finite_target(target) || !valid_motor_id(target->motor_id)) {
        return ESP_ERR_INVALID_ARG;
    }

    motor_state_t state;
    if (!motor_manager_get_state(target->motor_id, &state)) {
        return ESP_ERR_NOT_FOUND;
    }
    if (state.vendor == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (state.vendor->supports_mode != NULL &&
        !state.vendor->supports_mode(target->mode)) {
        return ESP_ERR_NOT_SUPPORTED;
    }
    if(snapshot!=NULL)*snapshot=state;
    *command = (motor_service_mode_command_t) {
        .motor_id = target->motor_id,
        .mode = target->mode,
        .command = {
            .pos = target->value.position,
            .vel = target->value.velocity,
            .tau = target->value.torque,
            .vlim = target->value.velocity_limit,
            .ratio = target->value.force_ratio,
            .acc = target->value.acceleration,
        },
    };
    return ESP_OK;
}

esp_err_t can_console_motor_set_target(
    const can_console_motor_target_t *target)
{
    motor_service_mode_command_t command;
    const esp_err_t error = prepare_target(target, &command, NULL);
    return error == ESP_OK
               ? motor_service_submit_mode(command.motor_id, command.mode,
                                           &command.command)
               : error;
}

esp_err_t can_console_motor_set_targets(
    const can_console_motor_target_t *targets, size_t count)
{
    if (targets == NULL || count == 0U ||
        count > CAN_CONSOLE_MOTOR_MAX_TARGET_GROUP) {
        return ESP_ERR_INVALID_ARG;
    }

    motor_service_mode_command_t commands[CAN_CONSOLE_MOTOR_MAX_TARGET_GROUP];
    for (size_t i = 0U; i < count; ++i) {
        const esp_err_t error = prepare_target(&targets[i], &commands[i], NULL);
        if (error != ESP_OK) {
            return error;
        }
    }
    return motor_service_submit_modes(commands, count);
}

esp_err_t can_console_motor_submit_reference(const can_console_motor_target_t *targets,
    size_t count, bool acquire, uint32_t *sequence)
{
    if (targets == NULL || sequence == NULL || count == 0 || count > CAN_CONSOLE_MOTOR_MAX_TARGET_GROUP)
        return ESP_ERR_INVALID_ARG;
    motor_service_mode_command_t commands[CAN_CONSOLE_MOTOR_MAX_TARGET_GROUP];
    int channel = -1;
    for (size_t i = 0; i < count; ++i) {
        /* The tracked delivery/confirmation reference path drives MIT or the
         * vendor position servo; a group must be uniform and match each
         * motor's active mode. */
        if ((targets[i].mode != MOTOR_MODE_MIT && targets[i].mode != MOTOR_MODE_POS_VEL) ||
            targets[i].mode != targets[0].mode) return ESP_ERR_INVALID_ARG;
        motor_state_t state;
        const esp_err_t error = prepare_target(&targets[i], &commands[i], &state);
        if (error != ESP_OK) return error;
        if (!state.online || !state.enabled || !state.mode_ready || state.mode != targets[i].mode ||
            (i != 0 && state.channel != channel)) return ESP_ERR_INVALID_STATE;
        channel = state.channel;
        for (size_t j = 0; j < i; ++j) if (targets[j].motor_id == targets[i].motor_id) return ESP_ERR_INVALID_ARG;
    }
    return motor_service_submit_reference(commands, count, acquire, false, sequence);
}

esp_err_t can_console_motor_release_reference(uint32_t *sequence)
{
    if (sequence == NULL) return ESP_ERR_INVALID_ARG;
    return motor_service_submit_reference(NULL, 0, false, true, sequence);
}

esp_err_t can_console_motor_get_reference_status(uint32_t sequence, can_console_motor_reference_status_t *status)
{
    return motor_service_get_reference(sequence, status);
}

static bool admin_opcode(can_console_motor_admin_action_t action,
                         host_admin_opcode_t *opcode)
{
    switch (action) {
    case CAN_CONSOLE_MOTOR_ADMIN_ENABLE:
        *opcode = HOST_OP_ENABLE;
        break;
    case CAN_CONSOLE_MOTOR_ADMIN_DISABLE:
        *opcode = HOST_OP_DISABLE;
        break;
    case CAN_CONSOLE_MOTOR_ADMIN_STOP:
        *opcode = HOST_OP_STOP;
        break;
    case CAN_CONSOLE_MOTOR_ADMIN_ESTOP:
        *opcode = HOST_OP_ESTOP;
        break;
    case CAN_CONSOLE_MOTOR_ADMIN_SET_ZERO:
        *opcode = HOST_OP_SET_ZERO;
        break;
    case CAN_CONSOLE_MOTOR_ADMIN_CLEAR_ERROR:
        *opcode = HOST_OP_CLEAR_ERROR;
        break;
    default:
        return false;
    }
    return true;
}

esp_err_t can_console_motor_submit_admin(
    const can_console_motor_admin_command_t *command)
{
    if (command == NULL || !valid_motor_id(command->motor_id)) {
        return ESP_ERR_INVALID_ARG;
    }

    host_admin_opcode_t opcode;
    if (!admin_opcode(command->action, &opcode)) {
        return ESP_ERR_INVALID_ARG;
    }

    const host_admin_cmd_t internal = {
        .op = opcode,
        .motor_id = command->motor_id,
    };
    return motor_service_submit_admin(&internal);
}

esp_err_t can_console_motor_submit_admin_range(
    const can_console_motor_admin_range_t *command)
{
    if (command == NULL || command->first_motor_id > command->last_motor_id ||
        !valid_motor_id(command->last_motor_id) ||
        command->action == CAN_CONSOLE_MOTOR_ADMIN_ESTOP) {
        return ESP_ERR_INVALID_ARG;
    }

    host_admin_opcode_t opcode;
    if (!admin_opcode(command->action, &opcode)) {
        return ESP_ERR_INVALID_ARG;
    }
    return motor_service_submit_admin_range(opcode, command->first_motor_id,
                                            command->last_motor_id);
}

esp_err_t can_console_motor_scan(
    const can_console_motor_scan_request_t *request)
{
    if (request == NULL || request->channel < 0 ||
        request->channel >= CAN_MAX_CHANNELS ||
        request->first_motor_id > request->last_motor_id ||
        !valid_motor_id(request->last_motor_id)) {
        return ESP_ERR_INVALID_ARG;
    }
    can_manager_trigger_scan(request->channel, request->first_motor_id,
                             request->last_motor_id);
    return ESP_OK;
}

esp_err_t can_console_motor_stop_scan(int channel)
{
    if (channel < 0 || channel >= CAN_MAX_CHANNELS) {
        return ESP_ERR_INVALID_ARG;
    }
    can_manager_stop_scan(channel);
    return ESP_OK;
}

esp_err_t can_console_motor_get_bus_state(
    int channel, int32_t recent_window_ms,
    can_console_motor_bus_state_t *state)
{
    if (state == NULL || channel < 0 || channel >= CAN_MAX_CHANNELS ||
        recent_window_ms <= 0) {
        return ESP_ERR_INVALID_ARG;
    }

    can_console_can_status_t internal = {0};
    const esp_err_t error = can_manager_get_status(channel, &internal);
    if (error != ESP_OK) {
        return error;
    }

    can_console_motor_bus_status_t status = CAN_CONSOLE_MOTOR_BUS_STOPPED;
    switch (internal.state) {
    case CAN_CONSOLE_CAN_STATE_ERROR_ACTIVE:
        status = CAN_CONSOLE_MOTOR_BUS_ACTIVE;
        break;
    case CAN_CONSOLE_CAN_STATE_ERROR_WARNING:
        status = CAN_CONSOLE_MOTOR_BUS_WARNING;
        break;
    case CAN_CONSOLE_CAN_STATE_ERROR_PASSIVE:
        status = CAN_CONSOLE_MOTOR_BUS_PASSIVE;
        break;
    case CAN_CONSOLE_CAN_STATE_BUS_OFF:
        status = CAN_CONSOLE_MOTOR_BUS_OFF;
        break;
    default:
        break;
    }

    *state = (can_console_motor_bus_state_t) {
        .started = internal.started,
        .recent_rx = can_manager_recent_rx(channel, recent_window_ms),
        .status = status,
        .tx_error_count = internal.tx_error_count,
        .rx_error_count = internal.rx_error_count,
        .bus_error_count = internal.bus_error_count,
        .tx_queued_count = internal.tx_queued_count,
        .tx_completed_count = internal.tx_completed_count,
        .tx_failed_count = internal.tx_failed_count,
        .rx_received_count = internal.rx_received_count,
        .rx_dropped_count = internal.rx_dropped_count,
    };
    return ESP_OK;
}

esp_err_t can_console_motor_fence_begin(int channel,uint32_t *token)
{return motor_service_fence_begin(channel,token);}
esp_err_t can_console_motor_fence_poll(uint32_t token,can_console_motor_fence_status_t *status)
{return motor_service_fence_poll(token,status);}
