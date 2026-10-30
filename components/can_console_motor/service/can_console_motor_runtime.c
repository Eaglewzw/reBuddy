/*
 * SPDX-FileCopyrightText: 2026 XIAO-C5-CAN
 * SPDX-License-Identifier: Apache-2.0
 */

#include "can_console_motor_runtime.h"

#include <math.h>
#include "freertos/FreeRTOS.h"
#include <string.h>

#include "config/app_config.h"
#include "core/motor_manager.h"
#include "service/motor_service.h"
#include "transport/can_manager.h"

static bool s_initialized;

void can_console_motor_config_set_defaults(can_console_motor_config_t *config)
{
    if (config == NULL) {
        return;
    }
    const app_config_t *defaults = app_config_get_defaults();
    *config = (can_console_motor_config_t) {
        .control_period_ms = defaults->control_period_ms,
        .min_tx_interval_ms = defaults->min_tx_interval_ms,
        .offline_timeout_ms = defaults->offline_timeout_ms,
        .log_period_ms = defaults->log_period_ms,
        .max_motors = defaults->max_motors,
        .default_kp = defaults->default_kp,
        .default_kd = defaults->default_kd,
        .default_motor_id = defaults->default_motor_id,
        .default_feedback_id = defaults->default_feedback_id,
        .apply_default_id_map_at_boot = defaults->apply_default_id_map_at_boot,
        .bulk_op_gap_ms = defaults->bulk_op_gap_ms,
        .robstride_default_model = defaults->robstride_default_model,
        .damiao_default_model = defaults->damiao_default_model,
        .service_task_priority = defaults->service_task_priority,
        .can_task_priority = defaults->can_task_priority,
        .scan_task_priority = defaults->scan_task_priority,
    };
}

esp_err_t can_console_motor_init(const can_console_motor_config_t *config)
{
    if (config == NULL || config->control_period_ms <= 0 ||
        config->max_motors < MOTORBRIDGE_MIN_MOTOR_ID + 1 ||
        config->max_motors > MOTORBRIDGE_MAX_MOTOR_ID ||
        config->min_tx_interval_ms < 0 || config->offline_timeout_ms < 0 ||
        config->service_task_priority >= configMAX_PRIORITIES ||
        config->can_task_priority >= configMAX_PRIORITIES ||
        config->scan_task_priority >= configMAX_PRIORITIES ||
        config->bulk_op_gap_ms < 0 || config->bulk_op_gap_ms > 60000 ||
        !isfinite(config->default_kp) ||
        !isfinite(config->default_kd)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    const app_config_t internal = {
        .control_period_ms = config->control_period_ms,
        .min_tx_interval_ms = config->min_tx_interval_ms,
        .offline_timeout_ms = config->offline_timeout_ms,
        .log_period_ms = config->log_period_ms,
        .max_motors = config->max_motors,
        .default_kp = config->default_kp,
        .default_kd = config->default_kd,
        .default_motor_id = config->default_motor_id,
        .default_feedback_id = config->default_feedback_id,
        .apply_default_id_map_at_boot = config->apply_default_id_map_at_boot,
        .bulk_op_gap_ms = config->bulk_op_gap_ms,
        .robstride_default_model = config->robstride_default_model,
        .damiao_default_model = config->damiao_default_model,
        .service_task_priority = config->service_task_priority,
        .can_task_priority = config->can_task_priority,
        .scan_task_priority = config->scan_task_priority,
    };
    app_config_set(&internal);
    motor_manager_init(config->max_motors);
    s_initialized = true;
    return ESP_OK;
}

esp_err_t can_console_motor_add_bus(const can_console_motor_bus_config_t *config,
                                    int *out_channel)
{
    if (config == NULL || out_channel == NULL ||
        config->tx_gpio < 0 || config->rx_gpio < 0 ||
        config->controller_id < 0 || config->controller_id >= CAN_MAX_CHANNELS ||
        config->bitrate <= 0 || config->data_bitrate<0 || config->mode<0 || config->mode>2) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    const can_channel_t channel = can_manager_add_channel(
        config->tx_gpio, config->rx_gpio, config->controller_id,
        config->bitrate, config->data_bitrate, config->mode);
    if (channel < 0) {
        return ESP_FAIL;
    }
    *out_channel = channel;
    return ESP_OK;
}

esp_err_t can_console_motor_set_bus_observer(
    int channel, can_console_motor_bus_observer_t observer, void *context)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    can_console_can_status_t status;
    const esp_err_t error = can_manager_get_status(channel, &status);
    if (error != ESP_OK) {
        return error;
    }
    if (status.started) {
        return ESP_ERR_INVALID_STATE;
    }
    return motor_service_set_observer(channel, observer, context);
}

esp_err_t can_console_motor_start_bus(int channel)
{
    if (channel < 0 || channel >= CAN_MAX_CHANNELS) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    can_console_can_status_t status;
    const esp_err_t status_error = can_manager_get_status(channel, &status);
    if (status_error != ESP_OK) {
        return status_error;
    }
    if (status.started) {
        return ESP_ERR_INVALID_STATE;
    }
    esp_err_t error = motor_service_start();
    if (error != ESP_OK) {
        return error;
    }
    can_manager_set_rx_callback(channel, motor_service_handle_can_rx);
    return can_manager_start_channel(channel);
}

esp_err_t can_console_motor_send_bus_frame(
    int channel, const can_console_motor_frame_t *frame)
{
    if (frame == NULL || frame->data_len > 8U ||
        (frame->is_extended ? frame->id > 0x1FFFFFFFU : frame->id > 0x7FFU)) {
        return ESP_ERR_INVALID_ARG;
    }
    twai_message_t message = {
        .identifier = frame->id,
        .data_length_code = frame->data_len,
        .extd = frame->is_extended,
        .rtr = frame->is_remote,
    };
    memcpy(message.data, frame->data, frame->data_len);
    return can_manager_send(channel, &message, 1);
}

esp_err_t can_console_motor_tick(float dt_s)
{
    if (!isfinite(dt_s) || dt_s <= 0.0f) {
        return ESP_ERR_INVALID_ARG;
    }
    return motor_service_submit_tick(dt_s);
}
