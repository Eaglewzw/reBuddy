/*
 * SPDX-FileCopyrightText: 2026 XIAO-C5-CAN
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Startup policy. Keep vendor model strings alive for the lifetime of the service. */
typedef struct {
    int control_period_ms;
    int min_tx_interval_ms;
    int offline_timeout_ms;
    int log_period_ms;
    int max_motors;
    float default_kp;
    float default_kd;
    uint8_t default_motor_id;
    uint8_t default_feedback_id;
    bool apply_default_id_map_at_boot;
    /** Minimum wall-clock spacing between per-motor admin commands within one
     *  range operation (software-level pacing; not a bus frame interval or
     *  device-processing guarantee). Range 0..60000 ms; the wait honors the
     *  configured value in full, quantized by the FreeRTOS tick. */
    int bulk_op_gap_ms;
    const char *robstride_default_model;
    const char *damiao_default_model;
    /* 0 keeps component defaults: service 6, CAN I/O 5, discovery 6. */
    uint8_t service_task_priority, can_task_priority, scan_task_priority;
} can_console_motor_config_t;

typedef struct {
    int tx_gpio;
    int rx_gpio;
    int controller_id;
    int bitrate;
    int data_bitrate; /* 0: classic CAN/no BRS. */
    int mode; /* can_console_can_mode_t; 0: normal. */
} can_console_motor_bus_config_t;

/** Classic CAN frame for optional application diagnostics and link probes. */
typedef struct {
    uint32_t id;
    uint8_t data_len;
    bool is_extended;
    bool is_remote;
    uint8_t data[8];
} can_console_motor_frame_t;

/** Called on the CAN RX task before the frame is queued for motor processing. */
typedef void (*can_console_motor_bus_observer_t)(
    int channel, const can_console_motor_frame_t *frame, void *context);

void can_console_motor_config_set_defaults(can_console_motor_config_t *config);
esp_err_t can_console_motor_init(const can_console_motor_config_t *config);

/** Create channels in controller order (0, then 1). */
esp_err_t can_console_motor_add_bus(const can_console_motor_bus_config_t *config,
                                    int *out_channel);

/** Optional observer must be installed before starting the channel. */
esp_err_t can_console_motor_set_bus_observer(
    int channel, can_console_motor_bus_observer_t observer, void *context);

/** Start one configured channel and the shared motor-service task. */
esp_err_t can_console_motor_start_bus(int channel);

/** Send a raw classic CAN frame for diagnostics; this bypasses motor command validation. */
esp_err_t can_console_motor_send_bus_frame(
    int channel, const can_console_motor_frame_t *frame);

/** Queue a control, vendor-maintenance and safety cycle. */
esp_err_t can_console_motor_tick(float dt_s);

#ifdef __cplusplus
}
#endif
