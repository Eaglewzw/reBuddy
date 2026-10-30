/*
 * SPDX-FileCopyrightText: 2026 XIAO-C5-CAN
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CAN_CONSOLE_CAN_MAX_DATA_LEN            64U
#define CAN_CONSOLE_CAN_DEFAULT_TX_QUEUE_DEPTH  16U
#define CAN_CONSOLE_CAN_DEFAULT_RX_QUEUE_DEPTH  16U
#define CAN_CONSOLE_CAN_DEFAULT_TASK_STACK_SIZE 4096U
#define CAN_CONSOLE_CAN_DEFAULT_TASK_PRIORITY   5U
#define CAN_CONSOLE_CAN_WAIT_FOREVER            UINT32_MAX
#define CAN_CONSOLE_CAN_MASK_FILTER_COUNT       3U

/** Component handle. One handle represents one on-chip TWAI controller. */
typedef struct can_console_can_device can_console_can_device_t;
typedef can_console_can_device_t *can_console_can_handle_t;

/** Operating mode selected while creating a CAN device. */
typedef enum {
    CAN_CONSOLE_CAN_MODE_NORMAL = 0,       /**< Normal CAN/CAN FD node. */
    CAN_CONSOLE_CAN_MODE_LISTEN_ONLY,      /**< Receive only; this node neither transmits nor ACKs. */
    CAN_CONSOLE_CAN_MODE_LOOPBACK,         /**< Internal self-test loopback; transmission does not require a bus ACK. */
} can_console_can_mode_t;

/** Current CAN controller error state. */
typedef enum {
    CAN_CONSOLE_CAN_STATE_STOPPED = 0,
    CAN_CONSOLE_CAN_STATE_ERROR_ACTIVE,
    CAN_CONSOLE_CAN_STATE_ERROR_WARNING,
    CAN_CONSOLE_CAN_STATE_ERROR_PASSIVE,
    CAN_CONSOLE_CAN_STATE_BUS_OFF,
} can_console_can_state_t;

/** CAN/CAN FD frame exchanged with the application. */
typedef struct {
    uint32_t id;                                    /**< 11-bit or 29-bit arbitration ID. */
    uint8_t data[CAN_CONSOLE_CAN_MAX_DATA_LEN];
    uint8_t data_len;                               /**< Classic CAN: 0..8; CAN FD: valid FD lengths. */
    bool is_extended;                               /**< false: 11-bit ID, true: 29-bit ID. */
    bool is_remote;                                 /**< Classic CAN remote frame. Remote CAN FD frames are invalid. */
    bool is_fd;                                     /**< true: CAN FD frame. */
    bool bit_rate_switch;                           /**< CAN FD BRS flag; requires data_bitrate != 0. */
} can_console_can_frame_t;

/** Mask filter configuration. A mask bit of 1 means that the ID bit must match. */
typedef struct {
    bool enabled;
    uint32_t id;
    uint32_t mask;
    bool is_extended;
    bool no_classic;
    bool no_fd;
} can_console_can_mask_filter_t;

/** Range filter configuration. The range is inclusive when enabled. */
typedef struct {
    bool enabled;
    uint32_t range_low;
    uint32_t range_high;
    bool is_extended;
    bool no_classic;
    bool no_fd;
} can_console_can_range_filter_t;

/** Configuration needed to create one CAN device. */
typedef struct {
    gpio_num_t tx_gpio;
    gpio_num_t rx_gpio;
    uint32_t nominal_bitrate;                       /**< Arbitration bitrate; component default is 500000. */
    uint32_t data_bitrate;                          /**< CAN FD data bitrate; 0 disables BRS support. */
    can_console_can_mode_t mode;
    can_console_can_mask_filter_t mask_filters[CAN_CONSOLE_CAN_MASK_FILTER_COUNT];
    can_console_can_range_filter_t range_filter;
    uint16_t tx_queue_depth;                        /**< 0 selects CAN_CONSOLE_CAN_DEFAULT_TX_QUEUE_DEPTH. */
    uint16_t rx_queue_depth;                        /**< 0 selects CAN_CONSOLE_CAN_DEFAULT_RX_QUEUE_DEPTH. */
    uint32_t task_stack_size;                       /**< Bytes; 0 selects CAN_CONSOLE_CAN_DEFAULT_TASK_STACK_SIZE. */
    uint8_t task_priority;                          /**< 0 selects CAN_CONSOLE_CAN_DEFAULT_TASK_PRIORITY. */
    int8_t fail_retry_cnt;
} can_console_can_config_t;

/** Snapshot of driver state and component statistics. */
typedef struct {
    can_console_can_state_t state;
    bool started;
    uint16_t tx_error_count;
    uint16_t rx_error_count;
    uint32_t bus_error_count;
    uint32_t tx_queued_count;
    uint32_t tx_completed_count;
    uint32_t tx_failed_count;
    uint32_t rx_received_count;
    uint32_t rx_dropped_count;
} can_console_can_status_t;

/** TX task results for frames ahead of a FIFO fence; not device execution. */
typedef struct {
    int64_t last_tx_us; /**< Task-observed completion time of the last successful TX. */
    uint32_t failed_count;
    esp_err_t error;
} can_console_can_tx_fence_t;

/**
 * @brief Receive notification, invoked from the component RX task, never from a TWAI ISR.
 *
 * The frame pointer is valid only for the duration of this callback. Copy it when it must
 * be retained by the application.
 */
typedef void (*can_console_can_rx_callback_t)(can_console_can_handle_t handle,
                                              const can_console_can_frame_t *frame,
                                              void *user_ctx);

/**
 * @brief Fill a configuration structure with safe component defaults.
 *
 * Board defaults are CAN1 TX=GPIO5, RX=GPIO3, nominal bitrate=500 kbps.
 * Override them before passing the configuration to can_console_can_new() when needed.
 */
void can_console_can_config_set_defaults(can_console_can_config_t *config);

/** @brief Fill defaults for board CAN channel 0 or 1. */
void can_console_can_config_set_defaults_for_channel(can_console_can_config_t *config,
                                                     uint8_t channel);

/** @brief Allocate and configure one on-chip TWAI node. The node remains stopped after this call. */
esp_err_t can_console_can_new(const can_console_can_config_t *config,
                              can_console_can_handle_t *out_handle);

/** @brief Set or clear the receive callback. Set it before can_console_can_start() in normal use. */
esp_err_t can_console_can_set_rx_callback(can_console_can_handle_t handle,
                                          can_console_can_rx_callback_t callback,
                                          void *user_ctx);

/** @brief Change the operating mode. The node must be stopped. */
esp_err_t can_console_can_set_mode(can_console_can_handle_t handle,
                                   can_console_can_mode_t mode);

/** @brief Get the configured operating mode. */
esp_err_t can_console_can_get_mode(can_console_can_handle_t handle,
                                   can_console_can_mode_t *out_mode);

/** @brief Change nominal and CAN FD data bitrates. The node must be stopped. */
esp_err_t can_console_can_set_bitrate(can_console_can_handle_t handle,
                                      uint32_t nominal_bitrate,
                                      uint32_t data_bitrate);

/** @brief Get the configured nominal and CAN FD data bitrates. */
esp_err_t can_console_can_get_bitrate(can_console_can_handle_t handle,
                                      uint32_t *out_nominal_bitrate,
                                      uint32_t *out_data_bitrate);

/** @brief Configure one mask filter. The node must be stopped. */
esp_err_t can_console_can_set_mask_filter(
    can_console_can_handle_t handle,
    uint8_t filter_id,
    const can_console_can_mask_filter_t *filter);

/** @brief Configure the range filter. The node must be stopped. */
esp_err_t can_console_can_set_range_filter(
    can_console_can_handle_t handle,
    const can_console_can_range_filter_t *filter);

/** @brief Read one mask filter configuration. */
esp_err_t can_console_can_get_mask_filter(
    can_console_can_handle_t handle,
    uint8_t filter_id,
    can_console_can_mask_filter_t *out_filter);

/** @brief Read the range filter configuration. */
esp_err_t can_console_can_get_range_filter(
    can_console_can_handle_t handle,
    can_console_can_range_filter_t *out_filter);

/**
 * @brief Apply a complete configuration.
 *
 * If the node is running, this function stops and recreates it. Reconfiguration is
 * rejected while a frame is being transmitted or waiting in the software TX queue.
 */
esp_err_t can_console_can_reconfigure(can_console_can_handle_t handle,
                                      const can_console_can_config_t *config);

/**
 * @brief Run a local CAN self-test using the TWAI loopback mode.
 *
 * The device must be stopped. A fixed classic CAN frame is transmitted through
 * the internal loopback path and must be received by the component within
 * timeout_ms. The original configuration is restored before returning.
 */
esp_err_t can_console_can_self_test(can_console_can_handle_t handle,
                                    uint32_t timeout_ms);

/** @brief Enable the CAN node and start the component's RX/TX tasks. */
esp_err_t can_console_can_start(can_console_can_handle_t handle);

/** @brief Stop the component tasks and disable the CAN node. It may be started again afterwards. */
esp_err_t can_console_can_stop(can_console_can_handle_t handle);

/**
 * @brief Queue one CAN/CAN FD frame for transmission.
 *
 * A successful return means that the frame has been copied into the component TX queue;
 * it does not by itself mean that the frame has already appeared on the CAN bus.
 * Pass CAN_CONSOLE_CAN_WAIT_FOREVER to wait for queue space indefinitely.
 */
esp_err_t can_console_can_transmit(can_console_can_handle_t handle,
                                  const can_console_can_frame_t *frame,
                                  uint32_t timeout_ms);

/** @brief One fence at a time per device, owned by one client. No CAN frame is sent.
 *
 * Enqueues a FIFO marker after previously queued frames. Queue full or a fence
 * already pending returns ESP_ERR_TIMEOUT; receipt storage belongs to the device.
 */
esp_err_t can_console_can_tx_fence_begin(can_console_can_handle_t handle, uint32_t *token);

/** @brief Poll a fence. ESP_ERR_TIMEOUT means pending; success copies a completed receipt.
 *
 * Compare the receipt's failure count with the snapshot taken before the frames
 * being confirmed to detect TX failures between the two calls.
 */
esp_err_t can_console_can_tx_fence_poll(can_console_can_handle_t handle, uint32_t token,
                                       can_console_can_tx_fence_t *receipt);

/** @brief Begin the ISO 11898-1 bus-off recovery sequence. */
esp_err_t can_console_can_recover(can_console_can_handle_t handle);

/** @brief Obtain controller state and component counters. */
esp_err_t can_console_can_get_status(can_console_can_handle_t handle,
                                     can_console_can_status_t *out_status);

/** @brief Stop, release the TWAI controller, and free all component resources. */
esp_err_t can_console_can_delete(can_console_can_handle_t handle);

#ifdef __cplusplus
}
#endif
