/*
 * SPDX-FileCopyrightText: 2026 XIAO-C5-CAN
 * SPDX-License-Identifier: Apache-2.0
 */

#include <inttypes.h>
#include <stdlib.h>
#include <stdatomic.h>
#include <string.h>

#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "can_console_can.h"
#include "can_console_can_tx_sync.h"
#include "can_console_bsp.h"

#define CAN_CONSOLE_CAN_TWAI_TX_QUEUE_DEPTH 1U
#define CAN_CONSOLE_CAN_RECOVERY_TASK_STACK_SIZE 2048U
static const char *TAG = "can_console_can";

/* A queued transmission owns its payload until the controller finishes.
 * A fence marker (fence != 0) carries no frame; it completes in FIFO order. */
typedef struct {
    uint32_t seq;
    uint32_t fence;
    can_console_can_frame_t frame;
} can_console_can_tx_item_t;

struct can_console_can_device {
    twai_node_handle_t node;
    QueueHandle_t tx_queue;
    QueueHandle_t rx_queue;

    SemaphoreHandle_t bus_off_sem;
    SemaphoreHandle_t self_test_sem;
    TaskHandle_t tx_task;
    TaskHandle_t rx_task;
    TaskHandle_t recovery_task;
    can_console_can_rx_callback_t rx_callback;
    void *rx_user_ctx;
    can_console_can_config_t config;
    volatile bool running;
    bool started;
    atomic_uint tx_queued_count;
    atomic_uint tx_completed_count;
    atomic_uint tx_failed_count;
    volatile bool tx_done;
    volatile bool tx_success;
    atomic_uint rx_received_count;
    atomic_uint rx_dropped_count;

    atomic_uint next_tx_seq;     //下一个发送编号
    volatile uint32_t active_tx_seq;   //当前正在发送的编号
    volatile bool active_tx_valid;     //是否存在当前发送帧

    volatile bool bus_off;             //当前是否 Bus-Off
    volatile uint32_t bus_off_tx_seq;  //进入 Bus-Off 时正在处理的帧
    volatile uint32_t last_error_flags;//最近一次 TWAI 错误类型

    volatile bool self_test_active;
    volatile bool self_test_passed;
    can_console_can_frame_t self_test_frame;

    SemaphoreHandle_t fence_lock;
    uint32_t fence_queued;
    uint32_t fence_completed;
    can_console_can_tx_fence_t fence_result;
    int64_t last_tx_us;
};

static bool can_console_can_frame_valid(const can_console_can_frame_t *frame)
{
    if (frame == NULL || frame->data_len > CAN_CONSOLE_CAN_MAX_DATA_LEN) {
        return false;
    }
    if (frame->is_extended) {
        if (frame->id > 0x1FFFFFFFU) {
            return false;
        }
    } else if (frame->id > 0x7FFU) {
        return false;
    }
    if (!frame->is_fd) {
        return !frame->bit_rate_switch && frame->data_len <= 8U;
    }
    if (frame->is_remote) {
        return false;
    }
    return twaifd_dlc2len(twaifd_len2dlc(frame->data_len)) == frame->data_len;
}

static void can_console_can_normalize_config(can_console_can_config_t *config)
{
    if (config->tx_queue_depth == 0U) {
        config->tx_queue_depth = CAN_CONSOLE_CAN_DEFAULT_TX_QUEUE_DEPTH;
    }
    if (config->rx_queue_depth == 0U) {
        config->rx_queue_depth = CAN_CONSOLE_CAN_DEFAULT_RX_QUEUE_DEPTH;
    }
    if (config->task_stack_size == 0U) {
        config->task_stack_size = CAN_CONSOLE_CAN_DEFAULT_TASK_STACK_SIZE;
    }
    if (config->task_priority == 0U) {
        config->task_priority = CAN_CONSOLE_CAN_DEFAULT_TASK_PRIORITY;
    }
}

static bool can_console_can_config_valid(const can_console_can_config_t *config)
{
    if (config == NULL || config->tx_gpio == GPIO_NUM_NC ||
        config->rx_gpio == GPIO_NUM_NC || config->nominal_bitrate == 0U ||
        (int)config->mode < CAN_CONSOLE_CAN_MODE_NORMAL ||
        config->mode > CAN_CONSOLE_CAN_MODE_LOOPBACK ||
        config->fail_retry_cnt < -1 || config->fail_retry_cnt > 15) {
        return false;
    }

    for (uint8_t i = 0U; i < CAN_CONSOLE_CAN_MASK_FILTER_COUNT; ++i) {
        const can_console_can_mask_filter_t *filter = &config->mask_filters[i];
        if (!filter->enabled) {
            continue;
        }
        if (filter->no_classic && filter->no_fd) {
            return false;
        }
        if (filter->is_extended) {
            if ((filter->id | filter->mask) & ~TWAI_EXT_ID_MASK) {
                return false;
            }
        } else if ((filter->id | filter->mask) & ~TWAI_STD_ID_MASK) {
            return false;
        }
    }

    if (config->range_filter.enabled) {
        const can_console_can_range_filter_t *filter = &config->range_filter;
        if (filter->no_classic && filter->no_fd) {
            return false;
        }
        if (filter->range_low > filter->range_high) {
            return false;
        }
        if (filter->is_extended) {
            if (filter->range_high > TWAI_EXT_ID_MASK) {
                return false;
            }
        } else if (filter->range_high > TWAI_STD_ID_MASK) {
            return false;
        }
    }
    return true;
}

static void can_console_can_make_twai_frame(const can_console_can_frame_t *src, twai_frame_t *raw)
{
    memset(raw, 0, sizeof(*raw));
    raw->header.id = src->id;
    raw->header.dlc = src->is_fd ? twaifd_len2dlc(src->data_len) : src->data_len;
    raw->header.ide = src->is_extended;
    raw->header.rtr = src->is_remote;
    raw->header.fdf = src->is_fd;
    raw->header.brs = src->bit_rate_switch;
    raw->buffer = (uint8_t *)src->data;
    raw->buffer_len = src->data_len;
}

static can_console_can_state_t can_console_can_map_state(twai_error_state_t state)
{
    switch (state) {
    case TWAI_ERROR_ACTIVE:
        return CAN_CONSOLE_CAN_STATE_ERROR_ACTIVE;
    case TWAI_ERROR_WARNING:
        return CAN_CONSOLE_CAN_STATE_ERROR_WARNING;
    case TWAI_ERROR_PASSIVE:
        return CAN_CONSOLE_CAN_STATE_ERROR_PASSIVE;
    case TWAI_ERROR_BUS_OFF:
    default:
        return CAN_CONSOLE_CAN_STATE_BUS_OFF;
    }
}

static IRAM_ATTR bool can_console_can_on_error(twai_node_handle_t node,const twai_error_event_data_t *edata,void *user_ctx)
{
    (void)node;

    can_console_can_handle_t dev =(can_console_can_handle_t)user_ctx;
    dev->last_error_flags = edata->err_flags.val;

    return false;
}

static IRAM_ATTR bool can_console_can_on_state_change(twai_node_handle_t node,const twai_state_change_event_data_t *edata,void *user_ctx)
{
    (void)node;
    can_console_can_handle_t dev =(can_console_can_handle_t)user_ctx;

    /*bus-off enter*/
    if (edata->new_sta == TWAI_ERROR_BUS_OFF) {
        dev->bus_off = true;
        dev->bus_off_tx_seq = dev->active_tx_valid ? dev->active_tx_seq : 0;
        if (dev->bus_off_sem != NULL) {
            BaseType_t task_woken = pdFALSE;
            (void)xSemaphoreGiveFromISR(dev->bus_off_sem, &task_woken);
            return task_woken == pdTRUE;
        }
    } else if (edata->old_sta == TWAI_ERROR_BUS_OFF) {
        dev->bus_off = false;
    }

    return false;
}

static IRAM_ATTR bool can_console_can_on_tx_done(twai_node_handle_t node,const twai_tx_done_event_data_t *edata,void *user_ctx)
{
    (void)node;

    can_console_can_handle_t dev =(can_console_can_handle_t)user_ctx;

    dev->tx_done = true;
    dev->tx_success = edata->is_tx_success;
    BaseType_t task_woken = pdFALSE;
    if (dev->tx_task != NULL) vTaskNotifyGiveFromISR(dev->tx_task, &task_woken);
    return task_woken == pdTRUE;
}

static IRAM_ATTR bool can_console_can_on_rx_done(twai_node_handle_t node,
                                       const twai_rx_done_event_data_t *event,
                                       void *user_ctx)
{
    (void)event;
    can_console_can_handle_t dev = (can_console_can_handle_t)user_ctx;
    can_console_can_frame_t frame = {0};
    twai_frame_t raw = {
        .buffer = frame.data,
        .buffer_len = sizeof(frame.data),
    };

    if (twai_node_receive_from_isr(node, &raw) != ESP_OK) {
        return false;
    }

    frame.id = raw.header.id;
    frame.data_len = (uint8_t)twaifd_dlc2len(raw.header.dlc);
    frame.is_extended = raw.header.ide;
    frame.is_remote = raw.header.rtr;
    frame.is_fd = raw.header.fdf;
    frame.bit_rate_switch = raw.header.brs;

    BaseType_t task_woken = pdFALSE;
    if (xQueueSendFromISR(dev->rx_queue, &frame, &task_woken) == pdPASS) {
        dev->rx_received_count++;
    } else {
        dev->rx_dropped_count++;
    }
    return task_woken == pdTRUE;
}

static void can_console_can_rx_task(void *arg)
{
    can_console_can_handle_t dev = (can_console_can_handle_t)arg;
    can_console_can_frame_t frame;

    while (true) {
        if (xQueueReceive(dev->rx_queue, &frame, portMAX_DELAY) == pdPASS && dev->running) {
            /*self test mode*/
            if (dev->self_test_active && dev->self_test_sem != NULL &&
                frame.id == dev->self_test_frame.id &&
                frame.data_len == dev->self_test_frame.data_len &&
                frame.is_extended == dev->self_test_frame.is_extended &&
                frame.is_fd == dev->self_test_frame.is_fd &&
                frame.bit_rate_switch == dev->self_test_frame.bit_rate_switch &&
                memcmp(frame.data, dev->self_test_frame.data, frame.data_len) == 0) {

                /*self test success!*/
                dev->self_test_passed = true;
                (void)xSemaphoreGive(dev->self_test_sem);
                ESP_LOGI(TAG,"===self test ok!===");
            }
            if (!dev->self_test_active && dev->rx_callback != NULL) {
                dev->rx_callback(dev, &frame, dev->rx_user_ctx);
            }
        }
    }
}

static void can_console_can_tx_task(void *arg)
{
    can_console_can_handle_t dev = (can_console_can_handle_t)arg;
    can_console_can_tx_item_t item;

    while (true) {
        if (xQueueReceive(dev->tx_queue, &item, portMAX_DELAY) != pdPASS || !dev->running) {
            continue;
        }
        if (item.fence != 0U) {
            xSemaphoreTake(dev->fence_lock, portMAX_DELAY);
            dev->fence_result = (can_console_can_tx_fence_t) {
                .last_tx_us = dev->last_tx_us,
                .failed_count = atomic_load(&dev->tx_failed_count),
                .error = dev->bus_off ? ESP_ERR_INVALID_STATE : ESP_OK,
            };
            dev->fence_completed = item.fence;
            xSemaphoreGive(dev->fence_lock);
            continue;
        }
        can_console_can_frame_t *frame = &item.frame;
        dev->active_tx_seq = item.seq;
        dev->active_tx_valid = true;
        ESP_LOGD(TAG,"TX[%lu] DEQUEUED: id=0x%lX, len=%u, pending=%u",
                (unsigned long)item.seq,
                (unsigned long)frame->id,
                frame->data_len,
                (unsigned)uxQueueMessagesWaiting(dev->tx_queue));

        /* Consume any previous completion before submitting the next frame. */
        ulTaskNotifyTake(pdTRUE, 0);
        dev->tx_done = false;
        dev->tx_success = false;

        /* create raw data */
        twai_frame_t raw;
        can_console_can_make_twai_frame(frame, &raw);

        /* begin enter twai internal queue and send data to hardware */
        esp_err_t ret = twai_node_transmit(dev->node, &raw, 1000);
        if (ret != ESP_OK) {
            dev->tx_failed_count++;
            ESP_LOGW(TAG,"TX[%lu] SUBMIT_FAILED: id=0x%lX, error=%s",(unsigned long)item.seq,(unsigned long)frame->id,esp_err_to_name(ret));
            dev->active_tx_valid = false;
            dev->active_tx_seq = 0;
            continue;
        }
        ESP_LOGD(TAG,"TX[%lu] SUBMITTED: id=0x%lX",(unsigned long)item.seq,(unsigned long)frame->id);

        /* wait all data ,until transmit done */
        /* TWAI queues frame/buffer pointers. Do not reuse this task's item
         * while a timed-out or bus-off transfer still references it. Recovery
         * runs independently; stop disables the node before deleting tasks. */
        ret = can_console_can_wait_tx_idle(dev->node, &dev->running, &dev->tx_done);
        if (!dev->running) continue;
        if (ret != ESP_OK) {
            dev->tx_failed_count++;
            ESP_LOGW(TAG,"TX[%lu] WAIT_FAILED: id=0x%lX, error=%s",(unsigned long)item.seq,(unsigned long)frame->id,esp_err_to_name(ret));
        } else if (!dev->tx_success) {
            dev->tx_failed_count++;
            ESP_LOGW(TAG,"TX[%lu] FAILED_AFTER_RETRY: id=0x%lX, len=%u, error_flags=0x%lX",(unsigned long)item.seq,(unsigned long)frame->id,
                    frame->data_len,(unsigned long)dev->last_error_flags);
        } else {
            dev->tx_completed_count++;
            dev->last_tx_us = esp_timer_get_time();
            ESP_LOGD(TAG,"TX[%lu] SUCCESS: id=0x%lX",(unsigned long)item.seq,(unsigned long)frame->id);
        }

        if (dev->bus_off && dev->bus_off_tx_seq == item.seq) {
            ESP_LOGW(TAG,"TX[%lu] CAUSED_OR_REACHED_BUS_OFF",(unsigned long)item.seq);
        }
        dev->active_tx_valid = false;
        dev->active_tx_seq = 0;
    }
}

static void can_console_can_recovery_task(void *arg)
{
    can_console_can_handle_t dev = (can_console_can_handle_t)arg;

    while (true) {
        if (xSemaphoreTake(dev->bus_off_sem, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        if (!dev->running || !dev->bus_off || dev->node == NULL) {
            continue;
        }

        ESP_LOGW(TAG, "Bus-off detected, starting automatic recovery");
        esp_err_t ret = twai_node_recover(dev->node);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Automatic bus-off recovery failed: %s", esp_err_to_name(ret));
        } else {
            ESP_LOGI(TAG, "Automatic bus-off recovery requested");
        }
    }
}

static void can_console_can_drain_queue(QueueHandle_t queue)
{
    can_console_can_tx_item_t item;

    while (xQueueReceive(queue, &item, 0) == pdPASS) {
        ESP_LOGW(TAG,"TX[%lu] dropped while draining queue: id=0x%lX",(unsigned long)item.seq,(unsigned long)item.frame.id);
    }
}

static esp_err_t can_console_can_apply_filters(twai_node_handle_t node, const can_console_can_config_t *config)
{
    const uint8_t filter_count = CAN_CONSOLE_CAN_MASK_FILTER_COUNT < SOC_TWAI_MASK_FILTER_NUM ?
                                 CAN_CONSOLE_CAN_MASK_FILTER_COUNT : SOC_TWAI_MASK_FILTER_NUM;

    for (uint8_t i = 0U; i < filter_count; ++i) {
        const can_console_can_mask_filter_t *source = &config->mask_filters[i];
        const twai_mask_filter_config_t filter = {
            .id = source->enabled ? source->id : UINT32_MAX,
            .mask = source->enabled ? source->mask : UINT32_MAX,
            .is_ext = source->is_extended,
            .no_classic = source->no_classic,
            .no_fd = source->no_fd,
            .dual_filter = false,
        };

        esp_err_t ret = twai_node_config_mask_filter(node, i, &filter);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Mask filter %u setup failed: %s", i, esp_err_to_name(ret));
            return ret;
        }
        ESP_LOGI(TAG, "Mask filter %u %s", i, source->enabled ? "configured" : "disabled");
    }

#if SOC_TWAI_RANGE_FILTER_NUM
    const can_console_can_range_filter_t *source = &config->range_filter;
    const twai_range_filter_config_t range_filter = {
        .range_low = source->enabled ? source->range_low : 1U,
        .range_high = source->enabled ? source->range_high : 0U,
        .is_ext = source->is_extended,
        .no_classic = source->no_classic,
        .no_fd = source->no_fd,
    };
    esp_err_t ret = twai_node_config_range_filter(node, 0U, &range_filter);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Range filter setup failed: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "Range filter %s", source->enabled ? "configured" : "disabled");
#endif

    return ESP_OK;
}

static esp_err_t can_console_can_create_node(can_console_can_handle_t dev)
{
    /* create can node */
    twai_onchip_node_config_t node_cfg = {
        .io_cfg = {
            .tx = dev->config.tx_gpio,
            .rx = dev->config.rx_gpio,
            .quanta_clk_out = GPIO_NUM_NC,
            .bus_off_indicator = GPIO_NUM_NC,
        },
        .bit_timing = {
            .bitrate = dev->config.nominal_bitrate,
        },
        .data_timing = {
            .bitrate = dev->config.data_bitrate,
        },
        .fail_retry_cnt = dev->config.fail_retry_cnt,
        .intr_priority = 1, /* Keep CAN at the lowest interrupt priority. */
        .tx_queue_depth = CAN_CONSOLE_CAN_TWAI_TX_QUEUE_DEPTH,
        .flags = {
            .enable_self_test = dev->config.mode   == CAN_CONSOLE_CAN_MODE_LOOPBACK,
            .enable_loopback =  dev->config.mode   == CAN_CONSOLE_CAN_MODE_LOOPBACK,
            .enable_listen_only = dev->config.mode == CAN_CONSOLE_CAN_MODE_LISTEN_ONLY,
        },
    };

    twai_node_handle_t node = NULL;
    ESP_LOGI(TAG, "Creating TWAI node (TX=%d, RX=%d, nominal=%lu, data=%lu)",
             dev->config.tx_gpio, dev->config.rx_gpio, dev->config.nominal_bitrate,
             dev->config.data_bitrate);
    esp_err_t ret = twai_new_node_onchip(&node_cfg, &node);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "TWAI node creation failed: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "TWAI node created");

    ret = can_console_can_apply_filters(node, &dev->config);

    /* register callback */
    if (ret == ESP_OK) {
        const twai_event_callbacks_t callbacks = {
            .on_tx_done = can_console_can_on_tx_done, //tx done
            .on_rx_done = can_console_can_on_rx_done, //rx done
            .on_state_change = can_console_can_on_state_change,
            .on_error = can_console_can_on_error,
        };
        ret = twai_node_register_event_callbacks(node, &callbacks, dev);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "TWAI callback registration failed: %s", esp_err_to_name(ret));
        } else {
            ESP_LOGI(TAG, "TWAI RX callback registered");
        }
    }
    if (ret != ESP_OK) {
        (void)twai_node_delete(node);
        return ret;
    }

    dev->node = node;
    dev->bus_off = false;
    dev->bus_off_tx_seq = 0U;
    ESP_LOGI(TAG, "TWAI node configured and ready to start");
    return ESP_OK;
}

static esp_err_t can_console_can_replace_config(can_console_can_handle_t dev, const can_console_can_config_t *config)
{
    const can_console_can_config_t old_config = dev->config;

    if (dev->node != NULL) {
        esp_err_t ret = twai_node_delete(dev->node);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Cannot replace CAN node: delete failed: %s", esp_err_to_name(ret));
            return ret;
        }
        dev->node = NULL;
    }

    dev->config = *config;
    can_console_can_normalize_config(&dev->config);

    esp_err_t ret = can_console_can_create_node(dev);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "CAN configuration applied");
        return ESP_OK;
    }

    ESP_LOGE(TAG, "New CAN configuration failed: %s", esp_err_to_name(ret));
    dev->config = old_config;
    if (can_console_can_create_node(dev) != ESP_OK) {
        ESP_LOGE(TAG, "Previous CAN configuration could not be restored");
    } else {
        ESP_LOGI(TAG, "Previous CAN configuration restored");
    }
    return ret;
}

void can_console_can_config_set_defaults(can_console_can_config_t *config)
{
    can_console_can_config_set_defaults_for_channel(config, 0U);
}

void can_console_can_config_set_defaults_for_channel(can_console_can_config_t *config,
                                                     uint8_t channel)
{
    if (config == NULL) {
        ESP_LOGE(TAG, "Cannot set defaults: config is NULL");
        return;
    }
    const can_console_bsp_can_pins_t *pins = can_console_bsp_get_can_pins(channel);
    *config = (can_console_can_config_t) {
        .tx_gpio = pins != NULL ? pins->tx_gpio : GPIO_NUM_NC,
        .rx_gpio = pins != NULL ? pins->rx_gpio : GPIO_NUM_NC,
        .nominal_bitrate = 500000,
        .data_bitrate = 0,
        .mode = CAN_CONSOLE_CAN_MODE_NORMAL,
        .mask_filters = {
            {
                .enabled = true,
                .id = 0U,
                .mask = 0U,
                .is_extended = false,
            },
            {
                .enabled = true,
                .id = 0U,
                .mask = 0U,
                .is_extended = true,
            },
        },
        .range_filter = {
            .enabled = false,
        },
        .tx_queue_depth = CAN_CONSOLE_CAN_DEFAULT_TX_QUEUE_DEPTH,
        .rx_queue_depth = CAN_CONSOLE_CAN_DEFAULT_RX_QUEUE_DEPTH,
        .task_stack_size = CAN_CONSOLE_CAN_DEFAULT_TASK_STACK_SIZE,
        .task_priority = CAN_CONSOLE_CAN_DEFAULT_TASK_PRIORITY,
        .fail_retry_cnt = 3,
    };
    ESP_LOGI(TAG, "Default CAN configuration loaded");
}

esp_err_t can_console_can_new(const can_console_can_config_t *config,
                              can_console_can_handle_t *out_handle)
{
    if (out_handle == NULL || !can_console_can_config_valid(config)) {
        ESP_LOGE(TAG, "CAN creation rejected: invalid configuration");
        return ESP_ERR_INVALID_ARG;
    }
    *out_handle = NULL;

    /* ISR callbacks access this object while NVS/Flash may disable the cache. */
    can_console_can_handle_t dev = heap_caps_calloc(1, sizeof(*dev), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (dev == NULL) {
        ESP_LOGE(TAG, "CAN creation failed: out of memory");
        return ESP_ERR_NO_MEM;
    }

    /* init can task & queue & semaphore*/
    dev->config = *config;
    can_console_can_normalize_config(&dev->config);

    dev->tx_queue = xQueueCreate(dev->config.tx_queue_depth, sizeof(can_console_can_tx_item_t));
    dev->rx_queue = xQueueCreate(dev->config.rx_queue_depth, sizeof(can_console_can_frame_t));

    dev->bus_off_sem   = xSemaphoreCreateBinary();
    dev->self_test_sem = xSemaphoreCreateBinary();
    dev->fence_lock    = xSemaphoreCreateMutex();

    if (dev->tx_queue == NULL || dev->rx_queue == NULL ||
        dev->bus_off_sem == NULL || dev->self_test_sem == NULL ||
        dev->fence_lock == NULL) {
        ESP_LOGE(TAG, "CAN queue/semaphore creation failed");
        can_console_can_delete(dev);
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "CAN queues created (TX=%u, RX=%u)", dev->config.tx_queue_depth,
             dev->config.rx_queue_depth);

    /* create can node*/
    esp_err_t ret = can_console_can_create_node(dev);
    if (ret != ESP_OK) {
        can_console_can_delete(dev);
        return ret;
    }

    *out_handle = dev;
    ESP_LOGI(TAG, "CAN device created");
    return ESP_OK;
}

esp_err_t can_console_can_set_rx_callback(can_console_can_handle_t dev,
                                          can_console_can_rx_callback_t callback,
                                          void *user_ctx)
{
    if (dev == NULL) {
        ESP_LOGE(TAG, "Cannot set RX callback: handle is NULL");
        return ESP_ERR_INVALID_ARG;
    }
    if (dev->started) {
        ESP_LOGE(TAG, "Cannot set RX callback while CAN is running");
        return ESP_ERR_INVALID_STATE;
    }
    dev->rx_callback = callback;
    dev->rx_user_ctx = user_ctx;
    ESP_LOGI(TAG, "RX callback %s", callback != NULL ? "installed" : "cleared");
    return ESP_OK;
}

esp_err_t can_console_can_set_mode(can_console_can_handle_t dev,
                                   can_console_can_mode_t mode)
{
    if (dev == NULL || (int)mode < CAN_CONSOLE_CAN_MODE_NORMAL || mode > CAN_CONSOLE_CAN_MODE_LOOPBACK) {
        ESP_LOGE(TAG, "Mode update rejected: invalid argument");
        return ESP_ERR_INVALID_ARG;
    }
    if (dev->started) {
        ESP_LOGE(TAG, "Mode update rejected: CAN is running");
        return ESP_ERR_INVALID_STATE;
    }
    if (dev->config.mode == mode) {
        ESP_LOGI(TAG, "CAN mode unchanged");
        return ESP_OK;
    }

    can_console_can_config_t config = dev->config;
    config.mode = mode;
    ESP_LOGI(TAG, "Updating CAN mode to %d", (int)mode);
    return can_console_can_replace_config(dev, &config);
}

esp_err_t can_console_can_get_mode(can_console_can_handle_t dev,
                                   can_console_can_mode_t *out_mode)
{
    if (dev == NULL || out_mode == NULL) {
        ESP_LOGE(TAG, "Mode query rejected: invalid argument");
        return ESP_ERR_INVALID_ARG;
    }
    *out_mode = dev->config.mode;
    return ESP_OK;
}

esp_err_t can_console_can_set_bitrate(can_console_can_handle_t dev,
                                      uint32_t nominal_bitrate,
                                      uint32_t data_bitrate)
{
    if (dev == NULL || nominal_bitrate == 0U) {
        ESP_LOGE(TAG, "Bitrate update rejected: invalid argument");
        return ESP_ERR_INVALID_ARG;
    }
    if (dev->started) {
        ESP_LOGE(TAG, "Bitrate update rejected: CAN is running");
        return ESP_ERR_INVALID_STATE;
    }

    can_console_can_config_t config = dev->config;
    config.nominal_bitrate = nominal_bitrate;
    config.data_bitrate = data_bitrate;
    if (!can_console_can_config_valid(&config)) {
        ESP_LOGE(TAG, "Bitrate update rejected: configuration is invalid");
        return ESP_ERR_INVALID_ARG;
    }
    ESP_LOGI(TAG, "Updating CAN bitrate: nominal=%" PRIu32 ", data=%" PRIu32,
             nominal_bitrate, data_bitrate);
    return can_console_can_replace_config(dev, &config);
}

esp_err_t can_console_can_get_bitrate(can_console_can_handle_t dev,
                                      uint32_t *out_nominal_bitrate,
                                      uint32_t *out_data_bitrate)
{
    if (dev == NULL || out_nominal_bitrate == NULL || out_data_bitrate == NULL) {
        ESP_LOGE(TAG, "Bitrate query rejected: invalid argument");
        return ESP_ERR_INVALID_ARG;
    }
    *out_nominal_bitrate = dev->config.nominal_bitrate;
    *out_data_bitrate = dev->config.data_bitrate;
    return ESP_OK;
}

esp_err_t can_console_can_set_mask_filter(
    can_console_can_handle_t dev, uint8_t filter_id,
    const can_console_can_mask_filter_t *filter)
{
    if (dev == NULL || filter == NULL || filter_id >= CAN_CONSOLE_CAN_MASK_FILTER_COUNT) {
        ESP_LOGE(TAG, "Mask filter update rejected: invalid argument");
        return ESP_ERR_INVALID_ARG;
    }
    if (dev->started) {
        ESP_LOGE(TAG, "Mask filter update rejected: CAN is running");
        return ESP_ERR_INVALID_STATE;
    }

    can_console_can_config_t config = dev->config;
    config.mask_filters[filter_id] = *filter;
    if (!can_console_can_config_valid(&config)) {
        ESP_LOGE(TAG, "Mask filter update rejected: configuration is invalid");
        return ESP_ERR_INVALID_ARG;
    }
    ESP_LOGI(TAG, "Updating mask filter %u", filter_id);
    return can_console_can_replace_config(dev, &config);
}

esp_err_t can_console_can_set_range_filter(
    can_console_can_handle_t dev, const can_console_can_range_filter_t *filter)
{
    if (dev == NULL || filter == NULL) {
        ESP_LOGE(TAG, "Range filter update rejected: invalid argument");
        return ESP_ERR_INVALID_ARG;
    }
    if (dev->started) {
        ESP_LOGE(TAG, "Range filter update rejected: CAN is running");
        return ESP_ERR_INVALID_STATE;
    }

    can_console_can_config_t config = dev->config;
    config.range_filter = *filter;
    if (!can_console_can_config_valid(&config)) {
        ESP_LOGE(TAG, "Range filter update rejected: configuration is invalid");
        return ESP_ERR_INVALID_ARG;
    }
    ESP_LOGI(TAG, "Updating range filter: %s", filter->enabled ? "enabled" : "disabled");
    return can_console_can_replace_config(dev, &config);
}

esp_err_t can_console_can_get_mask_filter(
    can_console_can_handle_t dev, uint8_t filter_id,
    can_console_can_mask_filter_t *out_filter)
{
    if (dev == NULL || out_filter == NULL || filter_id >= CAN_CONSOLE_CAN_MASK_FILTER_COUNT) {
        ESP_LOGE(TAG, "Mask filter query rejected: invalid argument");
        return ESP_ERR_INVALID_ARG;
    }
    *out_filter = dev->config.mask_filters[filter_id];
    return ESP_OK;
}

esp_err_t can_console_can_get_range_filter(
    can_console_can_handle_t dev, can_console_can_range_filter_t *out_filter)
{
    if (dev == NULL || out_filter == NULL) {
        ESP_LOGE(TAG, "Range filter query rejected: invalid argument");
        return ESP_ERR_INVALID_ARG;
    }
    *out_filter = dev->config.range_filter;
    return ESP_OK;
}

esp_err_t can_console_can_reconfigure(can_console_can_handle_t dev,
                                      const can_console_can_config_t *config)
{
    if (dev == NULL || !can_console_can_config_valid(config)) {
        ESP_LOGE(TAG, "CAN reconfiguration rejected: invalid configuration");
        return ESP_ERR_INVALID_ARG;
    }

    can_console_can_config_t normalized = *config;
    can_console_can_normalize_config(&normalized);

    if (dev->started && (dev->active_tx_valid ||
                         uxQueueMessagesWaiting(dev->tx_queue) != 0U)) {
        ESP_LOGW(TAG, "CAN reconfiguration rejected: TX activity is pending");
        return ESP_ERR_INVALID_STATE;
    }

    const bool restart = dev->started;
    if (restart) {
        esp_err_t ret = can_console_can_stop(dev);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "CAN reconfiguration could not stop node: %s", esp_err_to_name(ret));
            return ret;
        }
    }

    esp_err_t ret = can_console_can_replace_config(dev, &normalized);
    if (ret != ESP_OK) {
        if (restart && dev->node != NULL) {
            esp_err_t restart_ret = can_console_can_start(dev);
            if (restart_ret != ESP_OK) {
                ESP_LOGE(TAG, "Previous CAN configuration restart failed: %s",
                         esp_err_to_name(restart_ret));
            }
        }
        return ret;
    }

    if (restart) {
        ret = can_console_can_start(dev);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "CAN reconfiguration applied but restart failed: %s",
                     esp_err_to_name(ret));
            return ret;
        }
    }
    ESP_LOGI(TAG, "CAN reconfiguration complete");
    return ESP_OK;
}

esp_err_t can_console_can_self_test(can_console_can_handle_t dev, uint32_t timeout_ms)
{
    if (dev == NULL) {
        ESP_LOGE(TAG, "CAN self-test rejected: handle is NULL");
        return ESP_ERR_INVALID_ARG;
    }
    if (dev->started) {
        ESP_LOGE(TAG, "CAN self-test rejected: CAN must be stopped");
        return ESP_ERR_INVALID_STATE;
    }

    const can_console_can_config_t old_config = dev->config;
    can_console_can_config_t test_config = old_config;
    test_config.mode = CAN_CONSOLE_CAN_MODE_LOOPBACK;
    test_config.mask_filters[0] = (can_console_can_mask_filter_t) {
        .enabled = true,
        .id = 0x5A5U,
        .mask = TWAI_STD_ID_MASK,
        .is_extended = false,
        .no_classic = false,
        .no_fd = true,
    };
    for (uint8_t i = 1U; i < CAN_CONSOLE_CAN_MASK_FILTER_COUNT; ++i) {
        test_config.mask_filters[i] = (can_console_can_mask_filter_t) {
            .enabled = false,
        };
    }
    test_config.range_filter.enabled = false;

    const can_console_can_frame_t test_frame = {
        .id = 0x5A5U,
        .data = {0xC5U, 0xA5U, 0x5AU, 0x01U, 0x02U, 0x03U, 0x04U, 0x05U},
        .data_len = 8U,
        .is_extended = false,
        .is_remote = false,
        .is_fd = false,
        .bit_rate_switch = false,
    };

    while (xSemaphoreTake(dev->self_test_sem, 0U) == pdTRUE) {
    }

    ESP_LOGI(TAG, "Starting CAN loopback self-test");
    esp_err_t ret = can_console_can_replace_config(dev, &test_config);
    if (ret != ESP_OK) {
        return ret;
    }

    dev->self_test_frame = test_frame;
    dev->self_test_passed = false;
    dev->self_test_active = true;

    ret = can_console_can_start(dev);
    if (ret == ESP_OK) {
        ret = can_console_can_transmit(dev, &test_frame, timeout_ms);
    }
    if (ret == ESP_OK) {
        const TickType_t wait = timeout_ms == CAN_CONSOLE_CAN_WAIT_FOREVER
                                    ? portMAX_DELAY
                                    : pdMS_TO_TICKS(timeout_ms);
        if (xSemaphoreTake(dev->self_test_sem, wait) != pdTRUE || !dev->self_test_passed) {
            ret = ESP_ERR_TIMEOUT;
            ESP_LOGE(TAG, "CAN loopback self-test timed out");
        }
    }

    dev->self_test_active = false;
    esp_err_t stop_ret = ESP_OK;
    if (dev->started) {
        stop_ret = can_console_can_stop(dev);
    }

    esp_err_t restore_ret = can_console_can_replace_config(dev, &old_config);
    if (restore_ret != ESP_OK) {
        ESP_LOGE(TAG, "CAN self-test configuration restore failed: %s",
                 esp_err_to_name(restore_ret));
        return restore_ret;
    }
    if (ret == ESP_OK && stop_ret != ESP_OK) {
        ret = stop_ret;
    }
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "CAN loopback self-test passed");
    } else {
        ESP_LOGE(TAG, "CAN loopback self-test failed: %s", esp_err_to_name(ret));
    }
    return ret;
}

esp_err_t can_console_can_start(can_console_can_handle_t dev)
{
    if (dev == NULL) {
        ESP_LOGE(TAG, "Cannot start CAN: handle is NULL");
        return ESP_ERR_INVALID_ARG;
    }
    if (dev->started) {
        ESP_LOGE(TAG, "Cannot start CAN: already started");
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret = ESP_OK;
    if (dev->node == NULL) {
        ret = can_console_can_create_node(dev);
        if (ret != ESP_OK) {
            return ret;
        }
    }
    /*begin can componentx*/
    dev->running = true;
    if (xTaskCreate(can_console_can_rx_task, "can_rx", dev->config.task_stack_size, dev,
                    dev->config.task_priority, &dev->rx_task) != pdPASS ||
        xTaskCreate(can_console_can_tx_task, "can_tx", dev->config.task_stack_size, dev,
                    dev->config.task_priority, &dev->tx_task) != pdPASS ||
        xTaskCreate(can_console_can_recovery_task, "can_recover",
                    CAN_CONSOLE_CAN_RECOVERY_TASK_STACK_SIZE, dev,
                    dev->config.task_priority, &dev->recovery_task) != pdPASS) {

        ESP_LOGE(TAG, "CAN task creation failed");
        dev->running = false;
        if (dev->rx_task != NULL) {
            vTaskDelete(dev->rx_task);
            dev->rx_task = NULL;
        }
        if (dev->tx_task != NULL) {
            vTaskDelete(dev->tx_task);
            dev->tx_task = NULL;
        }
        if (dev->recovery_task != NULL) {
            vTaskDelete(dev->recovery_task);
            dev->recovery_task = NULL;
        }
        return ESP_ERR_NO_MEM;

    }
    ESP_LOGI(TAG, "CAN RX/TX & buff-off tasks created!");

    ret = twai_node_enable(dev->node);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "TWAI node enable failed: %s", esp_err_to_name(ret));
        dev->running = false;
        vTaskDelete(dev->rx_task);
        vTaskDelete(dev->tx_task);
        dev->rx_task = NULL;
        dev->tx_task = NULL;
        if (dev->recovery_task != NULL) {
            vTaskDelete(dev->recovery_task);
            dev->recovery_task = NULL;
        }
        return ret;
    }

    dev->started = true;
    ESP_LOGI(TAG, "CAN started");
    return ESP_OK;
}

esp_err_t can_console_can_stop(can_console_can_handle_t dev)
{
    if (dev == NULL) {
        ESP_LOGE(TAG, "Cannot stop CAN: handle is NULL");
        return ESP_ERR_INVALID_ARG;
    }
    if (!dev->started) {
        ESP_LOGE(TAG, "Cannot stop CAN: not started");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Stopping CAN");
    dev->running = false;
    if (dev->bus_off_sem != NULL) {
        (void)xSemaphoreGive(dev->bus_off_sem);
    }
    /* Hold the fence lock across task deletion and queue draining so a
     * pending fence cannot observe a half-stopped TX task. */
    xSemaphoreTake(dev->fence_lock, portMAX_DELAY);
    twai_node_status_t status;
    esp_err_t ret = twai_node_get_info(dev->node, &status, NULL);
    if (ret == ESP_OK && status.state != TWAI_ERROR_BUS_OFF) {
        ret = twai_node_disable(dev->node);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "TWAI node disable failed: %s", esp_err_to_name(ret));
        }
    } else if (ret != ESP_OK) {
        ESP_LOGE(TAG, "TWAI node status read failed: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGW(TAG, "TWAI node is bus-off; skipping disable before deletion");
    }
    if (dev->rx_task != NULL) {
        vTaskDelete(dev->rx_task);
        dev->rx_task = NULL;
    }
    if (dev->tx_task != NULL) {
        vTaskDelete(dev->tx_task);
        dev->tx_task = NULL;
    }
    if (dev->recovery_task != NULL) {
        vTaskDelete(dev->recovery_task);
        dev->recovery_task = NULL;
    }
    if (ret == ESP_OK) {
        ret = twai_node_delete(dev->node);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "TWAI node deletion failed: %s", esp_err_to_name(ret));
        }
    }
    if (ret == ESP_OK) {
        dev->node = NULL;
    }
    can_console_can_drain_queue(dev->tx_queue);
    xQueueReset(dev->rx_queue);
    /* Any fence still pending completes with an error; no frames follow a stop. */
    dev->fence_result.error = ESP_ERR_INVALID_STATE;
    dev->fence_completed = dev->fence_queued;
    xSemaphoreGive(dev->fence_lock);
    dev->started = false;
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "CAN stopped");
    }
    return ret;
}

esp_err_t can_console_can_transmit(can_console_can_handle_t dev,
                                   const can_console_can_frame_t *frame,
                                   uint32_t timeout_ms)
{
    if (dev == NULL || !can_console_can_frame_valid(frame)) {
        ESP_LOGE(TAG, "TX rejected: invalid handle or frame");
        return ESP_ERR_INVALID_ARG;
    }
    if (!dev->started || !dev->running) {
        ESP_LOGE(TAG, "TX rejected: CAN is not running");
        return ESP_ERR_INVALID_STATE;
    }
    if (dev->config.mode == CAN_CONSOLE_CAN_MODE_LISTEN_ONLY) {
        ESP_LOGE(TAG, "TX rejected: listen-only mode");
        return ESP_ERR_NOT_SUPPORTED;
    }
    if (frame->bit_rate_switch && dev->config.data_bitrate == 0) {
        ESP_LOGE(TAG, "TX rejected: BRS frame requires a data bitrate");
        return ESP_ERR_INVALID_ARG;
    }

    if (dev->bus_off) {
        ESP_LOGW(TAG,"TX rejected: BUS_OFF, id=0x%lX",(unsigned long)frame->id);return ESP_ERR_INVALID_STATE;
    }

    can_console_can_tx_item_t item = {
        .seq = ++dev->next_tx_seq,
        .frame = *frame,
    };

    TickType_t wait = (timeout_ms == CAN_CONSOLE_CAN_WAIT_FOREVER? portMAX_DELAY: pdMS_TO_TICKS(timeout_ms));

    if (xQueueSend(dev->tx_queue, &item, wait) != pdPASS) {
        ESP_LOGD(TAG,"TX[%lu] QUEUE_TIMEOUT: id=0x%lX, pending=%u/%u",(unsigned long)item.seq,(unsigned long)item.frame.id,
                                        (unsigned)uxQueueMessagesWaiting(dev->tx_queue),(unsigned)dev->config.tx_queue_depth);
        return ESP_ERR_TIMEOUT;
    }
    dev->tx_queued_count++;

    ESP_LOGD(TAG,
            "TX[%lu] QUEUED: id=0x%lX, pending=%u/%u",
            (unsigned long)item.seq,
            (unsigned long)item.frame.id,
            (unsigned)uxQueueMessagesWaiting(dev->tx_queue),
            (unsigned)dev->config.tx_queue_depth);

    return ESP_OK;
}

esp_err_t can_console_can_tx_fence_begin(can_console_can_handle_t dev, uint32_t *token)
{
    if (dev == NULL || token == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!dev->started || !dev->running || dev->bus_off) {
        return ESP_ERR_INVALID_STATE;
    }
    if (xSemaphoreTake(dev->fence_lock, 0) != pdTRUE) return ESP_ERR_TIMEOUT;
    esp_err_t error = ESP_ERR_TIMEOUT;
    if (dev->fence_queued == dev->fence_completed) {
        uint32_t next = dev->fence_queued + 1U;
        if (next == 0U) next = 1U;
        const can_console_can_tx_item_t item = {.fence = next};
        if (xQueueSend(dev->tx_queue, &item, 0) == pdPASS) {
            dev->fence_queued = next;
            *token = next;
            error = ESP_OK;
        }
    }
    xSemaphoreGive(dev->fence_lock);
    return error;
}

esp_err_t can_console_can_tx_fence_poll(can_console_can_handle_t dev, uint32_t token,
                                       can_console_can_tx_fence_t *receipt)
{
    if (dev == NULL || receipt == NULL || token == 0U) {
        return ESP_ERR_INVALID_ARG;
    }
    if (xSemaphoreTake(dev->fence_lock, 0) != pdTRUE) return ESP_ERR_TIMEOUT;
    esp_err_t error = ESP_ERR_TIMEOUT;
    if (token != dev->fence_queued) error = ESP_ERR_INVALID_STATE;
    else if (token == dev->fence_completed) {
        *receipt = dev->fence_result;
        error = ESP_OK;
    }
    xSemaphoreGive(dev->fence_lock);
    return error;
}

esp_err_t can_console_can_recover(can_console_can_handle_t dev)
{
    if (dev == NULL || !dev->started) {
        ESP_LOGE(TAG, "Bus-off recovery rejected: CAN is not running");
        return ESP_ERR_INVALID_STATE;
    }
    ESP_LOGI(TAG, "Starting bus-off recovery");
    const esp_err_t ret = twai_node_recover(dev->node);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Bus-off recovery requested");
    } else {
        ESP_LOGE(TAG, "Bus-off recovery request failed: %s", esp_err_to_name(ret));
    }
    return ret;
}

esp_err_t can_console_can_get_status(can_console_can_handle_t dev,
                                     can_console_can_status_t *out_status)
{
    if (dev == NULL || out_status == NULL) {
        ESP_LOGE(TAG, "Status request rejected: invalid argument");
        return ESP_ERR_INVALID_ARG;
    }

    *out_status = (can_console_can_status_t) {
        .state = CAN_CONSOLE_CAN_STATE_STOPPED,
        .started = dev->started,
        .tx_queued_count = dev->tx_queued_count,
        .tx_completed_count = dev->tx_completed_count,
        .tx_failed_count = dev->tx_failed_count,
        .rx_received_count = dev->rx_received_count,
        .rx_dropped_count = dev->rx_dropped_count,
    };
    if (dev->node != NULL) {
        twai_node_status_t twai_status;
        twai_node_record_t twai_record;
        esp_err_t ret = twai_node_get_info(dev->node, &twai_status, &twai_record);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "TWAI status read failed: %s", esp_err_to_name(ret));
            return ret;
        }
        out_status->state = dev->started ? can_console_can_map_state(twai_status.state) : CAN_CONSOLE_CAN_STATE_STOPPED;
        out_status->tx_error_count = twai_status.tx_error_count;
        out_status->rx_error_count = twai_status.rx_error_count;
        out_status->bus_error_count = twai_record.bus_err_num;
    }
    return ESP_OK;
}

esp_err_t can_console_can_delete(can_console_can_handle_t dev)
{
    if (dev == NULL) {
        ESP_LOGE(TAG, "CAN deletion rejected: handle is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Deleting CAN device");
    esp_err_t ret = ESP_OK;
    if (dev->started) {
        ret = can_console_can_stop(dev);
    }
    if (dev->node != NULL) {
        esp_err_t delete_ret = twai_node_delete(dev->node);
        if (delete_ret != ESP_OK) {
            ESP_LOGE(TAG, "TWAI node deletion failed: %s", esp_err_to_name(delete_ret));
            return ret == ESP_OK ? delete_ret : ret;
        }
        dev->node = NULL;
    }
    if (dev->tx_queue != NULL) {
        vQueueDelete(dev->tx_queue);
    }
    if (dev->rx_queue != NULL) {
        vQueueDelete(dev->rx_queue);
    }
    if (dev->bus_off_sem != NULL) {
        vSemaphoreDelete(dev->bus_off_sem);
    }
    if (dev->self_test_sem != NULL) {
        vSemaphoreDelete(dev->self_test_sem);
    }
    if (dev->fence_lock != NULL) {
        vSemaphoreDelete(dev->fence_lock);
    }
    free(dev);
    ESP_LOGI(TAG, "CAN device deleted");
    return ret;
}
