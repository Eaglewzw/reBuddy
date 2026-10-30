#include "transport/can_manager.h"
#include "service/motor_session.h"

#include <inttypes.h>
#include <stdint.h>
#include <string.h>

#include "can_console_can.h"
#include "esp_rom_sys.h"
#include "mb_port.h"

#include "config/app_config.h"
#include "core/motor_manager.h"
#include "vendors/motor_vendor.h"

static const char *TAG = "can_manager";

typedef struct {
    can_console_can_handle_t can;
    can_rx_callback_t user_rx_cb;
    int tx_gpio;
    int rx_gpio;
    int controller_id;
    uint32_t bitrate;
    int scan_min;
    int scan_max;
    int64_t scan_until_us;
    bool started;
    int64_t last_qfull_log_us;
    int64_t last_tx_fail_log_us;
    int64_t last_rx_us;
    uint32_t tx_gap_us;
} can_channel_ctx_t;

static can_channel_ctx_t s_ch[CAN_MAX_CHANNELS];
static int s_count;

static void task_can_scan(void *arg);

static can_channel_ctx_t *ctx_of(can_channel_t ch)
{
    if (ch < 0 || ch >= s_count) {
        return NULL;
    }
    return &s_ch[ch];
}

static uint32_t pending_tx_count(const can_console_can_status_t *status)
{
    const uint32_t finished = status->tx_completed_count + status->tx_failed_count;
    return status->tx_queued_count > finished ? status->tx_queued_count - finished : 0;
}

static void log_tx_failure(can_channel_ctx_t *ctx, uint32_t id, esp_err_t err)
{
    const int64_t now = mb_now_us();
    if (now - ctx->last_tx_fail_log_us > 500000) {
        MB_LOGW(TAG, "tx failed id=0x%03" PRIX32 " err=0x%x", id, (unsigned)err);
        ctx->last_tx_fail_log_us = now;
    }
}

static void on_can_rx(can_console_can_handle_t handle,
                      const can_console_can_frame_t *frame,
                      void *user_ctx)
{
    (void)handle;
    const can_channel_t ch = (can_channel_t)(intptr_t)user_ctx;
    can_channel_ctx_t *ctx = ctx_of(ch);
    if (ctx == NULL || frame == NULL || frame->is_fd || frame->data_len > 8) {
        return;
    }

    twai_message_t msg = {
        .identifier = frame->id,
        .data_length_code = frame->data_len,
        .extd = frame->is_extended,
        .rtr = frame->is_remote,
    };
    memcpy(msg.data, frame->data, frame->data_len);
    ctx->last_rx_us = mb_now_us();

    if (ctx->user_rx_cb != NULL) {
        ctx->user_rx_cb(ch, &msg);
    }
}

can_channel_t can_manager_add_channel(int tx_gpio, int rx_gpio, int controller_id, int bitrate, int data_bitrate, int mode)
{
    if (s_count >= CAN_MAX_CHANNELS || controller_id != s_count) {
        MB_LOGE(TAG, "channel/controller allocation must be sequential: channel=%d controller=%d",
                s_count, controller_id);
        return (can_channel_t)-1;
    }

    can_channel_ctx_t *ctx = &s_ch[s_count];
    memset(ctx, 0, sizeof(*ctx));
    ctx->tx_gpio = tx_gpio;
    ctx->rx_gpio = rx_gpio;
    ctx->controller_id = controller_id;
    ctx->bitrate = bitrate > 0 ? (uint32_t)bitrate : 1000000U;
    ctx->scan_min = MOTORBRIDGE_MIN_MOTOR_ID;
    ctx->scan_max = MOTORBRIDGE_MAX_MOTOR_ID;

    can_console_can_config_t config;
    can_console_can_config_set_defaults(&config);
    config.tx_gpio = (gpio_num_t)tx_gpio;
    config.rx_gpio = (gpio_num_t)rx_gpio;
    config.nominal_bitrate = ctx->bitrate;
    config.data_bitrate = data_bitrate;
    config.mode = (can_console_can_mode_t)mode;
    /* Motor control traffic is a few frames per 100 ms, so depth 16 (the
     * component default) is ample.  The 64-deep queues this replaced cost
     * ~10 KB of heap per channel (64 x ~88 B tx item + 64 x ~80 B rx item);
     * on SRAM-tight ESP32-C5 builds that pushed queue AND task-stack
     * allocation past the free-heap border. */
    config.tx_queue_depth = 16;
    config.rx_queue_depth = 16;
    /* Can_rx / can_tx walk further than "queue pop" suggests: the RX path reaches
     * the user callback (frame router + motor state updates), which brushes
     * newlib locks and log formatting — a 2 KB boot overflowed in can_rx inside
     * lock_acquire_generic, watermark ~2.1 KB on this stack.  2560 keeps
     * ~0.5 KB over that while still saving ~1.5 KB/task against the 4 KB
     * component default.  Demos that log from inside the RX callback should
     * raise this. */
    config.task_stack_size = 2560;
    if(app_config_get()->can_task_priority)config.task_priority=app_config_get()->can_task_priority;

    const esp_err_t err = can_console_can_new(&config, &ctx->can);
    if (err != ESP_OK) {
        MB_LOGE(TAG, "CAN create ctlr=%d tx=%d rx=%d bitrate=%" PRIu32 " err=0x%x",
                controller_id, tx_gpio, rx_gpio, ctx->bitrate, (unsigned)err);
        return (can_channel_t)-1;
    }

    const app_config_t *app_cfg = app_config_get();
    if (app_cfg != NULL && app_cfg->max_motors >= 2) {
        ctx->tx_gap_us = 120;
    }

    MB_LOGI(TAG, "CAN create ctlr=%d tx=%d rx=%d bitrate=%" PRIu32,
            controller_id, tx_gpio, rx_gpio, ctx->bitrate);
    return (can_channel_t)(s_count++);
}

esp_err_t can_manager_start_channel(can_channel_t ch)
{
    can_channel_ctx_t *ctx = ctx_of(ch);
    if (ctx == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = can_console_can_set_rx_callback(ctx->can, on_can_rx,
                                                     (void *)(intptr_t)ch);
    if (err != ESP_OK) {
        return err;
    }
    err = can_console_can_start(ctx->can);
    if (err != ESP_OK) {
        return err;
    }
    ctx->started = true;

    /* task_can_scan body is a 200 ms idle poll plus one shallow send per probe
     * (build_scan_request fills a small struct; can_manager_send is a queue
     * submit); 2 KB is ample, saves 1 KB per channel vs the 3 KB before. */
    if (xTaskCreatePinnedToCore(task_can_scan, "can_scan", 2048,
                                (void *)(intptr_t)ch, app_config_get()->scan_task_priority ? app_config_get()->scan_task_priority : 6, NULL,
                                tskNO_AFFINITY) != pdPASS) {
        ctx->started = false;
        (void)can_console_can_stop(ctx->can);
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

void can_manager_set_rx_callback(can_channel_t ch, can_rx_callback_t callback)
{
    can_channel_ctx_t *ctx = ctx_of(ch);
    if (ctx != NULL) {
        ctx->user_rx_cb = callback;
    }
}

esp_err_t can_manager_send(can_channel_t ch, const twai_message_t *msg, int timeout_ms)
{ return can_manager_send_for_client(ch,msg,timeout_ms,CAN_MOTOR_CLIENT_SERVICE); }
esp_err_t can_manager_send_for_client(can_channel_t ch,const twai_message_t *msg,int timeout_ms,can_motor_client_t client)
{
    can_channel_ctx_t *ctx = ctx_of(ch);
    if (ctx == NULL || msg == NULL || msg->data_length_code > 8) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!ctx->started) {
        return ESP_ERR_INVALID_STATE;
    }
    if (timeout_ms < 0 || (timeout_ms == 0 && client != CAN_MOTOR_CLIENT_RAW)) {
        timeout_ms = 1;
    }

    can_console_can_status_t status = {0};
    if (can_console_can_get_status(ctx->can, &status) == ESP_OK) {
        if (status.state == CAN_CONSOLE_CAN_STATE_STOPPED ||
            status.state == CAN_CONSOLE_CAN_STATE_BUS_OFF) {
            return ESP_ERR_INVALID_STATE;
        }
        /* Emergency disable uses the driver's bounded queue wait, so a full
         * raw stream cannot reject the entire stop fan-out at this soft cap. */
        if (client != CAN_MOTOR_CLIENT_EMERGENCY && pending_tx_count(&status) >= 16) {
            const int64_t log_now = mb_now_us();
            if (log_now - ctx->last_qfull_log_us > 500000) {
                MB_LOGW(TAG, "tx throttled: queue high txq=%" PRIu32 " state=%d",
                        pending_tx_count(&status), (int)status.state);
                ctx->last_qfull_log_us = log_now;
            }
            return ESP_ERR_TIMEOUT;
        }
    }

    can_console_can_frame_t frame = {
        .id = msg->identifier,
        .data_len = msg->data_length_code,
        .is_extended = msg->extd,
        .is_remote = msg->rtr,
        .is_fd = false,
        .bit_rate_switch = false,
    };
    memcpy(frame.data, msg->data, msg->data_length_code);

    if(!motor_session_lock_tx(ch,client,msg))return ESP_ERR_INVALID_STATE;
    const esp_err_t err = can_console_can_transmit(ctx->can, &frame,
                                                   (uint32_t)timeout_ms);
    motor_session_unlock_tx();

    if (err != ESP_OK) {
        log_tx_failure(ctx, msg->identifier, err);
        return err;
    }

    if (ctx->tx_gap_us > 0) {
        esp_rom_delay_us(ctx->tx_gap_us);
    }
    return ESP_OK;
}

esp_err_t can_manager_tx_fence_begin(can_channel_t ch, uint32_t *token)
{
    can_channel_ctx_t *ctx = ctx_of(ch);
    return ctx != NULL ? can_console_can_tx_fence_begin(ctx->can, token) : ESP_ERR_INVALID_ARG;
}

esp_err_t can_manager_tx_fence_poll(can_channel_t ch, uint32_t token,
    can_console_can_tx_fence_t *receipt)
{
    can_channel_ctx_t *ctx = ctx_of(ch);
    return ctx != NULL ? can_console_can_tx_fence_poll(ctx->can, token, receipt) : ESP_ERR_INVALID_ARG;
}

void can_manager_trigger_scan(can_channel_t ch, int min_id, int max_id)
{
    can_channel_ctx_t *ctx = ctx_of(ch);
    if (ctx == NULL) {
        return;
    }
    if (min_id < MOTORBRIDGE_MIN_MOTOR_ID) {
        min_id = MOTORBRIDGE_MIN_MOTOR_ID;
    }
    if (max_id > MOTORBRIDGE_MAX_MOTOR_ID) {
        max_id = MOTORBRIDGE_MAX_MOTOR_ID;
    }
    if (min_id > max_id) {
        return;
    }
    ctx->scan_min = min_id;
    ctx->scan_max = max_id;
    ctx->scan_until_us = mb_now_us() + 2000000LL;
    MB_LOGI(TAG, "scan range updated ch=%d [%d, %d]", (int)ch, min_id, max_id);
}

void can_manager_stop_scan(can_channel_t ch)
{
    can_channel_ctx_t *ctx = ctx_of(ch);
    if (ctx != NULL) {
        ctx->scan_until_us = 0;
    }
}

bool can_manager_recent_rx(can_channel_t ch, int32_t within_ms)
{
    can_channel_ctx_t *ctx = ctx_of(ch);
    if (ctx == NULL || within_ms <= 0 || ctx->last_rx_us <= 0) {
        return false;
    }
    return mb_now_us() - ctx->last_rx_us <= (int64_t)within_ms * 1000;
}

esp_err_t can_manager_get_status(can_channel_t ch,
                                 can_console_can_status_t *out_status)
{
    can_channel_ctx_t *ctx = ctx_of(ch);
    if (ctx == NULL || out_status == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    return can_console_can_get_status(ctx->can, out_status);
}

/* Minimum wait between adjacent scan probe submissions; software-level
 * pacing, not an on-wire frame interval (see the component README). */
#define CAN_SCAN_PROBE_GAP_US 2000LL

static void task_can_scan(void *arg)
{
    const can_channel_t ch = (can_channel_t)(intptr_t)arg;
    can_channel_ctx_t *ctx = ctx_of(ch);
    if (ctx == NULL) {
        vTaskDelete(NULL);
        return;
    }

    while (true) {
        if (mb_now_us() >= ctx->scan_until_us) {
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }
        for (int id = ctx->scan_min;
             id <= ctx->scan_max && mb_now_us() < ctx->scan_until_us;
             ++id) {
            motor_state_t motor;
            if (motor_manager_get_state((uint8_t)id, &motor) && motor.vendor != NULL &&
                motor.channel == ch && motor.vendor->build_scan_request != NULL) {
                twai_message_t msg;
                motor.vendor->build_scan_request((uint8_t)id, &msg);
                (void)can_manager_send(ch, &msg, 8);
                mb_sleep_min_us(CAN_SCAN_PROBE_GAP_US);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(80));
    }
}
