/*
 * SPDX-FileCopyrightText: 2026 XIAO-C5-CAN
 * SPDX-License-Identifier: Apache-2.0
 */

#include "service/motor_service.h"

#include <inttypes.h>
#include <string.h>

#include "config/app_config.h"
#include "can_console_motor_client.h"
#include "core/control_loop.h"
#include "core/safety_manager.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "mb_port.h"
#include "service/frame_router.h"
#include "vendors/motor_vendor.h"

#define MOTOR_SERVICE_QUEUE_DEPTH   96U
#define MOTOR_SERVICE_TASK_STACK    4096U
#define MOTOR_SERVICE_TASK_PRIORITY 6U
#define MOTOR_SERVICE_BATCH_SLOTS   8U

typedef enum {
    MOTOR_SERVICE_EVENT_RX = 0,
    MOTOR_SERVICE_EVENT_ADMIN,
    MOTOR_SERVICE_EVENT_ADMIN_RANGE,
    MOTOR_SERVICE_EVENT_MODE,
    MOTOR_SERVICE_EVENT_MODES,
    MOTOR_SERVICE_EVENT_TICK,
    MOTOR_SERVICE_EVENT_REBIND,
    MOTOR_SERVICE_EVENT_REFERENCE,
    MOTOR_SERVICE_EVENT_FENCE,
} motor_service_event_type_t;

typedef struct {
    size_t count;
    motor_service_mode_command_t commands[CAN_CONSOLE_MOTOR_MAX_TARGET_GROUP];
} motor_service_modes_t;

/* Caller owns this stack request until the owner signals completion. */
typedef struct {
    bool reset_only;
    int reset_channel;
    const can_console_motor_binding_t *bindings;
    size_t count;
    SemaphoreHandle_t done;
    esp_err_t result;
} rebind_request_t;

typedef struct {
    motor_service_event_type_t type;
    union {
        struct {
            can_channel_t channel;
            twai_message_t message;
        } rx;
        host_admin_cmd_t admin;
        struct {
            host_admin_opcode_t opcode;
            uint8_t first;
            uint8_t last;
        } range;
        motor_service_mode_command_t target;
        uint8_t target_batch_slot;
        float dt_s;
        rebind_request_t *rebind;
        uint32_t reference;
    } value;
} motor_service_event_t;

static const char *TAG = "motor_service";
static QueueHandle_t s_event_queue;
static QueueHandle_t s_emergency_queue;
static QueueHandle_t s_overflow_tick_queue;
static QueueHandle_t s_free_batches;
static SemaphoreHandle_t s_work_sem;
static motor_service_modes_t s_target_batches[MOTOR_SERVICE_BATCH_SLOTS];
static int64_t s_last_rx_drop_log_us;
static can_console_motor_bus_observer_t s_observers[CAN_MAX_CHANNELS];
static void *s_observer_contexts[CAN_MAX_CHANNELS];
static SemaphoreHandle_t s_reference_lock;
static struct {
    can_console_motor_reference_status_t status;
    motor_service_mode_command_t commands[CAN_CONSOLE_MOTOR_MAX_TARGET_GROUP];
    size_t count;
    int channel;
    bool acquire, release;
    unsigned stage;
    uint32_t fence, failures;
} s_reference;

static struct {
    uint32_t token, transport_token, failures;
    int channel;
    unsigned stage; /* 0=done, 1=service queued, 2=begin TX fence, 3=poll */
    can_console_motor_fence_status_t status;
} s_fence;

static void poll_fence(void)
{
    xSemaphoreTake(s_reference_lock,portMAX_DELAY);
    esp_err_t e=ESP_OK;
    if(s_fence.stage==2) {
        e=can_manager_tx_fence_begin(s_fence.channel,&s_fence.transport_token);
        if(e==ESP_ERR_TIMEOUT)goto done;
        if(e!=ESP_OK)goto complete;
        s_fence.stage=3;
    }
    if(s_fence.stage==3) {
        can_console_can_tx_fence_t receipt;
        e=can_manager_tx_fence_poll(s_fence.channel,s_fence.transport_token,&receipt);
        if(e==ESP_ERR_TIMEOUT)goto done;
        if(e==ESP_OK){e=receipt.error;if(e==ESP_OK && receipt.failed_count!=s_fence.failures)e=ESP_FAIL;
            s_fence.status.confirmed_us=receipt.last_tx_us;}
        goto complete;
    }
    goto done;
complete:
    s_fence.stage=0;s_fence.status.complete=true;s_fence.status.error=e;
done:
    xSemaphoreGive(s_reference_lock);
}

/* ID/range 0..UINT8_MAX means all motors (ESTOP). Unrelated admin commands
 * must not release a group, e.g. disabling a separate gripper. */
static void abort_reference(uint8_t first, uint8_t last)
{
    xSemaphoreTake(s_reference_lock, portMAX_DELAY);
    bool affected = false;
    for (size_t i = 0; i < s_reference.count; ++i) {
        const uint8_t id = s_reference.commands[i].motor_id;
        if (id >= first && id <= last) affected = true;
    }
    if (s_reference.status.owned && affected) {
        s_reference.status.owned = false;
        s_reference.status.state = CAN_CONSOLE_MOTOR_REFERENCE_FAILED;
        s_reference.status.error = ESP_ERR_INVALID_STATE;
        s_reference.stage = 0;
        control_loop_reference_ids(NULL, 0);
    }
    xSemaphoreGive(s_reference_lock);
}

static bool reference_pending(void)
{
    xSemaphoreTake(s_reference_lock, portMAX_DELAY);
    const bool pending = s_fence.stage!=0 || (s_reference.status.owned &&
        s_reference.status.state == CAN_CONSOLE_MOTOR_REFERENCE_PENDING);
    xSemaphoreGive(s_reference_lock);
    return pending;
}

static esp_err_t send_reference(const motor_service_mode_command_t *commands, size_t count)
{
    twai_message_t frames[CAN_CONSOLE_MOTOR_MAX_TARGET_GROUP];
    int channel = -1;
    for (size_t i = 0; i < count; ++i) {
        motor_state_t m;
        if (!motor_manager_get_state(commands[i].motor_id, &m) || !m.online || !m.enabled ||
            !m.mode_ready || (m.mode != MOTOR_MODE_MIT && m.mode != MOTOR_MODE_POS_VEL) ||
            m.mode != commands[i].mode || m.vendor == NULL ||
            m.vendor->build_control_frame == NULL || (i != 0 && m.channel != channel))
            return ESP_ERR_INVALID_STATE;
        channel = m.channel;
        if (!m.vendor->build_control_frame(m.id, m.mode, &commands[i].command, &m.params, &frames[i]))
            return ESP_ERR_INVALID_ARG;
    }
    for (size_t i = 0; i < count; ++i) {
        if (!frame_router_process_mode_cmd(commands[i].motor_id, commands[i].mode, &commands[i].command))
            return ESP_ERR_INVALID_STATE;
    }
    for (size_t i = 0; i < count; ++i) {
        const esp_err_t error = can_manager_send(channel, &frames[i], 1);
        if (error != ESP_OK) return error;
    }
    return ESP_OK;
}

/* Owner only. A real FIFO transport fence, not merely service processing. */
static void poll_reference(void)
{
    xSemaphoreTake(s_reference_lock, portMAX_DELAY);
    if (s_reference.status.state != CAN_CONSOLE_MOTOR_REFERENCE_PENDING || s_reference.stage == 0) goto done;
    esp_err_t error = ESP_OK;
    if (s_reference.stage == 1 || s_reference.stage == 4) {
        error = can_manager_tx_fence_begin(s_reference.channel, &s_reference.fence);
        if (error == ESP_ERR_TIMEOUT) goto done;
        if (error != ESP_OK) goto failure;
        ++s_reference.stage;
    }
    if (s_reference.stage == 2 || s_reference.stage == 5) {
        can_console_can_tx_fence_t receipt;
        error = can_manager_tx_fence_poll(s_reference.channel, s_reference.fence, &receipt);
        if (error == ESP_ERR_TIMEOUT) goto done;
        if (error != ESP_OK) goto failure;
        error = receipt.error;
        if (error == ESP_OK && receipt.failed_count != s_reference.failures) error = ESP_FAIL;
        if (error != ESP_OK) goto failure;
        if (s_reference.stage == 5) {
            s_reference.status.state = CAN_CONSOLE_MOTOR_REFERENCE_DELIVERED;
            s_reference.status.confirmed_us = receipt.last_tx_us;
            s_reference.stage = 0;
            goto done;
        }
        s_reference.stage = 3;
    }
    if (s_reference.stage == 3) {
        error = send_reference(s_reference.commands, s_reference.count);
        if (error != ESP_OK) goto failure;
        s_reference.stage = 4;
    }
    goto done;
failure:
    s_reference.status.state = CAN_CONSOLE_MOTOR_REFERENCE_FAILED;
    s_reference.status.error = error;
    s_reference.stage = 0;
    /* Failure retains suppression until disable/ESTOP; no uncertain replay. */
done:
    xSemaphoreGive(s_reference_lock);
}

static esp_err_t submit_event(const motor_service_event_t *event)
{
    if (event == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_event_queue == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    const BaseType_t queued = xQueueSend(s_event_queue, event, 0U);
    if (queued == pdTRUE) {
        (void)xSemaphoreGive(s_work_sem);
    }
    return queued == pdTRUE ? ESP_OK : ESP_ERR_TIMEOUT;
}

static bool reference_reserved(uint8_t id)
{
    if(s_reference_lock==NULL)return false;
    xSemaphoreTake(s_reference_lock,portMAX_DELAY);
    bool reserved=false;
    if(s_reference.status.owned)for(size_t i=0;i<s_reference.count;++i)
        if(s_reference.commands[i].motor_id==id)reserved=true;
    xSemaphoreGive(s_reference_lock);
    return reserved;
}

static void discard_queued_events(void)
{
    motor_service_event_t event;
    const UBaseType_t pending = uxQueueMessagesWaiting(s_event_queue);
    for (UBaseType_t i = 0U; i < pending; ++i) {
        if (xQueueReceive(s_event_queue, &event, 0U) != pdTRUE) {
            break;
        }
        if (event.type == MOTOR_SERVICE_EVENT_MODES) {
            (void)xQueueSend(s_free_batches, &event.value.target_batch_slot, 0U);
        } else if (event.type == MOTOR_SERVICE_EVENT_REBIND) {
            event.value.rebind->result = ESP_ERR_INVALID_STATE;
            (void)xSemaphoreGive(event.value.rebind->done);
        }
    }
}

static bool process_emergency(void)
{
    host_admin_cmd_t command;
    if (xQueueReceive(s_emergency_queue, &command, 0U) != pdTRUE) {
        return false;
    }
    discard_queued_events();
    abort_reference(0, UINT8_MAX);
    xSemaphoreTake(s_reference_lock,portMAX_DELAY);
    if(s_fence.stage){s_fence.stage=0;s_fence.status.complete=true;s_fence.status.error=ESP_ERR_INVALID_STATE;}
    xSemaphoreGive(s_reference_lock);
    frame_router_process_admin_cmd(&command);
    return true;
}

static void process_tick(float dt_s)
{
    control_loop_process_tick(dt_s);
    frame_router_process_reg_verify_tick();
    safety_manager_tick();
}

static esp_err_t process_rebind(const rebind_request_t *request)
{
    if(request->reset_only){
        int ch=request->reset_channel;
        if(!can_console_motor_session_raw_active(ch))return ESP_ERR_INVALID_STATE;
        for(unsigned id=1;id<=motor_manager_count();++id){
            motor_state_t m;if(!motor_manager_get_state(id,&m))return ESP_ERR_INVALID_STATE;
            if(m.channel!=ch || !m.vendor)continue;
            if(m.enabled)return ESP_ERR_INVALID_STATE;
            if(m.last_seen_us || m.last_feedback_us){
                int64_t age=mb_now_us()-m.last_feedback_us;
                if(!m.last_feedback_us || age<0 || age>1000000)return ESP_ERR_INVALID_STATE;
                bool disabled=!strcmp(m.vendor->name,"robstride")?(m.status&3)==0:
                              !strcmp(m.vendor->name,"damiao")?(m.status&15)==0:false;
                if(!disabled)return ESP_ERR_INVALID_STATE;
            }
        }
        for(unsigned id=1;id<=motor_manager_count();++id){
            motor_state_t m;motor_manager_get_state(id,&m);
            if(m.channel!=ch || !m.vendor)continue;
            if(m.vendor->reset_session)m.vendor->reset_session(id);
            motor_manager_reset_session(id);
        }
        return ESP_OK;
    }
    /* Check on the owner, after earlier queued enable/target commands ran. */
    for (size_t i = 0; i < request->count; ++i) {
        motor_state_t state;
        if (!motor_manager_get_state(request->bindings[i].motor_id, &state) || state.enabled) {
            return ESP_ERR_INVALID_STATE;
        }
    }
    for (size_t i = 0; i < request->count; ++i) {
        const can_console_motor_binding_t *b = &request->bindings[i];
        for (int v = 0; v < motor_vendor_count(); ++v) {
            const motor_vendor_ops_t *ops = motor_vendor_get_by_index(v);
            if (ops->reset_session != NULL) {
                ops->reset_session(b->motor_id);
            }
        }
        motor_manager_reset_session(b->motor_id);
        can_manager_stop_scan(b->channel);
        esp_err_t error = can_console_motor_bind(b->motor_id, b->vendor, b->model, b->channel);
        if (error == ESP_OK) {
            error = can_console_motor_set_gains(b->motor_id, b->kp, b->kd);
        }
        if (error != ESP_OK) {
            /* Do not expose a partly configured group to control traffic. */
            for (size_t j = 0; j < request->count; ++j) {
                (void)motor_manager_set_mode(request->bindings[j].motor_id, MOTOR_MODE_DISABLED);
                (void)motor_manager_set_channel(request->bindings[j].motor_id, -1);
            }
            return error;
        }
    }
    return ESP_OK;
}

static void motor_service_task(void *argument)
{
    (void)argument;
    motor_service_event_t event;

    while (true) {
        if (process_emergency()) {
            frame_router_refresh_mode_readiness();
            continue;
        }

        poll_reference();
        poll_fence();
        float overdue_dt_s;
        if (xQueueReceive(s_overflow_tick_queue, &overdue_dt_s, 0U) == pdTRUE) {
            process_tick(overdue_dt_s);
            frame_router_refresh_mode_readiness();
            continue;
        }

        if (xQueueReceive(s_event_queue, &event, 0U) != pdTRUE) {
            (void)xSemaphoreTake(s_work_sem, reference_pending() ? 1 : portMAX_DELAY);
            continue;
        }

        switch (event.type) {
        case MOTOR_SERVICE_EVENT_RX:
            frame_router_process_can_rx(event.value.rx.channel,
                                        &event.value.rx.message);
            break;
        case MOTOR_SERVICE_EVENT_ADMIN:
            if (event.value.admin.op == HOST_OP_DISABLE || event.value.admin.op == HOST_OP_STOP)
                abort_reference(event.value.admin.motor_id,
                    event.value.admin.motor_id == 0 ? UINT8_MAX : event.value.admin.motor_id);
            frame_router_process_admin_cmd(&event.value.admin);
            break;
        case MOTOR_SERVICE_EVENT_ADMIN_RANGE: {
            if (event.value.range.opcode == HOST_OP_DISABLE || event.value.range.opcode == HOST_OP_STOP)
                abort_reference(event.value.range.first, event.value.range.last);
            const int gap_ms = app_config_get()->bulk_op_gap_ms;
            for (uint8_t id = event.value.range.first;
                 id <= event.value.range.last; ++id) {
                if (process_emergency()) {
                    break;
                }
                const host_admin_cmd_t command = {
                    .op = event.value.range.opcode,
                    .motor_id = id,
                };
                frame_router_process_admin_cmd(&command);
                if (id < event.value.range.last && gap_ms > 0) {
                    /* Software-operation interval: after the previous motor's
                     * admin-command processing returned, wait at least
                     * gap_ms before starting the next motor. Not a
                     * bus-frame interval or device-processing guarantee:
                     * frames are queued and sent asynchronously. */
                    mb_sleep_min_us((int64_t)gap_ms * 1000);
                }
            }
            break;
        }
        case MOTOR_SERVICE_EVENT_MODE:
            (void)frame_router_process_mode_cmd(
                event.value.target.motor_id, event.value.target.mode,
                &event.value.target.command);
            break;
        case MOTOR_SERVICE_EVENT_MODES: {
            motor_service_modes_t *targets =
                &s_target_batches[event.value.target_batch_slot];
            for (size_t i = 0U; i < targets->count; ++i) {
                const motor_service_mode_command_t *target =
                    &targets->commands[i];
                (void)frame_router_process_mode_cmd(
                    target->motor_id, target->mode, &target->command);
            }
            (void)xQueueSend(s_free_batches, &event.value.target_batch_slot, 0U);
            break;
        }
        case MOTOR_SERVICE_EVENT_TICK:
            process_tick(event.value.dt_s);
            break;
        case MOTOR_SERVICE_EVENT_REBIND: {
            rebind_request_t *request = event.value.rebind;
            request->result = process_rebind(request);
            (void)xSemaphoreGive(request->done);
            break;
        }
        case MOTOR_SERVICE_EVENT_FENCE:
            xSemaphoreTake(s_reference_lock,portMAX_DELAY);
            if(event.value.reference==s_fence.token && s_fence.stage==1)s_fence.stage=2;
            xSemaphoreGive(s_reference_lock);
            break;
        case MOTOR_SERVICE_EVENT_REFERENCE: {
            xSemaphoreTake(s_reference_lock, portMAX_DELAY);
            if (event.value.reference == s_reference.status.sequence && s_reference.status.owned) {
                if (s_reference.release) {
                    control_loop_reference_ids(NULL, 0);
                    s_reference.status.owned = false;
                    s_reference.status.state = CAN_CONSOLE_MOTOR_REFERENCE_DELIVERED;
                    s_reference.status.confirmed_us = mb_now_us();
                } else {
                    uint8_t ids[CAN_CONSOLE_MOTOR_MAX_TARGET_GROUP];
                    for (size_t i = 0; i < s_reference.count; ++i) ids[i] = s_reference.commands[i].motor_id;
                    control_loop_reference_ids(ids, s_reference.count);
                    can_console_can_status_t bus;
                    const esp_err_t error = can_manager_get_status(s_reference.channel, &bus);
                    if (error == ESP_OK) {
                        s_reference.failures = bus.tx_failed_count;
                        s_reference.stage = s_reference.acquire ? 1 : 3;
                    } else {
                        s_reference.status.error = error;
                        s_reference.status.state = CAN_CONSOLE_MOTOR_REFERENCE_FAILED;
                    }
                }
            }
            xSemaphoreGive(s_reference_lock);
            break;
        }
        default:
            break;
        }

        frame_router_refresh_mode_readiness();
    }
}

esp_err_t motor_service_start(void)
{
    if (s_event_queue != NULL) {
        return ESP_OK;
    }

    s_event_queue = xQueueCreate(MOTOR_SERVICE_QUEUE_DEPTH,
                                 sizeof(motor_service_event_t));
    s_emergency_queue = xQueueCreate(1U, sizeof(host_admin_cmd_t));
    s_overflow_tick_queue = xQueueCreate(1U, sizeof(float));
    s_free_batches = xQueueCreate(MOTOR_SERVICE_BATCH_SLOTS, sizeof(uint8_t));
    s_work_sem = xSemaphoreCreateBinary();
    s_reference_lock = xSemaphoreCreateMutex();
    if (s_event_queue == NULL || s_emergency_queue == NULL ||
        s_overflow_tick_queue == NULL || s_free_batches == NULL ||
        s_work_sem == NULL || s_reference_lock == NULL) {
        if (s_event_queue != NULL) {
            vQueueDelete(s_event_queue);
        }
        if (s_emergency_queue != NULL) {
            vQueueDelete(s_emergency_queue);
        }
        if (s_overflow_tick_queue != NULL) {
            vQueueDelete(s_overflow_tick_queue);
        }
        if (s_free_batches != NULL) {
            vQueueDelete(s_free_batches);
        }
        if (s_work_sem != NULL) {
            vSemaphoreDelete(s_work_sem);
        }
        s_event_queue = NULL;
        s_emergency_queue = NULL;
        s_overflow_tick_queue = NULL;
        s_free_batches = NULL;
        s_work_sem = NULL;
        if (s_reference_lock != NULL) vSemaphoreDelete(s_reference_lock);
        s_reference_lock = NULL;
        return ESP_ERR_NO_MEM;
    }
    for (uint8_t slot = 0U; slot < MOTOR_SERVICE_BATCH_SLOTS; ++slot) {
        (void)xQueueSend(s_free_batches, &slot, 0U);
    }

    if (xTaskCreate(motor_service_task, "motor_service",
                    MOTOR_SERVICE_TASK_STACK, NULL,
                    app_config_get()->service_task_priority ? app_config_get()->service_task_priority : MOTOR_SERVICE_TASK_PRIORITY, NULL) != pdPASS) {
        vQueueDelete(s_event_queue);
        s_event_queue = NULL;
        vQueueDelete(s_emergency_queue);
        vQueueDelete(s_overflow_tick_queue);
        vQueueDelete(s_free_batches);
        vSemaphoreDelete(s_work_sem);
        s_emergency_queue = NULL;
        s_overflow_tick_queue = NULL;
        s_free_batches = NULL;
        s_work_sem = NULL;
        vSemaphoreDelete(s_reference_lock);
        s_reference_lock = NULL;
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

esp_err_t motor_service_rebind_disabled(
    const can_console_motor_binding_t *bindings, size_t count)
{
    rebind_request_t request = {
        .bindings = bindings, .count = count, .done = xSemaphoreCreateBinary(),
    };
    if (request.done == NULL) {
        return ESP_ERR_NO_MEM;
    }
    const motor_service_event_t event = {
        .type = MOTOR_SERVICE_EVENT_REBIND, .value.rebind = &request,
    };
    esp_err_t error = submit_event(&event);
    if (error == ESP_OK) {
        /* No timed-out stack pointer can remain queued. ESTOP's discard path
         * completes the request too, so its caller is never left waiting. */
        (void)xSemaphoreTake(request.done, portMAX_DELAY);
        error = request.result;
    }
    vSemaphoreDelete(request.done);
    return error;
}

esp_err_t motor_service_set_observer(int channel,
                                     can_console_motor_bus_observer_t observer,
                                     void *context)
{
    if (channel < 0 || channel >= CAN_MAX_CHANNELS) {
        return ESP_ERR_INVALID_ARG;
    }
    s_observers[channel] = observer;
    s_observer_contexts[channel] = context;
    return ESP_OK;
}

void motor_service_handle_can_rx(can_channel_t channel,
                                 const twai_message_t *message)
{
    if (message == NULL || message->data_length_code > 8U) {
        return;
    }
    if (channel >= 0 && channel < CAN_MAX_CHANNELS &&
        s_observers[channel] != NULL) {
        can_console_motor_frame_t frame = {
            .id = message->identifier,
            .data_len = message->data_length_code,
            .is_extended = message->extd,
            .is_remote = message->rtr,
        };
        memcpy(frame.data, message->data, frame.data_len);
        s_observers[channel](channel, &frame, s_observer_contexts[channel]);
    }
    host_admin_cmd_t admin;
    if (!can_console_motor_session_raw_active(channel) && host_protocol_parse_admin_cmd(message, &admin) &&
        admin.op == HOST_OP_ESTOP) {
        (void)motor_service_submit_admin(&admin);
        return;
    }
    const motor_service_event_t event = {
        .type = MOTOR_SERVICE_EVENT_RX,
        .value.rx = {
            .channel = channel,
            .message = *message,
        },
    };
    const esp_err_t error = submit_event(&event);
    if (error != ESP_OK) {
        const int64_t now_us = mb_now_us();
        if (now_us - s_last_rx_drop_log_us > 500000LL) {
            MB_LOGW(TAG, "RX dispatch dropped: ch=%d id=0x%" PRIX32
                         " error=0x%x",
                    (int)channel, message->identifier, (unsigned)error);
            s_last_rx_drop_log_us = now_us;
        }
    }
}

esp_err_t motor_service_submit_admin(const host_admin_cmd_t *command)
{
    if (command == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (command->op == HOST_OP_ESTOP) {
        if (s_emergency_queue == NULL) {
            return ESP_ERR_INVALID_STATE;
        }
        (void)xQueueOverwrite(s_emergency_queue, command);
        (void)xSemaphoreGive(s_work_sem);
        return ESP_OK;
    }
    const motor_service_event_t event = {
        .type = MOTOR_SERVICE_EVENT_ADMIN,
        .value.admin = *command,
    };
    return submit_event(&event);
}

esp_err_t motor_service_submit_admin_range(host_admin_opcode_t opcode,
                                           uint8_t first, uint8_t last)
{
    const motor_service_event_t event = {
        .type = MOTOR_SERVICE_EVENT_ADMIN_RANGE,
        .value.range = {
            .opcode = opcode,
            .first = first,
            .last = last,
        },
    };
    return submit_event(&event);
}

esp_err_t motor_service_submit_mode(uint8_t motor_id, motor_mode_t mode,
                                    const damiao_cmd_t *command)
{
    if (command == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if(reference_reserved(motor_id))return ESP_ERR_INVALID_STATE;
    const motor_service_event_t event = {
        .type = MOTOR_SERVICE_EVENT_MODE,
        .value.target = {
            .motor_id = motor_id,
            .mode = mode,
            .command = *command,
        },
    };
    return submit_event(&event);
}

esp_err_t motor_service_submit_modes(const motor_service_mode_command_t *commands,
                                     size_t count)
{
    if (s_free_batches == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    for(size_t i=0;i<count;++i)if(reference_reserved(commands[i].motor_id))return ESP_ERR_INVALID_STATE;
    uint8_t slot;
    if (xQueueReceive(s_free_batches, &slot, 0U) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    motor_service_modes_t *targets = &s_target_batches[slot];
    targets->count = count;
    memcpy(targets->commands, commands, count * sizeof(*commands));
    const motor_service_event_t event = {
        .type = MOTOR_SERVICE_EVENT_MODES,
        .value.target_batch_slot = slot,
    };
    const esp_err_t error = submit_event(&event);
    if (error != ESP_OK) {
        (void)xQueueSend(s_free_batches, &slot, 0U);
    }
    return error;
}

esp_err_t motor_service_submit_tick(float dt_s)
{
    const motor_service_event_t event = {
        .type = MOTOR_SERVICE_EVENT_TICK,
        .value.dt_s = dt_s,
    };
    const esp_err_t error = submit_event(&event);
    if (error != ESP_ERR_TIMEOUT) {
        return error;
    }
    (void)xQueueOverwrite(s_overflow_tick_queue, &dt_s);
    (void)xSemaphoreGive(s_work_sem);
    return ESP_OK;
}

esp_err_t motor_service_submit_reference(const motor_service_mode_command_t *commands,
    size_t count, bool acquire, bool release, uint32_t *sequence)
{
    if (s_reference_lock == NULL) return ESP_ERR_INVALID_STATE;
    xSemaphoreTake(s_reference_lock, portMAX_DELAY);
    esp_err_t error = ESP_ERR_INVALID_STATE;
    if (s_reference.status.owned && s_reference.status.state == CAN_CONSOLE_MOTOR_REFERENCE_PENDING) {
        error = ESP_ERR_TIMEOUT;
        goto done;
    }
    if(s_fence.stage!=0)goto done;
    if (acquire ? s_reference.status.owned : !s_reference.status.owned) goto done;
    if (!acquire && s_reference.status.state == CAN_CONSOLE_MOTOR_REFERENCE_FAILED) goto done;
    if (!release) {
        for (size_t i = 0; i < count; ++i) {
            if (!acquire && (count != s_reference.count ||
                commands[i].motor_id != s_reference.commands[i].motor_id)) goto done;
        }
    }
    uint32_t next = s_reference.status.sequence + 1U;
    if (next == 0) next = 1;
    const motor_service_event_t event = {.type=MOTOR_SERVICE_EVENT_REFERENCE,.value.reference=next};
    error = submit_event(&event);
    if (error != ESP_OK) goto done;
    if (!release) {
        memcpy(s_reference.commands, commands, count * sizeof(*commands));
        s_reference.count = count;
        motor_state_t state;
        (void)motor_manager_get_state(commands[0].motor_id, &state);
        s_reference.channel = state.channel;
    }
    s_reference.acquire = acquire; s_reference.release = release; s_reference.stage = 0;
    s_reference.status = (can_console_motor_reference_status_t) {
        .sequence=next,.owned=true,.state=CAN_CONSOLE_MOTOR_REFERENCE_PENDING,.queued_us=mb_now_us(),
    };
    *sequence=next;
done:
    xSemaphoreGive(s_reference_lock);
    return error;
}

esp_err_t motor_service_get_reference(uint32_t sequence, can_console_motor_reference_status_t *status)
{
    if (status == NULL || sequence == 0) return ESP_ERR_INVALID_ARG;
    if (s_reference_lock == NULL) return ESP_ERR_INVALID_STATE;
    xSemaphoreTake(s_reference_lock, portMAX_DELAY);
    *status = s_reference.status;
    xSemaphoreGive(s_reference_lock);
    return status->sequence == sequence ? ESP_OK : ESP_ERR_INVALID_STATE;
}

esp_err_t motor_service_fence_begin(int channel,uint32_t *token)
{
    if(token==NULL || channel<0 || channel>=CAN_MAX_CHANNELS)return ESP_ERR_INVALID_ARG;
    if(s_reference_lock==NULL)return ESP_ERR_INVALID_STATE;
    xSemaphoreTake(s_reference_lock,portMAX_DELAY);
    esp_err_t e=ESP_ERR_INVALID_STATE;
    if(s_reference.status.owned || s_fence.stage!=0)goto done;
    can_console_can_status_t bus;e=can_manager_get_status(channel,&bus);if(e!=ESP_OK)goto done;
    uint32_t next=s_fence.token+1;if(next==0)next=1;
    const motor_service_event_t event={.type=MOTOR_SERVICE_EVENT_FENCE,.value.reference=next};
    e=submit_event(&event);
    if(e==ESP_OK){s_fence.token=next;s_fence.channel=channel;s_fence.failures=bus.tx_failed_count;
        s_fence.stage=1;s_fence.status=(can_console_motor_fence_status_t){0};*token=next;}
done:
    xSemaphoreGive(s_reference_lock);return e;
}
esp_err_t motor_service_fence_poll(uint32_t token,can_console_motor_fence_status_t *status)
{
    if(token==0 || status==NULL)return ESP_ERR_INVALID_ARG;
    if(s_reference_lock==NULL)return ESP_ERR_INVALID_STATE;
    xSemaphoreTake(s_reference_lock,portMAX_DELAY);
    const esp_err_t e=token==s_fence.token?ESP_OK:ESP_ERR_INVALID_STATE;
    if(e==ESP_OK)*status=s_fence.status;
    xSemaphoreGive(s_reference_lock);return e;
}

esp_err_t can_console_motor_reset_disabled_bus(int channel)
{
    if(!can_console_motor_session_raw_active(channel))return ESP_ERR_INVALID_STATE;
    rebind_request_t request={.reset_only=true,.reset_channel=channel,.result=ESP_FAIL};
    request.done=xSemaphoreCreateBinary();if(!request.done)return ESP_ERR_NO_MEM;
    motor_service_event_t event={.type=MOTOR_SERVICE_EVENT_REBIND,.value.rebind=&request};
    esp_err_t e=submit_event(&event);
    if(e==ESP_OK){xSemaphoreTake(request.done,portMAX_DELAY);e=request.result;}
    vSemaphoreDelete(request.done);return e;
}
