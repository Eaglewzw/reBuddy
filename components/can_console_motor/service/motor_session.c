/* SPDX-License-Identifier: Apache-2.0 */
#include "service/motor_session.h"
#include "transport/can_manager.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <string.h>
static SemaphoreHandle_t gate;
static can_motor_client_t owners[CAN_MAX_CHANNELS];
static bool releasing[CAN_MAX_CHANNELS];
static bool valid(int ch) {return ch>=0 && ch<CAN_MAX_CHANNELS;}
static void settle(int ch)
{
    if(!releasing[ch])return;
    can_console_can_status_t s;
    if(can_manager_get_status(ch,&s)==ESP_OK &&
       s.tx_queued_count==s.tx_completed_count+s.tx_failed_count) {
        owners[ch]=CAN_MOTOR_CLIENT_NONE;releasing[ch]=false;
    }
}
esp_err_t can_console_motor_sessions_init(void)
{
    if(gate)return ESP_ERR_INVALID_STATE;
    gate=xSemaphoreCreateMutex();return gate?ESP_OK:ESP_ERR_NO_MEM;
}
bool can_console_motor_session_acquire(int ch,can_motor_client_t client)
{
    if(!gate || !valid(ch) || (client!=CAN_MOTOR_CLIENT_UI && client!=CAN_MOTOR_CLIENT_USB && client!=CAN_MOTOR_CLIENT_UI_NATIVE && client!=CAN_MOTOR_CLIENT_RAW && client!=CAN_MOTOR_CLIENT_WEB))return false;
    xSemaphoreTake(gate,portMAX_DELAY);settle(ch);
    bool ok=owners[ch]==CAN_MOTOR_CLIENT_NONE;
    if(ok)owners[ch]=client;
    xSemaphoreGive(gate);return ok;
}
void can_console_motor_session_release(int ch,can_motor_client_t client)
{
    if(!gate || !valid(ch) || client==CAN_MOTOR_CLIENT_NONE)return;
    xSemaphoreTake(gate,portMAX_DELAY);
    if(owners[ch]==client){releasing[ch]=true;settle(ch);}
    xSemaphoreGive(gate);
}
can_motor_client_t can_console_motor_session_owner(int ch)
{
    if(!gate || !valid(ch))return CAN_MOTOR_CLIENT_NONE;
    xSemaphoreTake(gate,portMAX_DELAY);settle(ch);
    can_motor_client_t owner=owners[ch];xSemaphoreGive(gate);return owner;
}
static bool telemetry(const twai_message_t *f)
{
    if(!f || !f->extd || f->rtr || f->data_length_code!=8)return false;
    unsigned type=f->identifier>>24;
    /* Type 17 register reads, or the exact Type 24 telemetry subscription. */
    if(type==17)return true;
    static const uint8_t report[8]={1,2,3,4,5,6,1,0};
    return type==24 && !memcmp(f->data,report,sizeof(report));
}
bool motor_session_lock_tx(int ch,can_motor_client_t client,const twai_message_t *frame)
{
    if(!gate)return true; /* Unmodified upstream standalone applications. */
    if(!valid(ch))return false;
    xSemaphoreTake(gate,portMAX_DELAY);settle(ch);
    bool ok=!releasing[ch] && (client==CAN_MOTOR_CLIENT_NONE ?
        owners[ch]!=CAN_MOTOR_CLIENT_UI && owners[ch]!=CAN_MOTOR_CLIENT_RAW && owners[ch]!=CAN_MOTOR_CLIENT_RAW_ESTOP && telemetry(frame) :
        client==CAN_MOTOR_CLIENT_EMERGENCY ? owners[ch]==CAN_MOTOR_CLIENT_RAW_ESTOP :
        client==CAN_MOTOR_CLIENT_SERVICE ?
            (owners[ch]==CAN_MOTOR_CLIENT_USB || owners[ch]==CAN_MOTOR_CLIENT_UI_NATIVE || owners[ch]==CAN_MOTOR_CLIENT_WEB) :
            (client!=CAN_MOTOR_CLIENT_RAW_ESTOP && owners[ch]==client));
    if(!ok)xSemaphoreGive(gate);
    return ok;
}
void motor_session_unlock_tx(void) {if(gate)xSemaphoreGive(gate);}
esp_err_t can_console_motor_session_send(int ch,can_motor_client_t client,
    const can_console_motor_frame_t *f,uint32_t timeout_ms)
{
    if(!f || f->data_len>8 || (f->is_extended?f->id>0x1FFFFFFFU:f->id>0x7FFU))return ESP_ERR_INVALID_ARG;
    twai_message_t msg={.identifier=f->id,.data_length_code=f->data_len,.extd=f->is_extended,.rtr=f->is_remote};
    memcpy(msg.data,f->data,f->data_len);
    return can_manager_send_for_client(ch,&msg,timeout_ms,client);
}

bool can_console_motor_session_raw_active(int ch)
{
    can_motor_client_t owner=can_console_motor_session_owner(ch);
    return owner==CAN_MOTOR_CLIENT_RAW || owner==CAN_MOTOR_CLIENT_RAW_ESTOP;
}
void can_console_motor_session_halt_raw(int ch)
{
    if(!gate || !valid(ch))return;
    xSemaphoreTake(gate,portMAX_DELAY);
    if(owners[ch]==CAN_MOTOR_CLIENT_RAW)owners[ch]=CAN_MOTOR_CLIENT_RAW_ESTOP;
    xSemaphoreGive(gate);
}
