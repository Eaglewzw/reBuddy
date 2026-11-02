#include "rb_arm_daily.h"
#include "rb_arm.h"
#include "can_console_motor_runtime.h"
#include "can_console_motor_session.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "esp_timer.h"
#include "esp_log.h"
static rb_daily_control_t control;
static SemaphoreHandle_t mutex;
static QueueHandle_t log_queue;
static bool ready;
static void log_worker(void *unused)
{
    (void)unused;rb_dc_status_t s;
    for(;;)if(xQueueReceive(log_queue,&s,portMAX_DELAY)==pdTRUE)
        ESP_LOGI("rb_daily","%s action=%u request=%u motor=%u detail=%s stop_detail=%s disable_confirmed=%d",
            rb_daily_control_state_name(s.state),s.action,s.request_id,s.bad_motor,
            s.error?s.error:"ok",s.stop_error?s.stop_error:"none",s.disable_confirmed);
}
static void worker(void *unused)
{
    (void)unused;
    TickType_t tick=xTaskGetTickCount();
    rb_dc_state_t previous=RB_DC_FAULT;
    const char *previous_error=NULL;
    const char *previous_stop_error=NULL;
    for(;;) {
        xSemaphoreTake(mutex,portMAX_DELAY);
        esp_err_t e=can_console_motor_tick(.02f);
        if(e!=ESP_OK && control.status.enabled)rb_daily_control_pause(&control);
        rb_daily_control_tick(&control,esp_timer_get_time()/1000);
        rb_dc_status_t snapshot=control.status;
        xSemaphoreGive(mutex);
        if(snapshot.state!=previous || snapshot.error!=previous_error || snapshot.stop_error!=previous_stop_error) {
            /* Logging must never wait for USB/UART in the 20ms control loop.
             * Coalesce blocked-reader transitions; trigger evidence is latched
             * in status and also emitted by the periodic application logger. */
            (void)xQueueOverwrite(log_queue,&snapshot);
            previous=snapshot.state;previous_error=snapshot.error;previous_stop_error=snapshot.stop_error;
        }
        vTaskDelayUntil(&tick,pdMS_TO_TICKS(20));
    }
}
esp_err_t rb_arm_daily_start(void)
{
    if(mutex)return ESP_ERR_INVALID_STATE;
    mutex=xSemaphoreCreateMutex();if(!mutex)return ESP_ERR_NO_MEM;
    esp_err_t e=can_console_motor_sessions_init();if(e!=ESP_OK)return e;
    e=rb_arm_init_control();if(e!=ESP_OK)return e;
    if(!can_console_motor_session_acquire(0,CAN_MOTOR_CLIENT_UI_NATIVE))return ESP_ERR_INVALID_STATE;
    e=rb_daily_control_init(&control,0);if(e!=ESP_OK)return e;
    log_queue=xQueueCreate(1,sizeof(rb_dc_status_t));if(!log_queue)return ESP_ERR_NO_MEM;
    TaskHandle_t logger=NULL;
    if(xTaskCreate(log_worker,"daily_log",4096,NULL,1,&logger)!=pdPASS) {
        vQueueDelete(log_queue);log_queue=NULL;return ESP_ERR_NO_MEM;
    }
    if(xTaskCreate(worker,"daily_arm",8192,NULL,7,NULL)!=pdPASS) {
        vTaskDelete(logger);vQueueDelete(log_queue);log_queue=NULL;return ESP_ERR_NO_MEM;
    }
    ready=true;return ESP_OK;
}
esp_err_t rb_arm_daily_request(unsigned action,unsigned request)
{
    if(!ready)return ESP_ERR_INVALID_STATE;
    if(xSemaphoreTake(mutex,pdMS_TO_TICKS(20))!=pdTRUE)return ESP_ERR_TIMEOUT;
    esp_err_t e=rb_daily_control_request(&control,action,request);
    xSemaphoreGive(mutex);return e;
}
esp_err_t rb_arm_daily_status(rb_dc_status_t *out)
{
    if(!ready || !out)return ESP_ERR_INVALID_STATE;
    if(xSemaphoreTake(mutex,0)!=pdTRUE)return ESP_ERR_TIMEOUT;
    *out=control.status;xSemaphoreGive(mutex);return ESP_OK;
}
void rb_arm_daily_home(void)
{
    if(!ready)return;
    xSemaphoreTake(mutex,portMAX_DELAY);rb_daily_control_home(&control);xSemaphoreGive(mutex);
}
void rb_arm_daily_pause(void)
{
    if(!ready)return;
    xSemaphoreTake(mutex,portMAX_DELAY);rb_daily_control_pause(&control);xSemaphoreGive(mutex);
}
void rb_arm_daily_heartbeat(uint64_t now)
{
    if(!ready)return;
    if(xSemaphoreTake(mutex,0)!=pdTRUE)return;
    rb_daily_control_heartbeat(&control,now);xSemaphoreGive(mutex);
}
