#include "rb_arm_daily.h"
#include "rb_arm.h"
#include "can_console_motor_runtime.h"
#include "can_console_motor_session.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "esp_log.h"
static rb_daily_control_t control;
static SemaphoreHandle_t mutex;
static bool ready;
static void worker(void *unused)
{
    (void)unused;
    TickType_t tick=xTaskGetTickCount();
    rb_dc_state_t previous=RB_DC_FAULT;
    const char *previous_error=NULL;
    for(;;) {
        xSemaphoreTake(mutex,portMAX_DELAY);
        esp_err_t e=can_console_motor_tick(.02f);
        if(e!=ESP_OK && control.status.enabled)rb_daily_control_pause(&control);
        rb_daily_control_tick(&control,esp_timer_get_time()/1000);
        rb_dc_status_t snapshot=control.status;
        xSemaphoreGive(mutex);
        if(snapshot.state!=previous || snapshot.error!=previous_error) {
            ESP_LOGI("rb_daily","%s action=%u request=%u motor=%u detail=%s disable_confirmed=%d",
                rb_daily_control_state_name(snapshot.state),snapshot.action,snapshot.request_id,
                snapshot.bad_motor,snapshot.error?snapshot.error:"ok",snapshot.disable_confirmed);
            previous=snapshot.state;previous_error=snapshot.error;
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
    if(xTaskCreate(worker,"daily_arm",8192,NULL,7,NULL)!=pdPASS)return ESP_ERR_NO_MEM;
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
