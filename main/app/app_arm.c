#include "app_arm.h"
#include "pet_ui.h"
#include "rb_board.h"
#include "rb_arm.h"
#include "rb_daily.h"
#include "rb_arm_daily.h"
#include "rb_daily_screen.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static rb_pet_t *pet;
static bool initialized;
#ifdef CONFIG_REBUDDY_ARM_DAILY_CONTROL
static bool arm_linked;

static void enter_key_worker(void *unused)
{
    (void)unused;rb_enter_key_t key={0};
    for(;;) {
        bool pressed=false;
        esp_err_t e=rb_board_read_enter(&pressed);
        if(rb_enter_key_update(&key,pressed,e==ESP_OK,rb_platform_now_ms()))
            rb_arm_daily_home();
        /* No LVGL access or motor I/O here. The existing serialized controller
         * handles HOME; input polling cannot block its 20ms control cycle. */
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
static void reset_sleep_screen(void)
{
    unsigned requests=pet->requests;
    rb_pet_init_sleep(pet,rb_platform_now_ms());pet->requests=requests;
    rb_pet_use_daily_timing(pet,true);pet->motion_clock=true;
    rb_pet_set_sink(pet,rb_app_arm_action,NULL);
}
static void arm_button_command(void)
{
    rb_dc_status_t s;
    if(rb_arm_daily_status(&s)!=ESP_OK)return;
    if(s.state==RB_DC_OFF || (s.state==RB_DC_FAULT && !s.enabled)) {
        arm_linked=false;reset_sleep_screen();
    } else {
        rb_arm_daily_home();rb_pet_stop(pet,"ARM HOMING");
    }
}
static void follow_arm(uint64_t now)
{
    rb_arm_daily_heartbeat(now);
    rb_dc_status_t s;
    if(rb_arm_daily_status(&s)!=ESP_OK)return;
    rb_ui_arm_status(rb_daily_control_state_name(s.state),s.state!=RB_DC_OFF);
    if(!arm_linked)return;
    if(s.state==RB_DC_FAULT || s.state==RB_DC_PROTECTIVE_HOLD || s.error) {
        rb_pet_stop(pet,s.error?s.error:"ARM FAULT");pet->stop_reason=s.error?s.error:"ARM FAULT";return;
    }
    if(s.state==RB_DC_HOME || s.state==RB_DC_HOME_SETTLE) {rb_pet_stop(pet,"ARM HOMING");return;}
    if(s.state==RB_DC_OFF && s.disable_confirmed && (s.action!=RB_DAILY_SLEEP ||
        (pet->state==RB_PET_STOPPED && pet->stop_reason && !strcmp(pet->stop_reason,"ARM HOMING")))) {
        arm_linked=false;reset_sleep_screen();return;
    }
    if(pet->suspended) {rb_arm_daily_home();return;}
    if(pet->state==RB_PET_STOPPED && pet->stop_reason && !strcmp(pet->stop_reason,"SCREEN STOP")) {
        rb_arm_daily_pause();return;
    }
    if(rb_daily_sync_screen(pet,&s))
        (void)rb_arm_daily_request(RB_DAILY_SLEEP,s.request_id|0x40000000u);
}
#endif
void rb_app_arm_action(void *context,const rb_pet_event_t *event)
{
    (void)context;
    const rb_daily_action_t *action=rb_daily_find(event->action_id);
    if (!action) { ESP_LOGE("rb_daily","Unmapped daily action %u",(unsigned)event->action_id);return; }
#ifdef CONFIG_REBUDDY_ARM_DAILY_CONTROL
    if(arm_linked || action->id==RB_DAILY_WAKE) {
        arm_linked=true;
        esp_err_t e=rb_arm_daily_request(action->id,event->request_id);
        if(e!=ESP_OK) {rb_arm_daily_pause();rb_pet_stop(pet,"ARM REQUEST REJECTED");}
        return;
    }
    rb_pet_stop(pet,"WAKE REQUIRED");return;
#endif
    ESP_LOGI("rb_daily","action=%s gif=%s request=%lu duration=%lums pieces=%u; mapped, motor output disconnected",
        action->name,action->gif_file,(unsigned long)event->request_id,
        (unsigned long)action->duration_ms,(unsigned)action->piece_count);
}
void rb_app_arm_init(rb_pet_t *state)
{
    pet=state;
#ifndef CONFIG_REBUDDY_ARM_DAILY_CONTROL
    bool monitor=false;
#ifdef CONFIG_REBUDDY_ARM_MONITOR
    monitor=true;
#endif
#endif
    esp_err_t arm_error;
#ifdef CONFIG_REBUDDY_ARM_DAILY_CONTROL
    arm_error=rb_arm_daily_start();
    if(arm_error==ESP_OK)rb_ui_set_arm_control(arm_button_command);
#else
    arm_error=rb_arm_init(monitor);
#endif
    if (arm_error!=ESP_OK)
        ESP_LOGE("rb_arm","Component initialization failed: %s; motion unavailable",esp_err_to_name(arm_error));
#ifdef CONFIG_REBUDDY_ARM_DAILY_CONTROL
    else ESP_LOGI("rb_arm","Native daily controller ready; motors not enabled; click wake arrow");
#else
    else ESP_LOGI("rb_arm","RS components ready, 6 joints + gripper; CAN1 %s; screen-only build",
                  monitor?"listen-only":"standby");
#endif
    initialized=arm_error==ESP_OK;
}

void rb_app_arm_tick(uint64_t now)
{
#ifdef CONFIG_REBUDDY_ARM_DAILY_CONTROL
    follow_arm(now);
#else
    (void)now;
#endif
}

void rb_app_arm_start_input(void)
{
#ifdef CONFIG_REBUDDY_ARM_DAILY_CONTROL
    if(initialized) {
        if(xTaskCreate(enter_key_worker,"enter_home",3072,NULL,2,NULL)!=pdPASS)
            ESP_LOGE("rb_input","ENTER input task unavailable; screen HOME remains available");
        else ESP_LOGI("rb_input","Physical ENTER: verified zero return then disable, same as HOME; does not wake");
    }
#endif
}
