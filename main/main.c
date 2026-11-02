#include "pet_ui.h"
#include "pet_assets.h"
#include "choreo_store.h"
#include "rb_board.h"
#include "rb_arm.h"
#include "rb_daily.h"
#include "rb_arm_daily.h"
#include "rb_daily_screen.h"
#include <string.h>
#include "sdkconfig.h"
#include "rb_memory.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_flash.h"
#include "esp_partition.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static rb_pet_t pet;
#ifdef CONFIG_REBUDDY_ARM_DAILY_CONTROL
static bool arm_linked;
static void daily_action(void *,const rb_pet_event_t *);
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
    unsigned requests=pet.requests;
    rb_pet_init_sleep(&pet,rb_platform_now_ms());pet.requests=requests;
    rb_pet_use_daily_timing(&pet,true);pet.motion_clock=true;
    rb_pet_set_sink(&pet,daily_action,NULL);
}
static void arm_button_command(void)
{
    rb_dc_status_t s;
    if(rb_arm_daily_status(&s)!=ESP_OK)return;
    if(s.state==RB_DC_OFF || (s.state==RB_DC_FAULT && !s.enabled)) {
        arm_linked=false;reset_sleep_screen();
    } else {
        rb_arm_daily_home();rb_pet_stop(&pet,"ARM HOMING");
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
        rb_pet_stop(&pet,s.error?s.error:"ARM FAULT");pet.stop_reason=s.error?s.error:"ARM FAULT";return;
    }
    if(s.state==RB_DC_HOME || s.state==RB_DC_HOME_SETTLE) {rb_pet_stop(&pet,"ARM HOMING");return;}
    if(s.state==RB_DC_OFF && s.disable_confirmed && (s.action!=RB_DAILY_SLEEP ||
        (pet.state==RB_PET_STOPPED && pet.stop_reason && !strcmp(pet.stop_reason,"ARM HOMING")))) {
        arm_linked=false;reset_sleep_screen();return;
    }
    if(pet.suspended) {rb_arm_daily_home();return;}
    if(pet.state==RB_PET_STOPPED && pet.stop_reason && !strcmp(pet.stop_reason,"SCREEN STOP")) {
        rb_arm_daily_pause();return;
    }
    if(rb_daily_sync_screen(&pet,&s))
        (void)rb_arm_daily_request(RB_DAILY_SLEEP,s.request_id|0x40000000u);
}
#endif
static void arm_log(void)
{
    rb_arm_status_t status;
    esp_err_t error=rb_arm_get_status(&status);
    ESP_LOGI("rb_arm","state=%u planner=%d CAN_started=%d CAN_healthy=%d feedback=0x%02x error=%s",
             (unsigned)status.state,status.planner_ready,status.bus_started,status.bus_healthy,
             status.feedback_valid_mask,esp_err_to_name(error));
#ifdef CONFIG_REBUDDY_ARM_DAILY_CONTROL
    /* Runs on the low-priority application task, outside the control mutex.
     * Keep a late-attached serial monitor useful even without a state change. */
    rb_dc_status_t daily;
    if(rb_arm_daily_status(&daily)==ESP_OK) {
        const rb_daily_action_t *action=rb_daily_find(daily.action);
        ESP_LOGI("rb_daily","snapshot state=%s action=%s request=%u completed=%u enabled=%d disable_confirmed=%d motor=%u detail=%s stop_detail=%s",
            rb_daily_control_state_name(daily.state),action?action->name:"unknown",
            daily.request_id,daily.completed_id,daily.enabled,daily.disable_confirmed,
            daily.bad_motor,daily.error?daily.error:"ok",daily.stop_error?daily.stop_error:"none");
        if(daily.error && daily.hold.valid) {
            const rb_dc_hold_snapshot_t *h=&daily.hold;
            ESP_LOGW("rb_daily","hold trigger at=%llu ms from=%s torque_checked=%d; following values are latched, not live",
                (unsigned long long)h->at_ms,rb_daily_control_state_name(h->source_state),h->torque_checked);
            for(unsigned j=0;j<7;++j)
                ESP_LOGW("rb_daily","trigger J%u measured=%.3f target=%.3f error=%.3f deg velocity=%.3f deg/s estimated_torque=%.3f budget=%.3f Nm",
                    j+1,h->measured[j]*57.2957795f,h->target[j]*57.2957795f,
                    (h->measured[j]-h->target[j])*57.2957795f,h->velocity[j]*57.2957795f,
                    h->estimated_torque[j],rb_daily_policy.torque_budget_nm[j]);
        }
    }
#endif
}
extern const uint8_t frames_start[] asm("_binary_frames_bin_start");
extern const uint8_t frames_end[] asm("_binary_frames_bin_end");
extern const uint8_t room_start[] asm("_binary_room_bin_start");
extern const uint8_t room_end[] asm("_binary_room_bin_end");
uint64_t rb_platform_now_ms(void) { return (uint64_t)esp_timer_get_time()/1000; }
static void daily_action(void *context,const rb_pet_event_t *event)
{
    (void)context;
    const rb_daily_action_t *action=rb_daily_find(event->action_id);
    if (!action) { ESP_LOGE("rb_daily","Unmapped daily action %u",(unsigned)event->action_id);return; }
#ifdef CONFIG_REBUDDY_ARM_DAILY_CONTROL
    if(arm_linked || action->id==RB_DAILY_WAKE) {
        arm_linked=true;
        esp_err_t e=rb_arm_daily_request(action->id,event->request_id);
        if(e!=ESP_OK) {rb_arm_daily_pause();rb_pet_stop(&pet,"ARM REQUEST REJECTED");}
        return;
    }
    rb_pet_stop(&pet,"WAKE REQUIRED");return;
#endif
    ESP_LOGI("rb_daily","action=%s gif=%s request=%lu duration=%lums pieces=%u; mapped, motor output disconnected",
        action->name,action->gif_file,(unsigned long)event->request_id,
        (unsigned long)action->duration_ms,(unsigned)action->piece_count);
}
static void update(lv_timer_t *timer)
{
    (void)timer;
    /* Keep gesture accumulation matched to the 20ms touch poll. Only the
       choreography/display clock runs at 10ms, so 50ms frames have no 40/60ms
       scheduling alternation. Running the gesture sampler at 10ms would
       undercount actual movement when the touch position updates at 20ms. */
    static uint64_t last_pet_tick;
    uint64_t now=rb_platform_now_ms();
    if (now-last_pet_tick>=20) { rb_pet_tick(&pet,now);last_pet_tick=now; }
#ifdef CONFIG_REBUDDY_ARM_DAILY_CONTROL
    follow_arm(now);
#endif
    rb_ui_update();
    static bool was_stopped;
    bool stopped=pet.state==RB_PET_STOPPED;
    if (stopped && !was_stopped)
        ESP_LOGW("rb_state","Screen paused: %s",pet.stop_reason?pet.stop_reason:"unspecified");
    else if (!stopped && was_stopped) ESP_LOGI("rb_state","Screen interaction resumed");
    was_stopped=stopped;
}
static void animation_log(lv_timer_t *timer)
{
    (void)timer;
    static unsigned previous;
    static uint64_t last;
    uint64_t now=rb_platform_now_ms(); unsigned updates=rb_ui_body_updates();
    if (last && now>last) ESP_LOGI("rb_animation","body_changes=%.1f/s feedback_to_ui=%llums cache=%u degraded=%d",
        (updates-previous)*1000.0/(now-last),(unsigned long long)rb_ui_last_feedback_ms(),
        (unsigned)rb_assets_cache_bytes(),rb_assets_degraded());
    previous=updates; last=now;
    rb_asset_stats_t stats=rb_assets_stats();
    ESP_LOGI("rb_decode","frames=%lu mean=%.2fms max=%.2fms failures=%lu",
        (unsigned long)stats.frames,stats.frames?stats.total_us/1000.0/stats.frames:0,
        stats.max_us/1000.0,(unsigned long)stats.failures);
}
void app_main(void)
{
    ESP_ERROR_CHECK(rb_board_init());
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
    uint32_t flash_size=0;
    if (esp_flash_get_size(NULL,&flash_size)==ESP_OK) ESP_LOGI("reBuddy","Detected Flash=%lu bytes",(unsigned long)flash_size);
    unsigned ota_slots=0;
    esp_partition_iterator_t it=esp_partition_find(ESP_PARTITION_TYPE_APP,ESP_PARTITION_SUBTYPE_ANY,NULL);
    while (it) {
        const esp_partition_t *part=esp_partition_get(it);
        ESP_LOGI("reBuddy","App partition %s offset=0x%lx size=%lu",part->label,(unsigned long)part->address,(unsigned long)part->size);
        if (part->subtype>=ESP_PARTITION_SUBTYPE_APP_OTA_0 && part->subtype<=ESP_PARTITION_SUBTYPE_APP_OTA_15) ++ota_slots;
        it=esp_partition_next(it);
    }
    ESP_LOGI("reBuddy","OTA slots=%u; animation blob=%u bytes",ota_slots,(unsigned)(frames_end-frames_start));
    rb_memory_log();
    rb_pet_init_sleep(&pet,rb_platform_now_ms());
#ifdef CONFIG_REBUDDY_ARM_DAILY_CONTROL
    pet.motion_clock=true;
#endif
    rb_pet_use_daily_timing(&pet,true);rb_pet_set_sink(&pet,daily_action,NULL);
    if (rb_choreo_store_init()!=RB_CHOREO_STORE_OK)
        ESP_LOGW("reBuddy","Choreography storage unavailable; existing NVS data preserved");
    if (!rb_board_display_lock(0)) return;
    bool assets_ok=rb_assets_init(frames_start,(size_t)(frames_end-frames_start),2);
    if (!rb_assets_set_room(room_start,(size_t)(room_end-room_start)))
        ESP_LOGW("reBuddy","Room resource invalid; plain-background fallback");
    if (!assets_ok || rb_assets_degraded()) ESP_LOGW("reBuddy","Animation resources degraded; idle/placeholder fallback");
    rb_ui_create(&pet);
    lv_timer_create(update,10,NULL); lv_timer_create(animation_log,5000,NULL);
    rb_board_display_unlock();
#ifdef CONFIG_REBUDDY_ARM_DAILY_CONTROL
    if(arm_error==ESP_OK) {
        if(xTaskCreate(enter_key_worker,"enter_home",3072,NULL,2,NULL)!=pdPASS)
            ESP_LOGE("rb_input","ENTER input task unavailable; screen HOME remains available");
        else ESP_LOGI("rb_input","Physical ENTER: verified zero return then disable, same as HOME; does not wake");
    }
#endif
    ESP_LOGI("reBuddy","Screen starts asleep; click wake arrow to begin WAKE; motor output only in control builds");
    for (;;) { vTaskDelay(pdMS_TO_TICKS(30000)); rb_memory_log(); arm_log(); }
}
