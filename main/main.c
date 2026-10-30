#include "pet_ui.h"
#include "pet_assets.h"
#include "choreo_store.h"
#include "rb_board.h"
#include "rb_arm.h"
#include "sdkconfig.h"
#include "rb_memory.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_flash.h"
#include "esp_partition.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static rb_pet_t pet;
static void arm_log(void)
{
    rb_arm_status_t status;
    esp_err_t error=rb_arm_get_status(&status);
    ESP_LOGI("rb_arm","state=%u planner=%d CAN_started=%d CAN_healthy=%d feedback=0x%02x error=%s; motion unavailable",
             (unsigned)status.state,status.planner_ready,status.bus_started,status.bus_healthy,
             status.feedback_valid_mask,esp_err_to_name(error));
}
extern const uint8_t frames_start[] asm("_binary_frames_bin_start");
extern const uint8_t frames_end[] asm("_binary_frames_bin_end");
extern const uint8_t room_start[] asm("_binary_room_bin_start");
extern const uint8_t room_end[] asm("_binary_room_bin_end");
uint64_t rb_platform_now_ms(void) { return (uint64_t)esp_timer_get_time()/1000; }
static void mock_action(void *context,const rb_pet_event_t *event)
{
    (void)context;
    ESP_LOGI("rb_mock","action=%s request=%lu source=%u target=%u style=%s timestamp=%llu; no commissioned arm action",
        rb_pet_action_name(event->action_id),(unsigned long)event->request_id,(unsigned)event->source,
        (unsigned)event->target_state,event->lazy?"lazy":"lively",(unsigned long long)event->timestamp_ms);
}
static void update(lv_timer_t *timer)
{
    (void)timer;
    /* Screen-only demo. No motor backend, key poller or GPIO-stop input. */
    /* Keep gesture accumulation matched to the 20ms touch poll. Only the
       choreography/display clock runs at 10ms, so 50ms frames have no 40/60ms
       scheduling alternation. Running the gesture sampler at 10ms would
       undercount actual movement when the touch position updates at 20ms. */
    static uint64_t last_pet_tick;
    uint64_t now=rb_platform_now_ms();
    if (now-last_pet_tick>=20) { rb_pet_tick(&pet,now);last_pet_tick=now; }
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
    bool monitor=false;
#ifdef CONFIG_REBUDDY_ARM_MONITOR
    monitor=true;
#endif
    esp_err_t arm_error=rb_arm_init(monitor);
    if (arm_error!=ESP_OK)
        ESP_LOGE("rb_arm","Component initialization failed: %s; motion unavailable",esp_err_to_name(arm_error));
    else ESP_LOGI("rb_arm","RS components ready, 6 joints + gripper; CAN1 %s; motion unavailable",
                  monitor?"listen-only":"standby");
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
    rb_pet_init(&pet,rb_platform_now_ms()); rb_pet_set_sink(&pet,mock_action,NULL);
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
    ESP_LOGI("reBuddy","Screen ready; arm components integrated, awaiting calibrated actions");
    for (;;) { vTaskDelay(pdMS_TO_TICKS(30000)); rb_memory_log(); arm_log(); }
}
