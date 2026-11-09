#include "app_screen.h"
#include "app_arm.h"
#include "app_diagnostics.h"
#include "pet_ui.h"
#include "pet_assets.h"
#include "choreo_store.h"
#include "rb_board.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_timer.h"

static rb_pet_t *pet;
extern const uint8_t frames_start[] asm("_binary_frames_bin_start");
extern const uint8_t frames_end[] asm("_binary_frames_bin_end");
extern const uint8_t room_start[] asm("_binary_room_bin_start");
extern const uint8_t room_end[] asm("_binary_room_bin_end");
uint64_t rb_platform_now_ms(void) { return (uint64_t)esp_timer_get_time()/1000; }
static void update(lv_timer_t *timer)
{
    (void)timer;
    /* Keep gesture accumulation matched to the 20ms touch poll. Only the
       choreography/display clock runs at 10ms, so 50ms frames have no 40/60ms
       scheduling alternation. Running the gesture sampler at 10ms would
       undercount actual movement when the touch position updates at 20ms. */
    static uint64_t last_pet_tick;
    uint64_t now=rb_platform_now_ms();
    if (now-last_pet_tick>=20) { rb_pet_tick(pet,now);last_pet_tick=now; }
    rb_app_arm_tick(now);
    rb_ui_update();
    static bool was_stopped;
    bool stopped=pet->state==RB_PET_STOPPED;
    if (stopped && !was_stopped)
        ESP_LOGW("rb_state","Screen paused: %s",pet->stop_reason?pet->stop_reason:"unspecified");
    else if (!stopped && was_stopped) ESP_LOGI("rb_state","Screen interaction resumed");
    was_stopped=stopped;
}
size_t rb_app_animation_bytes(void)
{
    return (size_t)(frames_end-frames_start);
}

bool rb_app_screen_start(rb_pet_t *state)
{
    pet=state;
    rb_pet_init_sleep(pet,rb_platform_now_ms());
#ifdef CONFIG_REBUDDY_ARM_DAILY_CONTROL
    pet->motion_clock=true;
#endif
    rb_pet_use_daily_timing(pet,true);rb_pet_set_sink(pet,rb_app_arm_action,NULL);
    if (rb_choreo_store_init()!=RB_CHOREO_STORE_OK)
        ESP_LOGW("reBuddy","Choreography storage unavailable; existing NVS data preserved");
    if (!rb_board_display_lock(0)) return false;
    bool assets_ok=rb_assets_init(frames_start,(size_t)(frames_end-frames_start),2);
    if (!rb_assets_set_room(room_start,(size_t)(room_end-room_start)))
        ESP_LOGW("reBuddy","Room resource invalid; plain-background fallback");
    if (!assets_ok || rb_assets_degraded()) ESP_LOGW("reBuddy","Animation resources degraded; idle/placeholder fallback");
    rb_ui_create(pet);
    lv_timer_create(update,10,NULL); lv_timer_create(rb_app_log_animation,5000,NULL);
    rb_board_display_unlock();
    return true;
}
