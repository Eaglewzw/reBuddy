#include "pet_ui.h"
#include "lvgl.h"
#include "rb_arm.h"
#include "rb_board.h"
#include "rb_memory.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static rb_engine_t pet;
static rb_gesture_t gesture;
static rb_arm_t arm; /* No physical adapter attached in v0.1. */
uint64_t rb_platform_now_ms(void) { return (uint64_t)esp_timer_get_time()/1000; }
static void update(lv_timer_t *timer)
{
    (void)timer;
    uint64_t now=rb_platform_now_ms();
    bool released=rb_board_estop_released();
    if (!released) rb_engine_stop(&pet,"E-STOP");
    rb_board_input_t input;
    for (unsigned n=0;n<8 && rb_board_read_input(&input);++n) {
        if (input.fault) rb_engine_stop(&pet,"BOARD INPUT FAULT");
        else if (input.back) rb_engine_stop(&pet,"BACK BUTTON");
        else if (input.enter && pet.state!=RB_STOPPED) (void)rb_engine_event(&pet,RB_HAPPY,now);
        else if (input.dial) rb_engine_style(&pet,input.dial>0?RB_LIVELY:RB_LAZY);
    }
    rb_gesture_tick(&gesture,&pet,now);
    rb_engine_tick(&pet,now);
    (void)rb_arm_update(&arm,&pet,now,released);
    rb_ui_update();
}
void app_main(void)
{
    ESP_ERROR_CHECK(rb_board_init());
    esp_err_t ret=nvs_flash_init();
    if (ret!=ESP_OK) ESP_LOGW("reBuddy","NVS unavailable (%s); using session defaults",esp_err_to_name(ret));
    rb_config_t config=rb_config_default();
    config.drowsy_ms=CONFIG_REBUDDY_DROWSY_SECONDS*1000;
    config.sleep_ms=CONFIG_REBUDDY_SLEEP_SECONDS*1000;
    if (!rb_engine_init(&pet,&config,rb_platform_now_ms())) {
        ESP_LOGE("reBuddy","Invalid idle timers: sleep must exceed drowsy"); return;
    }
    rb_gesture_init(&gesture);
    if (!rb_board_display_lock(0)) return;
    rb_ui_create(&pet,&gesture);
    if (!rb_board_estop_released()) rb_engine_stop(&pet,"E-STOP");
    rb_ui_update();
    lv_timer_create(update,20,NULL);
    rb_board_display_unlock();
    ESP_LOGI("reBuddy","v0.1 screen demo ready: PREVIEW ONLY; no motor enable or RS motion commands");
    for (;;) { vTaskDelay(pdMS_TO_TICKS(30000)); rb_memory_log(); }
}
