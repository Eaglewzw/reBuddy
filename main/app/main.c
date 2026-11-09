#include "app_arm.h"
#include "app_diagnostics.h"
#include "app_screen.h"
#include "rb_board.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static rb_pet_t pet;

void app_main(void)
{
    ESP_ERROR_CHECK(rb_board_init());
    rb_app_arm_init(&pet);
    rb_app_log_startup(rb_app_animation_bytes());

    if (!rb_app_screen_start(&pet)) return;
    rb_app_arm_start_input();
    ESP_LOGI("reBuddy","Screen starts asleep; click wake arrow to begin WAKE; motor output only in control builds");

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(30000));
        rb_app_log_health();
    }
}
