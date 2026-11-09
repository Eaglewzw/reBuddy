#pragma once
#include <stddef.h>
#include "lvgl.h"

void rb_app_log_startup(size_t animation_bytes);
void rb_app_log_animation(lv_timer_t *timer);
void rb_app_log_health(void);
