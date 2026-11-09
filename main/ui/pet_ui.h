#pragma once
#include "rb_pet.h"
void rb_ui_create(rb_pet_t *pet);
void rb_ui_update(void);
unsigned rb_ui_body_updates(void);
uint64_t rb_ui_last_feedback_ms(void);
uint64_t rb_platform_now_ms(void);
/* Optional physical-arm control; NULL keeps the screen-only UI unchanged. */
void rb_ui_set_arm_control(void (*command)(void));
void rb_ui_arm_status(const char *status,bool active);
