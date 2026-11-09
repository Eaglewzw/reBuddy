#pragma once
#include "rb_pet.h"

/* Initialization never wakes the arm. Input starts only after UI setup. */
void rb_app_arm_init(rb_pet_t *state);
void rb_app_arm_action(void *context, const rb_pet_event_t *event);
void rb_app_arm_tick(uint64_t now);
void rb_app_arm_start_input(void);
