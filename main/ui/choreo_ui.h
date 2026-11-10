#pragma once
#include "lvgl.h"
#include <stdbool.h>
#include <stdint.h>

/* Caller suspends Daily before open. Explicit motor playback is delegated to
 * the registered choreo_motion backend; screen previews never request motors. */
void rb_choreo_ui_open(lv_obj_t *parent,void (*on_exit)(void *),void *context,uint64_t now);
bool rb_choreo_ui_active(void);
void rb_choreo_ui_update(uint64_t now);
void rb_choreo_ui_fault(void);
