#pragma once
#include "lvgl.h"
#include <stdbool.h>
#include <stdint.h>

/* Independent screen-only editor. Caller suspends daily behavior before open,
 * and resumes it from on_exit. No pet/motor commands are emitted here. */
void rb_choreo_ui_open(lv_obj_t *parent,void (*on_exit)(void *),void *context,uint64_t now);
bool rb_choreo_ui_active(void);
void rb_choreo_ui_update(uint64_t now);
void rb_choreo_ui_fault(void);
