#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "lvgl.h"
#include "pet_animation_table.h"
bool rb_assets_init(const uint8_t *data,size_t bytes,unsigned cache_slots);
void rb_assets_close(void);
const lv_image_dsc_t *rb_assets_frame(unsigned frame,const rb_frame_meta_t **meta);
size_t rb_assets_cache_bytes(void);
bool rb_assets_degraded(void);
