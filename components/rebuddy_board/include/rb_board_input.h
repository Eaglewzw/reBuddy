// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <stdbool.h>
#include <stdint.h>

/* Native GT911 portrait coordinates -> fixed 320x240 landscape. */
bool rb_touch_map(uint16_t raw_x, uint16_t raw_y, int16_t *x, int16_t *y);
typedef struct { bool active, blocked; uint8_t id; } rb_primary_touch_t;
/* Index of original contact; -1 released, -2 cancelled until all contacts lift. */
int rb_primary_touch_select(rb_primary_touch_t *tracker, const uint8_t *ids, unsigned count, bool valid);
