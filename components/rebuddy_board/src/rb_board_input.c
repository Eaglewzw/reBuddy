// SPDX-License-Identifier: Apache-2.0
#include "rb_board_input.h"
bool rb_touch_map(uint16_t raw_x, uint16_t raw_y, int16_t *x, int16_t *y)
{
    if (!x || !y || raw_x >= 240 || raw_y >= 320) return false;
    *x = (int16_t)raw_y;
    *y = (int16_t)(239 - raw_x);
    return true;
}

int rb_primary_touch_select(rb_primary_touch_t *tracker, const uint8_t *ids, unsigned count, bool valid)
{
    if (!valid) { tracker->active=false; tracker->blocked=true; return -2; }
    if (!count) { *tracker=(rb_primary_touch_t){0}; return -1; }
    if (tracker->blocked) return -2;
    if (!tracker->active) { tracker->active=true; tracker->id=ids[0]; }
    for (unsigned i=0;i<count;++i) if (ids[i]==tracker->id) return (int)i;
    tracker->active=false; tracker->blocked=true;
    return -2;
}
