// SPDX-License-Identifier: Apache-2.0
#include "rb_board_input.h"
#define UP (1u << 2)
#define DOWN (1u << 3)
#define ENTER (1u << 6)
#define BACK (1u << 7)
#define KEYS (UP | DOWN | ENTER | BACK)

rb_board_input_t rb_keys_sample(rb_keys_t *keys, uint16_t levels, bool valid, uint64_t now)
{
    rb_board_input_t event = {0};
    if (!valid) {
        keys->initialized = false;
        event.fault = true;
        return event;
    }
    uint16_t pressed = (uint16_t)(~levels) & KEYS;
    /* STOP is level-sensitive; debounce must not delay it. */
    event.back = (pressed & BACK) != 0;
    if (!keys->initialized) {
        keys->initialized = true;
        keys->candidate = keys->stable = pressed;
        keys->changed_ms = now;
        return event;
    }
    if (pressed != keys->candidate || now < keys->changed_ms) {
        keys->candidate = pressed;
        keys->changed_ms = now;
    }
    if (now - keys->changed_ms < 20) return event;
    uint16_t rising = pressed & (uint16_t)~keys->stable;
    keys->stable = pressed;
    if (!event.back) {
        event.enter = (rising & ENTER) != 0;
        if ((pressed & (UP | DOWN)) == UP && (rising & UP)) event.dial = 1;
        if ((pressed & (UP | DOWN)) == DOWN && (rising & DOWN)) event.dial = -1;
    }
    return event;
}

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
