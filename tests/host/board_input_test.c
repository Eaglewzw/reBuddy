// SPDX-License-Identifier: Apache-2.0
#include "rb_board_input.h"
#include <assert.h>
#include <stdio.h>
#define RELEASED UINT16_MAX
#define HELD(bit) (uint16_t)(RELEASED ^ (1u << (bit)))
int main(void)
{
    int16_t x, y;
    assert(rb_touch_map(0, 0, &x, &y) && x == 0 && y == 239);
    assert(rb_touch_map(239, 319, &x, &y) && x == 319 && y == 0);
    assert(!rb_touch_map(240, 100, &x, &y));
    assert(!rb_touch_map(0, 320, &x, &y));
    assert(!rb_touch_map(0, 0, NULL, &y));
    for (unsigned rx = 0; rx < 240; ++rx) for (unsigned ry = 0; ry < 320; ++ry) {
        assert(rb_touch_map(rx, ry, &x, &y));
        assert(x >= 0 && x < 320 && y >= 0 && y < 240);
        assert(rx == (unsigned)(239 - y) && ry == (unsigned)x);
    }
    rb_keys_t keys = {0};
    assert(!rb_keys_sample(&keys, HELD(6), true, 0).enter); // Held at boot.
    assert(!rb_keys_sample(&keys, HELD(6), true, 40).enter);
    rb_keys_sample(&keys, RELEASED, true, 50);
    rb_keys_sample(&keys, RELEASED, true, 70);
    assert(!rb_keys_sample(&keys, HELD(6), true, 80).enter);
    rb_keys_sample(&keys, RELEASED, true, 90); // Contact bounce.
    assert(!rb_keys_sample(&keys, HELD(6), true, 100).enter);
    assert(!rb_keys_sample(&keys, HELD(6), true, 119).enter);
    assert(rb_keys_sample(&keys, HELD(6), true, 120).enter);
    assert(!rb_keys_sample(&keys, HELD(6), true, 140).enter);
    assert(rb_keys_sample(&keys, HELD(7), true, 141).back); // STOP has no debounce delay.
    rb_keys_sample(&keys, RELEASED, true, 160);
    rb_keys_sample(&keys, RELEASED, true, 180);
    rb_keys_sample(&keys, HELD(2), true, 200);
    assert(rb_keys_sample(&keys, HELD(2), true, 220).dial == 1);
    rb_keys_sample(&keys, HELD(3), true, 240);
    assert(rb_keys_sample(&keys, HELD(3), true, 260).dial == -1);
    uint16_t both = HELD(2) & HELD(3);
    rb_keys_sample(&keys, both, true, 280);
    assert(rb_keys_sample(&keys, both, true, 300).dial == 0);
    assert(rb_keys_sample(&keys, 0, false, 320).fault);
    assert(!rb_keys_sample(&keys, HELD(6), true, 340).enter); // No ghost event on reconnect.
    assert(!rb_keys_sample(&keys, HELD(6), true, 360).enter);
    keys = (rb_keys_t){0};
    rb_keys_sample(&keys, RELEASED, true, 0);
    uint16_t stop_and_enter = HELD(7) & HELD(6);
    rb_keys_sample(&keys, stop_and_enter, true, 20);
    rb_board_input_t event = rb_keys_sample(&keys, stop_and_enter, true, 40);
    assert(event.back && !event.enter && event.dial == 0);
    puts("PASS board coordinate mapping, debounce, stop priority and I2C fault recovery");
}
