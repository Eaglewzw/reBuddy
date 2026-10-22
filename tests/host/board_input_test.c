// SPDX-License-Identifier: Apache-2.0
#include "rb_board_input.h"
#include <assert.h>
#include <stdio.h>
int main(void)
{
    rb_primary_touch_t primary={0};
    const uint8_t one[]={4}, two[]={8,4}, remaining[]={8};
    assert(rb_primary_touch_select(&primary,one,1,true)==0);
    assert(rb_primary_touch_select(&primary,two,2,true)==1);
    assert(rb_primary_touch_select(&primary,remaining,1,true)==-2);
    assert(rb_primary_touch_select(&primary,two,2,true)==-2);
    assert(rb_primary_touch_select(&primary,NULL,0,true)==-1);
    assert(rb_primary_touch_select(&primary,remaining,1,true)==0);
    assert(rb_primary_touch_select(&primary,NULL,0,false)==-2);
    assert(rb_primary_touch_select(&primary,remaining,1,true)==-2);
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
    puts("PASS touch coordinate mapping, primary-contact tracking and error cancellation");
}
