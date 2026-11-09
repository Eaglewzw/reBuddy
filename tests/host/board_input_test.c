// SPDX-License-Identifier: Apache-2.0
#include "rb_board_input.h"
#include <assert.h>
#include <stdio.h>
int main(void)
{
    rb_enter_key_t key={0};
    /* Held at boot is not a new press. */
    assert(!rb_enter_key_update(&key,true,true,0));
    assert(!rb_enter_key_update(&key,true,true,100));
    assert(!rb_enter_key_update(&key,false,true,120));
    assert(!rb_enter_key_update(&key,false,true,160));
    /* Short bounce cannot trigger HOME. */
    assert(!rb_enter_key_update(&key,true,true,180));
    assert(!rb_enter_key_update(&key,false,true,190));
    assert(!rb_enter_key_update(&key,true,true,200));
    assert(!rb_enter_key_update(&key,true,true,220));
    assert(rb_enter_key_update(&key,true,true,240));
    for(unsigned t=260;t<1000;t+=20)assert(!rb_enter_key_update(&key,true,true,t));
    /* I2C failure and recovery while held cannot generate a phantom press. */
    assert(!rb_enter_key_update(&key,false,false,1000));
    assert(!rb_enter_key_update(&key,true,true,1020));
    assert(!rb_enter_key_update(&key,true,true,1060));
    assert(!rb_enter_key_update(&key,false,true,1080));
    assert(!rb_enter_key_update(&key,false,true,1120));
    assert(!rb_enter_key_update(&key,true,true,1140));
    assert(rb_enter_key_update(&key,true,true,1180));
    assert(!rb_enter_key_update(&key,true,true,1200));
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
    assert(rb_touch_map(0, 0, &x, &y) && x == 319 && y == 0);
    assert(rb_touch_map(239, 319, &x, &y) && x == 0 && y == 239);
    assert(!rb_touch_map(240, 100, &x, &y));
    assert(!rb_touch_map(0, 320, &x, &y));
    assert(!rb_touch_map(0, 0, NULL, &y));
    for (unsigned rx = 0; rx < 240; ++rx) for (unsigned ry = 0; ry < 320; ++ry) {
        assert(rb_touch_map(rx, ry, &x, &y));
        assert(x >= 0 && x < 320 && y >= 0 && y < 240);
        assert(rx == (unsigned)y && ry == (unsigned)(319 - x));
    }
    puts("PASS touch mapping/tracking and physical ENTER debounce, hold, boot and I2C-error handling");
}
