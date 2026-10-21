#include "rb_core.h"
#include <stdlib.h>
#include <string.h>
void rb_gesture_init(rb_gesture_t *g) { if (g) memset(g,0,sizeof(*g)); }
void rb_gesture_tick(rb_gesture_t *g, rb_engine_t *e, uint64_t now)
{
    if (!g || !e) return;
    if (e->state==RB_STOPPED) { rb_gesture_init(g); return; }
    if (g->tap_pending && !g->down && now>=g->last_tap_ms && now-g->last_tap_ms>=350) {
        (void)rb_engine_event(e,g->taps>=2?RB_MULTI_TAP:RB_TAP,now);
        g->tap_pending=false; g->taps=0;
    }
    if (g->down && now>=g->last_sample_ms && now-g->last_sample_ms>1500) rb_gesture_init(g);
}
void rb_gesture_touch(rb_gesture_t *g, rb_engine_t *e, rb_touch_phase_t phase, int x, int y, uint64_t now)
{
    if (!g || !e || now<e->now_ms) return;
    if (phase==RB_TOUCH_CANCEL || e->state==RB_STOPPED) { rb_gesture_init(g); return; }
    if (x<0 || x>=320 || y<0 || y>=240) { rb_gesture_init(g); return; }
    if (phase==RB_TOUCH_DOWN) {
        rb_gesture_tick(g,e,now);
        g->down=true; g->x=x; g->y=y; g->anchor_x=x; g->path=0;
        g->direction=g->reversals=0; g->started_ms=g->last_sample_ms=now;
        g->wake_only=e->state==RB_SLEEPING || e->state==RB_DROWSY;
        if (g->wake_only) { g->tap_pending=false; g->taps=0; (void)rb_engine_event(e,RB_WAKE,now); }
        e->last_input_ms=now;
        return;
    }
    if (!g->down || now<g->started_ms) return;
    int dx=x-g->x, dy=y-g->y;
    g->path+=(unsigned)(abs(dx)+abs(dy));
    int travel=x-g->anchor_x;
    if (abs(travel)>=22) {
        int direction=travel>0?1:-1;
        if (g->direction && direction!=g->direction) ++g->reversals;
        g->direction=direction; g->anchor_x=x;
    }
    g->x=x; g->y=y; g->last_sample_ms=now; e->last_input_ms=now;
    if (phase!=RB_TOUCH_UP) return;
    g->down=false;
    if (g->wake_only) return;
    uint64_t duration=now-g->started_ms;
    if (g->reversals>=1 && g->path>=70 && duration<=800) {
        g->tap_pending=false; g->taps=0; (void)rb_engine_event(e,RB_ZIGZAG,now);
    } else if (g->path>=45 && duration>=400 && duration<=5000 && g->path*1000ULL<=duration*450ULL) {
        g->tap_pending=false; g->taps=0; (void)rb_engine_event(e,RB_STROKE,now);
    } else if (g->path<=15 && duration<=500) {
        if (!g->tap_pending || now-g->last_tap_ms>350) g->taps=0;
        if (g->taps<3) ++g->taps;
        g->tap_pending=true; g->last_tap_ms=now;
        if (g->taps>=3) { g->tap_pending=false; g->taps=0; (void)rb_engine_event(e,RB_MULTI_TAP,now); }
    }
}
