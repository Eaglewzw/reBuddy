#include "rb_core.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

void rb_gesture_init(rb_gesture_t *g) { if (g) memset(g,0,sizeof(*g)); }
void rb_gesture_cancel(rb_gesture_t *g,rb_engine_t *e)
{
    rb_gesture_init(g);
    if (e) {
        e->life.contact=e->life.stroking=false;
        e->life.release_ms=e->now_ms;
    }
}
void rb_gesture_tick(rb_gesture_t *g,rb_engine_t *e,uint64_t now)
{
    if (!g || !e) return;
    if (e->state==RB_STOPPED || (g->down && now>=g->last_sample_ms && now-g->last_sample_ms>1500))
        rb_gesture_cancel(g,e);
}
void rb_gesture_touch_region(rb_gesture_t *g,rb_engine_t *e,rb_touch_phase_t phase,
                             int x,int y,rb_region_t region,uint64_t now)
{
    if (!g || !e || now<e->now_ms) return;
    if (phase==RB_TOUCH_CANCEL || e->state==RB_STOPPED || x<0 || x>=320 || y<0 || y>=240) {
        rb_gesture_cancel(g,e); return;
    }
    rb_life_t *l=&e->life;
    if (phase==RB_TOUCH_DOWN) {
        rb_gesture_tick(g,e,now);
        g->down=true; g->x=x; g->y=y; g->anchor_x=x; g->path=0;
        g->direction=g->reversals=0; g->started_ms=g->last_sample_ms=now;
        l->contact_ms=now; l->stroking=false;
        if (region!=RB_REGION_NEAR) {
            if (now-l->last_poke_ms>1400) l->pokes=0;
            l->last_poke_ms=now;
            if (l->pokes<8) ++l->pokes;
            if (l->pokes>=3) l->disturbance=fminf(1,l->disturbance+.32f);
            if (region==RB_REGION_BODY || region==RB_REGION_CLAW) l->startled_until=now+450;
        }
    } else {
        if (!g->down || now<g->last_sample_ms) return;
        unsigned travel=(unsigned)(abs(x-g->x)+abs(y-g->y));
        uint64_t dt=now-g->last_sample_ms;
        g->path+=travel;
        /* A fast segment must not remain a gentle stroke on stationary samples. */
        if (travel) {
            l->stroking=dt>0 && travel*1000ULL<=dt*350ULL;
            if (l->stroking) l->stroke_until=now+180;
            else l->rough_until=now+400;
        }
        int dx=x-g->anchor_x;
        if (abs(dx)>=22) {
            int direction=dx>0?1:-1;
            if (g->direction && direction!=g->direction) ++g->reversals;
            g->direction=direction; g->anchor_x=x;
        }
        if (g->reversals && g->path>=70 && now-g->started_ms<=800) {
            l->puzzled_until=now+1600; l->stroking=false;
            l->disturbance=fminf(1,l->disturbance+.08f);
            g->reversals=0; g->path=0;
        }
        /* Restart the reversal window so long contacts can still become fast play. */
        if (now-g->started_ms>800) {
            g->started_ms=now; g->path=0; g->reversals=0; g->direction=0; g->anchor_x=x;
        }
        g->x=x; g->y=y; g->last_sample_ms=now;
    }
    if (l->region!=region) { l->contact_ms=now; l->stroking=false; }
    l->region=region;
    l->contact=phase!=RB_TOUCH_UP;
    if (l->contact && region!=RB_REGION_NEAR &&
        (e->state==RB_SLEEPING || e->state==RB_DROWSY || e->action==RB_DOZE))
        (void)rb_engine_event(e,RB_WAKE,now);
    l->target_x=fmaxf(-1,fminf(1,(x-200)/105.0f));
    l->target_y=fmaxf(-1,fminf(1,(y-85)/95.0f));
    if (e->state!=RB_SLEEPING && e->state!=RB_DROWSY && e->action!=RB_DOZE) e->last_input_ms=now;
    if (phase==RB_TOUCH_UP) {
        g->down=false; l->stroking=false; l->release_ms=now;
    }
}
void rb_gesture_touch(rb_gesture_t *g,rb_engine_t *e,rb_touch_phase_t phase,int x,int y,uint64_t now)
{ rb_gesture_touch_region(g,e,phase,x,y,RB_REGION_BODY,now); }
