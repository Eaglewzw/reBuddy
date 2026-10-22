#include "rb_core.h"
#include <math.h>

static float clamp(float v, float lo, float hi) { return fmaxf(lo, fminf(hi, v)); }
static uint32_t random_next(rb_life_t *l)
{ l->random=l->random*1664525u+1013904223u; return l->random; }
static float approach(float value,float target,float dt,float rate)
{ return value+(target-value)*(1.0f-expf(-dt*rate)); }
void rb_life_tick(rb_engine_t *e,uint64_t now,uint64_t elapsed)
{
    rb_life_t *l=&e->life;
    float dt=(float)elapsed*.001f;
    if (e->state==RB_STOPPED) {
        l->contact=l->stroking=l->blink=false; l->lean=l->lift=0;
        l->face=RB_FACE_STOP; l->openness=1; return;
    }
    bool sleeping=e->state==RB_SLEEPING;
    bool dozing=sleeping || e->state==RB_DROWSY || e->action==RB_DOZE;
    if (now>=l->stroke_until) l->stroking=false;
    bool gentle=l->contact && l->region==RB_REGION_HEAD && now>=l->contact_ms+250 && now>=l->rough_until;
    bool calm=gentle && now>=l->puzzled_until;
    float style=e->style==RB_LAZY?.7f:1.0f;
    l->comfort=clamp(l->comfort+dt*(calm?(l->stroking?.28f:.12f):-.065f),0,1);
    l->excitement=clamp(l->excitement+dt*(l->contact && l->region==RB_REGION_CLAW?.25f:-.10f),0,1);
    l->disturbance=clamp(l->disturbance-dt*(calm?.38f:.12f),0,1);
    if (!l->contact && now>l->release_ms+1500 && !dozing) {
        if (now>=l->next_look_ms) {
            l->target_x=((int)(random_next(l)%201)-100)/100.0f;
            l->target_y=((int)(random_next(l)%101)-50)/100.0f;
            l->next_look_ms=now+2500+random_next(l)%4500+(e->style==RB_LAZY?2500:0);
        }
    }
    float tx=dozing?0:l->target_x, ty=dozing?.3f:l->target_y;
    if (l->disturbance>.55f) tx=-tx*.7f;
    l->gaze_x=approach(l->gaze_x,tx,dt,7*style);
    l->gaze_y=approach(l->gaze_y,ty,dt,7*style);
    if (now>=l->next_blink_ms) {
        l->blink_until=now+130;
        l->next_blink_ms=now+2200+random_next(l)%3500;
    }
    l->blink=now<l->blink_until;
    if (l->comfort>.8f && l->disturbance<.2f && !l->contact && !dozing && now>=l->celebrate_after) {
        l->celebrate_until=now+1800; l->celebrate_after=now+25000;
    }
    l->face=RB_FACE_IDLE;
    if (dozing) l->face=sleeping?RB_FACE_SLEEP:RB_FACE_DROWSY;
    else if (l->disturbance>.55f) l->face=RB_FACE_SHY;
    else if (now<l->puzzled_until) l->face=RB_FACE_PUZZLED;
    else if (now<l->celebrate_until || l->excitement>.55f) l->face=RB_FACE_HAPPY;
    else if (l->comfort>.3f) l->face=RB_FACE_RELAXED;
    else if (l->contact || (l->release_ms && now<l->release_ms+1500) || now<l->startled_until) l->face=RB_FACE_CURIOUS;
    else if (e->action!=RB_NONE) l->face=e->face;
    float open=sleeping?.08f:dozing?.25f:l->comfort>.3f?.45f:1.0f;
    l->openness=approach(l->openness,open,dt,e->state==RB_WAKING?1.8f:7);
    float wave=sinf((float)(now%60000)*.0018f*style);
    float lean=dozing?0:l->gaze_x*(l->contact?3.0f:1.2f);
    if (calm) lean+=sinf((float)(now%60000)*.004f)*l->comfort*2;
    if (l->disturbance>.55f) lean-=3;
    float lift=sleeping?2:wave*(e->style==RB_LAZY?.65f:1.1f);
    if (now<l->startled_until) lift+=1.8f;
    if (now<l->celebrate_until && !dozing) {
        lean+=sinf((float)(now%60000)*.014f)*3;
        lift-=fabsf(sinf((float)(now%60000)*.014f))*3;
    }
    if (l->region==RB_REGION_CLAW && l->contact) lift-=l->excitement*1.5f;
    l->lean=approach(l->lean,lean,dt,5);
    l->lift=approach(l->lift,lift,dt,5);
}
