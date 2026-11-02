#include "rb_daily.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

const rb_daily_action_t *rb_daily_find(unsigned id)
{
    for (unsigned i=0;i<8;++i) if (rb_daily_actions[i].id==id) return &rb_daily_actions[i];
    return NULL;
}
const rb_daily_action_t *rb_daily_for_screen(unsigned action,unsigned pose)
{ return rb_daily_find(action==RB_DAILY_IDLE && pose==2 ? RB_DAILY_SLEEP:action); }
bool rb_daily_sample(unsigned id,float seconds,float q[7],float v[7],float a[7])
{
    const rb_daily_action_t *action=rb_daily_find(id);
    if (!action || !q || !v || !a || !isfinite(seconds) || seconds<0) return false;
    if (!action->piece_count) {
        memcpy(q,action->anchor_rad,7*sizeof(float));
        memset(v,0,7*sizeof(float));memset(a,0,7*sizeof(float));return true;
    }
    float t=fminf(seconds,action->duration_s);
    unsigned lo=0,hi=action->piece_count;
    while (lo+1<hi) {
        unsigned mid=(lo+hi)/2;
        if (action->pieces[mid].begin_s<=t) lo=mid; else hi=mid;
    }
    const rb_daily_piece_t *p=&action->pieces[lo];
    float dt=p->end_s-p->begin_s;
    if (!(dt>0)) return false;
    float u=fmaxf(0,fminf(1,(t-p->begin_s)/dt));
    for (unsigned j=0;j<7;++j) {
        const float *c=p->coefficient[j];
        q[j]=c[0]+u*(c[1]+u*(c[2]+u*(c[3]+u*(c[4]+u*c[5]))));
        v[j]=(c[1]+u*(2*c[2]+u*(3*c[3]+u*(4*c[4]+u*5*c[5]))))/dt;
        a[j]=(2*c[2]+u*(6*c[3]+u*(12*c[4]+u*20*c[5])))/(dt*dt);
        if (seconds==0 || seconds>=action->duration_s) v[j]=a[j]=0;
    }
    return true;
}
uint16_t rb_daily_clip_time(unsigned id,float phase)
{
    const rb_daily_action_t *a=rb_daily_find(id);
    if (!a || !isfinite(phase)) return 0;
    phase=fmaxf(0,fminf(1,phase));
    if (a->sync_count<2) return (uint16_t)(phase*a->clip_duration_ms);
    float t=phase*a->duration_s;
    for (unsigned i=1;i<a->sync_count;++i) {
        const rb_daily_sync_t *left=&a->sync[i-1],*right=&a->sync[i];
        if (t<=right->motion_s) {
            float span=right->motion_s-left->motion_s;
            float f=span>0?(t-left->motion_s)/span:1;
            return (uint16_t)fmaxf(0,fminf(a->clip_duration_ms,left->clip_ms+f*(right->clip_ms-left->clip_ms)));
        }
    }
    return a->clip_duration_ms;
}
