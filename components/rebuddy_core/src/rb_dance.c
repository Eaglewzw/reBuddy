#include "rb_dance.h"
#include <math.h>
#include <string.h>
static bool polynomial(const float c[7][6],float t,float duration,float q[7],float v[7],float a[7])
{
    if(!q || !v || !a || !isfinite(t) || !(duration>0))return false;
    float u=fmaxf(0,fminf(1,t/duration));
    for(unsigned j=0;j<7;++j) {
        const float *b=c[j];
        q[j]=b[0]+u*(b[1]+u*(b[2]+u*(b[3]+u*(b[4]+u*b[5]))));
        v[j]=(b[1]+u*(2*b[2]+u*(3*b[3]+u*(4*b[4]+u*5*b[5]))))/duration;
        a[j]=(2*b[2]+u*(6*b[3]+u*(12*b[4]+u*20*b[5])))/(duration*duration);
        if(!isfinite(q[j]) || !isfinite(v[j]) || !isfinite(a[j]))return false;
    }
    return true;
}
bool rb_dance_action_sample(unsigned number,float t,float q[7],float v[7],float a[7])
{
    if(number<1 || number>10 || !isfinite(t) || t<0)return false;
    const rb_dance_action_t *action=&rb_dance_actions[number-1];
    t=fminf(t,action->duration_s);
    unsigned lo=0,hi=action->count;
    while(lo+1<hi) {unsigned mid=(lo+hi)/2;if(action->pieces[mid].begin_s<=t)lo=mid;else hi=mid;}
    const rb_dance_piece_t *p=&action->pieces[lo];
    return polynomial(p->c,t-p->begin_s,p->end_s-p->begin_s,q,v,a);
}
bool rb_dance_compile(rb_dance_plan_t *out,const uint8_t *numbers,unsigned count,float speed)
{
    if(!out)return false;
    memset(out,0,sizeof(*out));
    if(!numbers || !count || count>RB_DANCE_MAX_STEPS || (speed!=.5f && speed!=1.f))return false;
    for(unsigned i=0;i<count;++i)if(numbers[i]<1 || numbers[i]>10)return false;
    float elapsed=0;
    for(unsigned i=0;i<count;++i) {
        unsigned n=numbers[i];
        float start=i?rb_dance_joins[numbers[i-1]-1][n-1].right_trim_s:0;
        float end=rb_dance_actions[n-1].duration_s-(i+1<count?rb_dance_joins[n-1][numbers[i+1]-1].left_trim_s:0);
        if(!(end>start))return false;
        out->parts[out->part_count++]=(rb_dance_part_t){elapsed,elapsed+end-start,start,(uint8_t)n,0};
        elapsed+=end-start;
        if(i+1<count) {
            const rb_dance_join_t *join=&rb_dance_joins[n-1][numbers[i+1]-1];
            float q0[7],v0[7],a0[7],q1[7],v1[7],a1[7],T=join->duration_s;
            if(!(T>0) || !rb_dance_action_sample(n,end,q0,v0,a0) ||
                !rb_dance_action_sample(numbers[i+1],join->right_trim_s,q1,v1,a1))return false;
            for(unsigned j=0;j<7;++j) {
                float *b=out->bridges[i][j];b[0]=q0[j];b[1]=T*v0[j];b[2]=.5f*T*T*a0[j];
                float D=q1[j]-b[0]-b[1]-b[2],V=T*v1[j]-b[1]-2*b[2],A=T*T*a1[j]-2*b[2];
                b[3]=10*D-4*V+.5f*A;b[4]=-15*D+7*V-A;b[5]=6*D-3*V+.5f*A;
            }
            out->parts[out->part_count++]=(rb_dance_part_t){elapsed,elapsed+T,0,(uint8_t)i,1};elapsed+=T;
        }
    }
    memcpy(out->numbers,numbers,count);out->count=count;out->speed=speed;
    out->nominal_s=elapsed;out->duration_s=elapsed/speed;return true;
}
bool rb_dance_sample(const rb_dance_plan_t *p,float t,float q[7],float v[7],float a[7],unsigned *number,bool *bridge)
{
    if(!p || !p->count || !p->part_count || !isfinite(t) || t<0)return false;
    float stamp=fminf(t*p->speed,p->nominal_s);
    unsigned index=0;
    while(index+1<p->part_count && p->parts[index+1].begin_s<=stamp)++index;
    const rb_dance_part_t *part=&p->parts[index];
    bool ok=part->bridge?polynomial(p->bridges[part->number],stamp-part->begin_s,part->end_s-part->begin_s,q,v,a):
        rb_dance_action_sample(part->number,stamp-part->begin_s+part->source_s,q,v,a);
    if(!ok)return false;
    for(unsigned j=0;j<7;++j) {v[j]*=p->speed;a[j]*=p->speed*p->speed;}
    if(number)*number=part->bridge?0:part->number;
    if(bridge)*bridge=part->bridge!=0;
    return true;
}
bool rb_dance_random(uint8_t *out,unsigned count,uint32_t seed)
{
    if(!out || !count || count>10)return false;
    if(!seed)seed=0x6d2b79f5u;
    uint8_t pool[10];for(unsigned i=0;i<10;++i)pool[i]=(uint8_t)(i+1);
    for(unsigned i=0;i<count;++i) {
        uint32_t bound=10-i,threshold=(uint32_t)(-bound)%bound,value;
        do {seed^=seed<<13;seed^=seed>>17;seed^=seed<<5;value=seed;}while(value<threshold);
        unsigned j=i+value%bound;uint8_t tmp=pool[i];pool[i]=pool[j];pool[j]=tmp;out[i]=pool[i];
    }
    return true;
}
