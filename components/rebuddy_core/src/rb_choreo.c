#include "rb_choreo.h"
#include <stdio.h>
#include <string.h>

const rb_choreo_action_t rb_choreo_actions[]={
    {RB_CHOREO_2D_NOD,"点头打拍",RB_CHOREO_PERFORMANCE,3,true,2400,RB_CHOREO_POSE_IDLE,RB_CHOREO_POSE_IDLE},
    {RB_CHOREO_2D_LIFT,"抬臂舒展",RB_CHOREO_PERFORMANCE,3,true,2400,RB_CHOREO_POSE_IDLE,RB_CHOREO_POSE_IDLE},
    {RB_CHOREO_2D_DIP,"深蹲回升",RB_CHOREO_PERFORMANCE,3,true,2400,RB_CHOREO_POSE_IDLE,RB_CHOREO_POSE_IDLE},
    {RB_CHOREO_2D_REACH,"前探伸展",RB_CHOREO_PERFORMANCE,3,true,2400,RB_CHOREO_POSE_IDLE,RB_CHOREO_POSE_IDLE},
    {RB_CHOREO_2D_BASE_TURN,"底盘回转",RB_CHOREO_PERFORMANCE,3,true,2400,RB_CHOREO_POSE_IDLE,RB_CHOREO_POSE_IDLE},
    {RB_CHOREO_2D_SWAY,"大幅摇摆",RB_CHOREO_PERFORMANCE,3,true,2400,RB_CHOREO_POSE_IDLE,RB_CHOREO_POSE_IDLE},
    {RB_CHOREO_2D_ELBOW,"折肘展开",RB_CHOREO_PERFORMANCE,3,true,2400,RB_CHOREO_POSE_IDLE,RB_CHOREO_POSE_IDLE},
    {RB_CHOREO_2D_WRIST,"抬头俯看",RB_CHOREO_PERFORMANCE,3,true,2400,RB_CHOREO_POSE_IDLE,RB_CHOREO_POSE_IDLE},
    {RB_CHOREO_2D_GRIP,"夹爪开合",RB_CHOREO_PERFORMANCE,3,true,2400,RB_CHOREO_POSE_IDLE,RB_CHOREO_POSE_IDLE},
    {RB_CHOREO_2D_WAVE,"大幅波浪",RB_CHOREO_PERFORMANCE,3,true,2400,RB_CHOREO_POSE_IDLE,RB_CHOREO_POSE_IDLE},
    {RB_CHOREO_WAIT,"停留",RB_CHOREO_CONTROL,1,true,0,RB_CHOREO_POSE_IDLE,RB_CHOREO_POSE_IDLE}
};
const size_t rb_choreo_action_count=sizeof(rb_choreo_actions)/sizeof(rb_choreo_actions[0]);
const rb_choreo_action_t *rb_choreo_action(unsigned id)
{
    static const rb_choreo_action_t bookends[]={
        {RB_CHOREO_START,"开场",RB_CHOREO_CONTROL,1,true,3000,RB_CHOREO_POSE_IDLE,RB_CHOREO_POSE_IDLE},
        {RB_CHOREO_END,"收尾",RB_CHOREO_CONTROL,1,true,3000,RB_CHOREO_POSE_IDLE,RB_CHOREO_POSE_IDLE}
    };
    if(id==RB_CHOREO_START || id==RB_CHOREO_END) return &bookends[id-RB_CHOREO_START];
    for (size_t i=0;i<rb_choreo_action_count;++i)
        if (rb_choreo_actions[i].action_id==id) return &rb_choreo_actions[i];
    return NULL;
}
void rb_choreo_work_init(rb_choreo_work_t *work,unsigned slot)
{
    if (!work) return;
    memset(work,0,sizeof(*work)); work->schema_version=RB_CHOREO_SCHEMA_VERSION;
    work->tempo=RB_CHOREO_STANDARD;
    if (slot<RB_CHOREO_SLOTS) {
        work->work_id=(uint8_t)(slot+1);
        (void)snprintf(work->name,sizeof(work->name),"舞蹈 %u",slot+1);
    }
}
void rb_choreo_work_demo(rb_choreo_work_t *work,unsigned slot)
{
    rb_choreo_work_init(work,slot);
    /* Users compose the selected primitives themselves. */
}
static rb_choreo_status_t validate_step(const rb_choreo_step_t *step)
{
    const rb_choreo_action_t *action=rb_choreo_action(step->action_id);
    if (!action || step->action_id==RB_CHOREO_START || step->action_id==RB_CHOREO_END) return RB_CHOREO_UNKNOWN_ACTION;
    if (!action->available) return RB_CHOREO_UNAVAILABLE;
    if (!step->repeat || step->repeat>action->repeat_max) return RB_CHOREO_BAD_REPEAT;
    if (step->hold_ms>RB_CHOREO_HOLD_MAX_MS || step->hold_ms%RB_CHOREO_TIME_STEP_MS)
        return RB_CHOREO_BAD_HOLD;
    if (step->action_id==RB_CHOREO_WAIT) {
        if (step->hold_ms) return RB_CHOREO_BAD_HOLD;
        if (step->duration_ms<RB_CHOREO_WAIT_MIN_MS || step->duration_ms>RB_CHOREO_WAIT_MAX_MS ||
            step->duration_ms%RB_CHOREO_TIME_STEP_MS) return RB_CHOREO_BAD_DURATION;
    } else if (step->duration_ms) return RB_CHOREO_BAD_DURATION;
    return RB_CHOREO_OK;
}
static bool valid_name(const char *name)
{
    const unsigned char *p=(const unsigned char *)name;
    const unsigned char *end=memchr(name,0,RB_CHOREO_NAME_BYTES);
    if (!end || p==end) return false;
    while (p<end) {
        uint32_t value=*p++;
        if (value<0x80) { if (value<0x20 || value==0x7f) return false; continue; }
        unsigned more; uint32_t minimum;
        if (value>=0xc2 && value<=0xdf) { more=1; minimum=0x80; value&=0x1f; }
        else if (value>=0xe0 && value<=0xef) { more=2; minimum=0x800; value&=0x0f; }
        else if (value>=0xf0 && value<=0xf4) { more=3; minimum=0x10000; value&=0x07; }
        else return false;
        if ((size_t)(end-p)<more) return false;
        while (more--) { if ((*p&0xc0)!=0x80) return false; value=(value<<6)|(*p++&0x3f); }
        if (value<minimum || value>0x10ffff || (value>=0xd800 && value<=0xdfff)) return false;
    }
    return true;
}
rb_choreo_status_t rb_choreo_validate(const rb_choreo_work_t *work,uint8_t *bad_step)
{
    if (bad_step) *bad_step=RB_CHOREO_NO_STEP;
    if (!work) return RB_CHOREO_INVALID_ARGUMENT;
    if (work->schema_version!=RB_CHOREO_SCHEMA_VERSION) return RB_CHOREO_BAD_VERSION;
    if (!work->work_id || work->work_id>RB_CHOREO_SLOTS) return RB_CHOREO_BAD_WORK_ID;
    if (!valid_name(work->name)) return RB_CHOREO_BAD_NAME;
    if (work->tempo>=RB_CHOREO_TEMPO_COUNT) return RB_CHOREO_BAD_TEMPO;
    if (work->count>RB_CHOREO_MAX_STEPS) return RB_CHOREO_TOO_MANY_STEPS;
    for (unsigned i=0;i<work->count;++i) {
        rb_choreo_status_t status=validate_step(&work->steps[i]);
        if (status!=RB_CHOREO_OK) { if (bad_step) *bad_step=(uint8_t)i; return status; }
    }
    return RB_CHOREO_OK;
}
rb_choreo_status_t rb_choreo_add_after(rb_choreo_work_t *work,int selected,const rb_choreo_step_t *step)
{
    rb_choreo_status_t status=rb_choreo_validate(work,NULL);
    if (status!=RB_CHOREO_OK) return status;
    if (!step) return RB_CHOREO_INVALID_ARGUMENT;
    if (work->count>=RB_CHOREO_MAX_STEPS) return RB_CHOREO_TOO_MANY_STEPS;
    if (selected < -1 || (selected>=0 && (unsigned)selected>=work->count)) return RB_CHOREO_BAD_INDEX;
    status=validate_step(step); if (status!=RB_CHOREO_OK) return status;
    /* Copy first: callers may duplicate an existing step in this same work. */
    rb_choreo_step_t value=*step;
    unsigned at=selected<0?work->count:(unsigned)selected+1;
    memmove(work->steps+at+1,work->steps+at,(work->count-at)*sizeof(value));
    work->steps[at]=value; ++work->count;
    return RB_CHOREO_OK;
}
rb_choreo_status_t rb_choreo_delete(rb_choreo_work_t *work,unsigned index)
{
    rb_choreo_status_t status=rb_choreo_validate(work,NULL);
    if (status!=RB_CHOREO_OK) return status;
    if (index>=work->count) return RB_CHOREO_BAD_INDEX;
    memmove(work->steps+index,work->steps+index+1,(work->count-index-1)*sizeof(work->steps[0]));
    --work->count; memset(work->steps+work->count,0,sizeof(work->steps[0]));
    return RB_CHOREO_OK;
}
rb_choreo_status_t rb_choreo_move(rb_choreo_work_t *work,unsigned index,int direction)
{
    rb_choreo_status_t status=rb_choreo_validate(work,NULL);
    if (status!=RB_CHOREO_OK) return status;
    if (index>=work->count || (direction!=-1 && direction!=1) ||
        (direction<0 && index==0) || (direction>0 && index+1>=work->count)) return RB_CHOREO_BAD_INDEX;
    unsigned target=(unsigned)((int)index+direction);
    rb_choreo_step_t value=work->steps[index]; work->steps[index]=work->steps[target]; work->steps[target]=value;
    return RB_CHOREO_OK;
}
const char *rb_choreo_error_name(rb_choreo_status_t status)
{
    static const char *const names[]={"OK","INVALID_ARGUMENT","BAD_VERSION","BAD_WORK_ID","BAD_NAME",
        "BAD_TEMPO","TOO_MANY_STEPS","EMPTY","UNKNOWN_ACTION","UNAVAILABLE","BAD_REPEAT",
        "BAD_HOLD","BAD_DURATION","BAD_INDEX","TOO_MANY_EVENTS"};
    return (unsigned)status<sizeof(names)/sizeof(names[0])?names[status]:"UNKNOWN_ERROR";
}
static uint32_t scaled(unsigned duration,unsigned tempo)
{
    static const unsigned numerator[]={3,1,4},denominator[]={2,1,5};
    return (uint32_t)duration*numerator[tempo]/denominator[tempo];
}
uint16_t rb_choreo_beat_ms(unsigned tempo)
{
    static const uint16_t beats[]={900,600,480};
    return tempo<RB_CHOREO_TEMPO_COUNT?beats[tempo]:0;
}
/* Only the first/last half-beat changes speed. The polynomial reaches the
 * shared neutral with zero speed/acceleration, and joins the central linear
 * phase with unit speed and zero acceleration. Internal beat markers stay put.
 * This is time remapping of a visual clip, not a calibrated arm trajectory. */
static unsigned joined_time(unsigned time,unsigned duration)
{
    const unsigned edge=RB_CHOREO_HALF_BEAT_MS;
    if(duration<=edge*2 || time>=duration) return time;
    bool tail=time>duration-edge;
    unsigned distance=tail?duration-time:time;
    if(distance>=edge) return time;
    double u=(double)distance/edge;
    unsigned mapped=(unsigned)(edge*u*u*u*(6+u*(-8+3*u)));
    unsigned result=tail?duration-mapped:mapped;
    return result>=duration?duration-1:result;
}
static bool append(rb_choreo_plan_t *plan,unsigned action,unsigned step,bool transition,bool holding,
    unsigned source_duration,unsigned clip_time,unsigned duration)
{
    if (plan->count>=RB_CHOREO_MAX_EVENTS || !duration || UINT32_MAX-plan->total_ms<duration) return false;
    rb_choreo_event_t *event=&plan->events[plan->count++];
    *event=(rb_choreo_event_t){.action_id=(uint8_t)action,.step_index=(uint8_t)step,
        .transition=transition,.holding=holding,.start_ms=plan->total_ms,.duration_ms=duration,
        .source_duration_ms=(uint16_t)source_duration,.clip_time_ms=(uint16_t)clip_time};
    plan->total_ms+=duration;
    if (!transition) plan->base_ms+=duration;
    return true;
}
rb_choreo_status_t rb_choreo_compile(const rb_choreo_work_t *work,rb_choreo_plan_t *plan,uint8_t *bad_step)
{
    if (!plan) return RB_CHOREO_INVALID_ARGUMENT;
    memset(plan,0,sizeof(*plan));
    rb_choreo_status_t status=rb_choreo_validate(work,bad_step);
    if (status!=RB_CHOREO_OK) return status;
    if (!work->count) return RB_CHOREO_EMPTY;
    if(!append(plan,RB_CHOREO_START,RB_CHOREO_NO_STEP,true,false,3000,0,scaled(2400,work->tempo))) return RB_CHOREO_TOO_MANY_EVENTS;
    unsigned last_action=RB_CHOREO_START,last_source=3000,last_time=2999;
    for (unsigned i=0;i<work->count;++i) {
        const rb_choreo_step_t *step=&work->steps[i];
        const rb_choreo_action_t *action=rb_choreo_action(step->action_id);
        if (step->action_id==RB_CHOREO_WAIT) {
            if (!append(plan,last_action,i,false,true,last_source,last_time,scaled(step->duration_ms,work->tempo))) goto full;
            continue;
        }
        for (unsigned repeat=0;repeat<step->repeat;++repeat) {
            if (!append(plan,step->action_id,i,false,false,action->duration_ms,0,scaled(action->duration_ms,work->tempo))) goto full;
            last_action=step->action_id;
            last_source=action->duration_ms; last_time=last_source-1;
        }
        if (step->hold_ms && !append(plan,last_action,i,false,true,last_source,last_time,scaled(step->hold_ms,work->tempo))) goto full;
        continue;
full:
        memset(plan,0,sizeof(*plan));
        if (bad_step) *bad_step=(uint8_t)i;
        return RB_CHOREO_TOO_MANY_EVENTS;
    }
    if(!append(plan,RB_CHOREO_END,RB_CHOREO_NO_STEP,true,false,3000,0,scaled(2400,work->tempo))) {
        memset(plan,0,sizeof(*plan));return RB_CHOREO_TOO_MANY_EVENTS;
    }
    return RB_CHOREO_OK;
}
bool rb_choreo_sample(const rb_choreo_plan_t *plan,uint32_t elapsed_ms,rb_choreo_sample_t *sample)
{
    if (!plan || !sample || !plan->count || plan->count>RB_CHOREO_MAX_EVENTS || !plan->total_ms) return false;
    bool done=elapsed_ms>=plan->total_ms;
    unsigned index=0;
    while (index+1<plan->count && plan->events[index+1].start_ms<=elapsed_ms) ++index;
    const rb_choreo_event_t *event=&plan->events[index];
    if (!event->duration_ms || elapsed_ms<event->start_ms) return false;
    uint32_t local=elapsed_ms-event->start_ms;
    if (local>=event->duration_ms) local=event->duration_ms-1;
    unsigned time=event->holding?event->clip_time_ms:
        (unsigned)((uint64_t)local*event->source_duration_ms/event->duration_ms);
    if (done && !event->holding) time=event->source_duration_ms?event->source_duration_ms-1:0;
    if (!event->holding && !event->transition && !done) time=joined_time(time,event->source_duration_ms);
    *sample=(rb_choreo_sample_t){.action_id=event->action_id,.step_index=event->step_index,
        .clip_time_ms=(uint16_t)time,.holding=event->holding || done,.transition=event->transition,.done=done,
        .phase_q16=event->source_duration_ms>1?(uint16_t)((uint32_t)time*65535/(event->source_duration_ms-1)):0};
    return true;
}
bool rb_choreo_player_start(rb_choreo_player_t *player,const rb_choreo_plan_t *plan,uint64_t now_ms)
{
    if (!player) return false;
    memset(player,0,sizeof(*player));
    if (!rb_choreo_sample(plan,0,&player->sample)) return false;
    player->plan=plan; player->started_ms=now_ms; player->running=true;
    return true;
}
void rb_choreo_player_tick(rb_choreo_player_t *player,uint64_t now_ms)
{
    if (!player || !player->running || now_ms<player->started_ms) return;
    uint64_t elapsed=now_ms-player->started_ms;
    if (elapsed<player->elapsed_ms) return;
    player->elapsed_ms=elapsed>player->plan->total_ms?player->plan->total_ms:(uint32_t)elapsed;
    if (!rb_choreo_sample(player->plan,player->elapsed_ms,&player->sample) || player->sample.done) player->running=false;
}
void rb_choreo_player_stop(rb_choreo_player_t *player,uint64_t now_ms)
{
    if (!player) return;
    rb_choreo_player_tick(player,now_ms); player->running=false;
    player->sample.holding=true;
}
