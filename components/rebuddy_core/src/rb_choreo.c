#include "rb_choreo.h"
#include <stdio.h>
#include <string.h>

const rb_choreo_action_t rb_choreo_actions[]={
    {RB_CHOREO_PROBE,"抬头看你",RB_CHOREO_INTERACTION,1,true,5000,RB_CHOREO_POSE_IDLE,RB_CHOREO_POSE_IDLE},
    {RB_CHOREO_NUZZLE,"亲昵轻蹭",RB_CHOREO_INTERACTION,1,true,6000,RB_CHOREO_POSE_IDLE,RB_CHOREO_POSE_IDLE},
    {RB_CHOREO_TILT,"疑惑歪头",RB_CHOREO_INTERACTION,1,true,5000,RB_CHOREO_POSE_IDLE,RB_CHOREO_POSE_IDLE},
    {RB_CHOREO_WITHDRAW,"害羞收拢",RB_CHOREO_EMOTION,1,true,3000,RB_CHOREO_POSE_IDLE,RB_CHOREO_POSE_FOLDED},
    {RB_CHOREO_RECOVER,"安抚展开",RB_CHOREO_EMOTION,1,true,5000,RB_CHOREO_POSE_FOLDED,RB_CHOREO_POSE_IDLE},
    {RB_CHOREO_HAPPY,"开心回应",RB_CHOREO_EMOTION,1,true,8000,RB_CHOREO_POSE_IDLE,RB_CHOREO_POSE_IDLE},
    {RB_CHOREO_BASE_TURN,"底盘转拍",RB_CHOREO_PERFORMANCE,3,true,2000,RB_CHOREO_POSE_IDLE,RB_CHOREO_POSE_IDLE},
    {RB_CHOREO_SIDE_BEAT,"侧摆重拍",RB_CHOREO_PERFORMANCE,3,true,2000,RB_CHOREO_POSE_IDLE,RB_CHOREO_POSE_IDLE},
    {RB_CHOREO_BOUNCE,"上下律动",RB_CHOREO_PERFORMANCE,3,true,2000,RB_CHOREO_POSE_IDLE,RB_CHOREO_POSE_IDLE},
    {RB_CHOREO_ALTERNATE,"左右交替",RB_CHOREO_PERFORMANCE,3,true,2000,RB_CHOREO_POSE_IDLE,RB_CHOREO_POSE_IDLE},
    {RB_CHOREO_WAVE,"连环波浪",RB_CHOREO_PERFORMANCE,3,true,4000,RB_CHOREO_POSE_IDLE,RB_CHOREO_POSE_IDLE},
    {RB_CHOREO_FINALE,"舒展收拍",RB_CHOREO_PERFORMANCE,1,true,4000,RB_CHOREO_POSE_IDLE,RB_CHOREO_POSE_IDLE},
    {RB_CHOREO_NOD,"点头回应",RB_CHOREO_PERFORMANCE,3,true,2000,RB_CHOREO_POSE_IDLE,RB_CHOREO_POSE_IDLE},
    {RB_CHOREO_SHAKE,"俏皮摇头",RB_CHOREO_PERFORMANCE,3,true,2000,RB_CHOREO_POSE_IDLE,RB_CHOREO_POSE_IDLE},
    {RB_CHOREO_SWAY,"左右摇摆",RB_CHOREO_PERFORMANCE,3,true,4000,RB_CHOREO_POSE_IDLE,RB_CHOREO_POSE_IDLE},
    {RB_CHOREO_BOW,"礼貌鞠躬",RB_CHOREO_PERFORMANCE,1,true,4000,RB_CHOREO_POSE_IDLE,RB_CHOREO_POSE_IDLE},
    {RB_CHOREO_CLAW_BEAT,"夹爪打拍",RB_CHOREO_PERFORMANCE,3,true,2000,RB_CHOREO_POSE_IDLE,RB_CHOREO_POSE_IDLE},
    {RB_CHOREO_POSE,"得意亮相",RB_CHOREO_PERFORMANCE,1,true,4000,RB_CHOREO_POSE_IDLE,RB_CHOREO_POSE_IDLE},
    {RB_CHOREO_WAIT,"停留",RB_CHOREO_CONTROL,1,true,0,RB_CHOREO_POSE_IDLE,RB_CHOREO_POSE_IDLE}
};
const size_t rb_choreo_action_count=sizeof(rb_choreo_actions)/sizeof(rb_choreo_actions[0]);
const rb_choreo_action_t *rb_choreo_action(unsigned id)
{
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
    if (!work || slot>=RB_CHOREO_SLOTS) return;
    (void)snprintf(work->name,sizeof(work->name),"活力律动");
    work->count=6;
    work->steps[0]=(rb_choreo_step_t){RB_CHOREO_BASE_TURN,2,0,0};
    work->steps[1]=(rb_choreo_step_t){RB_CHOREO_SIDE_BEAT,2,0,0};
    work->steps[2]=(rb_choreo_step_t){RB_CHOREO_BOUNCE,2,0,0};
    work->steps[3]=(rb_choreo_step_t){RB_CHOREO_ALTERNATE,2,0,0};
    work->steps[4]=(rb_choreo_step_t){RB_CHOREO_WAVE,1,0,0};
    work->steps[5]=(rb_choreo_step_t){RB_CHOREO_FINALE,1,0,0};
}
static rb_choreo_status_t validate_step(const rb_choreo_step_t *step)
{
    const rb_choreo_action_t *action=rb_choreo_action(step->action_id);
    if (!action) return RB_CHOREO_UNKNOWN_ACTION;
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
    unsigned pose=RB_CHOREO_POSE_IDLE,last_action=0,last_source=0,last_time=0;
    for (unsigned i=0;i<work->count;++i) {
        const rb_choreo_step_t *step=&work->steps[i];
        const rb_choreo_action_t *action=rb_choreo_action(step->action_id);
        if (step->action_id==RB_CHOREO_WAIT) {
            if (!append(plan,last_action,i,false,true,last_source,last_time,scaled(step->duration_ms,work->tempo))) goto full;
            continue;
        }
        for (unsigned repeat=0;repeat<step->repeat;++repeat) {
            if (pose!=action->entry_pose) {
                unsigned bridge=pose==RB_CHOREO_POSE_FOLDED?RB_CHOREO_RECOVER:RB_CHOREO_WITHDRAW;
                const rb_choreo_action_t *connection=rb_choreo_action(bridge);
                if (!append(plan,bridge,i,true,false,connection->duration_ms,0,scaled(connection->duration_ms,work->tempo))) goto full;
                pose=connection->exit_pose;
            }
            if (!append(plan,step->action_id,i,false,false,action->duration_ms,0,scaled(action->duration_ms,work->tempo))) goto full;
            pose=action->exit_pose; last_action=step->action_id;
            last_source=action->duration_ms; last_time=last_source-1;
        }
        if (step->hold_ms && !append(plan,last_action,i,false,true,last_source,last_time,scaled(step->hold_ms,work->tempo))) goto full;
        continue;
full:
        memset(plan,0,sizeof(*plan));
        if (bad_step) *bad_step=(uint8_t)i;
        return RB_CHOREO_TOO_MANY_EVENTS;
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
    *sample=(rb_choreo_sample_t){.action_id=event->action_id,.step_index=event->step_index,
        .clip_time_ms=(uint16_t)time,.holding=event->holding || done,.transition=event->transition,.done=done};
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
