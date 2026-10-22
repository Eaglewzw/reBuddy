#include "rb_pet.h"
#include <math.h>
#include <string.h>

static bool inside(int x,int y) { return x>=RB_ROLE_X && x<RB_ROLE_X+RB_ROLE_WIDTH && y>=RB_ROLE_Y && y<RB_ROLE_Y+RB_ROLE_HEIGHT; }
static unsigned priority(rb_pet_action_t a)
{
    const unsigned values[]={0,2,2,6,8,5,4,1,9};
    return a<RB_ANIM_COUNT?values[a]:0;
}
static void clear_happy(rb_pet_t *p)
{ p->happy_head=p->happy_count=p->happy_ms=0; }
static void clear_contact(rb_pet_t *p)
{
    p->head_moving_ms=0;
    p->stroke=false; p->candidate_count=p->candidate_ms=p->moving_ms=p->fast_ms=0;
    p->position_count=0; p->path=p->max_offset=p->speed=p->candidate_path=0;
}
void rb_pet_init(rb_pet_t *p,uint64_t now)
{
    memset(p,0,sizeof(*p)); p->now=p->last_interaction=now;
    p->estop_released=true; p->random=42; p->next_blink=now+2500;
    p->direction=p->active_direction=p->pending_direction=1;
}
void rb_pet_set_sink(rb_pet_t *p,rb_pet_sink_t sink,void *context)
{ p->sink=sink; p->sink_context=context; }
void rb_pet_set_head_region(rb_pet_t *p,const int16_t points[4][2])
{ memcpy(p->head_region,points,sizeof(p->head_region)); }
static bool on_head(const rb_pet_t *p,int x,int y)
{
    int sign=0;
    for (unsigned i=0;i<4;++i) {
        unsigned j=(i+1)%4;
        int cross=(p->head_region[j][0]-p->head_region[i][0])*(y-p->head_region[i][1])-
            (p->head_region[j][1]-p->head_region[i][1])*(x-p->head_region[i][0]);
        if (cross) {
            int next=cross>0?1:-1;
            if (sign && next!=sign) return false;
            sign=next;
        }
    }
    return sign!=0;
}
static bool allowed(const rb_pet_t *p,rb_pet_action_t a)
{
    if (p->state==RB_PET_STOPPED) return false;
    if (a==RB_ANIM_WAKE) return true;
    if (p->state==RB_PET_RESTING) return false;
    if (a==RB_ANIM_WITHDRAW || a==RB_ANIM_REST) return true;
    if (a==RB_ANIM_RECOVER) return p->state==RB_PET_FOLDED;
    return p->state==RB_PET_NORMAL;
}
static void start(rb_pet_t *p,rb_pet_action_t a,rb_pet_source_t source,int direction)
{
    const unsigned ms[]={0,RB_MS_PROBE,RB_MS_NUZZLE_CYCLE,RB_MS_TILT,RB_MS_WITHDRAW,
        RB_MS_RECOVER,RB_MS_HAPPY,RB_MS_REST,RB_MS_WAKE};
    p->action=a; p->origin=p->pose; p->phase=0; p->started=p->now;
    p->active_direction=direction;
    p->active_lazy=p->lazy; p->duration=(uint64_t)(ms[a]*(p->active_lazy?RB_STYLE_LAZY:RB_STYLE_LIVELY));
    p->returning=false;
    if (a==RB_ANIM_WITHDRAW) p->state=RB_PET_FOLDED;
    if (a==RB_ANIM_HAPPY) { p->happy_latched=true; clear_happy(p); }
    p->last_event=(rb_pet_event_t){a,++p->requests,source,
        a==RB_ANIM_REST?RB_PET_RESTING:a==RB_ANIM_WITHDRAW?RB_PET_FOLDED:
        (a==RB_ANIM_RECOVER || a==RB_ANIM_WAKE)?RB_PET_NORMAL:p->state,p->active_lazy,p->now};
    if (p->sink) p->sink(p->sink_context,&p->last_event);
    if (a==RB_ANIM_WITHDRAW && p->pose==RB_POSE_FOLDED) p->action=RB_ANIM_IDLE;
}
static void request(rb_pet_t *p,rb_pet_action_t a,rb_pet_source_t source)
{
    if (!allowed(p,a)) return;
    if (a==RB_ANIM_NUZZLE && (p->action==a || p->now<p->nuzzle_hold)) return;
    if (a==RB_ANIM_WAKE && p->action==a) return;
    if (a==RB_ANIM_WITHDRAW && p->state==RB_PET_FOLDED && p->action!=RB_ANIM_RECOVER) return;
    if (p->action==RB_ANIM_IDLE) { start(p,a,source,p->direction); return; }
    if (!p->pending_valid || priority(a)>=priority(p->pending)) {
        p->pending=a; p->pending_source=source; p->pending_valid=true;
        p->pending_direction=p->direction;
    }
    /* Urgent changes first retrace this clip to its known origin, never jump poses. */
    if (!p->returning && priority(a)>priority(p->action) && priority(a)>=6) {
        p->returning=true; p->return_started=p->now; p->return_phase=p->phase; p->return_duration=180;
        if (p->action==RB_ANIM_HAPPY) p->happy_cooldown_until=p->now+RB_CFG_HAPPY_COOLDOWN_MS;
    }
}
static void run_pending(rb_pet_t *p)
{
    if (!p->pending_valid) return;
    rb_pet_action_t a=p->pending; rb_pet_source_t source=p->pending_source;
    p->pending_valid=false;
    if (a==RB_ANIM_RECOVER && (p->soothe_ms<RB_CFG_SOOTHE_VALID_MS ||
        p->now-p->last_gentle>RB_CFG_SOOTHE_RESET_PAUSE_MS)) return;
    if (a==RB_ANIM_HAPPY && (p->happy_ms<RB_CFG_HAPPY_VALID_MS || p->happy_latched ||
        !on_head(p,p->raw_x,p->raw_y))) return;
    if (allowed(p,a) && (a!=RB_ANIM_NUZZLE || p->stroke)) start(p,a,source,p->pending_direction);
}
static void finish(rb_pet_t *p)
{
    rb_pet_action_t old=p->action; p->action=RB_ANIM_IDLE; p->phase=0; ++p->completed;
    p->last_interaction=p->now;
    if (old==RB_ANIM_WITHDRAW) p->pose=RB_POSE_FOLDED;
    else if (old==RB_ANIM_REST) { p->pose=RB_POSE_REST; p->state=RB_PET_RESTING; p->pending_valid=false; }
    else { p->pose=RB_POSE_IDLE; }
    if (old==RB_ANIM_RECOVER || old==RB_ANIM_WAKE) {
        p->state=RB_PET_NORMAL; p->consumed=p->contact || p->raw_down;
        p->soothe_ms=0; p->tap_count=0; clear_happy(p); p->pending_valid=false;
    }
    if (old==RB_ANIM_HAPPY) p->happy_cooldown_until=p->now+RB_CFG_HAPPY_COOLDOWN_MS;
    if (old==RB_ANIM_NUZZLE) {
        if (++p->nuzzle_cycles>=2) { p->nuzzle_hold=p->now+500; p->nuzzle_cycles=0; }
        else if (p->stroke && !p->pending_valid) request(p,RB_ANIM_NUZZLE,RB_INPUT_STROKE);
    }
    run_pending(p);
}
static void animate(rb_pet_t *p)
{
    if (p->action==RB_ANIM_IDLE) { run_pending(p); return; }
    if (p->returning) {
        float t=(float)(p->now-p->return_started)/(float)p->return_duration;
        p->phase=p->return_phase*fmaxf(0,1-t);
        if (t>=1) { p->pose=p->origin; p->action=RB_ANIM_IDLE; p->returning=false; run_pending(p); }
        return;
    }
    p->phase=p->duration?(float)(p->now-p->started)/p->duration:1;
    if (p->phase>=1) finish(p);
}
void rb_pet_cancel(rb_pet_t *p)
{
    p->contact=false; p->blocked=p->raw_down; p->consumed=false;
    clear_contact(p); clear_happy(p); p->soothe_ms=0; p->tap_count=0;
    p->last_interaction=p->release_at=p->now;
    if (p->pending_valid && priority(p->pending)<=4) p->pending_valid=false;
}
void rb_pet_input(rb_pet_t *p,bool down,int x,int y,uint64_t now)
{
    if (now<p->now) return;
    if (down!=p->raw_down) { p->raw_changed=now; p->raw_down=down; if (down && !inside(x,y)) p->blocked=true; }
    p->raw_x=x; p->raw_y=y;
    if (p->contact && down && !inside(x,y)) rb_pet_cancel(p);
}
static void down(rb_pet_t *p)
{
    p->contact=true; p->consumed=false; p->swiped=false;
    p->down_at=p->sampled=p->last_motion=p->now; p->last_interaction=p->now;
    p->x=p->start_x=p->previous_raw_x=p->raw_x; p->y=p->start_y=p->previous_raw_y=p->raw_y;
    clear_contact(p); p->positions[0].x=p->x; p->positions[0].y=p->y;
    p->positions[0].at=p->now; p->position_count=1;
    if (p->state==RB_PET_RESTING || p->action==RB_ANIM_REST || p->action==RB_ANIM_WAKE) {
        p->consumed=true; p->tap_count=p->soothe_ms=0; clear_happy(p);
        /* Rest is persistent. Only the explicit wake control can end it. */
    }
}
static void tap(rb_pet_t *p)
{
    clear_happy(p);
    if (p->pending_valid && p->pending==RB_ANIM_HAPPY) p->pending_valid=false;
    if (p->action==RB_ANIM_RECOVER) {
        p->soothe_ms=0; p->pending_valid=false; request(p,RB_ANIM_WITHDRAW,RB_INPUT_TAP); return;
    }
    if (p->state!=RB_PET_NORMAL) { p->blink_until=p->now+120; return; }
    unsigned count=0;
    for (unsigned i=0;i<p->tap_count;++i)
        if (p->now-p->taps[i]<=RB_CFG_POKE_WINDOW_MS) p->taps[count++]=p->taps[i];
    p->taps[count++]=p->now; p->tap_count=count;
    if (count>=RB_CFG_POKE_COUNT) {
        p->tap_count=0; p->pending_valid=false; request(p,RB_ANIM_WITHDRAW,RB_INPUT_POKE);
    } else request(p,RB_ANIM_PROBE,RB_INPUT_TAP);
}
static void up(rb_pet_t *p)
{
    if (!p->consumed && !p->swiped && !p->stroke && p->now-p->down_at<=RB_CFG_TAP_MAX_MS &&
        p->max_offset<=RB_CFG_TAP_MAX_OFFSET_PX && p->path<=RB_CFG_TAP_MAX_PATH_PX) tap(p);
    p->contact=false; p->release_at=p->last_interaction=p->now;
    p->consumed=false; clear_contact(p);
    if (p->pending_valid && p->pending==RB_ANIM_NUZZLE) p->pending_valid=false;
    if (p->action==RB_ANIM_NUZZLE && !p->returning) {
        p->returning=true; p->return_phase=p->phase; p->return_started=p->now; p->return_duration=400;
    }
}
static void expire_happy(rb_pet_t *p)
{
    while (p->happy_count) {
        rb_valid_interval_t *v=&p->happy_history[p->happy_head];
        if (p->now-v->at<=RB_CFG_HAPPY_WINDOW_MS) break;
        p->happy_ms-=v->duration; p->happy_head=(p->happy_head+1)%512; --p->happy_count;
    }
}
static void credit(rb_pet_t *p,rb_valid_interval_t v)
{
    if (p->state==RB_PET_FOLDED) {
        if (p->action!=RB_ANIM_RECOVER) {
            p->soothe_ms+=v.duration;
            if (p->soothe_ms>=RB_CFG_SOOTHE_VALID_MS) request(p,RB_ANIM_RECOVER,RB_INPUT_STROKE);
        }
        return;
    }
    if (p->happy_latched || p->state!=RB_PET_NORMAL || !v.head_duration) return;
    v.duration=v.head_duration;
    expire_happy(p);
    if (p->happy_count==512) {
        p->happy_ms-=p->happy_history[p->happy_head].duration;
        p->happy_head=(p->happy_head+1)%512; --p->happy_count;
    }
    p->happy_history[(p->happy_head+p->happy_count++)%512]=v; p->happy_ms+=v.duration;
    if (p->happy_ms>=RB_CFG_HAPPY_VALID_MS && on_head(p,p->raw_x,p->raw_y))
        request(p,RB_ANIM_HAPPY,RB_INPUT_STROKE);
}
static void motion(rb_pet_t *p)
{
    uint64_t elapsed=p->now-p->sampled; p->sampled=p->now;
    if (!elapsed || p->consumed || p->swiped || !p->raw_down) return;
    if (elapsed>120) elapsed=120;
    bool head=on_head(p,p->raw_x,p->raw_y);
    if (!head && p->pending_valid && p->pending==RB_ANIM_HAPPY) p->pending_valid=false;
    if (p->raw_x!=p->previous_raw_x || p->raw_y!=p->previous_raw_y) {
        p->moving_ms+=(unsigned)elapsed;
        if (head && on_head(p,p->previous_raw_x,p->previous_raw_y))
            p->head_moving_ms+=(unsigned)elapsed;
    }
    p->previous_raw_x=p->raw_x; p->previous_raw_y=p->raw_y;
    float dx=p->raw_x-p->x,dy=p->raw_y-p->y,dist=hypotf(dx,dy);
    float offset=hypotf(p->raw_x-p->start_x,p->raw_y-p->start_y);
    p->max_offset=fmaxf(p->max_offset,offset);
    bool moved=dist>RB_CFG_JITTER_PX;
    unsigned moving=0,head_moving=0;
    if (moved) {
        p->x=p->raw_x; p->y=p->raw_y; p->path+=dist;
        moving=p->moving_ms>120?120:p->moving_ms; p->moving_ms=0; p->last_motion=p->now;
        head_moving=p->head_moving_ms>moving?moving:p->head_moving_ms; p->head_moving_ms=0;
    }
    if (p->position_count==8) { memmove(p->positions,p->positions+1,7*sizeof(p->positions[0])); --p->position_count; }
    unsigned n=p->position_count++;
    p->positions[n].x=p->x; p->positions[n].y=p->y; p->positions[n].at=p->now;
    unsigned first=0;
    while (first+1<n && p->now-p->positions[first].at>RB_CFG_SPEED_WINDOW_MS) ++first;
    float path=0;
    for (unsigned i=first+1;i<=n;++i)
        path+=hypotf(p->positions[i].x-p->positions[i-1].x,p->positions[i].y-p->positions[i-1].y);
    uint64_t window=p->now-p->positions[first].at;
    p->speed=window?path*1000/window:0;
    if (moved && p->speed>=RB_CFG_SWIPE_SPEED_MIN) p->fast_ms+=(unsigned)elapsed;
    else p->fast_ms=0;
    if (p->fast_ms>=RB_CFG_SWIPE_MIN_MS && offset>=RB_CFG_SWIPE_MIN_DISPLACEMENT_PX) {
        p->direction=p->raw_x-p->start_x < -(int)RB_CFG_JITTER_PX?-1:1;
        p->swiped=true; p->stroke=false; p->tap_count=p->soothe_ms=0; clear_happy(p);
        p->pending_valid=false;
        if (p->action==RB_ANIM_RECOVER) request(p,RB_ANIM_WITHDRAW,RB_INPUT_SWIPE);
        else request(p,RB_ANIM_TILT,RB_INPUT_SWIPE);
        return;
    }
    if (moved && p->speed>=RB_CFG_STROKE_SPEED_MIN && p->speed<=RB_CFG_STROKE_SPEED_MAX) {
        p->last_gentle=p->now;
        rb_valid_interval_t v={p->now,(uint16_t)moving,(uint16_t)head_moving};
        if (!p->stroke) {
            if (p->candidate_count<64) p->candidate[p->candidate_count++]=v;
            p->candidate_ms+=moving; p->candidate_path+=dist;
            if (p->candidate_ms>=RB_CFG_STROKE_MIN_MS && p->candidate_path>=RB_CFG_STROKE_MIN_PATH_PX) {
                p->direction=p->raw_x-p->start_x < -(int)RB_CFG_JITTER_PX?-1:1;
                p->stroke=true; p->tap_count=0;
                for (unsigned i=0;i<p->candidate_count;++i) credit(p,p->candidate[i]);
                p->candidate_count=0;
            }
        } else credit(p,v);
        if (p->stroke && p->state==RB_PET_NORMAL && p->action!=RB_ANIM_HAPPY)
            request(p,RB_ANIM_NUZZLE,RB_INPUT_STROKE);
    } else if (moved && p->speed>RB_CFG_STROKE_SPEED_MAX) {
        p->candidate_count=p->candidate_ms=0; p->candidate_path=0;
    }
    if (p->now-p->last_gentle>RB_CFG_STROKE_PAUSE_GRACE_MS) {
        p->stroke=false; p->candidate_count=p->candidate_ms=0; p->candidate_path=0;
        if (p->pending_valid && p->pending==RB_ANIM_NUZZLE) p->pending_valid=false;
        if (p->action==RB_ANIM_NUZZLE && !p->returning) {
            p->returning=true; p->return_started=p->now; p->return_phase=p->phase; p->return_duration=400;
        }
    }
}
void rb_pet_tick(rb_pet_t *p,uint64_t now)
{
    if (now<p->now) return;
    p->now=now;
    if (!p->estop_released) rb_pet_stop(p,"E-STOP");
    if (p->state==RB_PET_STOPPED) return;
    expire_happy(p);
    animate(p);
    if (now-p->last_gentle>RB_CFG_SOOTHE_RESET_PAUSE_MS) p->soothe_ms=0;
    if (now-p->raw_changed>=RB_CFG_CONTACT_DEBOUNCE_MS) {
        if (!p->raw_down) {
            if (p->contact) up(p);
            p->blocked=false;
        } else if (!p->contact && !p->blocked) {
            if (inside(p->raw_x,p->raw_y)) down(p); else p->blocked=true;
        }
    }
    if (p->contact) motion(p);
    expire_happy(p);
    if (!p->contact && p->release_at && now-p->release_at>=RB_CFG_HAPPY_RELEASE_RESET_MS) {
        clear_happy(p);
        if (now>=p->happy_cooldown_until && p->action!=RB_ANIM_HAPPY) p->happy_latched=false;
    }
    if (p->action==RB_ANIM_IDLE && p->state!=RB_PET_RESTING && !p->contact && !p->raw_down &&
        now-p->last_interaction>=RB_CFG_SLEEP_IDLE_MS) request(p,RB_ANIM_REST,RB_INPUT_IDLE);
    if (p->stroke && p->state==RB_PET_NORMAL && p->action==RB_ANIM_IDLE && !p->consumed)
        request(p,RB_ANIM_NUZZLE,RB_INPUT_STROKE);
    if (now>=p->next_blink) {
        p->random=p->random*1664525u+1013904223u; p->blink_until=now+120;
        p->next_blink=now+2200+p->random%3500;
    }
    p->blink=now<p->blink_until;
}
void rb_pet_stop(rb_pet_t *p,const char *reason)
{
    if (p->state==RB_PET_STOPPED) {
        /* Hardware stops take precedence over a reversible screen pause. */
        if (p->stop_reason && !strcmp(p->stop_reason,"SCREEN STOP") &&
            reason && strcmp(reason,"SCREEN STOP")) p->stop_reason=reason;
        return;
    }
    /* Keep action/phase/origin frozen so STOP never changes the rendered pose. */
    p->state=RB_PET_STOPPED; p->stop_reason=reason; p->pending_valid=false;
    p->nuzzle_cycles=0; p->contact=false; p->blocked=true; p->consumed=true; p->soothe_ms=p->tap_count=0;
    clear_contact(p); clear_happy(p);
}
void rb_pet_estop(rb_pet_t *p,bool released)
{ p->estop_released=released; if (!released) rb_pet_stop(p,"E-STOP"); }
bool rb_pet_reset(rb_pet_t *p,uint64_t now)
{
    if (p->state!=RB_PET_STOPPED || !p->estop_released || now<p->now) return false;
    p->now=p->last_interaction=now; p->stop_reason=NULL;
    p->state=p->pose==RB_POSE_FOLDED?RB_PET_FOLDED:RB_PET_NORMAL;
    p->happy_latched=false; p->happy_cooldown_until=0;
    p->pending=RB_ANIM_WAKE; p->pending_source=RB_INPUT_RESET; p->pending_valid=true;
    if (p->action!=RB_ANIM_IDLE) { p->returning=true; p->return_phase=p->phase; p->return_started=now; p->return_duration=180; }
    else run_pending(p);
    return true;
}
void rb_pet_rest(rb_pet_t *p)
{
    if (p->state==RB_PET_RESTING || p->action==RB_ANIM_REST ||
        (p->pending_valid && p->pending==RB_ANIM_REST)) return;
    rb_pet_cancel(p);
    request(p,RB_ANIM_REST,RB_INPUT_SETTINGS);
}
void rb_pet_wake(rb_pet_t *p)
{
    if (p->state==RB_PET_STOPPED || !p->estop_released) return;
    if (p->state!=RB_PET_RESTING && p->action!=RB_ANIM_REST &&
        !(p->pending_valid && p->pending==RB_ANIM_REST)) return;
    rb_pet_cancel(p);
    request(p,RB_ANIM_WAKE,RB_INPUT_WAKE);
}
const char *rb_pet_action_name(rb_pet_action_t action)
{
    static const char *names[]={"IDLE","PROBE","NUZZLE","TILT","WITHDRAW","RECOVER","HAPPY","REST","WAKE"};
    return action<RB_ANIM_COUNT?names[action]:"INVALID";
}
