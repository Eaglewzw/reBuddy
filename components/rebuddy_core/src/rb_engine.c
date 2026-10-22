#include "rb_core.h"
#include <math.h>
#include <string.h>

static float clamp(float x, float lo, float hi) { return fmaxf(lo, fminf(hi, x)); }
static float time_scale(rb_style_t s) { return s == RB_LAZY ? 1.65f : 1.0f; }
static float amplitude(rb_style_t s) { return s == RB_LAZY ? .55f : 1.0f; }
static float smooth(float t) { t = clamp(t, 0, 1); return t*t*t*(t*(6*t-15)+10); }
rb_config_t rb_config_default(void)
{ return (rb_config_t){.drowsy_ms=30000, .sleep_ms=60000, .max_speed_deg_s=5, .max_offset_deg=8}; }
bool rb_engine_init(rb_engine_t *e, const rb_config_t *config, uint64_t now)
{
    if (!e || !config || config->drowsy_ms == 0 || config->sleep_ms <= config->drowsy_ms ||
        !isfinite(config->max_speed_deg_s) || config->max_speed_deg_s <= 0 || config->max_speed_deg_s > 10 ||
        !isfinite(config->max_offset_deg) || config->max_offset_deg <= 0 || config->max_offset_deg > 10) return false;
    memset(e, 0, sizeof(*e)); e->config=*config; e->action=RB_NONE;
    e->now_ms=e->last_input_ms=now; e->life.openness=1; e->life.random=42;
    e->life.next_blink_ms=now+2300; e->life.next_look_ms=now+1800; return true;
}
static void start_action(rb_engine_t *e, rb_action_id_t action, uint64_t now)
{
    memcpy(e->origin, e->pose, sizeof(e->pose));
    e->action=action; e->started_ms=now; e->active_style=e->style;
    e->state=RB_PLAYING; e->face=rb_actions[action].frames[0].face;
    e->last_input_ms=now; ++e->actions_started;
}
bool rb_engine_action(rb_engine_t *e, rb_action_id_t action, uint64_t now)
{
    if (!e || e->state==RB_STOPPED || action>=RB_ACTION_COUNT || now<e->now_ms) return false;
    rb_engine_tick(e, now);
    if (e->state==RB_STOPPED) return false;
    start_action(e, action, now); return true;
}
bool rb_engine_event(rb_engine_t *e, rb_event_t event, uint64_t now)
{
    if (!e || e->state==RB_STOPPED || now<e->now_ms || event>RB_WAKE) return false;
    bool wake = e->state==RB_SLEEPING || e->state==RB_DROWSY || event==RB_WAKE;
    rb_action_id_t action=RB_LOOK;
    if (event==RB_REST) action=RB_DOZE;
    else if (!wake) {
        switch (event) {
        case RB_STROKE: action=RB_NUZZLE; break;
        case RB_MULTI_TAP: action=RB_SHY; break;
        case RB_ZIGZAG: action=RB_PUZZLED; break;
        case RB_HAPPY: action=RB_DANCE; break;
        default: break;
        }
    }
    bool ok=rb_engine_action(e, action, now);
    if (ok && wake && action!=RB_DOZE) e->state=RB_WAKING;
    return ok;
}
static void motion_tick(rb_engine_t *e, uint64_t now)
{
    if (!e || now<e->now_ms) return;
    uint64_t elapsed=now-e->now_ms; e->now_ms=now;
    if (e->state==RB_STOPPED) return;
    if (e->action!=RB_NONE) {
        if (elapsed>250) { rb_engine_stop(e, "CONTROL TICK TIMEOUT"); return; }
        const rb_action_t *a=&rb_actions[e->action];
        float local=(float)(now-e->started_ms)/time_scale(e->active_style);
        unsigned n=0;
        while (n+1<a->count && local>=(float)a->frames[n].at_ms) ++n;
        float start=n ? (float)a->frames[n-1].at_ms : 0;
        float end=(float)a->frames[n].at_ms;
        float t=smooth(end>start ? (local-start)/(end-start) : 1);
        e->face=a->frames[n].face;
        bool settled=true;
        for (unsigned j=0;j<RB_JOINTS;++j) {
            float from=n ? a->frames[n-1].offset_deg[j]*amplitude(e->active_style) : e->origin[j];
            float to=a->frames[n].offset_deg[j]*amplitude(e->active_style);
            float target=clamp(from+(to-from)*t, -e->config.max_offset_deg, e->config.max_offset_deg);
            float speed=e->config.max_speed_deg_s*(e->active_style==RB_LAZY ? .55f : 1);
            float max_step=speed*(float)elapsed*.001f;
            e->pose[j]+=clamp(target-e->pose[j], -max_step, max_step);
            if (!isfinite(e->pose[j])) { rb_engine_stop(e, "INVALID POSE"); return; }
            settled &= fabsf(e->pose[j]-to)<.001f;
        }
        if (local>=(float)a->frames[a->count-1].at_ms && settled) {
            bool sleep=e->action==RB_DOZE;
            e->action=RB_NONE; ++e->actions_completed;
            e->state=sleep?RB_SLEEPING:RB_IDLE; e->face=sleep?RB_FACE_SLEEP:RB_FACE_IDLE;
        }
        return;
    }
    if (e->state==RB_SLEEPING) return;
    uint64_t idle=now-e->last_input_ms;
    if (idle>=e->config.sleep_ms) { start_action(e,RB_DOZE,now); return; }
    if (idle>=e->config.drowsy_ms) { e->state=RB_DROWSY; e->face=RB_FACE_DROWSY; }
}
void rb_engine_tick(rb_engine_t *e, uint64_t now)
{
    if (!e || now<e->now_ms) return;
    uint64_t elapsed=now-e->now_ms;
    motion_tick(e,now);
    rb_life_tick(e,now,elapsed);
}
void rb_engine_style(rb_engine_t *e, rb_style_t style)
{ if (e && style<=RB_LAZY) e->style=style; }
void rb_engine_stop(rb_engine_t *e, const char *reason)
{ if (e) { e->state=RB_STOPPED; e->face=RB_FACE_STOP; e->action=RB_NONE; e->stop_reason=reason?reason:"STOP"; e->life.contact=e->life.stroking=false; e->life.celebrate_until=0; } }
bool rb_engine_reset(rb_engine_t *e, uint64_t now)
{
    if (!e || e->state!=RB_STOPPED || now<e->now_ms) return false;
    e->state=RB_IDLE; e->face=RB_FACE_IDLE; e->now_ms=e->last_input_ms=now; e->stop_reason=NULL;
    memset(&e->life,0,sizeof(e->life)); e->life.openness=1; e->life.random=42;
    e->life.next_blink_ms=now+2300; e->life.next_look_ms=now+1800;
    return true;
}
const char *rb_state_name(rb_state_t state)
{
    static const char *names[]={"IDLE","PLAYING","DROWSY","SLEEPING","WAKING","STOPPED"};
    return state<=RB_STOPPED?names[state]:"INVALID";
}
