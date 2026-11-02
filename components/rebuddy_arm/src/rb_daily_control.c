/* Daily orchestration over the same native motor/reference/gravity components
 * as esp32c5_ui_console. No RS protocol encoding, register-position decoder,
 * second CAN driver, zero calibration write, or alternate motor control loop. */
#include "rb_daily_control.h"
#include "can_console_motor_runtime.h"
#include <math.h>
#include <string.h>
#define RAD .0174532925199433f
#define PERIOD .02f
static const float bounds_lo[7]={-2.8f,0,0,-1.57f,-1.57f,-3.14f,-.12f};
static const float bounds_hi[7]={2.8f,3.14f,3.14f,1.57f,1.57f,3.14f,4.48332313f};
static void state(rb_daily_control_t *c,rb_dc_state_t s)
{ c->status.state=s;c->changed=c->now;c->still_since=0;c->elapsed=0; }
static void disable(rb_daily_control_t *c,const char *reason,unsigned motor)
{
    if (reason && !c->status.error) { c->status.error=reason;c->status.bad_motor=motor; }
    /* Precheck has not changed modes or enabled anything: do not disable
     * motors that may belong to a different physical controller. */
    if(c->status.state==RB_DC_PRECHECK) {state(c,RB_DC_FAULT);return;}
    if (c->status.state==RB_DC_DISABLING) return;
    c->status.disable_confirmed=false;c->disable_sent=false;c->pending=false;c->fence=0;
    state(c,RB_DC_DISABLING);
}
static bool bus_ready(rb_daily_control_t *c)
{
    can_console_motor_bus_state_t b;
    return can_console_motor_get_bus_state(c->channel,100,&b)==ESP_OK && b.started &&
        (b.status==CAN_CONSOLE_MOTOR_BUS_ACTIVE || b.status==CAN_CONSOLE_MOTOR_BUS_WARNING);
}
static bool feedback(rb_daily_control_t *c)
{
    bool valid=true;
    for(unsigned j=0;j<7;++j) {
        can_console_motor_state_t *m=&c->motors[j];
        if(can_console_motor_get_state(j+1,m)!=ESP_OK || !m->online || m->motor_id!=j+1 ||
            m->channel!=c->channel || strcmp(m->vendor_name,"robstride") ||
            m->last_feedback_us<=0 || m->last_feedback_us>(int64_t)c->now*1000+20000 ||
            (int64_t)c->now*1000-m->last_feedback_us>100000 ||
            !isfinite(m->position) || !isfinite(m->velocity) || !isfinite(m->torque) ||
            !isfinite(m->mos_temperature)) { valid=false;c->status.bad_motor=j+1;continue; }
        if(m->last_feedback_us!=c->previous_feedback[j]) {
            float dt=(m->last_feedback_us-c->previous_feedback[j])/1e6f;
            if(c->previous_feedback[j] && dt>0 && dt<=.1f)
                c->estimated_v[j]+=dt/(.04f+dt)*((m->position-c->previous_q[j])/dt-c->estimated_v[j]);
            else c->estimated_v[j]=m->velocity;
            c->previous_feedback[j]=m->last_feedback_us;c->previous_q[j]=m->position;
        }
    }
    return valid;
}
static bool check_feedback(rb_daily_control_t *c,bool require_enable)
{
    for(unsigned j=0;j<7;++j) {
        const can_console_motor_state_t *m=&c->motors[j];
        const char *error=NULL;
        if(m->fault_raw || (m->status&0x1fc)) error="motor fault";
        else if(m->position<bounds_lo[j]-3*RAD || m->position>bounds_hi[j]+3*RAD) error="position outside measured limits";
        else if(require_enable && (!m->enabled || !m->mode_ready || (m->status&3)!=2 || m->mode!=MOTOR_MODE_MIT)) error="enable or mode lost";
        if(error) { disable(c,error,j+1);return false; }
    }
    return true;
}
static bool reference_ready(rb_daily_control_t *c)
{
    if(!c->pending)return true;
    can_console_motor_reference_status_t r;
    if(can_console_motor_get_reference_status(c->sequence,&r)!=ESP_OK ||
        r.state==CAN_CONSOLE_MOTOR_REFERENCE_FAILED) { disable(c,"reference delivery failed",0);return false; }
    if(r.state==CAN_CONSOLE_MOTOR_REFERENCE_PENDING) {
        if(c->now-c->sent>100)disable(c,"reference delivery timeout",0);
        return false;
    }
    c->pending=false;c->owned=true;c->elapsed=c->queued_elapsed;
    if(c->status.state==RB_DC_PLAY) {
        const rb_daily_action_t *a=rb_daily_find(c->status.action);
        c->status.phase=fminf(1,c->elapsed/a->duration_s);
    } else if(c->status.state==RB_DC_ENTRY && c->status.action==RB_DAILY_WAKE && c->duration>0) {
        c->status.phase=fminf(1,c->elapsed/c->duration);
    }
    return true;
}
static void hold(rb_daily_control_t *c,const char *reason,const float *attempted_target)
{
    rb_dc_hold_snapshot_t *h=&c->status.hold;
    h->valid=true;h->torque_checked=attempted_target!=NULL;
    h->source_state=c->status.state;h->at_ms=c->now;
    for(unsigned j=0;j<7;++j) {
        h->measured[j]=c->motors[j].position;
        h->target[j]=attempted_target?attempted_target[j]:c->target[j];
        h->velocity[j]=c->estimated_v[j];
        h->estimated_torque[j]=attempted_target?c->estimated_torque[j]:0;
    }
    for(unsigned j=0;j<7;++j)c->target[j]=c->motors[j].position;
    c->status.error=reason;state(c,RB_DC_PROTECTIVE_HOLD);
}
/* Budget rejection is a checked hold, never silently saturated feedforward. */
static bool send_target(rb_daily_control_t *c,const float q[7],const float v[7],float gain,float next)
{
    float actual[6],g[7]={0};
    for(unsigned j=0;j<6;++j)actual[j]=c->motors[j].position;
    if(can_console_arm_gravity_compute(&c->gravity,actual,g)!=ESP_OK) {disable(c,"gravity calculation failed",0);return false;}
    bool exceeded=false;
    for(unsigned j=0;j<7;++j) {
        if(!isfinite(q[j]) || !isfinite(v[j]) || q[j]<c->lower[j]-1e-5f || q[j]>c->upper[j]+1e-5f ||
            fabsf(g[j])>rb_daily_policy.torque_budget_nm[j]) {disable(c,"invalid target or gravity budget",j+1);return false;}
        float limit=(j==6?10.8f:5.f)*RAD;
        if(fabsf(q[j]-c->motors[j].position)>limit) {disable(c,"following error",j+1);return false;}
        float estimate=gain*(g[j]+rb_daily_policy.kp[j]*(q[j]-c->motors[j].position)+
                            rb_daily_policy.kd[j]*(v[j]-c->estimated_v[j]));
        c->estimated_torque[j]=estimate;
        if(fabsf(estimate)>rb_daily_policy.torque_budget_nm[j]) {
            exceeded=true;c->status.bad_motor=j+1;
        }
    }
    if(exceeded) {
        if(gain>=1 && c->status.state!=RB_DC_PROTECTIVE_HOLD && c->status.state!=RB_DC_HOME && c->status.state!=RB_DC_HOME_SETTLE)
            hold(c,"torque budget: protective hold",q);
        else disable(c,"hold/home torque budget",0);
        return false;
    }
    can_console_motor_target_t targets[7];
    for(unsigned j=0;j<7;++j) {
        if(can_console_motor_set_gains(j+1,rb_daily_policy.kp[j]*gain,rb_daily_policy.kd[j]*gain)!=ESP_OK) {
            disable(c,"gain update failed",j+1);return false;
        }
        targets[j]=(can_console_motor_target_t){.motor_id=j+1,.mode=MOTOR_MODE_MIT,
            .value={.position=q[j],.velocity=v[j],.torque=g[j]*gain}};
    }
    esp_err_t e=can_console_motor_submit_reference(targets,7,!c->owned,&c->sequence);
    if(e==ESP_ERR_TIMEOUT) {if(c->now-c->sent>100)disable(c,"reference queue timeout",0);return false;}
    if(e!=ESP_OK) {disable(c,"reference rejected",0);return false;}
    c->pending=true;c->sent=c->now;c->queued_elapsed=next;
    memcpy(c->target,q,sizeof(c->target));return true;
}
static void move(rb_daily_control_t *c,const float to[7],bool home)
{
    float largest=0;
    for(unsigned j=0;j<7;++j) {
        c->from[j]=c->motors[j].position;c->to[j]=to[j];
        largest=fmaxf(largest,fabsf(to[j]-c->from[j]));
        c->lower[j]=fminf(bounds_lo[j],c->from[j]);c->upper[j]=fmaxf(bounds_hi[j],c->from[j]);
    }
    float speed=home?rb_daily_policy.exit_speed:rb_daily_policy.position_speed;
    float acceleration=home?rb_daily_policy.exit_acceleration:rb_daily_policy.position_acceleration;
    c->duration=fmaxf(2,fmaxf(1.875f*largest/speed,sqrtf(5.773503f*largest/acceleration)));
    state(c,home?RB_DC_HOME:RB_DC_ENTRY);
}
static void approach(rb_daily_control_t *c)
{
    float q[7],v[7],a[7];
    /* WAKE is a measured-position -> idle move. Do not home first or replay
     * another zero -> idle curve after arriving at the idle target. */
    unsigned source=c->status.action==RB_DAILY_WAKE?RB_DAILY_IDLE:c->status.action;
    if(!rb_daily_sample(source,0,q,v,a)) {disable(c,"invalid action",0);return;}
    bool already=true;
    for(unsigned j=0;j<7;++j)already &= fabsf(c->target[j]-q[j])<1e-5f &&
        fabsf(c->motors[j].position-q[j])<(j==6?3.f:1.f)*RAD && fabsf(c->estimated_v[j])<RAD;
    if(already) {memcpy(c->target,q,sizeof(q));state(c,RB_DC_ENTRY_SETTLE);return;}
    move(c,q,false);
}
static bool arrived(rb_daily_control_t *c,bool zero)
{
    bool yes=true;
    for(unsigned j=0;j<7;++j) {
        float tolerance=(j==6?rb_daily_policy.gripper_tolerance_deg:
                         zero?rb_daily_policy.zero_tolerance_deg:rb_daily_policy.idle_tolerance_deg)*RAD;
        yes &= fabsf(c->motors[j].position-c->target[j])<=tolerance && fabsf(c->estimated_v[j])<=RAD;
    }
    if(!yes)c->still_since=0;
    else if(!c->still_since)c->still_since=c->now;
    return c->still_since && c->now-c->still_since>=500;
}
esp_err_t rb_daily_control_init(rb_daily_control_t *c,int channel)
{
    if(!c || channel<0)return ESP_ERR_INVALID_ARG;
    memset(c,0,sizeof(*c));c->channel=channel;
    can_console_arm_gravity_config_t g;
    can_console_arm_gravity_config_set_defaults(&g);g.model=CAN_CONSOLE_IK_MODEL_REBOTARM_RS;
    esp_err_t e=can_console_arm_gravity_init(&c->gravity,&g);
    c->initialized=e==ESP_OK;return e;
}
esp_err_t rb_daily_control_request(rb_daily_control_t *c,unsigned action,unsigned id)
{
    if(!c || !c->initialized || !rb_daily_find(action) || !id)return ESP_ERR_INVALID_ARG;
    rb_dc_state_t s=c->status.state;
    bool retry=s==RB_DC_FAULT && !c->status.enabled && !c->owned;
    if(s!=RB_DC_OFF && s!=RB_DC_HOLD && s!=RB_DC_ZERO_HOLD && !retry)return ESP_ERR_INVALID_STATE;
    if(action==RB_DAILY_SLEEP && s!=RB_DC_ZERO_HOLD)return ESP_ERR_INVALID_STATE;
    c->status.action=action;c->status.request_id=id;c->status.phase=0;c->status.error=NULL;c->status.bad_motor=0;
    memset(&c->status.hold,0,sizeof(c->status.hold));
    c->home_requested=c->pause_requested=false;c->ui_at=c->now;
    if(action==RB_DAILY_SLEEP)disable(c,NULL,0);
    else if(s==RB_DC_OFF || retry) {c->status.disable_confirmed=false;state(c,RB_DC_PRECHECK);}
    else {state(c,RB_DC_ENTRY);c->duration=0;} /* plan from next fresh sample */
    return ESP_OK;
}
void rb_daily_control_home(rb_daily_control_t *c) { if(c)c->home_requested=true; }
void rb_daily_control_pause(rb_daily_control_t *c) { if(c)c->pause_requested=true; }
void rb_daily_control_heartbeat(rb_daily_control_t *c,uint64_t now) {c->ui_at=now;}
void rb_daily_control_tick(rb_daily_control_t *c,uint64_t now)
{
    if(!c || !c->initialized)return;
    c->now=now;
    if(c->status.state==RB_DC_OFF || c->status.state==RB_DC_FAULT){c->last_tick=now;return;}
    if(c->last_tick && now-c->last_tick>100 && c->status.enabled)disable(c,"control deadline exceeded",0);
    c->last_tick=now;
    bool fresh=feedback(c),bus=bus_ready(c);
    if(c->status.state==RB_DC_DISABLING) {
        if(!c->disable_sent) {
            if(can_console_motor_submit_admin(&(can_console_motor_admin_command_t){0,CAN_CONSOLE_MOTOR_ADMIN_ESTOP})==ESP_OK) {
                c->disable_sent=true;c->enable_at=now;
            }
        } else if(fresh) {
            bool off=true;
            for(unsigned j=0;j<7;++j)off &= !c->motors[j].enabled && (c->motors[j].status&3)==0 &&
                c->motors[j].last_feedback_us>(int64_t)c->enable_at*1000;
            if(off) {
                c->status.enabled=false;c->status.disable_confirmed=true;c->owned=false;
                c->status.completed_id=c->status.request_id;
                state(c,c->status.error?RB_DC_FAULT:RB_DC_OFF);
            }
        }
        if(now-c->changed>4000 && !c->status.disable_confirmed)c->status.error="disable unconfirmed: use independent stop";
        return;
    }
    if(!bus) {disable(c,"CAN offline",0);return;}
    if(c->home_requested && !c->status.enabled) {disable(c,NULL,0);return;}
    if(!fresh) {
        if(c->status.state==RB_DC_PRECHECK && now-c->changed<4000) {
            if(now-c->report_at>=200) {
                for(unsigned j=1;j<=7;++j)(void)can_console_motor_request_rs_report(j,253);
                c->report_at=now;
            }
        } else disable(c,"stale status feedback",c->status.bad_motor);
        return;
    }
    bool running=c->status.state>=RB_DC_RAMP;
    if(!check_feedback(c,running))return;
    for(unsigned j=0;j<7;++j)if(c->motors[j].mos_temperature>=100) {
        if(!c->status.enabled){disable(c,"temperature before enable",j+1);return;}
        if(c->status.state!=RB_DC_HOME && c->status.state!=RB_DC_HOME_SETTLE)c->home_requested=true;
    }
    if(!reference_ready(c))return;
    if(c->home_requested && c->status.enabled && c->status.state!=RB_DC_RAMP) {
        float zero[7]={0};zero[6]=c->motors[6].position;
        c->home_requested=false;c->pause_requested=false;c->status.error=NULL;
        move(c,zero,true);
    } else if(c->status.enabled && c->status.state!=RB_DC_HOME && c->status.state!=RB_DC_HOME_SETTLE &&
              (c->pause_requested || now-c->ui_at>500) && c->status.state!=RB_DC_PROTECTIVE_HOLD) {
        bool requested=c->pause_requested;c->pause_requested=false;
        if(c->status.state==RB_DC_RAMP) {disable(c,"startup cancelled",0);return;}
        hold(c,requested?"pause requested: HOME to exit":"UI heartbeat timeout: HOME to exit",NULL);
    }
    float q[7],v[7]={0},a[7];
    switch(c->status.state) {
    case RB_DC_PRECHECK: {
        bool still=true;
        for(unsigned j=0;j<7;++j) {
            if(c->motors[j].enabled || (c->motors[j].status&3)!=0) {disable(c,"motor already controlled",j+1);return;}
            still &= fabsf(c->estimated_v[j])<=RAD;
        }
        if(!still)c->still_since=0;
        else if(!c->still_since)c->still_since=now;
        if(now-c->changed>4000){disable(c,"startup not stationary",0);return;}
        if(!c->still_since || now-c->still_since<200)return;
        float actual[6],gravity[6];
        for(unsigned j=0;j<6;++j)actual[j]=c->motors[j].position;
        if(can_console_arm_gravity_compute(&c->gravity,actual,gravity)!=ESP_OK) {disable(c,"startup gravity failed",0);return;}
        for(unsigned j=0;j<6;++j)if(fabsf(gravity[j])>rb_daily_policy.torque_budget_nm[j]) {
            disable(c,"startup gravity exceeds budget",j+1);return;
        }
        /* Any partial failure after this point must cancel the service mode
         * work, even when enable has not yet been requested. */
        state(c,RB_DC_MODE);
        can_console_motor_target_t targets[7];
        for(unsigned j=0;j<7;++j) {
            c->anchor[j]=c->motors[j].position;
            c->lower[j]=fminf(bounds_lo[j],c->anchor[j]);c->upper[j]=fmaxf(bounds_hi[j],c->anchor[j]);
            if(can_console_motor_set_gains(j+1,0,0)!=ESP_OK){disable(c,"initial gains failed",j+1);return;}
            targets[j]=(can_console_motor_target_t){.motor_id=j+1,.mode=MOTOR_MODE_MIT,.value={.position=c->anchor[j]}};
        }
        if(can_console_motor_set_targets(targets,7)!=ESP_OK){disable(c,"initial targets failed",0);return;}
        state(c,RB_DC_MODE);c->fence=0;break;
    }
    case RB_DC_MODE: {
        if(now-c->changed>4000){disable(c,"mode preparation timeout",0);return;}
        if(!c->fence) {
            esp_err_t e=can_console_motor_fence_begin(c->channel,&c->fence);
            if(e!=ESP_OK && e!=ESP_ERR_TIMEOUT)disable(c,"mode fence failed",0);
            return;
        }
        can_console_motor_fence_status_t f;
        if(can_console_motor_fence_poll(c->fence,&f)!=ESP_OK || (f.complete && f.error!=ESP_OK)) {disable(c,"mode delivery failed",0);return;}
        if(!f.complete)return;
        if(can_console_motor_submit_admin_range(&(can_console_motor_admin_range_t){1,7,CAN_CONSOLE_MOTOR_ADMIN_ENABLE})!=ESP_OK) {
            disable(c,"enable rejected",0);return;
        }
        c->enable_at=now;state(c,RB_DC_ENABLE);break;
    }
    case RB_DC_ENABLE: {
        bool enabled=true;
        for(unsigned j=0;j<7;++j) enabled &= c->motors[j].enabled && c->motors[j].mode_ready &&
            c->motors[j].mode==MOTOR_MODE_MIT && (c->motors[j].status&3)==2 && c->motors[j].last_feedback_us>(int64_t)c->enable_at*1000;
        if(enabled) {
            for(unsigned j=0;j<7;++j) {
                c->anchor[j]=c->motors[j].position;
                c->lower[j]=fminf(bounds_lo[j],c->anchor[j]);
                c->upper[j]=fmaxf(bounds_hi[j],c->anchor[j]);
            }
            c->status.enabled=true;c->sent=now;state(c,RB_DC_RAMP);
        } else if(now-c->changed>4000)disable(c,"enable confirmation timeout",0);
        break;
    }
    case RB_DC_RAMP:
        if(c->elapsed>=1.5f) {
            if(c->home_requested) {
                float zero[7]={0};zero[6]=c->motors[6].position;c->home_requested=false;
                move(c,zero,true);
            } else approach(c);
            break;
        }
        (void)send_target(c,c->anchor,v,fminf(1,(c->elapsed+PERIOD)/1.5f),c->elapsed+PERIOD);break;
    case RB_DC_ENTRY: case RB_DC_HOME: {
        if(c->duration==0){approach(c);break;}
        if(c->elapsed>=c->duration) {state(c,c->status.state==RB_DC_HOME?RB_DC_HOME_SETTLE:RB_DC_ENTRY_SETTLE);break;}
        float t=fminf(c->duration,c->elapsed+PERIOD),u=t/c->duration;
        float s=u*u*u*(10+u*(-15+6*u)),d=30*u*u*(1-u)*(1-u)/c->duration;
        for(unsigned j=0;j<7;++j){q[j]=c->from[j]+s*(c->to[j]-c->from[j]);v[j]=d*(c->to[j]-c->from[j]);}
        (void)send_target(c,q,v,1,t);break;
    }
    case RB_DC_PLAY: {
        const rb_daily_action_t *plan=rb_daily_find(c->status.action);
        if(c->elapsed>=plan->duration_s) {state(c,RB_DC_SETTLE);break;}
        float t=fminf(plan->duration_s,c->elapsed+PERIOD);
        if(!rb_daily_sample(plan->id,t,q,v,a)){disable(c,"plan evaluation failed",0);break;}
        (void)send_target(c,q,v,1,t);break;
    }
    case RB_DC_ENTRY_SETTLE: case RB_DC_SETTLE: case RB_DC_HOME_SETTLE: {
        bool home=c->status.state==RB_DC_HOME_SETTLE;
        if(arrived(c,home || c->status.action==RB_DAILY_REST || c->status.action==RB_DAILY_SLEEP)) {
            if(home) {disable(c,NULL,0);break;}
            if(c->status.state==RB_DC_ENTRY_SETTLE && c->status.action!=RB_DAILY_WAKE &&
                rb_daily_find(c->status.action)->piece_count)state(c,RB_DC_PLAY);
            else {
                c->status.completed_id=c->status.request_id;c->status.phase=1;
                state(c,c->status.action==RB_DAILY_REST?RB_DC_ZERO_HOLD:RB_DC_HOLD);
            }
        } else if(now-c->changed>8000) {hold(c,"arrival timeout: HOME to exit",NULL);break;}
        (void)send_target(c,c->target,v,1,c->elapsed);break;
    }
    case RB_DC_HOLD: case RB_DC_ZERO_HOLD: case RB_DC_PROTECTIVE_HOLD:
        (void)send_target(c,c->target,v,1,c->elapsed);break;
    default:break;
    }
}
const char *rb_daily_control_state_name(rb_dc_state_t s)
{
    static const char *names[]={"OFF","CHECK","MODE","ENABLE","RAMP","APPROACH","SETTLE",
        "PLAY","SETTLE","IDLE HOLD","ZERO HOLD","PROTECTIVE HOLD","HOME","HOME SETTLE","DISABLING","FAULT"};
    return s<=RB_DC_FAULT?names[s]:"UNKNOWN";
}
