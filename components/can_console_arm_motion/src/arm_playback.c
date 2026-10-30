#include "can_console_arm_playback.h"
#include "esp_timer.h"
#include "arm_trajectory.h"
#include <math.h>
#include <string.h>

#define PERIOD_US 20000LL
#define DELIVERY_TIMEOUT_US 100000LL
#define PLANNING_TIMEOUT_US 2000000LL

static bool clocked(const arm_playback_t *p)
{
    return p->recorded_timing || p->planned_timing;
}

const char *arm_playback_phase_name(arm_playback_phase_t phase)
{
    static const char *const names[]={"idle","preparing","acquiring","running","stopping","draining","releasing","cancelling"};
    return phase<=ARM_PLAYBACK_CANCELLING?names[phase]:"unknown";
}

static bool valid_budgets(const arm_playback_calibration_t *c)
{
    if(c==NULL || c->coordinate_version==0) return false;
    for(unsigned j=0;j<6;++j) {
        const float fields[]={c->limits.velocity[j],c->limits.acceleration[j],c->start_tolerance[j]};
        for(unsigned k=0;k<sizeof(fields)/sizeof(fields[0]);++k) if(!isfinite(fields[k]) || fields[k]<=0)return false;
    }
    return true;
}

static float gripper_target(const arm_playback_t *p,const can_console_arm_motion_sample_t *point,unsigned kind)
{
    if(kind==1)return p->seek?p->gripper_start:p->gripper_first;
    if(p->gripper_hold)return p->gripper_stop_position;
    if(kind==3)return p->gripper_confirmed;
    if(p->seek) {
        const float u=fminf(1,fmaxf(0,point->reference_time_s/p->gripper_seek_duration));
        return p->gripper_start+(p->gripper_first-p->gripper_start)*arm_trajectory_profile(u);
    }
    /* Use the planner's source time, never a second replay clock. */
    const double time_ms=point->reference_time_s*1000.0;
    uint32_t lo=0,hi=p->gripper_sample_count-1;
    while(lo+1<hi) {const uint32_t mid=lo+(hi-lo)/2;
        if(p->gripper_samples[mid].time_ms<=time_ms)lo=mid;else hi=mid;}
    const arm_teach_sample_t *a=&p->gripper_samples[lo],*b=&p->gripper_samples[hi];
    const double u=fmin(1,fmax(0,(time_ms-a->time_ms)/(b->time_ms-a->time_ms)));
    const float opening=(float)((a->gripper_q+u*((double)b->gripper_q-a->gripper_q))/65535.0);
    const arm_gripper_profile_t *gripper=&p->profile->gripper;
    return gripper->closed_position+opening*(gripper->open_position-gripper->closed_position);
}

static void targets(const arm_playback_t *p,const can_console_arm_motion_sample_t *point,
    can_console_motor_target_t out[7])
{
    for(unsigned j=0;j<6;++j) {
        if(p->position_mode) {
            /* POS_VEL frames carry position+vlim only; the vendor servo closes
             * the loop. Original-time playback uses the configured move vlim;
             * ordinary seeks use normal motion vlim, probes use their budgets. */
            out[j]=(can_console_motor_target_t){
                .motor_id=p->profile->joints[j].motor_id,.mode=CAN_CONSOLE_MOTOR_MODE_POS_VEL,
                .value={.position=point->position[j],.velocity_limit=p->drive_velocity_limit[j]},
            };
            continue;
        }
        out[j]=(can_console_motor_target_t){
            .motor_id=p->profile->joints[j].motor_id,.mode=CAN_CONSOLE_MOTOR_MODE_MIT,
            .value={.position=point->position[j],.torque=p->torque[j],.velocity_limit=p->drive_velocity_limit[j]},
        };
    }
}

static esp_err_t submit(arm_playback_t *p,const can_console_arm_motion_sample_t *point,
    bool acquire,unsigned kind)
{
    can_console_motor_target_t group[7];targets(p,point,group);
    const float gripper=p->gripper_owned?gripper_target(p,point,kind):0;
    if(p->gripper_owned)group[6]=(can_console_motor_target_t){.motor_id=p->profile->gripper.motor_id,
        .mode=p->position_mode?CAN_CONSOLE_MOTOR_MODE_POS_VEL:CAN_CONSOLE_MOTOR_MODE_MIT,
        .value={.position=gripper,.velocity_limit=p->profile->gripper.velocity_limit}};
    const esp_err_t error=can_console_motor_submit_reference(group,p->gripper_owned?7:6,acquire,&p->sequence);
    if(error==ESP_OK) {
        p->pending=true;p->pending_kind=kind;p->queued=*point;
        if(p->gripper_owned)p->gripper_queued=gripper;
        memcpy(p->queued_torque,p->torque,sizeof(p->queued_torque));
        p->submitted_us=esp_timer_get_time();
        if(acquire)p->acquired=true;
    }
    return error;
}

static esp_err_t begin(arm_playback_t *p,const arm_teach_trajectory_t *trajectory,
    const arm_hardware_profile_t *profile,const arm_playback_feedback_t *feedback,bool seek,float speed,
    bool probe,bool position_mode,bool follow)
{
    if(p==NULL || trajectory==NULL || profile==NULL || feedback==NULL)return ESP_ERR_INVALID_ARG;
    if(p->phase!=ARM_PLAYBACK_IDLE) return ESP_ERR_INVALID_STATE;
    if(!valid_budgets(profile->playback))
        return ESP_ERR_NOT_SUPPORTED;
    const arm_playback_calibration_t *c=profile->playback;
    const bool normal_seek=seek && !probe;
    const bool recorded_timing=c->recorded_timing && !seek && !probe;
    float drive_velocity_limit[6];
    float configured_velocity[6];
    if((normal_seek || recorded_timing) &&
        can_console_arm_motion_get_velocity_limits(configured_velocity)!=ESP_OK)return ESP_ERR_INVALID_STATE;
    const arm_teach_metadata_t *m=&trajectory->metadata;
    const uint8_t model=profile->ik_model==CAN_CONSOLE_IK_MODEL_REBOTARM_RS?ARM_TEACH_MODEL_RS:ARM_TEACH_MODEL_DM;
    if(trajectory->samples==NULL || m->sample_count<(seek?1U:2U) || m->sample_count>trajectory->capacity ||
        m->model_id!=model ||
        (m->sample_format!=ARM_TEACH_SAMPLE_FORMAT_GRIPPER || m->channel_mask!=ARM_TEACH_CHANNELS_GRIPPER) ||
        m->arm_dof!=6 ||
        m->coordinate_version!=c->coordinate_version || m->calibration_id!=c->calibration_id) return ESP_ERR_INVALID_ARG;
    const bool gripper=!probe;
    float gripper_first=0,seek_duration=.1f;
    if(gripper) {
        if(!arm_gripper_profile_valid(&profile->gripper) || !feedback->gripper_valid ||
            !isfinite(feedback->gripper_position) || m->gripper_motor_id!=profile->gripper.motor_id ||
            m->gripper_calibration_id!=profile->gripper.calibration_id ||
            arm_gripper_position(&profile->gripper,trajectory->samples[0].gripper_q/65535.0f,&gripper_first)!=ESP_OK)
            return ESP_ERR_INVALID_ARG;
        if(!seek && !follow && fabsf(feedback->gripper_position-gripper_first)>profile->gripper.start_tolerance)
            return ESP_ERR_INVALID_STATE;
        seek_duration=fmaxf(seek_duration,1.875f*fabsf(gripper_first-feedback->gripper_position)/profile->gripper.velocity_limit);
    }
    float first[6];
    for(unsigned j=0;j<6;++j) {
        if(m->motor_ids[j]!=profile->joints[j].motor_id || !isfinite(feedback->position[j]) || !isfinite(feedback->velocity[j])) return ESP_ERR_INVALID_STATE;
        first[j]=trajectory->samples[0].position_q[j]/ARM_TEACH_POSITION_SCALE;
        /* Automatic handoff continues the confirmed seek reference without
         * imposing another measured-position arrival gate. */
        if(!seek && !follow && fabsf(feedback->position[j]-first[j])>c->start_tolerance[j]) return ESP_ERR_INVALID_STATE;
        drive_velocity_limit[j]=c->limits.velocity[j];
        if(normal_seek || recorded_timing) {
            if(!isfinite(configured_velocity[j]) || configured_velocity[j]<=0)return ESP_ERR_INVALID_STATE;
            drive_velocity_limit[j]=configured_velocity[j];
        }
        if(recorded_timing && position_mode) {
            can_console_motor_state_t state;
            if(can_console_motor_get_state(profile->joints[j].motor_id,&state)!=ESP_OK ||
                !isfinite(state.velocity_max) || state.velocity_max<=0)return ESP_ERR_INVALID_STATE;
            if(!isfinite(configured_velocity[j]) || configured_velocity[j]<=0)return ESP_ERR_INVALID_STATE;
            drive_velocity_limit[j]=fminf(state.velocity_max,configured_velocity[j]);
        }
    }
    const esp_err_t error=seek?(normal_seek?
        can_console_arm_motion_plan_joints(feedback->position,first,seek_duration):
        can_console_arm_motion_plan_joints_limited(feedback->position,first,seek_duration,&c->limits)):
        can_console_arm_motion_plan_joint_path(&(arm_joint_path_view_t){trajectory->samples,m->sample_count},
            recorded_timing?NULL:&c->limits,speed);
    if(error!=ESP_OK)return error;
    /* Clear the app-owned state in place rather than on the control stack. */
    memset(p,0,sizeof(*p));
    p->phase=ARM_PLAYBACK_PREPARING;p->profile=profile;p->seek=seek;p->follow=follow;p->probe=probe;
    p->gripper_owned=gripper;p->gripper_samples=trajectory->samples;p->gripper_sample_count=m->sample_count;
    p->gripper_start=feedback->gripper_position;p->gripper_first=gripper_first;
    p->gripper_confirmed=seek?feedback->gripper_position:gripper_first;
    p->gripper_queued=p->gripper_confirmed;p->gripper_seek_duration=seek_duration;
    p->position_mode=position_mode;p->step_ms=40;p->begun_us=esp_timer_get_time();
    p->recorded_timing=recorded_timing;p->planned_timing=normal_seek;p->speed=normal_seek?1:speed;
    p->max_feedback_age_ms=feedback->age_ms;p->max_feedback_skew_ms=feedback->skew_ms;
    memcpy(p->measured,feedback->position,sizeof(p->measured));
    memcpy(p->drive_velocity_limit,drive_velocity_limit,sizeof(p->drive_velocity_limit));
    memcpy(p->confirmed.position,feedback->position,sizeof(p->confirmed.position));
    if(!seek)memcpy(p->confirmed.position,first,sizeof(p->confirmed.position));
    return ESP_OK;
}

esp_err_t arm_playback_begin(arm_playback_t *p,const arm_teach_trajectory_t *trajectory,
    const arm_hardware_profile_t *profile,const arm_playback_feedback_t *feedback,bool seek,float speed)
{
    if(profile==NULL)return ESP_ERR_INVALID_ARG;
    const bool position_mode=profile->playback!=NULL && profile->playback->position_mode;
    return begin(p,trajectory,profile,feedback,seek,speed,false,position_mode,false);
}

/* teach replay: play after the seek's final reference has been delivered. */
esp_err_t arm_playback_begin_follow(arm_playback_t *p,const arm_teach_trajectory_t *trajectory,
    const arm_hardware_profile_t *profile,const arm_playback_feedback_t *feedback,float speed)
{
    if(profile==NULL)return ESP_ERR_INVALID_ARG;
    const bool position_mode=profile->playback!=NULL && profile->playback->position_mode;
    return begin(p,trajectory,profile,feedback,false,speed,false,position_mode,true);
}

esp_err_t arm_playback_begin_home(arm_playback_t *p,
    const arm_playback_feedback_t *feedback, float duration_s)
{
    if(p==NULL || feedback==NULL || !isfinite(duration_s) || duration_s<=0)
        return ESP_ERR_INVALID_ARG;
    if(p->phase!=ARM_PLAYBACK_IDLE || p->reason!=ARM_PLAYBACK_COMPLETED ||
        p->seek || p->probe || p->home || p->profile==NULL)return ESP_ERR_INVALID_STATE;
    float velocity[6];
    esp_err_t error=can_console_arm_motion_get_velocity_limits(velocity);
    if(error!=ESP_OK)return error;
    for(unsigned j=0;j<6;++j)
        if(!isfinite(feedback->position[j]) || !isfinite(feedback->velocity[j]) ||
            !isfinite(velocity[j]) || velocity[j]<=0)return ESP_ERR_INVALID_STATE;
    const float home[6]={0};
    /* Feedback may lag the delivered endpoint. Keep the reference continuous;
     * measured position is still used for gravity and user-stop capture. */
    error=can_console_arm_motion_plan_joints(p->confirmed.position,home,duration_s);
    if(error!=ESP_OK)return error;
    const arm_hardware_profile_t *profile=p->profile;
    const bool gravity=p->gravity_enabled, position_mode=p->position_mode;
    const can_console_arm_motion_sample_t initial=p->confirmed;
    float torque[6];memcpy(torque,p->queued_torque,sizeof(torque));
    memset(p,0,sizeof(*p));
    p->profile=profile;p->home=true;p->follow=true;
    p->phase=ARM_PLAYBACK_PREPARING;p->planned_timing=true;p->speed=1;
    p->position_mode=position_mode;p->gravity_enabled=gravity;p->step_ms=40;
    p->begun_us=esp_timer_get_time();p->confirmed=initial;
    memcpy(p->drive_velocity_limit,velocity,sizeof(velocity));
    memcpy(p->measured,feedback->position,sizeof(p->measured));
    memcpy(p->torque,torque,sizeof(torque));
    memcpy(p->queued_torque,torque,sizeof(torque));
    memcpy(p->initial_torque,torque,sizeof(torque));
    return ESP_OK;
}

esp_err_t arm_playback_begin_probe(arm_playback_t *p,arm_playback_probe_t *scratch,
    const arm_hardware_profile_t *profile,const arm_playback_feedback_t *feedback,
    bool seek,unsigned joint,float delta,float velocity,float acceleration,bool position_mode)
{
    if(p==NULL || scratch==NULL || profile==NULL || feedback==NULL)return ESP_ERR_INVALID_ARG;
    if(p->phase!=ARM_PLAYBACK_IDLE)return ESP_ERR_INVALID_STATE;
    /* RS commissioning is MIT-only: the POS_VEL probe would need the
     * multi-frame RS parameter path that the reference delivery does not
     * provide. */
    if(position_mode && profile->ik_model!=CAN_CONSOLE_IK_MODEL_REBOTARM_DM)return ESP_ERR_NOT_SUPPORTED;
    /* Synthetic single-joint commissioning path. Budgets are temporary and
     * never modify the profile, recording or stored trajectories. */
    if(joint<1 || joint>6 || !isfinite(delta) || fabsf(delta)<.001f || fabsf(delta)>.3f ||
        !isfinite(velocity) || velocity<.005f || velocity>.06f ||
        !isfinite(acceleration) || acceleration<.01f || acceleration>.15f ||
        velocity*velocity/acceleration>.03f)return ESP_ERR_INVALID_ARG;
    const bool dm=profile->ik_model==CAN_CONSOLE_IK_MODEL_REBOTARM_DM;
    *scratch=(arm_playback_probe_t){.profile=*profile,.budgets={.coordinate_version=1}};
    scratch->profile.playback=&scratch->budgets;
    arm_teach_metadata_t metadata={.sample_format=ARM_TEACH_SAMPLE_FORMAT_GRIPPER,.model_id=dm?ARM_TEACH_MODEL_DM:ARM_TEACH_MODEL_RS,
        .arm_dof=6,.channel_mask=ARM_TEACH_CHANNELS_GRIPPER,.sample_period_ms=20,.coordinate_version=1,.sample_count=2,
        .duration_ms=100,.end_reason=ARM_TEACH_END_USER};
    const float start_tolerance=dm?.001f:.002f;
    for(unsigned j=0;j<6;++j) {
        scratch->budgets.limits.velocity[j]=velocity;
        scratch->budgets.limits.acceleration[j]=acceleration;
        scratch->budgets.start_tolerance[j]=start_tolerance;
        metadata.motor_ids[j]=profile->joints[j].motor_id;
        const float q=feedback->position[j];
        if(!isfinite(q) || fabsf(q)>3.2f)return ESP_ERR_INVALID_ARG;
        const int32_t start=(int32_t)lroundf(q*ARM_TEACH_POSITION_SCALE);
        const int32_t end=(int32_t)lroundf((q+(j==joint-1?delta:0))*ARM_TEACH_POSITION_SCALE);
        if(start<INT16_MIN || start>INT16_MAX || end<INT16_MIN || end>INT16_MAX)return ESP_ERR_INVALID_ARG;
        scratch->samples[0].position_q[j]=seek?end:start;
        scratch->samples[1].position_q[j]=end;
    }
    scratch->samples[1].time_ms=100;
    const arm_teach_trajectory_t trajectory={.metadata=metadata,.capacity=2,.samples=scratch->samples};
    return begin(p,&trajectory,&scratch->profile,feedback,seek,.1f,true,position_mode,false);
}

void arm_playback_stop(arm_playback_t *p)
{
    if(p->stop_requested || p->phase==ARM_PLAYBACK_IDLE || p->phase==ARM_PLAYBACK_CANCELLING)return;
    p->stop_requested=true;p->reason=ARM_PLAYBACK_USER_STOP;
    p->stop_requested_us=esp_timer_get_time();
    memcpy(p->stop_origin,p->measured,sizeof(p->stop_origin));
    /* A release already in flight must still remember the user's stop so
     * completion cannot schedule a loop or automatic home afterwards. */
    if(p->phase!=ARM_PLAYBACK_RELEASING)can_console_arm_motion_cancel();
}

void arm_playback_abort(arm_playback_t *p,arm_playback_reason_t reason,esp_err_t error)
{
    if(p->phase==ARM_PLAYBACK_IDLE)return;
    can_console_arm_motion_cancel();p->phase=ARM_PLAYBACK_CANCELLING;p->reason=reason;p->last_error=error;
}

void arm_playback_retire(arm_playback_t *p)
{
    if(p->phase!=ARM_PLAYBACK_CANCELLING)return;
    can_console_arm_motion_status_t motion;
    if(can_console_arm_motion_get_status(&motion)!=ESP_OK || motion.planner_busy)return;
    if(p->acquired) {
        can_console_motor_reference_status_t receipt;
        if(can_console_motor_get_reference_status(p->sequence,&receipt)!=ESP_OK || receipt.owned)return;
    }
    p->phase=ARM_PLAYBACK_IDLE;p->pending=false;p->acquired=false;p->ended_us=esp_timer_get_time();
}

static esp_err_t confirmation(arm_playback_t *p,int64_t now)
{
    if(!p->pending)return ESP_OK;
    can_console_motor_reference_status_t receipt;
    esp_err_t error=can_console_motor_get_reference_status(p->sequence,&receipt);
    if(error!=ESP_OK)return error;
    if(receipt.state==CAN_CONSOLE_MOTOR_REFERENCE_FAILED)return receipt.error!=ESP_OK?receipt.error:ESP_FAIL;
    if(receipt.state==CAN_CONSOLE_MOTOR_REFERENCE_PENDING)
        return p->probe && now-p->submitted_us>=DELIVERY_TIMEOUT_US?ESP_ERR_TIMEOUT:ESP_OK;
    const int64_t ack=now-p->submitted_us;
    if(ack>p->max_ack_us)p->max_ack_us=(uint32_t)ack;
    if(p->pending_kind==4) {
        if(receipt.owned)return ESP_ERR_INVALID_STATE;
        p->phase=ARM_PLAYBACK_IDLE;p->acquired=false;p->pending=false;p->ended_us=now;
        if(p->reason==ARM_PLAYBACK_NONE)p->reason=ARM_PLAYBACK_COMPLETED;
        return ESP_OK;
    }
    if(!receipt.owned || receipt.confirmed_us<p->submitted_us || receipt.confirmed_us>now ||
        (p->probe && now-receipt.confirmed_us>DELIVERY_TIMEOUT_US)) {
        return ESP_ERR_INVALID_STATE;
    }
    /* A confirmed duplicate hold refreshes delivery liveness, but never the
     * motion secant clock or its velocity estimate. DEFER may legitimately
     * stretch the interval between distinct motion points beyond 100 ms. */
    p->last_delivery_us=receipt.confirmed_us;
    if(p->pending_kind==1) {
        p->confirmed=p->queued;p->gripper_confirmed=p->gripper_queued;p->confirmed_us=receipt.confirmed_us;
        p->timeline_us=receipt.confirmed_us;
        p->phase=ARM_PLAYBACK_RUNNING;
    } else if(p->pending_kind==2) {
        const int64_t dt=receipt.confirmed_us-p->confirmed_us;
        if(dt<=0)return ESP_ERR_INVALID_STATE;
        const arm_playback_calibration_t *c=p->profile->playback;
        for(unsigned j=0;j<6;++j) {
            const float velocity=(p->queued.position[j]-p->confirmed.position[j])*1000000.f/dt;
            if(!clocked(p) && (fabsf(velocity)>c->limits.velocity[j]+1e-6f ||
               fabsf(velocity-p->execution_velocity[j])>c->limits.acceleration[j]*(dt*1e-6f)+1e-6f)) {
                return ESP_ERR_INVALID_STATE;
            }
            p->execution_velocity[j]=velocity;
        }
        if(p->min_delivery_us==0 || dt<p->min_delivery_us)p->min_delivery_us=(uint32_t)dt;
        if(dt>p->max_delivery_us)p->max_delivery_us=(uint32_t)dt;
        p->step_ms=(uint32_t)((dt+10000)/20000)*20;
        if(p->step_ms<20)p->step_ms=20;
        p->confirmed=p->queued;p->gripper_confirmed=p->gripper_queued;p->confirmed_us=receipt.confirmed_us;++p->confirmed_points;
    }
    p->pending=false;
    return ESP_OK;
}

static can_console_arm_motion_sample_result_t apply(const uint8_t ids[6],
    const can_console_arm_motion_sample_t *point,const float limits[6],void *context)
{
    (void)limits;arm_playback_t *p=context;
    for(unsigned j=0;j<6;++j)if(ids[j]!=p->profile->joints[j].motor_id)return CAN_CONSOLE_ARM_MOTION_SAMPLE_ERROR;
    const int64_t dt=esp_timer_get_time()-p->confirmed_us;
    if(dt<=0)return CAN_CONSOLE_ARM_MOTION_SAMPLE_ERROR;
    for(unsigned j=0;j<6;++j) {
        const float velocity=(point->position[j]-p->confirmed.position[j])*1000000.f/dt;
        const arm_joint_path_limits_t *cap=&p->profile->playback->limits;
        if(fabsf(velocity)>cap->velocity[j]+1e-6f ||
            fabsf(velocity-p->execution_velocity[j])>cap->acceleration[j]*(dt*1e-6f)+1e-6f) {
            /* This point has not been queued. A too-early offer is backpressure,
             * not evidence that a delivered reference broke its limits. Keep
             * the same point; confirmation still checks actual TX timing. */
            ++p->deferrals;
            return CAN_CONSOLE_ARM_MOTION_SAMPLE_DEFER;
        }
    }
    const esp_err_t error=submit(p,point,false,2);
    if(error==ESP_ERR_TIMEOUT){++p->deferrals;return CAN_CONSOLE_ARM_MOTION_SAMPLE_DEFER;}
    if(error!=ESP_OK) {p->last_error=error;return CAN_CONSOLE_ARM_MOTION_SAMPLE_ERROR;}
    p->motion_accepted=true;p->empty_us=0;
    return CAN_CONSOLE_ARM_MOTION_SAMPLE_APPLIED;
}

/* Advance by the recording's clock, coalescing overdue planner points while
 * a CAN receipt was pending. Transport latency must not multiply path time.
 * A staged point survives queue backpressure until a newer due point replaces
 * it or it is submitted; the final point is always submitted and confirmed. */
static can_console_arm_motion_sample_result_t collect_due(const uint8_t ids[6],
    const can_console_arm_motion_sample_t *point,const float limits[6],void *context)
{
    (void)limits;arm_playback_t *p=context;
    for(unsigned j=0;j<6;++j)if(ids[j]!=p->profile->joints[j].motor_id)
        return CAN_CONSOLE_ARM_MOTION_SAMPLE_ERROR;
    const double elapsed=(esp_timer_get_time()-p->timeline_us)*1e-6;
    if(point->reference_time_s>elapsed*p->speed+1e-6)return CAN_CONSOLE_ARM_MOTION_SAMPLE_DEFER;
    if(p->due_ready)++p->skipped_points;
    p->due=*point;p->due_ready=true;
    return CAN_CONSOLE_ARM_MOTION_SAMPLE_APPLIED;
}

/* Position error is diagnostic only. Do not interrupt a trajectory because
 * the position servo is behind the current reference. */
static void measure_tracking(arm_playback_t *p,const arm_playback_feedback_t *f)
{
    for(unsigned j=0;j<6;++j) {
        const float error=fabsf(p->confirmed.position[j]-f->position[j]);
        p->max_tracking[j]=fmaxf(p->max_tracking[j],error);
    }
}

esp_err_t arm_playback_tick(arm_playback_t *p,const arm_playback_feedback_t *feedback)
{
    if(p->phase==ARM_PLAYBACK_IDLE || p->phase==ARM_PLAYBACK_CANCELLING)return ESP_OK;
    const int64_t now=esp_timer_get_time();
    if(p->gripper_owned && (!feedback->gripper_valid || !isfinite(feedback->gripper_position)))return ESP_ERR_INVALID_STATE;
    if(feedback->age_ms>p->max_feedback_age_ms)p->max_feedback_age_ms=feedback->age_ms;
    if(feedback->skew_ms>p->max_feedback_skew_ms)p->max_feedback_skew_ms=feedback->skew_ms;
    for(unsigned j=0;j<6;++j) {
        if(!isfinite(feedback->position[j]) || !isfinite(feedback->velocity[j]))return ESP_ERR_INVALID_STATE;
        p->measured[j]=feedback->position[j];
        p->max_measured_velocity[j]=fmaxf(p->max_measured_velocity[j],fabsf(feedback->velocity[j]));
        if(p->stop_requested)p->max_stop_travel[j]=fmaxf(p->max_stop_travel[j],fabsf(p->measured[j]-p->stop_origin[j]));
    }
    esp_err_t error=confirmation(p,now);
    if(error!=ESP_OK || p->phase==ARM_PLAYBACK_IDLE)return error;
    if(p->probe && p->last_delivery_us!=0 && now-p->last_delivery_us>DELIVERY_TIMEOUT_US)return ESP_ERR_TIMEOUT;
    can_console_arm_motion_status_t motion;
    error=can_console_arm_motion_get_status(&motion);
    if(error!=ESP_OK)return error;
    if(motion.state==CAN_CONSOLE_ARM_MOTION_ERROR) {
        if(motion.error_source==CAN_CONSOLE_ARM_MOTION_ERROR_PLANNING && !p->acquired) {
            arm_playback_abort(p,ARM_PLAYBACK_PLANNING_ERROR,motion.last_error);return ESP_OK;
        }
        return motion.last_error!=ESP_OK?motion.last_error:ESP_FAIL;
    }
    if(p->phase==ARM_PLAYBACK_PREPARING) {
        if(p->stop_requested && !p->follow) {arm_playback_abort(p,ARM_PLAYBACK_USER_STOP,ESP_OK);return ESP_OK;}
        if(p->probe && now-p->begun_us>=PLANNING_TIMEOUT_US) {
            arm_playback_abort(p,ARM_PLAYBACK_PLANNING_ERROR,ESP_ERR_TIMEOUT);return ESP_OK;
        }
        if(motion.buffered_points==0 && !p->stop_requested)return ESP_OK;
        if(p->gripper_owned && p->seek && motion.duration_s>0)p->gripper_seek_duration=motion.duration_s;
        can_console_arm_motion_sample_t hold={0};
        memcpy(hold.position,feedback->position,sizeof(hold.position));
        /* Recheck the first point before ownership changes; never jump to a
         * recorded start that drifted away while preflight was running. */
        if(!p->follow && !p->stop_requested) {
            if(p->gripper_owned && fabsf(feedback->gripper_position-(p->seek?p->gripper_start:p->gripper_first))>
                p->profile->gripper.start_tolerance) {
                arm_playback_abort(p,ARM_PLAYBACK_PLANNING_ERROR,ESP_ERR_INVALID_STATE);return ESP_OK;
            }
            const arm_playback_calibration_t *c=p->profile->playback;
            for(unsigned j=0;j<6;++j) {
                if(fabsf(hold.position[j]-p->confirmed.position[j])>c->start_tolerance[j]) {
                    arm_playback_abort(p,ARM_PLAYBACK_PLANNING_ERROR,ESP_ERR_INVALID_STATE);return ESP_OK;
                }
            }
        }
        /* Seek already holds the recorded start reference. Keep that target
         * across play acquisition instead of replacing it with a lagging
         * encoder sample and then rate-limiting a return to the first point.
         * Standalone play still validates its measured start above. */
        if(!p->seek && !p->stop_requested)memcpy(hold.position,p->confirmed.position,sizeof(hold.position));
        error=submit(p,&hold,true,1);
        if(error==ESP_OK)p->phase=ARM_PLAYBACK_ACQUIRING;
        return error==ESP_ERR_TIMEOUT?ESP_OK:error;
    }
    measure_tracking(p,feedback);
    if(p->pending)return ESP_OK;
    const int64_t guard=(p->stop_planned?p->step_ms-20:20)*1000LL;
    if(now-p->submitted_us<PERIOD_US)return ESP_OK;
    if(!clocked(p) && now-p->confirmed_us<guard) {
        /* A long stop cadence still needs fresh telemetry while ownership
         * suppresses normal motor retransmission. */
        error=submit(p,&p->confirmed,false,3);
        return error==ESP_ERR_TIMEOUT?ESP_OK:error;
    }
    if(p->stop_requested && !p->stop_planned) {
        if(clocked(p)) {
            /* Original-time replay has no low-speed braking envelope. Cancel
             * the path and command a measured-position hold after draining
             * the outstanding receipt; no queued path point can follow it. */
            can_console_arm_motion_sample_t hold={0};
            memcpy(hold.position,feedback->position,sizeof(hold.position));
            if(p->gripper_owned){p->gripper_hold=true;p->gripper_stop_position=feedback->gripper_position;}
            error=submit(p,&hold,false,2);
            if(error==ESP_OK) {
                p->due_ready=false;p->stop_planned=true;p->phase=ARM_PLAYBACK_DRAINING;
            }
            return error==ESP_ERR_TIMEOUT?ESP_OK:error;
        }
        if(p->motion_accepted) {
            arm_joint_path_limits_t braking=p->profile->playback->limits;
            for(unsigned j=0;j<6;++j)braking.acceleration[j]*=.5f;
            error=can_console_arm_motion_plan_stop(p->confirmed.position,p->execution_velocity,&braking,p->step_ms);
            if(error!=ESP_OK)return error;
            p->stop_planned=true;p->phase=ARM_PLAYBACK_STOPPING;
        } else p->phase=ARM_PLAYBACK_DRAINING;
    }
    if(clocked(p) && p->phase==ARM_PLAYBACK_RUNNING) {
        /* Bounded by the queue snapshot: the producer may refill concurrently,
         * so never loop unboundedly under the application control mutex. */
        for(uint32_t i=0;i<motion.buffered_points;++i)
            if(!can_console_arm_motion_tick_reference(collect_due,p))break;
        error=can_console_arm_motion_get_status(&motion);if(error!=ESP_OK)return error;
        if(motion.state==CAN_CONSOLE_ARM_MOTION_ERROR)
            return motion.last_error!=ESP_OK?motion.last_error:ESP_FAIL;
        if(p->due_ready) {
            error=submit(p,&p->due,false,2);
            if(error==ESP_OK){p->due_ready=false;p->motion_accepted=true;p->empty_us=0;}
            return error==ESP_ERR_TIMEOUT?ESP_OK:error;
        }
        if(motion.state==CAN_CONSOLE_ARM_MOTION_COMPLETE) {
            p->phase=ARM_PLAYBACK_DRAINING;
        } else if(motion.buffered_points==0) {
            if(p->empty_us==0)p->empty_us=now;
            if(p->probe && now-p->empty_us>=DELIVERY_TIMEOUT_US)return ESP_ERR_TIMEOUT;
        } else p->empty_us=0; /* Waiting for a future timestamp is not starvation. */
    }
    if(!clocked(p) && (p->phase==ARM_PLAYBACK_RUNNING || p->phase==ARM_PLAYBACK_STOPPING)) {
        if(can_console_arm_motion_tick_reference(apply,p))return ESP_OK;
        error=can_console_arm_motion_get_status(&motion);if(error!=ESP_OK)return error;
        if(motion.state==CAN_CONSOLE_ARM_MOTION_ERROR)return motion.last_error!=ESP_OK?motion.last_error:ESP_FAIL;
        if(motion.state==CAN_CONSOLE_ARM_MOTION_COMPLETE) {
            p->phase=ARM_PLAYBACK_DRAINING;
        } else {
            if(p->empty_us==0)p->empty_us=now;
            if(p->probe && now-p->empty_us>=DELIVERY_TIMEOUT_US)return ESP_ERR_TIMEOUT;
        }
    }
    if(p->phase==ARM_PLAYBACK_DRAINING) {
        /* Completion means the last group is delivered and the producer is
         * retired. Do not wait on measured position or an arrival timeout. */
        if(!motion.planner_busy) {
            error=can_console_motor_release_reference(&p->sequence);
            if(error==ESP_OK) {p->phase=ARM_PLAYBACK_RELEASING;p->pending=true;p->pending_kind=4;p->submitted_us=now;}
            return error==ESP_ERR_TIMEOUT?ESP_OK:error;
        }
    }
    /* Heartbeats maintain feedback while awaiting producer retirement;
     * duplicate holds never fabricate a zero execution-velocity estimate. */
    error=submit(p,&p->confirmed,false,3);
    return error==ESP_ERR_TIMEOUT?ESP_OK:error;
}
