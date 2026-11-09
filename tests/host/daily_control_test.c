#include "rb_daily_control.h"
#include "rb_daily_screen.h"
#include "can_console_motor_runtime.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
static can_console_motor_state_t motors[7];
static uint64_t now;
static unsigned enables,disables,references,reports;
static bool stale,stall_tx,fail_tx,ignore_disable,fault_bus;
static float tracking_bias[7];
static int feedback_skew_us;
static can_console_motor_target_t queued[7];
static uint32_t serial;
int64_t esp_timer_get_time(void){return now*1000;}
esp_err_t can_console_motor_get_state(uint8_t id,can_console_motor_state_t *s)
{*s=motors[id-1];return ESP_OK;}
esp_err_t can_console_motor_get_bus_state(int ch,int32_t window,can_console_motor_bus_state_t *s)
{assert(ch==0 && window==100);*s=(can_console_motor_bus_state_t){.started=true,.status=fault_bus?CAN_CONSOLE_MOTOR_BUS_OFF:CAN_CONSOLE_MOTOR_BUS_ACTIVE};return ESP_OK;}
esp_err_t can_console_motor_request_rs_report(uint8_t id,uint8_t host)
{assert(id>=1 && id<=7 && host==253);++reports;return ESP_OK;}
esp_err_t can_console_motor_set_gains(uint8_t id,float kp,float kd)
{motors[id-1].kp=kp;motors[id-1].kd=kd;return ESP_OK;}
esp_err_t can_console_motor_set_targets(const can_console_motor_target_t *t,size_t n)
{assert(n==7);for(unsigned j=0;j<7;++j){assert(t[j].motor_id==j+1);motors[j].mode=t[j].mode;}return ESP_OK;}
esp_err_t can_console_motor_fence_begin(int ch,uint32_t *token)
{assert(ch==0);*token=1;return ESP_OK;}
esp_err_t can_console_motor_fence_poll(uint32_t token,can_console_motor_fence_status_t *s)
{assert(token==1);*s=(can_console_motor_fence_status_t){.complete=true};return ESP_OK;}
esp_err_t can_console_motor_submit_admin_range(const can_console_motor_admin_range_t *c)
{
    assert(c->first_motor_id==1 && c->last_motor_id==7 && c->action==CAN_CONSOLE_MOTOR_ADMIN_ENABLE);
    ++enables;for(unsigned j=0;j<7;++j){motors[j].enabled=true;motors[j].mode_ready=true;motors[j].status=2;}return ESP_OK;
}
esp_err_t can_console_motor_submit_admin(const can_console_motor_admin_command_t *c)
{
    assert(c->motor_id==0 && c->action==CAN_CONSOLE_MOTOR_ADMIN_ESTOP);++disables;
    if(!ignore_disable)for(unsigned j=0;j<7;++j){motors[j].enabled=false;motors[j].status=0;}
    return ESP_OK;
}
esp_err_t can_console_motor_submit_reference(const can_console_motor_target_t *t,size_t n,bool acquire,uint32_t *seq)
{
    assert(n==7);(void)acquire;memcpy(queued,t,sizeof(queued));*seq=++serial;++references;return ESP_OK;
}
esp_err_t can_console_motor_get_reference_status(uint32_t seq,can_console_motor_reference_status_t *s)
{
    assert(seq==serial);*s=(can_console_motor_reference_status_t){.owned=true,.sequence=seq,
        .state=fail_tx?CAN_CONSOLE_MOTOR_REFERENCE_FAILED:stall_tx?CAN_CONSOLE_MOTOR_REFERENCE_PENDING:CAN_CONSOLE_MOTOR_REFERENCE_DELIVERED};
    if(!stall_tx && !fail_tx)for(unsigned j=0;j<7;++j){motors[j].position=queued[j].value.position+tracking_bias[j];motors[j].velocity=queued[j].value.velocity;}
    return ESP_OK;
}
static rb_daily_control_t c;
static void tick(bool ui)
{
    now+=20;
    if(!stale)for(unsigned j=0;j<7;++j)motors[j].last_feedback_us=now*1000-feedback_skew_us;
    if(ui)rb_daily_control_heartbeat(&c,now);
    rb_daily_control_tick(&c,now);
}
static void until(rb_dc_state_t target,unsigned limit)
{
    for(unsigned i=0;i<limit && c.status.state!=target;++i)tick(true);
    if(c.status.state!=target)fprintf(stderr,"state=%s expected=%s err=%s motor=%u time=%llu\n",
        rb_daily_control_state_name(c.status.state),rb_daily_control_state_name(target),c.status.error?c.status.error:"ok",c.status.bad_motor,(unsigned long long)now);
    assert(c.status.state==target);
}
static void init(void)
{
    memset(motors,0,sizeof(motors));now=1000;
    for(unsigned j=0;j<7;++j){motors[j]=(can_console_motor_state_t){.motor_id=j+1,.online=true,.channel=0,.mos_temperature=30,.last_feedback_us=now*1000};strcpy(motors[j].vendor_name,"robstride");}
    assert(rb_daily_control_init(&c,0)==ESP_OK);
    for(unsigned i=0;i<20;++i)tick(true);
    assert(!enables && !references && c.status.state==RB_DC_OFF);
}
static void idle(void)
{
    assert(rb_daily_control_request(&c,RB_DAILY_IDLE,1)==ESP_OK);until(RB_DC_HOLD,2000);
    assert(c.status.completed_id==1 && enables==1 && !disables);
}
static void screen_request(void *context,const rb_pet_event_t *event)
{
    (void)context;
    assert(rb_daily_control_request(&c,event->action_id,event->request_id)==ESP_OK);
}
static void screen_tick(rb_pet_t *p)
{
    tick(true);rb_pet_tick(p,now);
    if(rb_daily_sync_screen(p,&c.status))
        assert(rb_daily_control_request(&c,RB_DAILY_SLEEP,c.status.request_id|0x40000000u)==ESP_OK);
}
static void screen_lifecycle(void)
{
    rb_pet_t p;rb_pet_init_sleep(&p,now);rb_pet_use_daily_timing(&p,true);p.motion_clock=true;
    rb_pet_set_sink(&p,screen_request,NULL);
    for(unsigned i=0;i<100;++i)screen_tick(&p);
    assert(p.state==RB_PET_RESTING && !enables && !references);
    /* A displaced sleeping arm must move directly toward idle, not zero first. */
    motors[1].position=.2f;motors[2].position=.3f;
    rb_pet_wake(&p);assert(p.action==RB_ANIM_WAKE);
    for(unsigned i=0;i<1600 && p.action==RB_ANIM_WAKE;++i) {
        screen_tick(&p);
        assert(motors[1].position>=.2f-1e-5f && motors[2].position>=.3f-1e-5f);
        if(c.status.state!=RB_DC_HOLD)assert(p.state==RB_PET_RESTING && p.action==RB_ANIM_WAKE);
    }
    assert(c.status.state==RB_DC_HOLD && p.state==RB_PET_NORMAL && p.action==RB_ANIM_IDLE && enables==1);
    for(unsigned i=0;i<2999;++i)screen_tick(&p);
    assert(p.action==RB_ANIM_IDLE);
    screen_tick(&p);assert(p.action==RB_ANIM_REST && c.status.action==RB_DAILY_REST);
    ignore_disable=true;
    for(unsigned i=0;i<1600 && c.status.state!=RB_DC_DISABLING;++i)screen_tick(&p);
    assert(c.status.state==RB_DC_DISABLING && p.action==RB_ANIM_REST && p.state!=RB_PET_RESTING);
    for(unsigned i=0;i<40;++i)screen_tick(&p);
    assert(!c.status.disable_confirmed && p.action==RB_ANIM_REST);
    ignore_disable=false;
    for(unsigned j=0;j<7;++j){motors[j].enabled=false;motors[j].status=0;}
    screen_tick(&p);
    assert(c.status.state==RB_DC_OFF && c.status.disable_confirmed && p.state==RB_PET_RESTING && p.action==RB_ANIM_IDLE);
    rb_pet_wake(&p);
    for(unsigned i=0;i<1600 && p.action==RB_ANIM_WAKE;++i)screen_tick(&p);
    assert(p.state==RB_PET_NORMAL && c.status.state==RB_DC_HOLD && enables==2);
}
int main(int argc,char **argv)
{
    assert(argc==2);init();
    if(!strcmp(argv[1],"screen_lifecycle")) {screen_lifecycle();puts("screen_lifecycle");return 0;}
    if(!strcmp(argv[1],"boot_wake")) {
        assert(!enables && !references);
        assert(rb_daily_control_request(&c,RB_DAILY_WAKE,1)==ESP_OK);
        until(RB_DC_ENTRY,1200);assert(enables==1 && c.status.action==RB_DAILY_WAKE);
        until(RB_DC_HOLD,1000);assert(c.status.completed_id==1 && !disables);return 0;
    }
    if(!strcmp(argv[1],"stale_start")) {
        stale=true;assert(rb_daily_control_request(&c,RB_DAILY_IDLE,1)==ESP_OK);until(RB_DC_FAULT,300);
        assert(!enables && !disables && !references && reports);return 0;
    }
    if(!strcmp(argv[1],"already_enabled")) {
        motors[2].enabled=true;motors[2].status=2;
        assert(rb_daily_control_request(&c,RB_DAILY_IDLE,1)==ESP_OK);until(RB_DC_FAULT,20);assert(!enables && !disables);return 0;
    }
    idle();
    if(!strcmp(argv[1],"home_request_idempotent")) {
        rb_daily_control_home(&c);until(RB_DC_HOME,10);
        uint64_t began=c.changed;
        for(unsigned i=0;i<20;++i) {
            rb_daily_control_home(&c);tick(true);
            assert(c.status.state==RB_DC_HOME && c.changed==began);
        }
        until(RB_DC_OFF,1500);assert(c.status.disable_confirmed && enables==1);
        rb_daily_control_home(&c);
        for(unsigned i=0;i<20;++i)tick(true);
        assert(c.status.state==RB_DC_OFF && !c.home_requested && enables==1);
    } else if(!strcmp(argv[1],"feedback_frame_spacing")) {
        assert(rb_daily_control_request(&c,RB_DAILY_PROBE,2)==ESP_OK);until(RB_DC_PLAY,1000);
        for(unsigned i=0;i<80;++i) {
            float before=c.estimated_v[1],q=c.previous_q[1];
            feedback_skew_us=i%2?19000:0;tick(true);
            float expected=before+.02f/.06f*((c.previous_q[1]-q)/.02f-before);
            assert(fabsf(c.estimated_v[1]-expected)<1e-6f && !disables);
        }
    } else if(!strcmp(argv[1],"hold_ramp_delivery")) {
        rb_daily_control_pause(&c);until(RB_DC_HOLD_RAMP,100);
        for(unsigned i=0;i<50 && motors[0].kp<rb_daily_policy.kp[0];++i)tick(true);
        assert(c.status.state==RB_DC_HOLD_RAMP && c.pending);
        stall_tx=true;
        for(unsigned i=0;i<3;++i){tick(true);assert(c.status.state==RB_DC_HOLD_RAMP);}
        stall_tx=false;until(RB_DC_PROTECTIVE_HOLD,10);assert(!disables);
    } else if(!strcmp(argv[1],"home_budget")) {
        rb_daily_control_home(&c);until(RB_DC_HOME,10);
        c.estimated_v[6]=-10.f;until(RB_DC_BRAKING,10);
        assert(c.status.hold.source_state==RB_DC_HOME && !disables);
        for(unsigned i=0;i<15;++i) {
            tick(true);
            assert(c.status.state==RB_DC_BRAKING && !disables);
            float q[6],g[6];for(unsigned j=0;j<6;++j)q[j]=c.motors[j].position;
            assert(can_console_arm_gravity_compute(&c.gravity,q,g)==ESP_OK);
            for(unsigned j=0;j<7;++j) {
                assert(motors[j].kp==0 && motors[j].kd>=0 && motors[j].kd<=rb_daily_policy.kd[j]);
                float ff=j<6?g[j]:0;
                assert(fabsf(queued[j].value.torque-ff)<1e-5f);
                assert(fabsf(ff-motors[j].kd*c.estimated_v[j])<=rb_daily_policy.torque_budget_nm[j]+1e-5f);
            }
        }
        until(RB_DC_PROTECTIVE_HOLD,150);assert(!disables && c.status.enabled);
        for(unsigned i=0;i<20;++i)tick(true);
        assert(c.status.state==RB_DC_PROTECTIVE_HOLD);
        rb_daily_control_home(&c);until(RB_DC_OFF,1500);
    } else if(!strcmp(argv[1],"following_recovery")) {
        c.target[0]+=.11f;until(RB_DC_BRAKING,10);
        assert(strstr(c.status.error,"following") && c.status.bad_motor==1 && !disables);
        until(RB_DC_PROTECTIVE_HOLD,150);assert(!disables);
    } else if(!strcmp(argv[1],"pause_home")) {
        rb_daily_control_home(&c);until(RB_DC_HOME,10);
        rb_daily_control_pause(&c);until(RB_DC_PROTECTIVE_HOLD,150);
        assert(!disables && c.status.hold.source_state==RB_DC_HOME);
        assert(rb_daily_control_request(&c,RB_DAILY_PROBE,88)==ESP_ERR_INVALID_STATE);
        float held[7];memcpy(held,c.target,sizeof(held));
        for(unsigned i=0;i<100;++i)tick(true);
        assert(!disables && !memcmp(held,c.target,sizeof(held)));
    } else if(!strcmp(argv[1],"invalid_target_hold")) {
        c.target[0]=NAN;until(RB_DC_PROTECTIVE_HOLD,150);
        assert(!disables && strstr(c.status.error,"invalid target"));
    } else if(!strcmp(argv[1],"recovery_stale")) {
        rb_daily_control_pause(&c);until(RB_DC_BRAKING,10);
        stale=true;until(RB_DC_DISABLING,20);
        assert(strstr(c.status.error,"pause requested") && strstr(c.status.stop_error,"stale"));
    } else if(!strcmp(argv[1],"recovery_timeout")) {
        rb_daily_control_pause(&c);until(RB_DC_BRAKING,10);
        for(unsigned i=0;i<270 && c.status.state!=RB_DC_DISABLING;++i) {
            c.estimated_v[6]=.1f;tick(true);
        }
        assert(c.status.state==RB_DC_DISABLING && strstr(c.status.stop_error,"within 5s"));
    } else if(!strcmp(argv[1],"home_arrival_timeout")) {
        rb_daily_control_home(&c);until(RB_DC_HOME_SETTLE,1500);
        tracking_bias[1]=3.5f*.01745329252f;until(RB_DC_BRAKING,450);
        assert(c.status.hold.source_state==RB_DC_HOME_SETTLE && !disables);
        tracking_bias[1]=0;until(RB_DC_PROTECTIVE_HOLD,150);
        assert(!disables && !c.status.disable_confirmed);
    } else if(!strcmp(argv[1],"all_actions")) {
        unsigned ids[]={RB_DAILY_PROBE,RB_DAILY_NUZZLE,RB_DAILY_TILT,RB_DAILY_HAPPY};
        for(unsigned i=0;i<4;++i) {
            assert(rb_daily_control_request(&c,ids[i],i+2)==ESP_OK);
            assert(rb_daily_control_request(&c,RB_DAILY_HAPPY,99)==ESP_ERR_INVALID_STATE);
            until(RB_DC_PLAY,1000);until(RB_DC_HOLD,2000);
            assert(c.status.completed_id==i+2 && c.status.phase==1);
        }
        assert(rb_daily_control_request(&c,RB_DAILY_REST,7)==ESP_OK);until(RB_DC_ZERO_HOLD,1500);assert(!disables);
        assert(rb_daily_control_request(&c,RB_DAILY_SLEEP,8)==ESP_OK);until(RB_DC_OFF,50);assert(c.status.disable_confirmed);
        assert(rb_daily_control_request(&c,RB_DAILY_WAKE,9)==ESP_OK);until(RB_DC_HOLD,1500);assert(enables==2);
        rb_daily_control_home(&c);until(RB_DC_OFF,1500);assert(c.status.disable_confirmed && !c.status.enabled);
        for(unsigned j=0;j<6;++j)assert(fabsf(motors[j].position)<.001f);
    } else if(!strcmp(argv[1],"tx_stall")) {
        stall_tx=true;until(RB_DC_DISABLING,30);assert(strstr(c.status.error,"delivery timeout"));
    } else if(!strcmp(argv[1],"tx_failed")) {
        fail_tx=true;until(RB_DC_DISABLING,30);
    } else if(!strcmp(argv[1],"stale_running")) {
        stale=true;until(RB_DC_DISABLING,30);assert(strstr(c.status.error,"stale"));
    } else if(!strcmp(argv[1],"bus_off")) {
        fault_bus=true;until(RB_DC_DISABLING,5);
    } else if(!strcmp(argv[1],"fault")) {
        motors[3].fault_raw=8;until(RB_DC_DISABLING,5);assert(c.status.bad_motor==4);
    } else if(!strcmp(argv[1],"nan")) {
        motors[3].position=NAN;until(RB_DC_DISABLING,5);
    } else if(!strcmp(argv[1],"ui_loss")) {
        for(unsigned i=0;i<30;++i)tick(false);
        assert(c.status.state==RB_DC_BRAKING && !disables);
        until(RB_DC_PROTECTIVE_HOLD,150);
        assert(c.status.state==RB_DC_PROTECTIVE_HOLD && c.status.enabled && !disables);
        assert(strstr(c.status.error,"UI heartbeat timeout") && c.status.hold.valid);
        assert(rb_daily_control_request(&c,RB_DAILY_HAPPY,3)==ESP_ERR_INVALID_STATE);
        rb_daily_control_home(&c);until(RB_DC_OFF,1500);
    } else if(!strcmp(argv[1],"budget")) {
        c.estimated_v[6]=-10.f;
        until(RB_DC_BRAKING,10);assert(!disables);
        rb_dc_hold_snapshot_t trigger=c.status.hold;
        assert(trigger.valid && trigger.torque_checked && c.status.bad_motor==7);
        assert(fabsf(trigger.estimated_torque[6])>rb_daily_policy.torque_budget_nm[6]);
        until(RB_DC_PROTECTIVE_HOLD,150);
        assert(c.status.enabled);
        assert(memcmp(&trigger,&c.status.hold,sizeof(trigger))==0);
        rb_daily_control_home(&c);until(RB_DC_OFF,1500);
    } else if(!strcmp(argv[1],"rest_arrival_tolerance")) {
        assert(rb_daily_policy.zero_tolerance_deg==3.f);
        assert(rb_daily_control_request(&c,RB_DAILY_REST,2)==ESP_OK);
        until(RB_DC_SETTLE,1500);
        tracking_bias[1]=2.5f*.01745329252f;
        until(RB_DC_ZERO_HOLD,200);
        assert(c.status.completed_id==2 && !c.status.error && !disables);
        assert(fabsf(motors[1].position-tracking_bias[1])<1e-5f);
        assert(rb_daily_control_request(&c,RB_DAILY_SLEEP,3)==ESP_OK);
        until(RB_DC_OFF,50);assert(c.status.disable_confirmed);
    } else if(!strcmp(argv[1],"rest_arrival_timeout")) {
        assert(rb_daily_control_request(&c,RB_DAILY_REST,2)==ESP_OK);
        until(RB_DC_SETTLE,1500);
        tracking_bias[1]=3.5f*.01745329252f;
        until(RB_DC_BRAKING,450);
        rb_dc_hold_snapshot_t trigger=c.status.hold;
        assert(strstr(c.status.error,"arrival timeout"));
        assert(trigger.valid && !trigger.torque_checked && trigger.source_state==RB_DC_SETTLE);
        assert(fabsf(trigger.target[1])<1e-5f);
        assert(fabsf(trigger.measured[1]-trigger.target[1]-tracking_bias[1])<1e-5f);
        assert(c.status.completed_id!=2 && !disables && c.status.enabled);
        assert(rb_daily_control_request(&c,RB_DAILY_SLEEP,3)==ESP_ERR_INVALID_STATE);
        tracking_bias[1]=0;
        until(RB_DC_PROTECTIVE_HOLD,150);
        for(unsigned i=0;i<30;++i)tick(true);
        assert(memcmp(&trigger,&c.status.hold,sizeof(trigger))==0);
        assert(c.status.state==RB_DC_PROTECTIVE_HOLD && !disables);
        rb_daily_control_home(&c);until(RB_DC_OFF,1500);
    } else if(!strcmp(argv[1],"temperature")) {
        motors[0].mos_temperature=100;until(RB_DC_HOME,10);until(RB_DC_OFF,1500);
    } else if(!strcmp(argv[1],"disable_unconfirmed")) {
        ignore_disable=true;rb_daily_control_home(&c);until(RB_DC_DISABLING,1500);
        for(unsigned i=0;i<250;++i)tick(true);
        assert(c.status.state==RB_DC_DISABLING && !c.status.disable_confirmed && c.status.error);
    } else assert(0);
    puts(argv[1]);return 0;
}
