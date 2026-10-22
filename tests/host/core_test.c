#include "rb_core.h"
#include "rb_arm.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static rb_engine_t pet;
static rb_gesture_t g;
static uint64_t now;
static void init(void)
{ rb_config_t c=rb_config_default(); now=1000; assert(rb_engine_init(&pet,&c,now)); rb_gesture_init(&g); }
static void tick(unsigned duration)
{
    for (unsigned ms=0;ms<duration;ms+=20) {
        float previous[RB_JOINTS]; memcpy(previous,pet.pose,sizeof(previous));
        now+=20; rb_gesture_tick(&g,&pet,now); rb_engine_tick(&pet,now);
        for (unsigned j=0;j<RB_JOINTS;++j) {
            assert(isfinite(pet.pose[j])); assert(fabsf(pet.pose[j])<=pet.config.max_offset_deg+.001f);
            assert(fabsf(pet.pose[j]-previous[j])<=pet.config.max_speed_deg_s*.02f+.001f);
        }
    }
}
static void tap(void)
{ rb_gesture_touch(&g,&pet,RB_TOUCH_DOWN,160,100,now); tick(60); rb_gesture_touch(&g,&pet,RB_TOUCH_UP,160,100,now); }
static void config_test(void)
{
    rb_config_t c=rb_config_default(); c.sleep_ms=c.drowsy_ms; assert(!rb_engine_init(&pet,&c,0));
    c=rb_config_default(); c.max_speed_deg_s=NAN; assert(!rb_engine_init(&pet,&c,0));
    c=rb_config_default(); c.max_offset_deg=11; assert(!rb_engine_init(&pet,&c,0));
    init(); rb_engine_tick(&pet,now-1); assert(pet.now_ms==now);
    assert(!rb_engine_action(&pet,RB_NONE,now));
}
static void actions_test(void)
{
    for (unsigned style=0;style<2;++style) for (unsigned a=0;a<RB_ACTION_COUNT;++a) {
        init(); rb_engine_style(&pet,(rb_style_t)style);
        assert(rb_engine_action(&pet,(rb_action_id_t)a,now)); tick(20000);
        assert(pet.action==RB_NONE); assert(pet.actions_completed==1);
        assert(pet.state==(a==RB_DOZE?RB_SLEEPING:RB_IDLE));
        for (unsigned j=0;j<RB_JOINTS;++j) assert(fabsf(pet.pose[j])<.001f);
    }
}
static void styles_test(void)
{
    init(); assert(rb_engine_action(&pet,RB_LOOK,now)); tick(1600);
    float lively=fabsf(pet.pose[1]); assert(lively>2.9f);
    rb_engine_style(&pet,RB_LAZY); assert(pet.active_style==RB_LIVELY); tick(3400);
    assert(rb_engine_action(&pet,RB_LOOK,now)); tick(2640); assert(fabsf(pet.pose[1])<lively*.6f);
    assert(pet.active_style==RB_LAZY);
}
static void interrupt_test(void)
{
    init(); assert(rb_engine_action(&pet,RB_DANCE,now)); tick(1800);
    float before[7]; memcpy(before,pet.pose,sizeof(before));
    assert(rb_engine_event(&pet,RB_MULTI_TAP,now));
    assert(memcmp(before,pet.pose,sizeof(before))==0); tick(5800);
    assert(pet.state==RB_IDLE);
    for (unsigned n=0;n<300;++n) { assert(rb_engine_event(&pet,n%2?RB_STROKE:RB_HAPPY,now)); tick(40); }
    assert(pet.state!=RB_STOPPED);
}
static void sleep_test(void)
{
    init(); tick(30000); assert(pet.state==RB_DROWSY);
    tick(30000); assert(pet.action==RB_DOZE); tick(10000); assert(pet.state==RB_SLEEPING);
    tap(); assert(pet.state==RB_WAKING); tick(5000); assert(pet.state==RB_IDLE);
    assert(rb_engine_event(&pet,RB_REST,now)); tick(10000); assert(pet.state==RB_SLEEPING);
}
static void stop_test(void)
{
    init(); tap(); rb_engine_stop(&pet,"TEST STOP"); tick(600);
    assert(pet.state==RB_STOPPED); assert(pet.actions_started==0);
    assert(!rb_engine_event(&pet,RB_HAPPY,now));
    assert(rb_engine_reset(&pet,now)); assert(pet.state==RB_IDLE); tick(600);
    assert(pet.action==RB_NONE); assert(pet.actions_started==0);
}
static void contact(rb_touch_phase_t phase,int x,int y,rb_region_t region)
{ rb_gesture_touch_region(&g,&pet,phase,x,y,region,now); }
static void hold_head(unsigned duration,bool move)
{
    for (unsigned t=0;t<duration;t+=100) {
        contact(RB_TOUCH_MOVE,195+(move?(int)((t/100)%8)*3:0),80,RB_REGION_HEAD);
        tick(100);
    }
}
static void gestures_test(void)
{
    init(); contact(RB_TOUCH_DOWN,200,80,RB_REGION_HEAD); tick(40);
    assert(pet.life.contact && pet.life.face==RB_FACE_CURIOUS);
    assert(pet.action==RB_NONE); /* Immediate expression, no queued action. */
    hold_head(4000,true);
    assert(g.down && pet.life.comfort>.6f && pet.life.face==RB_FACE_RELAXED);
    contact(RB_TOUCH_UP,200,80,RB_REGION_HEAD); tick(100);
    assert(!pet.life.contact && pet.life.comfort>.5f);
    assert(pet.life.face==RB_FACE_HAPPY); /* Warm interaction earns a short celebration. */
    tick(16000); assert(pet.life.comfort==0 && pet.life.face==RB_FACE_IDLE);
    init(); contact(RB_TOUCH_DOWN,20,120,RB_REGION_NEAR);
    for (unsigned n=0;n<40;++n) { contact(RB_TOUCH_MOVE,20,120,RB_REGION_NEAR); tick(100); }
    assert(pet.life.comfort==0 && pet.life.gaze_x<-.9f);
    init(); contact(RB_TOUCH_DOWN,200,80,RB_REGION_HEAD); hold_head(2000,false);
    assert(pet.life.comfort>.15f && pet.life.comfort<.3f);
    rb_gesture_cancel(&g,&pet); assert(!g.down && !pet.life.contact);
    init(); contact(RB_TOUCH_DOWN,160,80,RB_REGION_HEAD); tick(100);
    contact(RB_TOUCH_MOVE,240,80,RB_REGION_HEAD); tick(100);
    contact(RB_TOUCH_MOVE,160,80,RB_REGION_HEAD); tick(20);
    assert(g.down && pet.life.face==RB_FACE_PUZZLED);
    assert(pet.life.comfort==0); /* Fast scratching is not a gentle stroke. */
    init(); for (unsigned n=0;n<5;++n) { tap(); tick(100); }
    assert(pet.life.disturbance>.55f && pet.life.face==RB_FACE_SHY);
    tick(10000); assert(pet.life.disturbance==0);
    init(); contact(RB_TOUCH_DOWN,200,80,RB_REGION_HEAD);
    contact(RB_TOUCH_CANCEL,200,80,RB_REGION_HEAD); tick(600);
    assert(!pet.life.contact && pet.action==RB_NONE);
    contact(RB_TOUCH_DOWN,200,80,RB_REGION_HEAD); tick(1600);
    assert(!g.down && !pet.life.contact); /* Missing input samples cancel contact. */
    contact(RB_TOUCH_DOWN,200,80,RB_REGION_HEAD);
    contact(RB_TOUCH_MOVE,-1,80,RB_REGION_HEAD); assert(!pet.life.contact);
}
static void life_test(void)
{
    init(); float start=pet.life.gaze_x; tick(10000);
    assert(fabsf(pet.life.gaze_x-start)>.01f);
    assert(pet.actions_started==0 && pet.last_input_ms==1000);
    tick(20000); assert(pet.state==RB_DROWSY);
    tick(40000); assert(pet.state==RB_SLEEPING && pet.life.face==RB_FACE_SLEEP);
    contact(RB_TOUCH_DOWN,20,120,RB_REGION_NEAR); tick(100);
    assert(pet.state==RB_SLEEPING);
    contact(RB_TOUCH_MOVE,200,80,RB_REGION_HEAD); tick(40);
    assert(pet.state==RB_WAKING && pet.life.openness<.3f);
    rb_gesture_cancel(&g,&pet); tick(5200); assert(pet.state==RB_IDLE);
    init(); contact(RB_TOUCH_DOWN,200,80,RB_REGION_HEAD); hold_head(65000,false);
    assert(pet.state==RB_IDLE && pet.life.comfort<=1 && pet.life.comfort>.9f);
    rb_engine_stop(&pet,"test"); tick(20);
    assert(!pet.life.contact && !g.down && pet.life.face==RB_FACE_STOP);
    assert(rb_engine_reset(&pet,now)); tick(20);
    assert(pet.life.comfort==0 && !pet.life.contact && pet.action==RB_NONE);
    /* Equal elapsed time, independent of render/tick frequency. */
    rb_engine_t a,b; rb_config_t c=rb_config_default();
    assert(rb_engine_init(&a,&c,0) && rb_engine_init(&b,&c,0));
    a.life.comfort=b.life.comfort=.8f;
    for (unsigned t=10;t<=2000;t+=10) rb_engine_tick(&a,t);
    for (unsigned t=40;t<=2000;t+=40) rb_engine_tick(&b,t);
    assert(fabsf(a.life.comfort-b.life.comfort)<.001f);
}
typedef struct { float actual[7]; uint64_t stamp; bool send_ok; unsigned sends,stops; } fake_arm_t;
static bool feedback(void *ctx,float p[7],uint64_t *stamp)
{ fake_arm_t *f=ctx; memcpy(p,f->actual,sizeof(f->actual)); *stamp=f->stamp; return true; }
static bool send_target(void *ctx,const float p[7])
{ fake_arm_t *f=ctx; ++f->sends; memcpy(f->actual,p,sizeof(f->actual)); return f->send_ok; }
static void stop_arm(void *ctx) { ++((fake_arm_t *)ctx)->stops; }
static void arm_test(const char *scenario)
{
    init(); fake_arm_t f={.stamp=now,.send_ok=true};
    rb_arm_driver_t d={.feedback=feedback,.send=send_target,.stop=stop_arm,.context=&f};
    rb_arm_calibration_t c={0}; rb_arm_t a={0};
    for (unsigned j=0;j<7;++j) { c.lower[j]=-10; c.upper[j]=10; }
    assert(!rb_arm_enable(&a,&d,&c,now)); c.calibrated=true;
    f.actual[0]=2; assert(!rb_arm_enable(&a,&d,&c,now)); f.actual[0]=0;
    assert(rb_arm_enable(&a,&d,&c,now)); assert(f.sends==0);
    assert(rb_engine_action(&pet,RB_DANCE,now));
    for (unsigned n=0;n<100;++n) { tick(20); f.stamp=now; assert(rb_arm_update(&a,&pet,now,true)); }
    if (strcmp(scenario,"arm_timeout")==0) {
        f.stamp=now-101; assert(!rb_arm_update(&a,&pet,now,true));
    } else if (strcmp(scenario,"arm_limit")==0) {
        pet.pose[0]=NAN; assert(!rb_arm_update(&a,&pet,now,true));
    } else {
        f.send_ok=false; assert(!rb_arm_update(&a,&pet,now,true));
    }
    assert(pet.state==RB_STOPPED); assert(!a.enabled); assert(f.stops==1);
    rb_arm_stop(&a); assert(f.stops==1); /* Stop is idempotent. */
    assert(rb_engine_reset(&pet,now)); assert(!a.enabled);
    assert(!rb_arm_update(&a,&pet,now,false)); assert(pet.state==RB_STOPPED);
}
static void control_timeout_test(void)
{ init(); assert(rb_engine_action(&pet,RB_LOOK,now)); now+=251; rb_engine_tick(&pet,now); assert(pet.state==RB_STOPPED); }
static void soak_test(void)
{
    init(); uint32_t random=42;
    for (unsigned t=0;t<1800000;t+=20) {
        if (t%1380==0) {
            random=random*1664525u+1013904223u;
            rb_engine_style(&pet,(random&1)?RB_LIVELY:RB_LAZY);
            assert(rb_engine_event(&pet,(rb_event_t)((random>>8)%6),now));
        }
        if (t && t%97000==0) { rb_engine_stop(&pet,"SOAK STOP"); assert(rb_engine_reset(&pet,now)); }
        tick(20); assert(pet.state!=RB_STOPPED);
    }
    printf("30 simulated minutes / 90000 ticks / %u actions; no heap allocations in core\n",pet.actions_started);
}
int main(int argc,char **argv)
{
    assert(argc==2);
    const char *s=argv[1];
    if (!strcmp(s,"config")) config_test();
    else if (!strcmp(s,"actions")) actions_test();
    else if (!strcmp(s,"styles")) styles_test();
    else if (!strcmp(s,"interrupt")) interrupt_test();
    else if (!strcmp(s,"sleep")) sleep_test();
    else if (!strcmp(s,"stop")) stop_test();
    else if (!strcmp(s,"gestures")) gestures_test();
    else if (!strcmp(s,"life")) life_test();
    else if (!strncmp(s,"arm_",4)) arm_test(s);
    else if (!strcmp(s,"control_timeout")) control_timeout_test();
    else if (!strcmp(s,"soak")) soak_test();
    else assert(!"unknown scenario");
    printf("PASS %s\n",s); return 0;
}
