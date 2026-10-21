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
static void gestures_test(void)
{
    init(); tap(); tick(360); assert(pet.action==RB_LOOK);
    init(); tap(); tick(100); tap(); tick(360); assert(pet.action==RB_SHY);
    init(); tap(); tick(80); tap(); tick(80); tap(); assert(pet.action==RB_SHY);
    init(); rb_gesture_touch(&g,&pet,RB_TOUCH_DOWN,100,100,now);
    for (int n=1;n<=8;++n) { tick(100); rb_gesture_touch(&g,&pet,RB_TOUCH_MOVE,100+n*8,100,now); }
    rb_gesture_touch(&g,&pet,RB_TOUCH_UP,164,100,now); assert(pet.action==RB_NUZZLE);
    init(); rb_gesture_touch(&g,&pet,RB_TOUCH_DOWN,100,100,now); tick(100);
    rb_gesture_touch(&g,&pet,RB_TOUCH_MOVE,170,100,now); tick(100);
    rb_gesture_touch(&g,&pet,RB_TOUCH_UP,100,100,now); assert(pet.action==RB_PUZZLED);
    init(); rb_gesture_touch(&g,&pet,RB_TOUCH_DOWN,100,100,now); tick(100);
    rb_gesture_touch(&g,&pet,RB_TOUCH_UP,200,100,now); assert(pet.action==RB_NONE); /* Single swipe is not zigzag. */
    init(); rb_gesture_touch(&g,&pet,RB_TOUCH_DOWN,100,100,now);
    rb_gesture_touch(&g,&pet,RB_TOUCH_CANCEL,100,100,now); tick(600); assert(pet.action==RB_NONE);
    tap(); rb_gesture_touch(&g,&pet,RB_TOUCH_CANCEL,-1,0,now); tick(600); assert(pet.action==RB_NONE);
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
    else if (!strncmp(s,"arm_",4)) arm_test(s);
    else if (!strcmp(s,"control_timeout")) control_timeout_test();
    else if (!strcmp(s,"soak")) soak_test();
    else assert(!"unknown scenario");
    printf("PASS %s\n",s); return 0;
}
