#include "rb_pet.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static rb_pet_t p;
static uint64_t now;
static unsigned events[RB_ANIM_COUNT];
static uint32_t last_request;
static void sink(void *ctx,const rb_pet_event_t *e)
{
    (void)ctx; assert(e->request_id>last_request); last_request=e->request_id;
    assert(e->timestamp_ms==now); ++events[e->action_id];
}
static void init(void)
{ now=1000; memset(events,0,sizeof(events)); last_request=0; rb_pet_init(&p,now);
  const int16_t claw[4][2]={{170,112},{215,112},{215,148},{170,148}};
  rb_pet_set_claw_region(&p,claw); rb_pet_set_sink(&p,sink,NULL); }
static void tick(unsigned ms)
{ for (unsigned i=0;i<ms;i+=20) { now+=20; rb_pet_tick(&p,now); } }
static void input(bool down,int x,int y)
{ rb_pet_input(&p,down,x,y,now); }
static void tap(void)
{ input(true,130,110); tick(100); input(false,130,110); tick(60); }
static void release(void) { input(false,p.raw_x,p.raw_y); tick(60); }
static void stroke(unsigned ms)
{
    for (unsigned i=0;i<ms;i+=20) {
        unsigned n=(i/20)%60; int x=130+(int)(n<=30?n:60-n)*2;
        input(true,x,110); tick(20);
    }
}
static void claw_stroke(unsigned ms)
{
    for (unsigned i=0;i<ms;i+=20) {
        unsigned n=(i/20)%30;
        input(true,180+(int)(n<=15?n:30-n),130); tick(20);
    }
}
int main(void)
{
    init(); input(true,130,110); tick(20); assert(!p.contact); tick(20); assert(p.contact);
    input(false,130,110); tick(20); assert(p.contact); tick(20);
    assert(events[RB_ANIM_PROBE]==1); tick(p.duration); assert(p.action==RB_ANIM_IDLE);
    init(); tap(); assert(p.duration==(unsigned)(RB_MS_PROBE*RB_STYLE_LIVELY*RB_PLAYBACK_DURATION_SCALE));
    tick(p.duration/2); assert(p.action==RB_ANIM_PROBE && p.phase>0.45f && p.phase<0.6f);
    tick(p.duration); assert(p.action==RB_ANIM_IDLE);
    init(); input(true,130,110); tick(40); tick(65000);
    assert(p.state==RB_PET_NORMAL && p.action==RB_ANIM_IDLE && p.happy_ms==0);
    release(); tick(60000); assert(p.action==RB_ANIM_REST); tick((unsigned)(RB_MS_REST*RB_PLAYBACK_DURATION_SCALE)); assert(p.state==RB_PET_RESTING);
    input(true,130,110); tick(40); stroke(4000); release(); tap(); tap(); tap();
    rb_pet_rest(&p); tick(1800000);
    assert(p.state==RB_PET_RESTING && p.action==RB_ANIM_IDLE && events[RB_ANIM_WAKE]==0);
    rb_pet_wake(&p); assert(p.action==RB_ANIM_WAKE); stroke(4000); release();
    assert(p.state==RB_PET_NORMAL && events[RB_ANIM_PROBE]==0 && events[RB_ANIM_HAPPY]==0);
    init(); tap(); tap(); tap(); tick((unsigned)(RB_MS_WITHDRAW*RB_PLAYBACK_DURATION_SCALE)+200);
    assert(p.state==RB_PET_FOLDED && p.pose==RB_POSE_FOLDED && events[RB_ANIM_WITHDRAW]==1);
    tap(); tap(); tick(1000); assert(events[RB_ANIM_WITHDRAW]==1);
    input(true,130,110); tick(40); stroke(1600); tick((unsigned)(RB_MS_RECOVER*RB_PLAYBACK_DURATION_SCALE));
    assert(events[RB_ANIM_RECOVER]==1 && p.state==RB_PET_NORMAL && events[RB_ANIM_HAPPY]==0);
    stroke(5000); assert(events[RB_ANIM_HAPPY]==0); release();
    init(); input(true,180,130); tick(40); claw_stroke(4500);
    assert(events[RB_ANIM_HAPPY]==1); claw_stroke(20000); assert(events[RB_ANIM_HAPPY]==1);
    release(); tick(18000); input(true,180,130); tick(40); claw_stroke(4500);
    assert(events[RB_ANIM_HAPPY]==2);
    init(); input(true,130,110); tick(40); stroke(1000); release(); tick((unsigned)(400*RB_PLAYBACK_DURATION_SCALE)+100);
    assert(events[RB_ANIM_NUZZLE]>=1 && events[RB_ANIM_HAPPY]==0 && p.action==RB_ANIM_IDLE);
    init(); input(true,100,110); tick(40);
    for (int x=108;x<=156;x+=8) { input(true,x,110); tick(20); }
    assert(p.swiped); tick(1200); assert(events[RB_ANIM_TILT]==1);
    stroke(3500); release(); assert(events[RB_ANIM_TILT]==1 && events[RB_ANIM_PROBE]==0 && events[RB_ANIM_HAPPY]==0);
    init(); input(true,200,110); tick(40);
    for (int x=192;x>=144;x-=8) { input(true,x,110); tick(20); }
    assert(p.swiped && p.active_direction==-1 && p.action==RB_ANIM_TILT);
    init(); input(true,20,110); tick(40); input(true,130,110); tick(300);
    assert(!p.contact && events[RB_ANIM_PROBE]==0); release();
    input(true,130,110); tick(40); stroke(1000); input(true,240,110); tick(40);
    assert(!p.contact && p.happy_ms==0); input(true,130,110); tick(300); assert(!p.contact);
    release(); input(true,130,110); tick(40); assert(p.contact);
    rb_pet_estop(&p,false); float phase=p.phase; tick(2000);
    assert(p.state==RB_PET_STOPPED && p.phase==phase && !p.pending_valid);
    assert(!rb_pet_reset(&p,now)); rb_pet_estop(&p,true); tick(100); assert(p.state==RB_PET_STOPPED);
    assert(rb_pet_reset(&p,now)); release(); tick((unsigned)(RB_MS_WAKE*RB_PLAYBACK_DURATION_SCALE)+200); assert(p.state==RB_PET_NORMAL);
    init(); tap(); tick(120); rb_pet_stop(&p,"test"); phase=p.phase; tick(1000); assert(p.phase==phase);
    init(); tap(); now+=10000; rb_pet_tick(&p,now); assert(p.action==RB_ANIM_IDLE);
    init(); p.lazy=true; tap(); assert(p.duration==(unsigned)(RB_MS_PROBE*RB_STYLE_LAZY*RB_PLAYBACK_DURATION_SCALE)); tick(p.duration); assert(p.action==RB_ANIM_IDLE);
    /* Stationary grace and the neutral speed band never add gentle-motion credit. */
    init(); input(true,180,130); tick(40); claw_stroke(500);
    unsigned valid=p.happy_ms; assert(valid>0); tick(200); assert(p.happy_ms==valid);
    tick(120); assert(!p.stroke && p.happy_ms==valid); release();
    init(); input(true,90,110); tick(40);
    for (int x=95;x<=225;x+=5) { input(true,x,110); tick(20); }
    assert(!p.stroke && !p.swiped && p.happy_ms==0 && events[RB_ANIM_NUZZLE]==0);
    /* Outside raw DOWN remains excluded even if moved inside during debounce. */
    init(); input(true,20,110); tick(20); input(true,130,110); tick(100);
    assert(!p.contact); release();
    /* Recovery is reversible; a new tap restores the folded stable pose. */
    init(); tap(); tap(); tap(); tick((unsigned)(RB_MS_WITHDRAW*RB_PLAYBACK_DURATION_SCALE)+200); input(true,130,110); tick(40); stroke(1400);
    assert(p.action==RB_ANIM_RECOVER); release(); tap(); tick((unsigned)(RB_MS_WITHDRAW*RB_PLAYBACK_DURATION_SCALE)+200);
    assert(p.state==RB_PET_FOLDED && p.pose==RB_POSE_FOLDED && p.action==RB_ANIM_IDLE);
    /* Wake cancels an in-progress rest from either stable origin. */
    rb_pet_rest(&p); tick(1200); assert(p.action==RB_ANIM_REST);
    input(true,130,110); tick(40); assert(p.action==RB_ANIM_REST);
    rb_pet_wake(&p); tick((unsigned)(RB_MS_WAKE*RB_PLAYBACK_DURATION_SCALE)+200);
    assert(p.state==RB_PET_NORMAL && p.action==RB_ANIM_IDLE && p.consumed);
    unsigned probes=events[RB_ANIM_PROBE]; release(); assert(events[RB_ANIM_PROBE]==probes);
    /* A tap removes a celebration waiting for a NUZZLE switching point. */
    init(); input(true,180,130); tick(40);
    for (unsigned i=0;i<250 && !(p.pending_valid && p.pending==RB_ANIM_HAPPY);++i) {
        unsigned n=i%30; input(true,180+(int)(n<=15?n:30-n),130); tick(20);
    }
    assert(p.pending_valid && p.pending==RB_ANIM_HAPPY);
    release(); tap(); tick(1500); assert(events[RB_ANIM_HAPPY]==0);
    /* Body-only stroking still nuzzles, but contributes no happiness. */
    init(); input(true,130,110); tick(40); stroke(6000);
    assert(events[RB_ANIM_NUZZLE]>0 && events[RB_ANIM_HAPPY]==0 && p.happy_ms==0);
    /* Candidate intervals on the body cannot be credited retroactively when
       the same contact reaches the claw. */
    release(); input(true,170,110); tick(40); stroke(300); claw_stroke(300);
    assert(p.happy_ms<=300 && events[RB_ANIM_HAPPY]==0);
    /* A finger held still on the claw is not a stroke. */
    init(); input(true,190,130); tick(10000);
    assert(p.happy_ms==0 && events[RB_ANIM_HAPPY]==0);
    /* Forehead rubbing cannot trigger happiness; one short claw stroke can. */
    init(); input(true,170,70); tick(40);
    for (unsigned i=0;i<200;++i) { unsigned n=i%30; input(true,170+(int)(n<=15?n:30-n),70); tick(20); }
    assert(events[RB_ANIM_HAPPY]==0 && p.happy_ms==0);
    init(); input(true,180,130); tick(40); claw_stroke(900); release(); tick(1200);
    assert(events[RB_ANIM_HAPPY]==1);
    /* Thirty simulated minutes of dense requests, stops and state changes. */
    init();
    for (unsigned t=0;t<1800000;t+=20) {
        if (t%5000==0) { input(true,130,110); }
        else if (t%5000<3500) input(true,130+(int)((t/20)%30)*2,110);
        else input(false,130,110);
        if (t && t%97000==0) { rb_pet_stop(&p,"soak"); assert(rb_pet_reset(&p,now)); }
        tick(20); assert(p.happy_count<=512 && p.candidate_count<=64 && p.tap_count<3);
    }
    printf("PASS reference gestures, stable poses, recovery interruption, queued HAPPY cancellation, neutral speed, stationary grace, wake consumption, stop and 30min simulated soak; model=%zu bytes\n",sizeof(p));
    return 0;
}
