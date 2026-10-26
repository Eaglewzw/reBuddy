#include "rb_pet.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static rb_pet_t pet;
static unsigned events;
static void event_sink(void *context,const rb_pet_event_t *event)
{ (void)context; (void)event; ++events; }
static void initialize(void)
{
    events=0; rb_pet_init(&pet,1000); rb_pet_set_sink(&pet,event_sink,NULL);
}
static void start_probe(void)
{
    initialize();
    rb_pet_input(&pet,true,130,110,1000); rb_pet_tick(&pet,1040);
    rb_pet_input(&pet,false,130,110,1080); rb_pet_tick(&pet,1120);
    assert(pet.action==RB_ANIM_PROBE && events==1);
    rb_pet_tick(&pet,2120);
    assert(fabsf(pet.phase-0.2f)<0.0001f);
}
static void assert_gestures_cleared(void)
{
    assert(!pet.pending_valid && pet.pending==RB_ANIM_IDLE);
    assert(!pet.raw_down && !pet.contact && !pet.stroke && !pet.swiped && !pet.consumed);
    assert(!pet.candidate_count && !pet.candidate_ms && !pet.position_count);
    assert(!pet.moving_ms && !pet.claw_moving_ms && !pet.fast_ms && !pet.soothe_ms);
    assert(!pet.happy_head && !pet.happy_count && !pet.happy_ms && !pet.tap_count);
    assert(!pet.path && !pet.max_offset && !pet.speed && !pet.candidate_path);
}
int main(void)
{
    start_probe();
    uint64_t started=pet.started,next_blink=pet.next_blink,last_interaction=pet.last_interaction;
    pet.pending=RB_ANIM_WITHDRAW; pet.pending_valid=true; pet.pending_source=RB_INPUT_POKE;
    pet.contact=pet.raw_down=pet.stroke=pet.swiped=pet.consumed=true;
    pet.candidate_count=pet.candidate_ms=pet.position_count=pet.tap_count=2;
    pet.moving_ms=pet.claw_moving_ms=pet.fast_ms=pet.soothe_ms=120;
    pet.happy_head=1; pet.happy_count=2; pet.happy_ms=120;
    pet.path=pet.max_offset=pet.speed=pet.candidate_path=10;
    pet.happy_latched=true; pet.happy_cooldown_until=12120;
    pet.blink=true; pet.blink_until=2220;
    rb_pet_suspend(&pet,2120); assert_gestures_cleared();
    assert(pet.suspended && pet.state==RB_PET_NORMAL && pet.action==RB_ANIM_PROBE);
    rb_pet_suspend(&pet,3000); /* repeated entry must not shorten the pause */
    assert(pet.suspended_at==2120);
    rb_pet_input(&pet,true,180,130,100000); rb_pet_tick(&pet,182120);
    rb_pet_rest(&pet); rb_pet_wake(&pet); rb_pet_cancel(&pet);
    assert(!rb_pet_reset(&pet,182120));
    assert_gestures_cleared();
    assert(events==1 && pet.completed==0 && pet.action==RB_ANIM_PROBE && pet.phase==0.2f);
    assert(pet.blink && pet.next_blink==next_blink);
    rb_pet_resume(&pet,182119); assert(pet.suspended); /* backwards time ignored */
    rb_pet_resume(&pet,182120);
    assert(!pet.suspended && pet.started==started+180000);
    assert(pet.last_interaction==last_interaction+180000);
    assert(pet.next_blink==next_blink+180000 && pet.blink_until==182220);
    assert(pet.happy_cooldown_until==192120 && pet.happy_latched);
    started=pet.started; rb_pet_resume(&pet,182120); assert(pet.started==started);
    rb_pet_input(&pet,true,130,110,182120); rb_pet_tick(&pet,182160);
    assert(!pet.raw_down && !pet.contact && pet.resume_wait_release);
    rb_pet_input(&pet,false,130,110,182160); rb_pet_tick(&pet,182620);
    assert(!pet.resume_wait_release && fabsf(pet.phase-0.3f)<0.0001f);
    rb_pet_tick(&pet,186120);
    assert(pet.action==RB_ANIM_IDLE && pet.completed==1 && events==1);

    /* Entering between ticks preserves the displayed phase, too. */
    start_probe(); started=pet.started;
    rb_pet_suspend(&pet,2130); rb_pet_resume(&pet,3130); rb_pet_tick(&pet,3130);
    assert(pet.started==started+1010 && fabsf(pet.phase-0.2f)<0.0001f);

    /* Retracing an interrupted clip retains its direction and remaining time. */
    start_probe(); pet.returning=true; pet.return_started=2120;
    pet.return_phase=0.2f; pet.return_duration=1000;
    rb_pet_suspend(&pet,2120); rb_pet_tick(&pet,12120); rb_pet_resume(&pet,12120);
    rb_pet_tick(&pet,12620);
    assert(pet.returning && fabsf(pet.phase-0.1f)<0.0001f);
    rb_pet_tick(&pet,13120); assert(!pet.returning && pet.action==RB_ANIM_IDLE);

    /* Editor time does not consume the daily idle sleep timer. */
    initialize(); rb_pet_tick(&pet,59000); assert(pet.action==RB_ANIM_IDLE);
    rb_pet_suspend(&pet,59000); rb_pet_tick(&pet,359000);
    assert(pet.state==RB_PET_NORMAL && pet.action==RB_ANIM_IDLE && events==0);
    rb_pet_resume(&pet,359000); rb_pet_input(&pet,false,0,0,359000);
    rb_pet_tick(&pet,360999); assert(pet.action==RB_ANIM_IDLE);
    rb_pet_tick(&pet,361000); assert(pet.action==RB_ANIM_REST && events==1);

    /* Both falling asleep and stable rest survive edit-mode suspension. */
    initialize(); rb_pet_rest(&pet); rb_pet_tick(&pet,2750);
    assert(pet.action==RB_ANIM_REST && fabsf(pet.phase-0.25f)<0.0001f);
    rb_pet_suspend(&pet,2750); rb_pet_tick(&pet,62750); rb_pet_wake(&pet);
    assert(pet.action==RB_ANIM_REST && events==1);
    rb_pet_resume(&pet,62750); rb_pet_tick(&pet,68000);
    assert(pet.state==RB_PET_RESTING && pet.pose==RB_POSE_REST && pet.action==RB_ANIM_IDLE);
    rb_pet_suspend(&pet,68000); rb_pet_wake(&pet); rb_pet_tick(&pet,368000);
    rb_pet_resume(&pet,368000); rb_pet_tick(&pet,368020);
    assert(pet.state==RB_PET_RESTING && pet.pose==RB_POSE_REST && events==1);

    /* A screen stop remains a screen stop and never resumes implicitly. */
    start_probe(); rb_pet_stop(&pet,"SCREEN STOP");
    rb_pet_suspend(&pet,2120); rb_pet_tick(&pet,72120); rb_pet_resume(&pet,72120);
    rb_pet_tick(&pet,73120);
    assert(pet.state==RB_PET_STOPPED && pet.phase==0.2f && !strcmp(pet.stop_reason,"SCREEN STOP"));
    assert(events==1);

    /* E-stop always wins, including while the editor owns ordinary input. */
    start_probe(); rb_pet_suspend(&pet,2120); rb_pet_estop(&pet,false);
    rb_pet_tick(&pet,12120); rb_pet_rest(&pet); rb_pet_wake(&pet);
    assert(pet.state==RB_PET_STOPPED && !strcmp(pet.stop_reason,"E-STOP"));
    assert(!rb_pet_reset(&pet,12120));
    rb_pet_estop(&pet,true); assert(!rb_pet_reset(&pet,12120));
    rb_pet_resume(&pet,12120); assert(pet.state==RB_PET_STOPPED && pet.phase==0.2f);
    assert(rb_pet_reset(&pet,12120));

    start_probe(); rb_pet_suspend(&pet,2120); rb_pet_stop(&pet,"SCREEN STOP");
    assert(pet.state==RB_PET_STOPPED);
    rb_pet_estop(&pet,false); assert(!strcmp(pet.stop_reason,"E-STOP"));
    puts("daily suspension: gestures cleared, clocks frozen/rebased, rest/stop preserved, E-stop priority passed");
    return 0;
}
