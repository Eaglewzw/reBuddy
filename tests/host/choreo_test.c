#include "rb_choreo.h"
#include "rb_pet.h"
#include "pet_animation_table.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static rb_choreo_work_t work;
static rb_choreo_plan_t plan;
static void init(void) { rb_choreo_work_init(&work,0); }
static void add(unsigned id,unsigned hold,unsigned duration)
{
    rb_choreo_step_t step={(uint8_t)id,1,(uint16_t)hold,(uint16_t)duration};
    assert(rb_choreo_add_after(&work,-1,&step)==RB_CHOREO_OK);
}
static void compile(void) { assert(rb_choreo_compile(&work,&plan,NULL)==RB_CHOREO_OK); }
static rb_choreo_sample_t at(unsigned ms)
{ rb_choreo_sample_t sample; assert(rb_choreo_sample(&plan,ms,&sample)); return sample; }
int main(void)
{
    _Static_assert((int)RB_CHOREO_PROBE==(int)RB_ANIM_PROBE,"existing action IDs stay stable");
    _Static_assert((int)RB_CHOREO_HAPPY==(int)RB_ANIM_HAPPY,"existing action IDs stay stable");
    /* Choreography source time must follow the actual generated GIF table.
       Otherwise a regenerated animation silently speeds up or stalls here. */
    unsigned available=0;
    for (size_t i=0;i<rb_choreo_action_count;++i) {
        const rb_choreo_action_t *action=&rb_choreo_actions[i];
        assert(action->available);
        ++available;
        if (action->action_id==RB_CHOREO_WAIT) continue;
        assert(action->action_id<sizeof(rb_choreo_clip_map)/sizeof(rb_choreo_clip_map[0]) && rb_choreo_clip_map[action->action_id]!=UINT8_MAX);
        const rb_clip_meta_t *clip=&rb_clips[rb_choreo_clip_map[action->action_id]];
        assert(clip->count && clip->count<=64 && clip->duration_ms==action->duration_ms);
        assert(clip->at[0]==0 && clip->at[clip->count-1]<clip->duration_ms);
        for (unsigned frame=0;frame<clip->count;++frame) {
            assert(clip->frames[frame]<RB_FRAME_COUNT);
            if (frame) assert(clip->at[frame]>clip->at[frame-1]);
        }
        if (action->action_id<RB_ANIM_REST) {
            for (unsigned origin=RB_POSE_IDLE;origin<=RB_POSE_REST;++origin)
                assert(rb_clips[rb_clip_map[origin][action->action_id]].duration_ms==action->duration_ms);
        }
    }
    assert(available==17); /* Sixteen authored actions and WAIT. */
    uint8_t bad=0;
    init(); assert(work.work_id==1 && work.tempo==RB_CHOREO_STANDARD);
    assert(rb_choreo_validate(&work,&bad)==RB_CHOREO_OK && bad==RB_CHOREO_NO_STEP);
    assert(rb_choreo_compile(&work,&plan,&bad)==RB_CHOREO_EMPTY && plan.count==0);
    assert(!rb_choreo_sample(&plan,0,&(rb_choreo_sample_t){0}));
    for (unsigned slot=0;slot<RB_CHOREO_SLOTS;++slot) {
        rb_choreo_work_init(&work,slot); assert(work.work_id==slot+1);
        assert(rb_choreo_validate(&work,NULL)==RB_CHOREO_OK);
    }
    rb_choreo_work_init(&work,RB_CHOREO_SLOTS); assert(rb_choreo_validate(&work,NULL)==RB_CHOREO_BAD_WORK_ID);
    assert(rb_choreo_validate(NULL,NULL)==RB_CHOREO_INVALID_ARGUMENT);
    init(); work.schema_version=2; assert(rb_choreo_validate(&work,NULL)==RB_CHOREO_BAD_VERSION);
    init(); work.tempo=RB_CHOREO_TEMPO_COUNT; assert(rb_choreo_validate(&work,NULL)==RB_CHOREO_BAD_TEMPO);
    init(); work.count=RB_CHOREO_MAX_STEPS+1; assert(rb_choreo_validate(&work,NULL)==RB_CHOREO_TOO_MANY_STEPS);
    init(); memset(work.name,'x',sizeof(work.name)); assert(rb_choreo_validate(&work,NULL)==RB_CHOREO_BAD_NAME);
    init(); work.name[0]=0; assert(rb_choreo_validate(&work,NULL)==RB_CHOREO_BAD_NAME);
    init(); strcpy(work.name,"bad\nname"); assert(rb_choreo_validate(&work,NULL)==RB_CHOREO_BAD_NAME);
    const unsigned char invalid_names[][5]={{0xc0,0xaf,0},{0xed,0xa0,0x80,0},{0xf4,0x90,0x80,0x80,0},{0xe4,0xb8,0},{0x80,0}};
    for (unsigned i=0;i<sizeof(invalid_names)/sizeof(invalid_names[0]);++i) {
        init(); memcpy(work.name,invalid_names[i],sizeof(invalid_names[i]));
        assert(rb_choreo_validate(&work,NULL)==RB_CHOREO_BAD_NAME);
    }
    init(); add(RB_CHOREO_PROBE,0,0);
    work.steps[0].action_id=RB_ANIM_REST; assert(rb_choreo_validate(&work,&bad)==RB_CHOREO_UNKNOWN_ACTION && bad==0);
    work.steps[0].action_id=RB_ANIM_WAKE; assert(rb_choreo_validate(&work,NULL)==RB_CHOREO_UNKNOWN_ACTION);
    /* Retired saved IDs never alias the replacement dance library. */
    for (unsigned id=9;id<=22;++id) {
        if (id==RB_CHOREO_WAIT) continue;
        work.steps[0].action_id=(uint8_t)id;
        assert(!rb_choreo_action(id) && rb_choreo_clip_map[id]==UINT8_MAX);
        assert(rb_choreo_validate(&work,&bad)==RB_CHOREO_UNKNOWN_ACTION && bad==0);
        assert(rb_choreo_compile(&work,&plan,NULL)==RB_CHOREO_UNKNOWN_ACTION && plan.count==0);
    }
    for (unsigned id=RB_CHOREO_GROOVE_BOUNCE;id<=RB_CHOREO_DIME_STOP;++id) {
        work.steps[0]=(rb_choreo_step_t){(uint8_t)id,3,0,0};
        assert(rb_choreo_validate(&work,NULL)==RB_CHOREO_OK);
        work.steps[0].repeat=4; assert(rb_choreo_validate(&work,NULL)==RB_CHOREO_BAD_REPEAT);
    }
    assert(rb_choreo_action_count==17 && !rb_choreo_action(99));
    init(); add(RB_CHOREO_PROBE,0,0); work.steps[0].repeat=2;
    assert(rb_choreo_validate(&work,NULL)==RB_CHOREO_BAD_REPEAT);
    work.steps[0].repeat=0; assert(rb_choreo_validate(&work,NULL)==RB_CHOREO_BAD_REPEAT);
    work.steps[0].repeat=1; work.steps[0].hold_ms=3500; assert(rb_choreo_validate(&work,NULL)==RB_CHOREO_BAD_HOLD);
    work.steps[0].hold_ms=1; assert(rb_choreo_validate(&work,NULL)==RB_CHOREO_BAD_HOLD);
    work.steps[0].hold_ms=0; work.steps[0].duration_ms=500; assert(rb_choreo_validate(&work,NULL)==RB_CHOREO_BAD_DURATION);
    init(); add(RB_CHOREO_WAIT,0,500);
    work.steps[0].duration_ms=0; assert(rb_choreo_validate(&work,NULL)==RB_CHOREO_BAD_DURATION);
    work.steps[0].duration_ms=3500; assert(rb_choreo_validate(&work,NULL)==RB_CHOREO_BAD_DURATION);
    work.steps[0].duration_ms=501; assert(rb_choreo_validate(&work,NULL)==RB_CHOREO_BAD_DURATION);
    work.steps[0].duration_ms=500; work.steps[0].hold_ms=500; assert(rb_choreo_validate(&work,NULL)==RB_CHOREO_BAD_HOLD);

    /* Bounded edits are transactional and support copying from the same array. */
    init(); add(RB_CHOREO_PROBE,0,0); add(RB_CHOREO_HAPPY,0,0);
    assert(rb_choreo_add_after(&work,0,&work.steps[1])==RB_CHOREO_OK);
    assert(work.count==3 && work.steps[1].action_id==RB_CHOREO_HAPPY);
    assert(rb_choreo_move(&work,1,-1)==RB_CHOREO_OK && work.steps[0].action_id==RB_CHOREO_HAPPY);
    assert(rb_choreo_delete(&work,1)==RB_CHOREO_OK && work.count==2);
    rb_choreo_work_t before=work;
    assert(rb_choreo_move(&work,0,-1)==RB_CHOREO_BAD_INDEX);
    assert(rb_choreo_move(&work,work.count-1,1)==RB_CHOREO_BAD_INDEX);
    assert(rb_choreo_move(&work,0,0)==RB_CHOREO_BAD_INDEX);
    assert(rb_choreo_delete(&work,UINT32_MAX)==RB_CHOREO_BAD_INDEX);
    assert(rb_choreo_add_after(&work,-2,&work.steps[0])==RB_CHOREO_BAD_INDEX);
    assert(rb_choreo_add_after(&work,work.count,&work.steps[0])==RB_CHOREO_BAD_INDEX);
    assert(!memcmp(&before,&work,sizeof(work)));
    while (work.count<RB_CHOREO_MAX_STEPS) add(RB_CHOREO_WAIT,0,3000);
    assert(rb_choreo_add_after(&work,-1,&work.steps[0])==RB_CHOREO_TOO_MANY_STEPS);
    while (work.count) assert(rb_choreo_delete(&work,work.count-1)==RB_CHOREO_OK);
    assert(rb_choreo_delete(&work,0)==RB_CHOREO_BAD_INDEX);

    /* Idle -> RECOVER needs a visible WITHDRAW bridge, included in total. */
    init(); add(RB_CHOREO_RECOVER,0,0); compile();
    assert(plan.count==2 && plan.base_ms==5000 && plan.total_ms==8000);
    assert(plan.events[0].action_id==RB_CHOREO_WITHDRAW && plan.events[0].transition);
    rb_choreo_sample_t sample=at(2999); assert(sample.action_id==RB_CHOREO_WITHDRAW && sample.clip_time_ms==2999 && sample.transition);
    sample=at(3000); assert(sample.action_id==RB_CHOREO_RECOVER && sample.clip_time_ms==0 && !sample.transition);
    sample=at(8000); assert(sample.done && sample.holding && sample.clip_time_ms==4999);
    sample=at(UINT32_MAX); assert(sample.done && sample.clip_time_ms==4999);

    /* Holds and WAIT preserve WITHDRAW; PROBE gets an explicit RECOVER. */
    init(); add(RB_CHOREO_WITHDRAW,1000,0); add(RB_CHOREO_WAIT,0,1500); add(RB_CHOREO_PROBE,0,0); compile();
    assert(plan.count==5 && plan.base_ms==10500 && plan.total_ms==15500);
    sample=at(3000); assert(sample.holding && sample.action_id==RB_CHOREO_WITHDRAW && sample.clip_time_ms==2999 && sample.step_index==0);
    sample=at(4000); assert(sample.holding && sample.action_id==RB_CHOREO_WITHDRAW && sample.step_index==1);
    sample=at(5500); assert(!sample.holding && sample.transition && sample.action_id==RB_CHOREO_RECOVER && sample.step_index==2);
    sample=at(10500); assert(sample.action_id==RB_CHOREO_PROBE && sample.clip_time_ms==0 && !sample.transition);
    init(); add(RB_CHOREO_WITHDRAW,0,0); add(RB_CHOREO_RECOVER,0,0); compile();
    assert(plan.count==2 && plan.base_ms==plan.total_ms);
    init(); add(RB_CHOREO_WITHDRAW,0,0); add(RB_CHOREO_WITHDRAW,0,0); compile();
    assert(plan.count==3 && plan.events[1].action_id==RB_CHOREO_RECOVER && plan.events[1].transition);
    init(); add(RB_CHOREO_WAIT,0,500); compile(); sample=at(0);
    assert(sample.holding && sample.action_id==0 && sample.clip_time_ms==0 && !sample.done);
    assert(at(500).done);

    /* Tempo affects wall time, never the source GIF coordinates. */
    init(); add(RB_CHOREO_PROBE,1000,0); work.tempo=RB_CHOREO_SLOW; compile();
    assert(plan.total_ms==9000 && at(3750).clip_time_ms==2500 && at(7500).holding);
    work.tempo=RB_CHOREO_FAST; compile();
    assert(plan.total_ms==4800 && at(2000).clip_time_ms==2500 && at(4000).holding);
    init(); for (unsigned i=0;i<RB_CHOREO_MAX_STEPS;++i) add(RB_CHOREO_RECOVER,3000,0); compile();
    assert(plan.count==24 && plan.count<=RB_CHOREO_MAX_EVENTS && plan.total_ms==88000 && plan.base_ms==64000);
    work.tempo=255; assert(rb_choreo_compile(&work,&plan,NULL)==RB_CHOREO_BAD_TEMPO && plan.count==0);

    /* Stable new wire IDs, independently selectable and repeatable. */
    _Static_assert(RB_CHOREO_WAIT==15 && RB_CHOREO_GROOVE_BOUNCE==23 && RB_CHOREO_DIME_STOP==32,"wire IDs");
    _Static_assert(sizeof(rb_choreo_clip_map)/sizeof(rb_choreo_clip_map[0])==33,"sparse mapping preserves retired IDs");
    for (unsigned id=RB_CHOREO_GROOVE_BOUNCE;id<=RB_CHOREO_DIME_STOP;++id) {
        init(); add(id,500,0); compile();
        const rb_choreo_action_t *action=rb_choreo_action(id);
        assert(action->entry_pose==RB_CHOREO_POSE_IDLE && action->exit_pose==RB_CHOREO_POSE_IDLE);
        assert(action->repeat_max==3 && action->group==RB_CHOREO_PERFORMANCE);
        assert(plan.count==2 && plan.total_ms==action->duration_ms+500u);
        assert(at(0).action_id==id && at(action->duration_ms-1).clip_time_ms==action->duration_ms-1);
        assert(at(plan.total_ms).done && at(plan.total_ms).holding);
    }
    /* Six basic dance steps / forty beats at 80 BPM. */
    for (unsigned slot=0;slot<RB_CHOREO_SLOTS;++slot) {
        rb_choreo_work_demo(&work,slot); compile();
        assert(work.work_id==slot+1 && !strcmp(work.name,"基础律动") && work.count==6);
        assert(plan.count==6 && plan.base_ms==30000 && plan.total_ms==30000);
        const unsigned ids[]={RB_CHOREO_GROOVE_BOUNCE,RB_CHOREO_GROOVE_SIDE,RB_CHOREO_DANCE_POINT,
            RB_CHOREO_ARM_WAVE,RB_CHOREO_DANCE_ROLL,RB_CHOREO_DIME_STOP};
        const unsigned starts[]={0,3000,6000,12000,18000,24000};
        for (unsigned event=0;event<6;++event) {
            assert(plan.events[event].action_id==ids[event] && plan.events[event].start_ms==starts[event]);
            assert(!plan.events[event].holding && !plan.events[event].transition);
            assert(at(starts[event]).clip_time_ms==0 && at(starts[event]).action_id==ids[event]);
        }
        assert(at(29999).action_id==RB_CHOREO_DIME_STOP && !at(29999).done);
        assert(at(30000).done && at(30000).holding && at(30000).clip_time_ms==5999);
        work.tempo=RB_CHOREO_SLOW; compile(); assert(plan.total_ms==45000);
        work.tempo=RB_CHOREO_FAST; compile(); assert(plan.total_ms==24000);
    }
    rb_choreo_work_demo(&work,RB_CHOREO_SLOTS); assert(rb_choreo_validate(&work,NULL)==RB_CHOREO_BAD_WORK_ID);
    rb_choreo_work_demo(NULL,0);
    init();
    for (unsigned i=0;i<RB_CHOREO_MAX_STEPS;++i) { add(RB_CHOREO_ARM_WAVE,3000,0); work.steps[i].repeat=3; }
    compile(); assert(plan.count==32 && plan.total_ms==168000);
    /* A dance action from folded state uses the existing recovery bridge. */
    init(); add(RB_CHOREO_WITHDRAW,0,0); add(RB_CHOREO_GROOVE_BOUNCE,0,0); compile();
    assert(plan.count==3 && plan.total_ms==11000 && plan.events[1].action_id==RB_CHOREO_RECOVER && plan.events[1].transition);

    /* Repeats and final hold sample the actual three/six-second clips. */
    init(); add(RB_CHOREO_DANCE_HIT,0,0); work.steps[0].repeat=3;
    add(RB_CHOREO_DIME_STOP,500,0); compile();
    assert(plan.count==5 && plan.total_ms==15500 && plan.base_ms==15500);
    assert(at(8999).action_id==RB_CHOREO_DANCE_HIT && at(8999).clip_time_ms==2999);
    assert(at(9000).action_id==RB_CHOREO_DIME_STOP && at(9000).clip_time_ms==0);
    assert(at(15000).holding && at(15000).clip_time_ms==5999 && !at(15000).done);
    assert(at(15500).done && at(15500).clip_time_ms==5999);

    /* Stop holds current pixels; clock jumps cannot overflow or auto-restart. */
    init(); add(RB_CHOREO_HAPPY,0,0); compile();
    rb_choreo_player_t player;
    assert(rb_choreo_player_start(&player,&plan,1000));
    rb_choreo_player_tick(&player,3000); assert(player.elapsed_ms==2000 && player.sample.clip_time_ms==2000);
    rb_choreo_player_tick(&player,2000); assert(player.elapsed_ms==2000);
    rb_choreo_player_stop(&player,3500); assert(!player.running && player.sample.holding && !player.sample.done && player.sample.clip_time_ms==2500);
    rb_choreo_player_tick(&player,999999); assert(player.sample.clip_time_ms==2500);
    assert(rb_choreo_player_start(&player,&plan,5000));
    rb_choreo_player_tick(&player,UINT64_MAX); assert(!player.running && player.elapsed_ms==8000 && player.sample.done && player.sample.clip_time_ms==7999);
    rb_choreo_player_tick(&player,UINT64_MAX); assert(player.elapsed_ms==8000);
    assert(!rb_choreo_player_start(&player,NULL,0) && !player.running);
    puts("PASS choreography bounds, UTF-8/schema validation, editing, visual bridges, WAIT/holds, tempo mapping and stop/finish retention");
    return 0;
}
