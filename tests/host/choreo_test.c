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
        if (!action->available) { assert(rb_choreo_clip_map[action->action_id]==UINT8_MAX); continue; }
        ++available;
        if (action->action_id==RB_CHOREO_WAIT) continue;
        assert(action->action_id<sizeof(rb_choreo_clip_map)/sizeof(rb_choreo_clip_map[0]) && rb_choreo_clip_map[action->action_id]!=UINT8_MAX);
        const rb_clip_meta_t *clip=&rb_clips[rb_choreo_clip_map[action->action_id]];
        assert(clip->count && clip->count<=128 && clip->duration_ms==action->duration_ms);
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
    assert(available==11); /* Ten selected dances and timing-only WAIT. */
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
    init(); add(RB_CHOREO_2D_NOD,0,0);
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
        assert(rb_choreo_validate(&work,NULL)==RB_CHOREO_UNKNOWN_ACTION);
        assert(rb_choreo_compile(&work,&plan,NULL)==RB_CHOREO_UNKNOWN_ACTION);
    }
    assert(rb_choreo_action_count==11 && !rb_choreo_action(99));
    init(); add(RB_CHOREO_2D_NOD,0,0); work.steps[0].repeat=4;
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
    init(); add(RB_CHOREO_2D_NOD,0,0); add(RB_CHOREO_2D_LIFT,0,0);
    assert(rb_choreo_add_after(&work,0,&work.steps[1])==RB_CHOREO_OK);
    assert(work.count==3 && work.steps[1].action_id==RB_CHOREO_2D_LIFT);
    assert(rb_choreo_move(&work,1,-1)==RB_CHOREO_OK && work.steps[0].action_id==RB_CHOREO_2D_LIFT);
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

    /* Only selected dances may be authored or loaded; no daily bridges. */
    for (unsigned id=0;id<9;++id) {
        init(); work.count=1; work.steps[0]=(rb_choreo_step_t){id,1,0,0};
        assert(rb_choreo_validate(&work,NULL)==RB_CHOREO_UNKNOWN_ACTION);
        assert(rb_choreo_compile(&work,&plan,NULL)==RB_CHOREO_UNKNOWN_ACTION && plan.count==0);
    }
    for(unsigned id=RB_CHOREO_START;id<=RB_CHOREO_END;++id) {
        init();rb_choreo_step_t step={id,1,0,0};
        assert(rb_choreo_add_after(&work,-1,&step)==RB_CHOREO_UNKNOWN_ACTION);
    }
    init(); add(RB_CHOREO_2D_NOD,1000,0); add(RB_CHOREO_WAIT,0,1500); add(RB_CHOREO_2D_LIFT,0,0); compile();
    assert(work.count==3 && plan.count==6 && plan.total_ms==12100 && plan.base_ms==7300);
    assert(at(0).action_id==RB_CHOREO_START && at(0).step_index==RB_CHOREO_NO_STEP);
    assert(at(2399).clip_time_ms>=2998 && at(2400).action_id==RB_CHOREO_2D_NOD);
    rb_choreo_sample_t sample=at(5800); assert(sample.holding && sample.action_id==RB_CHOREO_2D_NOD && sample.clip_time_ms==2399);
    sample=at(7300);assert(sample.action_id==RB_CHOREO_2D_LIFT && sample.clip_time_ms==0 && !sample.transition);
    assert(at(9700).action_id==RB_CHOREO_END && at(9700).clip_time_ms==0);
    assert(at(12100).done && at(12100).action_id==RB_CHOREO_END && at(12100).clip_time_ms==2999);
    init();add(RB_CHOREO_WAIT,0,500);compile();sample=at(2400);
    assert(sample.holding && sample.action_id==RB_CHOREO_START && sample.clip_time_ms==2999);
    assert(at(2900).action_id==RB_CHOREO_END && at(5300).done);
    /* Automatic bookends follow tempo, but never consume authored steps. */
    init();add(RB_CHOREO_2D_NOD,1000,0);work.tempo=RB_CHOREO_SLOW;compile();
    assert(plan.total_ms==12300 && at(5400).clip_time_ms==1200 && at(7200).holding);
    work.tempo=RB_CHOREO_FAST;compile();
    assert(plan.total_ms==6560 && at(2880).clip_time_ms==1200 && at(3840).holding);
    init();for(unsigned i=0;i<RB_CHOREO_MAX_STEPS;++i) {add(RB_CHOREO_2D_LIFT,3000,0);work.steps[i].repeat=3;}compile();
    assert(work.count==8 && plan.count==34 && plan.total_ms==86400 && plan.base_ms==81600);
    for(unsigned i=1;i+1<plan.count;++i)assert(!plan.events[i].transition);
    work.tempo=255;assert(rb_choreo_compile(&work,&plan,NULL)==RB_CHOREO_BAD_TEMPO && plan.count==0);
    for(unsigned id=RB_CHOREO_2D_NOD;id<=RB_CHOREO_2D_WAVE;++id) {
        init();add(id,500,0);work.steps[0].repeat=3;compile();
        assert(plan.total_ms==12500 && at(2400).action_id==id);
        assert(at(4799).clip_time_ms==2399 && at(4800).clip_time_ms==0);
        assert(at(9600).holding && at(10100).action_id==RB_CHOREO_END && at(12500).done);
    }

    /* Every new hold edit lies on a half-beat; legacy 500ms values remain valid.
       The shared authored phase is monotonic and slows into/out of neutral. */
    init();add(RB_CHOREO_2D_NOD,300,0);compile();
    assert(rb_choreo_beat_ms(RB_CHOREO_STANDARD)==600 && rb_choreo_beat_ms(99)==0);
    unsigned previous=0;
    for(unsigned t=0;t<2400;++t) {
        rb_choreo_sample_t s=at(2400+t);
        assert(s.clip_time_ms>=previous);
        previous=s.clip_time_ms;
    }
    assert(at(2450).clip_time_ms<50 && at(4750).clip_time_ms>2350);
    assert(at(2400).phase_q16==0 && at(4799).phase_q16==65535);

    /* Stable new wire IDs, independently selectable and repeatable. */
    _Static_assert(RB_CHOREO_WAIT==15 && RB_CHOREO_GROOVE_BOUNCE==23 && RB_CHOREO_DIME_STOP==32,"wire IDs");
    _Static_assert(sizeof(rb_choreo_clip_map)/sizeof(rb_choreo_clip_map[0])==45,"sparse mapping preserves retired IDs");
    for (unsigned slot=0;slot<RB_CHOREO_SLOTS;++slot) {
        rb_choreo_work_demo(&work,slot);
        assert(work.count==0 && rb_choreo_compile(&work,&plan,NULL)==RB_CHOREO_EMPTY);
    }
    rb_choreo_work_demo(NULL,0);

    /* Stop holds current pixels; clock jumps cannot overflow or auto-restart. */
    init(); add(RB_CHOREO_2D_LIFT,0,0); compile();
    rb_choreo_player_t player;
    assert(rb_choreo_player_start(&player,&plan,1000));
    rb_choreo_player_tick(&player,3000); assert(player.elapsed_ms==2000 && player.sample.clip_time_ms==2500);
    rb_choreo_player_tick(&player,2000); assert(player.elapsed_ms==2000);
    rb_choreo_player_stop(&player,3200); assert(!player.running && player.sample.holding && !player.sample.done && player.sample.clip_time_ms==2750);
    rb_choreo_player_tick(&player,999999); assert(player.sample.clip_time_ms==2750);
    assert(rb_choreo_player_start(&player,&plan,5000));
    rb_choreo_player_tick(&player,UINT64_MAX); assert(!player.running && player.elapsed_ms==7200 && player.sample.done && player.sample.clip_time_ms==2999);
    rb_choreo_player_tick(&player,UINT64_MAX); assert(player.elapsed_ms==7200);
    assert(rb_clips[rb_choreo_clip_map[RB_CHOREO_START]].duration_ms==3000);
    assert(rb_clips[rb_choreo_clip_map[RB_CHOREO_END]].duration_ms==3000);
    assert(rb_choreo_player_start(&player,&plan,0));rb_choreo_player_tick(&player,6000);
    assert(player.sample.action_id==RB_CHOREO_END && player.sample.clip_time_ms==1500);
    rb_choreo_player_stop(&player,6100);rb_choreo_player_tick(&player,100000);
    assert(!player.running && player.sample.action_id==RB_CHOREO_END && player.sample.clip_time_ms==1625);
    assert(rb_choreo_player_start(&player,&plan,200000));
    assert(player.sample.action_id==RB_CHOREO_START && player.sample.clip_time_ms==0);
    assert(!rb_choreo_player_start(&player,NULL,0) && !player.running);
    puts("PASS choreography bounds, UTF-8/schema validation, editing, visual bridges, WAIT/holds, tempo mapping and stop/finish retention");
    return 0;
}
