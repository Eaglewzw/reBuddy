#include "rb_daily.h"
#include "rb_pet.h"
#include "pet_animation_table.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

int main(void)
{
    const unsigned ids[]={0,1,2,3,6,7,9,8};
    for (unsigned i=0;i<8;++i) {
        const rb_daily_action_t *a=rb_daily_find(ids[i]);assert(a && a->gif_index==i);
        assert(a->clip_duration_ms==rb_clips[i].duration_ms);
        if (a->id!=RB_DAILY_SLEEP) assert(rb_clip_map[0][a->id]==i);
        else assert(rb_clip_map[2][RB_ANIM_IDLE]==i);
        uint16_t last=0;
        for (unsigned j=0;j<=1000;++j) {
            uint16_t t=rb_daily_clip_time(a->id,j/1000.f);
            assert(t>=last && t<=a->clip_duration_ms);last=t;
        }
        assert(last==a->clip_duration_ms);
        float q[7],v[7],acc[7];
        assert(rb_daily_sample(a->id,0,q,v,acc));
        assert(rb_daily_sample(a->id,a->duration_s+1,q,v,acc));
        for (unsigned j=0;j<7;++j) assert(v[j]==0 && acc[j]==0 && isfinite(q[j]));
        assert(!rb_daily_sample(a->id,NAN,q,v,acc));
    }
    assert(!rb_daily_find(4) && !rb_daily_find(5) && !rb_daily_find(100));
    assert(rb_daily_for_screen(RB_ANIM_IDLE,RB_POSE_REST)->id==RB_DAILY_SLEEP);
    const rb_daily_action_t *happy=rb_daily_find(RB_DAILY_HAPPY);
    for (unsigned i=0;i<happy->sync_count;++i)
        assert(abs((int)rb_daily_clip_time(happy->id,happy->sync[i].motion_s/happy->duration_s)-
                   (int)happy->sync[i].clip_ms)<=1);
    rb_pet_t pet;rb_pet_init(&pet,100);rb_pet_use_daily_timing(&pet,true);
    rb_pet_rest(&pet);assert(pet.duration==rb_daily_find(RB_DAILY_REST)->duration_ms);
    rb_pet_tick(&pet,100+pet.duration+1);assert(pet.state==RB_PET_RESTING);
    rb_pet_wake(&pet);assert(pet.duration==rb_daily_find(RB_DAILY_WAKE)->duration_ms);
    rb_pet_tick(&pet,pet.started+pet.duration+1);assert(pet.state==RB_PET_NORMAL);
    assert(rb_daily_policy.torque_budget_nm[6]==1.5f);
    assert(rb_daily_policy.position_speed==.5f && rb_daily_policy.exit_speed==1.f);
    rb_pet_init(&pet,100);rb_pet_use_daily_timing(&pet,true);pet.motion_clock=true;
    rb_pet_rest(&pet);unsigned request=pet.last_event.request_id;
    rb_pet_tick(&pet,100000);assert(pet.action==RB_ANIM_REST && pet.phase==0);
    rb_pet_follow_motion(&pet,request+1,1,true);assert(pet.action==RB_ANIM_REST);
    rb_pet_follow_motion(&pet,request,.5f,false);assert(pet.phase==.5f);
    rb_pet_wake(&pet);assert(pet.action==RB_ANIM_REST && !pet.returning && !pet.pending_valid);
    rb_pet_follow_motion(&pet,request,1,true);assert(pet.state==RB_PET_RESTING);
    rb_pet_init_sleep(&pet,100);pet.motion_clock=true;
    assert(pet.requests==0 && pet.pose==RB_POSE_REST && pet.state==RB_PET_RESTING);
    rb_pet_tick(&pet,100000);assert(pet.action==RB_ANIM_IDLE && pet.requests==0);
    rb_pet_wake(&pet);assert(pet.action==RB_ANIM_WAKE && pet.requests==1);
    rb_pet_wake(&pet);assert(pet.requests==1);
    rb_pet_follow_motion(&pet,1,1,true);assert(pet.state==RB_PET_NORMAL && pet.pose==RB_POSE_IDLE);
    puts("PASS 8 GIF bindings, shared timing, sleep/wake and C plan API (no motor I/O)");
}
