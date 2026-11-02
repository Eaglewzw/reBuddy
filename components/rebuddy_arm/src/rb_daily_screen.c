#include "rb_daily_screen.h"

bool rb_daily_sync_screen(rb_pet_t *p,const rb_dc_status_t *s)
{
    if(!p || !s || s->error || p->state==RB_PET_STOPPED || p->suspended)return false;
    if(p->action==RB_ANIM_REST) {
        unsigned request=p->last_event.request_id;
        if(s->action==RB_DAILY_SLEEP && s->request_id==(request|0x40000000u)) {
            if(s->state==RB_DC_OFF && s->disable_confirmed && !s->enabled &&
                s->completed_id==s->request_id)rb_pet_follow_motion(p,request,1,true);
        } else if(s->action==RB_DAILY_REST && s->request_id==request) {
            rb_pet_follow_motion(p,request,s->phase,false);
            return s->state==RB_DC_ZERO_HOLD && s->enabled && s->completed_id==request;
        }
        return false;
    }
    rb_pet_follow_motion(p,s->request_id,s->phase,
        s->state==RB_DC_HOLD && s->enabled && s->completed_id==s->request_id);
    return false;
}
