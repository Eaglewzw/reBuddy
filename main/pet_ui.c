#include "pet_ui.h"
#include "pet_assets.h"
#include "lvgl.h"
#include <math.h>
#include <stdint.h>
#include <string.h>

static rb_pet_t *pet;
static lv_obj_t *body,*eye_group[2],*eyes[2],*pupils[2],*happy_eyes[2],*blush[2];
static lv_obj_t *settings,*style_label,*rest_label,*stop_label,*emote,*placeholder;
static bool settings_open;
static uint64_t last_body_ms,feedback_ms,raw_down_at;
static bool feedback_pending;
static unsigned body_frame,body_updates;
static const rb_frame_meta_t *anchor;
static lv_obj_t *box(lv_obj_t *parent,int x,int y,int w,int h,uint32_t color,int radius)
{
    lv_obj_t *o=lv_obj_create(parent); lv_obj_remove_style_all(o);
    lv_obj_set_pos(o,x,y); lv_obj_set_size(o,w,h);
    lv_obj_set_style_bg_color(o,lv_color_hex(color),0); lv_obj_set_style_bg_opa(o,LV_OPA_COVER,0);
    lv_obj_set_style_radius(o,radius,0); lv_obj_remove_flag(o,LV_OBJ_FLAG_SCROLLABLE|LV_OBJ_FLAG_CLICKABLE);
    return o;
}
static lv_obj_t *text(lv_obj_t *parent,int x,int y,const char *value)
{
    lv_obj_t *o=lv_label_create(parent); lv_label_set_text_static(o,value); lv_obj_set_pos(o,x,y);
    lv_obj_set_style_text_color(o,lv_color_hex(0x66705c),0);
    lv_obj_set_style_text_font(o,&lv_font_montserrat_12,0);
    lv_obj_remove_flag(o,LV_OBJ_FLAG_CLICKABLE); return o;
}
static void hidden(lv_obj_t *o,bool hide)
{ if (hide) lv_obj_add_flag(o,LV_OBJ_FLAG_HIDDEN); else lv_obj_remove_flag(o,LV_OBJ_FLAG_HIDDEN); }
static void label_text(lv_obj_t *o,const char *value)
{ if (strcmp(lv_label_get_text(o),value)) lv_label_set_text_static(o,value); }
static void touch(lv_event_t *event)
{
    lv_event_code_t code=lv_event_get_code(event);
    if (code==LV_EVENT_PRESS_LOST || code==LV_EVENT_INDEV_RESET) {
        rb_pet_input(pet,false,0,0,rb_platform_now_ms()); rb_pet_cancel(pet); feedback_pending=false; return;
    }
    if (settings_open || (code!=LV_EVENT_PRESSED && code!=LV_EVENT_PRESSING && code!=LV_EVENT_RELEASED)) return;
    lv_indev_t *indev=lv_indev_active(); if (!indev) return;
    lv_point_t point; lv_indev_get_point(indev,&point);
    uint64_t now=rb_platform_now_ms();
    if (code==LV_EVENT_PRESSED && point.x>=RB_ROLE_X && point.x<RB_ROLE_X+RB_ROLE_WIDTH &&
        point.y>=RB_ROLE_Y && point.y<RB_ROLE_Y+RB_ROLE_HEIGHT) {
        raw_down_at=now; feedback_pending=true;
    }
    rb_pet_input(pet,code!=LV_EVENT_RELEASED,point.x,point.y,now);
    /* Only record input here: frame/cache work stays in the 20ms UI timer. */
}
static void button_event(lv_event_t *event)
{
    intptr_t id=(intptr_t)lv_event_get_user_data(event);
    lv_event_code_t code=lv_event_get_code(event);
    if (code!=LV_EVENT_PRESSED && code!=LV_EVENT_CLICKED && code!=LV_EVENT_LONG_PRESSED) return;
    if (id==3) {
        if (pet->state==RB_PET_STOPPED) {
            if (code==LV_EVENT_LONG_PRESSED) (void)rb_pet_reset(pet,rb_platform_now_ms());
        } else if (code==LV_EVENT_PRESSED) { rb_pet_stop(pet,"SCREEN STOP"); settings_open=false; }
    } else if (code==LV_EVENT_CLICKED) {
        rb_pet_cancel(pet); feedback_pending=false;
        if (id==0) settings_open=!settings_open;
        else if (id==4) settings_open=false;
        else if (pet->state!=RB_PET_STOPPED) {
            if (id==1) pet->lazy=!pet->lazy;
            if (id==2) { rb_pet_rest(pet); settings_open=false; }
        }
    }
    /* STOP has immediate overlay feedback, but never loads a different pose. */
    if (id==3) rb_ui_update();
}
static lv_obj_t *button(lv_obj_t *parent,int x,int y,int w,int h,const char *label,intptr_t id,uint32_t color)
{
    lv_obj_t *o=box(parent,x,y,w,h,color,10); lv_obj_add_flag(o,LV_OBJ_FLAG_CLICKABLE);
    lv_obj_t *l=text(o,0,0,label); lv_obj_center(l);
    lv_obj_add_event_cb(o,button_event,LV_EVENT_ALL,(void *)id); return l;
}
void rb_ui_create(rb_pet_t *engine)
{
    pet=engine; settings_open=false; body_frame=UINT32_MAX; body_updates=0;
    last_body_ms=feedback_ms=0; feedback_pending=false; anchor=&rb_frames[0];
    lv_obj_t *root=lv_screen_active(); lv_obj_clean(root); lv_obj_remove_style_all(root);
    lv_obj_set_style_bg_color(root,lv_color_hex(0xf8f7ef),0); lv_obj_set_style_bg_opa(root,LV_OPA_COVER,0);
    lv_obj_remove_flag(root,LV_OBJ_FLAG_SCROLLABLE);
    box(root,0,201,320,39,0xeeeddf,0);
    box(root,107,195,106,6,0xdedecb,LV_RADIUS_CIRCLE);
    body=lv_image_create(root); lv_obj_set_pos(body,80,40); lv_obj_remove_flag(body,LV_OBJ_FLAG_CLICKABLE);
    placeholder=text(root,146,109,"...");
    for (unsigned i=0;i<2;++i) {
        eye_group[i]=box(root,0,0,20,28,0,0); lv_obj_set_style_bg_opa(eye_group[i],LV_OPA_TRANSP,0);
        lv_obj_set_style_transform_pivot_x(eye_group[i],10,0); lv_obj_set_style_transform_pivot_y(eye_group[i],14,0);
        eyes[i]=box(eye_group[i],3,4,14,20,0xeaffed,LV_RADIUS_CIRCLE);
        pupils[i]=box(eyes[i],5,6,5,10,0x294439,LV_RADIUS_CIRCLE);
        happy_eyes[i]=lv_arc_create(eye_group[i]); lv_obj_remove_style_all(happy_eyes[i]);
        lv_obj_set_pos(happy_eyes[i],1,6); lv_obj_set_size(happy_eyes[i],18,18);
        lv_arc_set_bg_angles(happy_eyes[i],190,350);
        lv_obj_set_style_arc_width(happy_eyes[i],3,LV_PART_MAIN);
        lv_obj_set_style_arc_color(happy_eyes[i],lv_color_hex(0xeaffed),LV_PART_MAIN);
        lv_obj_set_style_arc_rounded(happy_eyes[i],true,LV_PART_MAIN);
        lv_obj_remove_flag(happy_eyes[i],LV_OBJ_FLAG_CLICKABLE);
        blush[i]=box(eye_group[i],4,24,10,2,0xf9afa0,2);
    }
    emote=text(root,250,75,"");
    lv_obj_t *hit=box(root,0,0,320,240,0,0); lv_obj_set_style_bg_opa(hit,LV_OPA_TRANSP,0);
    lv_obj_add_flag(hit,LV_OBJ_FLAG_CLICKABLE); lv_obj_add_event_cb(hit,touch,LV_EVENT_ALL,NULL);
    settings=box(root,10,45,133,153,0xfffef7,14); lv_obj_add_flag(settings,LV_OBJ_FLAG_CLICKABLE);
    style_label=button(settings,8,9,117,38,"LIVELY",1,0xe8edda);
    rest_label=button(settings,8,57,117,38,"REST",2,0xeeeee3);
    button(settings,8,105,117,38,"CLOSE",4,0xf3f2e9);
    stop_label=button(root,8,7,60,28,"STOP",3,0xeee7dd);
    button(root,276,7,36,28,LV_SYMBOL_SETTINGS,0,0xeeeee3);
    rb_ui_update();
}
void rb_ui_update(void)
{
    if (!pet) return;
    bool stopped=pet->state==RB_PET_STOPPED;
    if (body_frame==UINT32_MAX || (!stopped && pet->now-last_body_ms>=(1000u+RB_BODY_FPS-1)/RB_BODY_FPS)) {
        unsigned origin=pet->action==RB_ANIM_IDLE?pet->pose:pet->origin;
        unsigned clip_index=rb_clip_map[origin][pet->action];
        if (origin==RB_POSE_IDLE && pet->active_direction<0 && rb_clip_reverse[pet->action])
            clip_index=rb_clip_reverse[pet->action];
        const rb_clip_meta_t *clip=&rb_clips[clip_index];
        unsigned n=0,phase=(unsigned)(fmaxf(0,fminf(1,pet->phase))*1000);
        while (n+1<clip->count && clip->at[n+1]<=phase) ++n;
        unsigned frame=clip->frames[n];
        if (frame!=body_frame) {
            const lv_image_dsc_t *image=rb_assets_frame(frame,&anchor);
            hidden(body,!image); hidden(placeholder,image!=NULL);
            if (image) { lv_image_set_src(body,image); lv_obj_invalidate(body); ++body_updates; }
            body_frame=frame;
        }
        last_body_ms+=(pet->now-last_body_ms)/((1000u+RB_BODY_FPS-1)/RB_BODY_FPS)*((1000u+RB_BODY_FPS-1)/RB_BODY_FPS);
    }
    label_text(style_label,pet->lazy?"LAZY":"LIVELY");
    label_text(rest_label,pet->state==RB_PET_RESTING?"WAKE":"REST");
    label_text(stop_label,stopped?"RESET":"STOP"); hidden(settings,!settings_open);
    bool happy=pet->action==RB_ANIM_HAPPY || pet->action==RB_ANIM_NUZZLE;
    bool folded=pet->state==RB_PET_FOLDED;
    float open=1;
    if (pet->state==RB_PET_RESTING) open=.1f;
    if (pet->action==RB_ANIM_REST) open=1-.9f*pet->phase;
    if (pet->action==RB_ANIM_WAKE) open=.1f+.9f*pet->phase;
    if (folded && pet->action!=RB_ANIM_RECOVER) open=.65f;
    int h=stopped?15:(int)(20*open);
    if (pet->blink && !happy && !stopped) h=2;
    if (h<2) h=2;
    for (unsigned i=0;i<2;++i) {
        hidden(eye_group[i],lv_obj_has_flag(body,LV_OBJ_FLAG_HIDDEN));
        lv_obj_set_pos(eye_group[i],80+anchor->eyes[i][0]-10,40+anchor->eyes[i][1]-14);
        if (lv_obj_get_style_transform_rotation(eye_group[i],0)!=anchor->angle)
            lv_obj_set_style_transform_rotation(eye_group[i],anchor->angle,0);
        hidden(eyes[i],happy && !stopped); hidden(happy_eyes[i],!happy || stopped);
        hidden(blush[i],!happy || stopped);
        int height=(pet->action==RB_ANIM_TILT && i==1)?12:h;
        lv_obj_set_size(eyes[i],14,height); lv_obj_set_pos(eyes[i],3,4+(20-height)/2);
        hidden(pupils[i],height<14);
        lv_obj_set_pos(pupils[i],5+(int)(pet->gaze_x*3),height/2-5+(int)(pet->gaze_y*2));
        lv_color_t color=lv_color_hex(stopped?0xffc2ad:0xeaffed);
        if (!lv_color_eq(lv_obj_get_style_bg_color(eyes[i],0),color)) lv_obj_set_style_bg_color(eyes[i],color,0);
    }
    label_text(emote,stopped?"HOLD\nRESET":pet->state==RB_PET_RESTING?"z z":pet->action==RB_ANIM_TILT?"?":"");
    if (feedback_pending && pet->contact) { feedback_ms=pet->now-raw_down_at; feedback_pending=false; }
}
unsigned rb_ui_body_updates(void) { return body_updates; }
uint64_t rb_ui_last_feedback_ms(void) { return feedback_ms; }
