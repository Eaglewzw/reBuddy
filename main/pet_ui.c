#include "pet_ui.h"
#include "pet_assets.h"
#include "lvgl.h"
#include <math.h>
#include <stdint.h>
#include <string.h>

static rb_pet_t *pet;
static lv_obj_t *body;
static lv_obj_t *settings,*style_label,*rest_label,*stop_label,*placeholder;
static lv_obj_t *status_label,*contact_mark;
static lv_obj_t *settings_label,*wake_label;
static bool wake_armed;
extern const lv_font_t rb_room_font;
/* Idle and sleep use the same authored GIF timelines as other actions. */
static uint64_t animation_last_ms, idle_clock_ms;
static unsigned previous_clip;
static rb_pet_action_t previous_action;
static bool settings_open;
/* Decide on touch-down, so holding Pause cannot also restore that pause. */
static enum { RESTORE_NONE, RESTORE_CLICK, RESTORE_HOLD } restore_gesture;
static uint64_t feedback_ms,raw_down_at;
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
static bool resting(void)
{
    return pet->state!=RB_PET_STOPPED && pet->action!=RB_ANIM_WAKE &&
        ((pet->state==RB_PET_RESTING) || pet->action==RB_ANIM_REST ||
         (pet->pending_valid && pet->pending==RB_ANIM_REST));
}
static void touch(lv_event_t *event)
{
    lv_event_code_t code=lv_event_get_code(event);
    if (code==LV_EVENT_PRESS_LOST || code==LV_EVENT_INDEV_RESET) {
        rb_pet_input(pet,false,0,0,rb_platform_now_ms()); rb_pet_cancel(pet); feedback_pending=false; return;
    }
    if (settings_open) {
        if (code==LV_EVENT_CLICKED) { settings_open=false; rb_ui_update(); }
        return;
    }
    if (resting()) return;
    if (code!=LV_EVENT_PRESSED && code!=LV_EVENT_PRESSING && code!=LV_EVENT_RELEASED) return;
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
    if (code==LV_EVENT_PRESS_LOST || code==LV_EVENT_INDEV_RESET) {
        if (id==3) restore_gesture=RESTORE_NONE;
        if (id==5) wake_armed=false;
        return;
    }
    if (code!=LV_EVENT_PRESSED && code!=LV_EVENT_CLICKED && code!=LV_EVENT_LONG_PRESSED) return;
    /* Keep small finger drift captured, but dragging away must not activate. */
    lv_indev_t *indev=lv_indev_active();
    if (indev && code!=LV_EVENT_PRESSED) {
        lv_point_t p; lv_indev_get_point(indev,&p);
        lv_area_t a; lv_obj_get_coords(lv_event_get_current_target(event),&a);
        if (p.x<a.x1-4 || p.x>a.x2+4 || p.y<a.y1-4 || p.y>a.y2+4) {
            if (id==5) wake_armed=false;
            return;
        }
    }
    if (id==5) {
        if (code==LV_EVENT_PRESSED) wake_armed=resting();
        else if (code==LV_EVENT_LONG_PRESSED && wake_armed && resting()) {
            wake_armed=false; rb_pet_wake(pet);
            if (indev) lv_indev_wait_release(indev);
        } else if (code==LV_EVENT_CLICKED) wake_armed=false;
    } else if (id==3) {
        if (code==LV_EVENT_PRESSED) {
            restore_gesture=RESTORE_NONE;
            if (pet->state==RB_PET_STOPPED) {
                restore_gesture=pet->stop_reason && !strcmp(pet->stop_reason,"SCREEN STOP")?
                    RESTORE_CLICK:RESTORE_HOLD;
            } else {
                rb_pet_stop(pet,"SCREEN STOP"); settings_open=false; feedback_pending=false;
            }
        } else if (pet->state==RB_PET_STOPPED &&
                   ((code==LV_EVENT_CLICKED && restore_gesture==RESTORE_CLICK &&
                     pet->stop_reason && !strcmp(pet->stop_reason,"SCREEN STOP")) ||
                    (code==LV_EVENT_LONG_PRESSED && restore_gesture==RESTORE_HOLD))) {
            (void)rb_pet_reset(pet,rb_platform_now_ms());
            restore_gesture=RESTORE_NONE;
        }
    } else if (code==LV_EVENT_CLICKED) {
        rb_pet_cancel(pet); feedback_pending=false;
        if (id==0) settings_open=!settings_open;
        else if (pet->state!=RB_PET_STOPPED) {
            if (id==1) pet->lazy=!pet->lazy;
            if (id==2) { rb_pet_rest(pet); settings_open=false; }
        }
    }
    /* STOP has immediate overlay feedback, but never loads a different pose. */
    rb_ui_update();
}
static lv_obj_t *button(lv_obj_t *parent,int x,int y,int w,int h,const char *label,intptr_t id,uint32_t color)
{
    lv_obj_t *o=box(parent,x,y,w,h,color,10);
    lv_obj_add_flag(o,LV_OBJ_FLAG_CLICKABLE|LV_OBJ_FLAG_PRESS_LOCK);
    lv_obj_set_ext_click_area(o,4);
    lv_obj_set_style_bg_color(o,lv_color_hex(0xd4dfac),LV_STATE_PRESSED);
    lv_obj_set_style_bg_opa(o,LV_OPA_50,LV_STATE_DISABLED);
    lv_obj_t *l=text(o,0,0,label); lv_obj_center(l);
    lv_obj_add_event_cb(o,button_event,LV_EVENT_ALL,(void *)id); return l;
}
void rb_ui_create(rb_pet_t *engine)
{
    pet=engine; settings_open=false; restore_gesture=RESTORE_NONE; body_frame=UINT32_MAX; body_updates=0;
    wake_armed=false;
    feedback_ms=0; feedback_pending=false; anchor=&rb_frames[0];
    animation_last_ms=pet->now; idle_clock_ms=0; previous_clip=UINT32_MAX;
    previous_action=RB_ANIM_COUNT;
    lv_obj_t *root=lv_screen_active(); lv_obj_clean(root); lv_obj_remove_style_all(root);
    lv_obj_set_style_bg_color(root,lv_color_hex(0xf8f7ef),0); lv_obj_set_style_bg_opa(root,LV_OPA_COVER,0);
    lv_obj_remove_flag(root,LV_OBJ_FLAG_SCROLLABLE);
    const lv_image_dsc_t *room=rb_assets_room();
    if (room) {
        lv_obj_t *background=lv_image_create(root);
        lv_image_set_src(background,room); lv_obj_set_pos(background,0,0);
        lv_obj_remove_flag(background,LV_OBJ_FLAG_CLICKABLE);
    }
    status_label=text(root,0,0,"点击左下继续");
    lv_obj_set_style_text_font(status_label,&rb_room_font,0);
    lv_obj_set_style_bg_color(status_label,lv_color_hex(0xfffef7),0);
    lv_obj_set_style_bg_opa(status_label,LV_OPA_90,0);
    lv_obj_set_style_radius(status_label,8,0);
    lv_obj_set_style_pad_hor(status_label,6,0);
    lv_obj_set_style_pad_ver(status_label,3,0);
    lv_obj_align(status_label,LV_ALIGN_BOTTOM_MID,0,-8);
    hidden(status_label,true);
    contact_mark=box(root,0,0,8,8,0xcbe83c,LV_RADIUS_CIRCLE);
    hidden(contact_mark,true);
    body=lv_image_create(root); lv_obj_set_pos(body,RB_FRAME_X,RB_FRAME_Y); lv_obj_remove_flag(body,LV_OBJ_FLAG_CLICKABLE);
    placeholder=text(root,146,109,"...");
    lv_obj_move_foreground(status_label);
    lv_obj_move_foreground(contact_mark);
    lv_obj_t *hit=box(root,0,0,320,240,0,0); lv_obj_set_style_bg_opa(hit,LV_OPA_TRANSP,0);
    lv_obj_add_flag(hit,LV_OBJ_FLAG_CLICKABLE|LV_OBJ_FLAG_PRESS_LOCK); lv_obj_add_event_cb(hit,touch,LV_EVENT_ALL,NULL);
    settings=box(root,169,82,143,102,0xfffef7,14); lv_obj_add_flag(settings,LV_OBJ_FLAG_CLICKABLE);
    style_label=button(settings,8,8,127,40,"活泼",1,0xe8edda);
    rest_label=button(settings,8,54,127,40,"休息",2,0xeeeee3);
    lv_obj_set_style_text_font(style_label,&rb_room_font,0);
    lv_obj_set_style_text_font(rest_label,&rb_room_font,0);
    stop_label=button(root,8,192,44,40,LV_SYMBOL_PAUSE,3,0xeee7dd);
    settings_label=button(root,268,192,44,40,LV_SYMBOL_SETTINGS,0,0xeeeee3);
    wake_label=button(root,8,192,44,40,LV_SYMBOL_REFRESH,5,0xeee7dd);
    rb_ui_update();
}
void rb_ui_update(void)
{
    if (!pet) return;
    bool stopped=pet->state==RB_PET_STOPPED;
    bool rest=resting();
    if (rest) feedback_pending=false;
    uint64_t elapsed=pet->now>=animation_last_ms?pet->now-animation_last_ms:0;
    animation_last_ms=pet->now;
    /* Inspect the GIF clock at UI-timer cadence. A separate 125ms throttle
       would skip authored 120/130ms GIF boundaries and alter their pacing. */
    if (body_frame==UINT32_MAX || !stopped) {
        unsigned origin=pet->action==RB_ANIM_IDLE?pet->pose:pet->origin;
        unsigned clip_index=rb_clip_map[origin][pet->action];
        const rb_clip_meta_t *clip=&rb_clips[clip_index];
        if (clip_index!=previous_clip || pet->action!=previous_action) idle_clock_ms=0;
        else if (!stopped && pet->action==RB_ANIM_IDLE) idle_clock_ms+=elapsed;
        previous_clip=clip_index; previous_action=pet->action;
        unsigned phase_ms;
        if (pet->action==RB_ANIM_IDLE) {
            phase_ms=clip->duration_ms?(unsigned)(idle_clock_ms%clip->duration_ms):0;
        } else {
            phase_ms=(unsigned)(fmaxf(0,fminf(1,pet->phase))*clip->duration_ms);
        }
        unsigned n=0;
        while (n+1<clip->count && clip->at[n+1]<=phase_ms) ++n;
        unsigned frame=clip->frames[n];
        if (frame!=body_frame) {
            const lv_image_dsc_t *image=rb_assets_frame(frame,&anchor);
            hidden(body,!image); hidden(placeholder,image!=NULL);
            if (image) { lv_image_set_src(body,image); lv_obj_invalidate(body); ++body_updates; }
            int16_t claw[4][2]={{0}};
            if (image) for (unsigned i=0;i<4;++i) {
                claw[i][0]=RB_ROLE_X+anchor->claw_region[i][0];
                claw[i][1]=RB_ROLE_Y+anchor->claw_region[i][1];
            }
            rb_pet_set_claw_region(pet,claw);
            body_frame=frame;
        }
    }
    label_text(style_label,pet->lazy?"慵懒":"活泼");
    label_text(rest_label,"休息");
    bool screen_pause=stopped && pet->stop_reason && !strcmp(pet->stop_reason,"SCREEN STOP");
    label_text(stop_label,stopped?(screen_pause?LV_SYMBOL_PLAY:LV_SYMBOL_REFRESH):LV_SYMBOL_PAUSE);
    label_text(status_label,rest?"长按左下角恢复":!pet->estop_released?"请释放急停":screen_pause?"点击左下继续":"长按左下恢复");
    lv_obj_align(status_label,LV_ALIGN_BOTTOM_MID,0,-8);
    if (stopped) {
        lv_obj_add_state(lv_obj_get_parent(style_label),LV_STATE_DISABLED);
        lv_obj_add_state(lv_obj_get_parent(rest_label),LV_STATE_DISABLED);
    } else {
        lv_obj_remove_state(lv_obj_get_parent(style_label),LV_STATE_DISABLED);
        lv_obj_remove_state(lv_obj_get_parent(rest_label),LV_STATE_DISABLED);
    }
    if (rest) lv_obj_add_state(lv_obj_get_parent(rest_label),LV_STATE_DISABLED);
    hidden(settings,!settings_open);
    hidden(status_label,!stopped && !rest);
    hidden(lv_obj_get_parent(wake_label),!rest);
    hidden(lv_obj_get_parent(stop_label),rest);
    hidden(lv_obj_get_parent(settings_label),false);
    hidden(contact_mark,!pet->contact || stopped || settings_open || rest);
    if (pet->contact && !stopped && !settings_open)
        lv_obj_set_pos(contact_mark,pet->raw_x-4,pet->raw_y-4);
    if (feedback_pending && pet->contact) { feedback_ms=pet->now-raw_down_at; feedback_pending=false; }
}
unsigned rb_ui_body_updates(void) { return body_updates; }
uint64_t rb_ui_last_feedback_ms(void) { return feedback_ms; }
