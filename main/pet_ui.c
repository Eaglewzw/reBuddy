#include "pet_ui.h"
#include "character_asset.h"
#include "lvgl.h"
#include <math.h>
#include <stdint.h>

static rb_engine_t *pet;
static rb_gesture_t *gestures;
static lv_obj_t *root, *actor, *eyes[2], *happy_eyes[2], *pupils[2], *glints[2], *brows[2], *blush[2][3];
static lv_obj_t *status, *hint, *style_label, *stop_label, *emotion, *style_button;
static uint32_t frame;
static lv_obj_t *box(lv_obj_t *parent,int x,int y,int w,int h,uint32_t color,int radius)
{
    lv_obj_t *o=lv_obj_create(parent);
    lv_obj_remove_style_all(o); lv_obj_set_pos(o,x,y); lv_obj_set_size(o,w,h);
    lv_obj_set_style_bg_color(o,lv_color_hex(color),0); lv_obj_set_style_bg_opa(o,LV_OPA_COVER,0);
    lv_obj_set_style_radius(o,radius,0); lv_obj_remove_flag(o,LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_remove_flag(o,LV_OBJ_FLAG_CLICKABLE); return o;
}
static lv_obj_t *text(lv_obj_t *parent,int x,int y,const char *value,uint32_t color)
{
    lv_obj_t *o=lv_label_create(parent); lv_label_set_text(o,value); lv_obj_set_pos(o,x,y);
    lv_obj_set_style_text_color(o,lv_color_hex(color),0); return o;
}
static void small(lv_obj_t *o) { lv_obj_set_style_text_font(o,&lv_font_montserrat_12,0); }
static void touch(lv_event_t *event)
{
    lv_event_code_t code=lv_event_get_code(event);
    rb_touch_phase_t phase;
    if (code==LV_EVENT_PRESSED) phase=RB_TOUCH_DOWN;
    else if (code==LV_EVENT_PRESSING) phase=RB_TOUCH_MOVE;
    else if (code==LV_EVENT_RELEASED) phase=RB_TOUCH_UP;
    else if (code==LV_EVENT_PRESS_LOST) phase=RB_TOUCH_CANCEL;
    else return;
    lv_indev_t *indev=lv_indev_active(); if (!indev) return;
    lv_point_t point; lv_indev_get_point(indev,&point);
    rb_gesture_touch(gestures,pet,phase,point.x,point.y,rb_platform_now_ms()); rb_ui_update();
}
static void button_event(lv_event_t *event)
{
    intptr_t id=(intptr_t)lv_event_get_user_data(event);
    lv_event_code_t code=lv_event_get_code(event);
    if (code!=LV_EVENT_PRESSED && code!=LV_EVENT_CLICKED && code!=LV_EVENT_LONG_PRESSED) return;
    uint64_t now=rb_platform_now_ms();
    if (id==3) {
        if (pet->state==RB_STOPPED) {
            if (code==LV_EVENT_LONG_PRESSED) { rb_gesture_init(gestures); (void)rb_engine_reset(pet,now); }
        } else if (code==LV_EVENT_PRESSED) {
            rb_gesture_init(gestures); rb_engine_stop(pet,"SCREEN STOP");
        }
    } else if (code==LV_EVENT_CLICKED && pet->state!=RB_STOPPED) {
        rb_gesture_init(gestures);
        if (id==0) (void)rb_engine_event(pet,RB_HAPPY,now);
        else if (id==1) rb_engine_style(pet,pet->style==RB_LIVELY?RB_LAZY:RB_LIVELY);
        else if (id==2) (void)rb_engine_event(pet,pet->state==RB_SLEEPING?RB_WAKE:RB_REST,now);
        pet->last_input_ms=now;
    }
    rb_ui_update();
}
void rb_ui_create(rb_engine_t *engine, rb_gesture_t *gesture)
{
    pet=engine; gestures=gesture; frame=0;
    root=lv_screen_active(); lv_obj_clean(root); lv_obj_remove_style_all(root);
    lv_obj_set_style_bg_color(root,lv_color_hex(0xfbf9f1),0); lv_obj_set_style_bg_opa(root,LV_OPA_COVER,0);
    lv_obj_remove_flag(root,LV_OBJ_FLAG_SCROLLABLE);
    text(root,12,9,"reBuddy",0x272b28); small(text(root,86,11,"/ RS",0x7e8373));
    lv_obj_t *preview=box(root,239,7,69,21,0xeceee0,10);
    lv_obj_t *pl=text(preview,0,0,"PREVIEW",0x737b63); small(pl); lv_obj_center(pl);
    box(root,12,32,296,1,0xe7e6db,0);
    /* Flash-resident generated reBot RS shell; expression layers remain live. */
    box(root,48,201,110,7,0xeae5d6,LV_RADIUS_CIRCLE);
    actor=box(root,25,30,155,183,0,0); lv_obj_set_style_bg_opa(actor,LV_OPA_TRANSP,0);
    lv_obj_t *image=lv_image_create(actor); lv_image_set_src(image,&rb_character_body); lv_obj_set_pos(image,4,4);
    lv_obj_remove_flag(image,LV_OBJ_FLAG_CLICKABLE);
    for (unsigned n=0;n<2;++n) {
        int x=98+(int)n*24, y=38+(int)n*6;
        eyes[n]=box(actor,x,y,13,23,0xdfffe8,LV_RADIUS_CIRCLE);
        lv_obj_set_style_shadow_color(eyes[n],lv_color_hex(0xbdffd5),0);
        lv_obj_set_style_shadow_width(eyes[n],5,0); lv_obj_set_style_shadow_opa(eyes[n],LV_OPA_30,0);
        pupils[n]=box(eyes[n],6,9,5,11,0x263f34,LV_RADIUS_CIRCLE);
        glints[n]=box(eyes[n],3,3,3,4,0xffffff,LV_RADIUS_CIRCLE);
        happy_eyes[n]=lv_arc_create(actor); lv_obj_remove_style_all(happy_eyes[n]);
        lv_obj_set_pos(happy_eyes[n],x-2,y+5); lv_obj_set_size(happy_eyes[n],18,18);
        lv_arc_set_bg_angles(happy_eyes[n],190,350);
        lv_obj_set_style_arc_width(happy_eyes[n],3,LV_PART_MAIN);
        lv_obj_set_style_arc_color(happy_eyes[n],lv_color_hex(0xdfffe8),LV_PART_MAIN);
        lv_obj_set_style_arc_rounded(happy_eyes[n],true,LV_PART_MAIN);
        lv_obj_remove_flag(happy_eyes[n],LV_OBJ_FLAG_CLICKABLE);
        lv_obj_add_flag(happy_eyes[n],LV_OBJ_FLAG_HIDDEN);
        brows[n]=box(actor,x+2,y-7,9,3,0xe6ffe9,3);
        for (unsigned b=0;b<3;++b) blush[n][b]=box(actor,x-4+(int)b*4,y+25,2,6,0xffb2a3,2);
    }
    status=text(root,208,40,"LIVELY",0x838a77); small(status);
    emotion=text(root,208,55,"HELLO",0x394334); small(emotion);
    hint=text(root,5,218,"Tap me. I'm here.",0x777766); small(hint);
    lv_obj_set_width(hint,190); lv_obj_set_style_text_align(hint,LV_TEXT_ALIGN_CENTER,0);
    lv_obj_t *hit=box(root,0,34,196,180,0,0);
    lv_obj_set_style_bg_opa(hit,LV_OPA_TRANSP,0); lv_obj_add_flag(hit,LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(hit,touch,LV_EVENT_ALL,NULL);
    const char *labels[]={"HAPPY","LIVELY","REST","STOP"};
    for (intptr_t n=0;n<4;++n) {
        lv_obj_t *button=box(root,204,74+(int)n*40,104,34,n==0?0xd5f21c:n==3?0xf6e5dd:0xeeeee3,12);
        lv_obj_add_flag(button,LV_OBJ_FLAG_CLICKABLE);
        lv_obj_t *label=text(button,0,0,labels[n],n==3?0x9f594b:0x3a4630);
        small(label); lv_obj_center(label); lv_obj_add_event_cb(button,button_event,LV_EVENT_ALL,(void *)n);
        if (n==1) { style_label=label; style_button=button; }
        if (n==3) stop_label=label;
    }
    rb_ui_update();
}
static void hidden(lv_obj_t *o,bool hide)
{ if (hide) lv_obj_add_flag(o,LV_OBJ_FLAG_HIDDEN); else lv_obj_remove_flag(o,LV_OBJ_FLAG_HIDDEN); }
void rb_ui_update(void)
{
    if (!pet) return;
    ++frame;
    lv_label_set_text(status,pet->style==RB_LIVELY?"LIVELY":"LAZY");
    lv_label_set_text(style_label,pet->style==RB_LIVELY?"LIVELY":"LAZY"); lv_obj_center(style_label);
    lv_obj_set_style_bg_color(style_button,lv_color_hex(pet->style==RB_LAZY?0xe4e6f1:0xeeeee3),0);
    lv_label_set_text(stop_label,pet->state==RB_STOPPED?"RESET":"STOP"); lv_obj_center(stop_label);
    const char *message="Tap me. I'm here.", *mood="HELLO";
    if (pet->action!=RB_NONE) {
        static const char *messages[]={"Oh! There you are.","Mmm... so nice.","You make me shy...","A little happy dance!","Hmm? What is it?","Five more minutes..."};
        static const char *moods[]={"CURIOUS","SO COMFY","A LITTLE SHY","HAPPY!","HUH?","SLEEPY"};
        message=messages[pet->action]; mood=moods[pet->action];
    } else if (pet->state==RB_DROWSY) { message="Getting sleepy..."; mood="SLEEPY"; }
    else if (pet->state==RB_SLEEPING) { message="Zzz... Touch to wake."; mood="Z Z Z"; }
    else if (pet->state==RB_STOPPED) { message="Hold RESET to clear."; mood="STOPPED"; }
    lv_label_set_text(hint,message); lv_label_set_text(emotion,mood);
    bool sleepy=pet->face==RB_FACE_DROWSY || pet->face==RB_FACE_SLEEP;
    bool happy=pet->face==RB_FACE_HAPPY || pet->face==RB_FACE_RELAXED;
    bool blink=pet->face==RB_FACE_IDLE && frame%190<5;
    int height=sleepy?3:happy?7:blink?2:23;
    for (unsigned n=0;n<2;++n) {
        int h=pet->face==RB_FACE_PUZZLED && n==1?13:height;
        int x=98+(int)n*24, y=38+(int)n*6+(23-h)/2;
        lv_obj_set_pos(eyes[n],x,y); lv_obj_set_size(eyes[n],13,h);
        hidden(eyes[n],happy); hidden(happy_eyes[n],!happy);
        hidden(pupils[n],h<16); hidden(glints[n],h<16);
        hidden(brows[n],sleepy || blink);
        for (unsigned b=0;b<3;++b) {
            lv_obj_set_style_bg_opa(blush[n][b],pet->face==RB_FACE_SHY?LV_OPA_COVER:LV_OPA_60,0);
        }
        lv_obj_set_style_bg_color(eyes[n],lv_color_hex(pet->state==RB_STOPPED?0xffc2ad:0xdfffe8),0);
    }
    int bob=(int)(sinf((float)(pet->now_ms%8000)*.002f)*(pet->style==RB_LIVELY?1.5f:.7f));
    if (pet->state==RB_SLEEPING || pet->state==RB_STOPPED) bob=0;
    lv_obj_set_pos(actor,25+(int)(pet->pose[0]*.6f),30+bob+(int)(pet->pose[4]*.3f));
}
