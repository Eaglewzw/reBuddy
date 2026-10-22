#include "pet_ui.h"
#include "pet_assets.h"
#include <stdlib.h>
#include "lvgl.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <inttypes.h>
static uint16_t buffer[320*40], buffer2[320*40];
static uint16_t pixels[320*240];
static rb_pet_t pet;
static uint64_t now=1000;
static lv_point_t point;
static bool pressed;
static uint64_t flushed_pixels;
static unsigned flushed_frames;
uint64_t rb_platform_now_ms(void) { return now; }
static void flush(lv_display_t *d,const lv_area_t *area,uint8_t *data)
{
    flushed_pixels+=(uint64_t)lv_area_get_width(area)*lv_area_get_height(area);
    if (lv_display_flush_is_last(d)) ++flushed_frames;
    uint16_t *src=(uint16_t *)data;
    for (int y=area->y1;y<=area->y2;++y) for (int x=area->x1;x<=area->x2;++x) {
        assert(x>=0 && x<320 && y>=0 && y<240); pixels[y*320+x]=*src++;
    }
    lv_display_flush_ready(d);
}
static void read_touch(lv_indev_t *indev,lv_indev_data_t *data)
{ (void)indev; data->point=point; data->state=pressed?LV_INDEV_STATE_PRESSED:LV_INDEV_STATE_RELEASED; }
static void pump(unsigned ms)
{
    for (unsigned t=0;t<ms;t+=20) {
        now+=20; lv_tick_inc(20); rb_pet_tick(&pet,now); rb_ui_update(); lv_timer_handler();
    }
}
static void click(int x,int y)
{ point=(lv_point_t){x,y}; pressed=true; pump(100); pressed=false; pump(100); }
static lv_obj_t *find_label(lv_obj_t *parent,const char *value)
{
    if (lv_obj_check_type(parent,&lv_label_class) && !strcmp(lv_label_get_text(parent),value)) return parent;
    for (unsigned i=0;i<lv_obj_get_child_count(parent);++i) {
        lv_obj_t *found=find_label(lv_obj_get_child(parent,i),value);
        if (found) return found;
    }
    return NULL;
}
static unsigned visible_questions(lv_obj_t *o)
{
    if (!lv_obj_is_visible(o)) return 0;
    unsigned count=lv_obj_check_type(o,&lv_label_class) && !strcmp(lv_label_get_text(o),"?");
    for (unsigned i=0;i<lv_obj_get_child_count(o);++i)
        count+=visible_questions(lv_obj_get_child(o,i));
    return count;
}
static void bounds(lv_obj_t *o)
{
    if (lv_obj_has_flag(o,LV_OBJ_FLAG_HIDDEN)) return;
    lv_area_t a; lv_obj_get_coords(o,&a);
    if (a.x1<0 || a.y1<0 || a.x2>=320 || a.y2>=240) {
        fprintf(stderr,"bounds: %ld,%ld,%ld,%ld\n",(long)a.x1,(long)a.y1,(long)a.x2,(long)a.y2); assert(0);
    }
    for (unsigned n=0;n<lv_obj_get_child_count(o);++n) bounds(lv_obj_get_child(o,n));
}
static void snap(const char *name)
{
    lv_obj_update_layout(lv_screen_active()); lv_refr_now(NULL); bounds(lv_screen_active());
    FILE *f=fopen(name,"wb"); assert(f); fprintf(f,"P6\n320 240\n255\n");
    for (unsigned n=0;n<320*240;++n) {
        uint16_t p=pixels[n]; unsigned char rgb[]={((p>>11)&31)*255/31,((p>>5)&63)*255/63,(p&31)*255/31};
        assert(fwrite(rgb,1,3,f)==3);
    }
    fclose(f);
}
static uint32_t region_hash(int x0,int y0,int x1,int y1)
{
    uint32_t hash=2166136261u;
    lv_refr_now(NULL);
    for (int y=y0;y<y1;++y) for (int x=x0;x<x1;++x)
        hash=(hash^pixels[y*320+x])*16777619u;
    return hash;
}
int main(void)
{
    lv_init();
    FILE *f=fopen(RB_ASSET_FILE,"rb"); assert(f); fseek(f,0,SEEK_END); long bytes=ftell(f); rewind(f);
    uint8_t *blob=malloc((size_t)bytes); assert(blob); assert(fread(blob,1,(size_t)bytes,f)==(size_t)bytes); fclose(f);
    f=fopen(RB_ROOM_FILE,"rb"); assert(f);
    uint8_t *room=malloc(153612); assert(room); assert(fread(room,1,153612,f)==153612); fclose(f);
    assert(!rb_assets_set_room(room,10)); assert(rb_assets_room()==NULL);
    assert(rb_assets_set_room(room,153612));
    const rb_frame_meta_t *meta=NULL;
    assert(!rb_assets_init(blob,10,2)); assert(rb_assets_frame(0,&meta)==NULL);
    assert(rb_assets_init(blob,(size_t)bytes,0)); assert(rb_assets_degraded());
    assert(rb_assets_frame(3,&meta) && meta==&rb_frames[0]);
    assert(rb_assets_init(blob,(size_t)bytes,2)); assert(!rb_assets_degraded());
    assert(rb_assets_frame(RB_FRAME_COUNT+1,&meta) && meta==&rb_frames[0]);
    assert(rb_assets_init(blob,(size_t)bytes,2));
    lv_display_t *d=lv_display_create(320,240); assert(d);
    lv_display_set_color_format(d,LV_COLOR_FORMAT_RGB565);
    lv_display_set_buffers(d,buffer,buffer2,sizeof(buffer),LV_DISPLAY_RENDER_MODE_PARTIAL); lv_display_set_flush_cb(d,flush);
    lv_indev_t *indev=lv_indev_create(); lv_indev_set_type(indev,LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(indev,read_touch); lv_indev_set_display(indev,d);
    rb_pet_init(&pet,now); rb_ui_create(&pet); pump(200); snap("idle.ppm");
    flushed_pixels=0;
    for (unsigned n=0;n<100;++n) { rb_ui_update(); lv_refr_now(d); }
    assert(flushed_pixels==0);
    /* One held press must never both pause and restore. */
    point=(lv_point_t){28,212}; pressed=true; pump(1000);
    assert(pet.state==RB_PET_STOPPED);
    pressed=false; pump(100);
    click(28,212); pump((unsigned)(RB_MS_WAKE*RB_PLAYBACK_DURATION_SCALE*RB_STYLE_LAZY)+400); assert(pet.state==RB_PET_NORMAL);
    lv_obj_t *rest=find_label(lv_screen_active(),"休息"); assert(rest);
    lv_obj_t *menu=lv_obj_get_parent(lv_obj_get_parent(rest));
    /* Repeated taps, paused-state access and outside dismissal never leak
       into character gestures. The menu opens above its lower-right entry. */
    for (unsigned i=0;i<12;++i) {
        click(28,212); assert(pet.state==RB_PET_STOPPED);
        click(290,212); assert(!lv_obj_has_flag(menu,LV_OBJ_FLAG_HIDDEN));
        lv_area_t a; lv_obj_get_coords(menu,&a); assert(a.x1==169 && a.y2==183);
        bool lazy=pet.lazy; unsigned requests=pet.requests;
        click(240,110); click(240,156);
        assert(pet.lazy==lazy && pet.requests==requests);
        click(290,212); assert(lv_obj_has_flag(menu,LV_OBJ_FLAG_HIDDEN));
        click(28,212); assert(pet.state!=RB_PET_STOPPED);
        click(290,212); assert(!lv_obj_has_flag(menu,LV_OBJ_FLAG_HIDDEN));
        requests=pet.requests; click(145,110);
        assert(lv_obj_has_flag(menu,LV_OBJ_FLAG_HIDDEN) && !pet.contact && pet.requests==requests);
        pump((unsigned)(RB_MS_WAKE*RB_PLAYBACK_DURATION_SCALE*RB_STYLE_LAZY)+400);
    }
    /* A hardware stop arriving during screen pause requires a fresh hold,
       even if a resume tap was already down when the stop arrived. */
    click(28,212); point=(lv_point_t){28,212}; pressed=true; pump(100);
    rb_pet_estop(&pet,false); pump(100);
    rb_pet_estop(&pet,true); pressed=false; pump(100);
    assert(pet.state==RB_PET_STOPPED && !strcmp(pet.stop_reason,"E-STOP"));
    click(28,212); assert(pet.state==RB_PET_STOPPED);
    point=(lv_point_t){28,212}; pressed=true; pump(800); pressed=false; pump((unsigned)(RB_MS_WAKE*RB_PLAYBACK_DURATION_SCALE*RB_STYLE_LAZY)+400);
    assert(pet.state==RB_PET_NORMAL);
    /* Finger drift just beyond the visual edge is tolerated; a drag away
       or an input reset does not click. A long hold toggles only once. */
    point=(lv_point_t){310,212}; pressed=true; pump(80);
    point=(lv_point_t){313,214}; pump(80); pressed=false; pump(100);
    assert(!lv_obj_has_flag(menu,LV_OBJ_FLAG_HIDDEN)); click(290,212);
    point=(lv_point_t){290,212}; pressed=true; pump(80);
    point=(lv_point_t){240,212}; pump(80); pressed=false; pump(100);
    assert(lv_obj_has_flag(menu,LV_OBJ_FLAG_HIDDEN));
    point=(lv_point_t){290,212}; pressed=true; pump(80);
    lv_indev_reset(indev,NULL); pressed=false; pump(100);
    assert(lv_obj_has_flag(menu,LV_OBJ_FLAG_HIDDEN));
    point=(lv_point_t){290,212}; pressed=true; pump(1200);
    assert(lv_obj_has_flag(menu,LV_OBJ_FLAG_HIDDEN)); pressed=false; pump(100);
    assert(!lv_obj_has_flag(menu,LV_OBJ_FLAG_HIDDEN)); click(290,212);
    /* Dragging off the character toward settings must not become a click. */
    point=(lv_point_t){145,110}; pressed=true; pump(100);
    point=(lv_point_t){290,212}; pump(100); pressed=false; pump(100);
    assert(lv_obj_has_flag(menu,LV_OBJ_FLAG_HIDDEN) && !pet.contact);
    point=(lv_point_t){145,110}; pressed=true; pump(3000);
    assert(pet.contact && pet.action==RB_ANIM_IDLE && pet.happy_ms==0);
    pressed=false; pump(100); click(145,110); pump(300);
    assert(pet.action==RB_ANIM_PROBE); snap("curious.ppm"); pump(1200);
    click(145,110); click(145,110); click(145,110); pump(1600);
    assert(pet.state==RB_PET_FOLDED); snap("folded.ppm");
    point=(lv_point_t){130,110}; pressed=true; pump(100);
    for (unsigned n=0;n<100;++n) { unsigned t=n%60; point.x=130+(t<=30?t:60-t)*2; pump(20); }
    pump(2000); assert(pet.state==RB_PET_NORMAL); snap("relaxed.ppm");
    pressed=false; pump(100);
    /* Same moving contact on body cannot build happiness. */
    point=(lv_point_t){130,140}; pressed=true; pump(100);
    for (unsigned n=0;n<250;++n) { unsigned t=n%60; point.x=130+(t<=30?t:60-t)*2; pump(20); }
    assert(pet.happy_ms==0 && !pet.happy_latched); pressed=false; pump((unsigned)(RB_MS_WAKE*RB_PLAYBACK_DURATION_SCALE*RB_STYLE_LAZY)+400);
    /* Rubbing the face/eyes below the forehead is not head petting. */
    point=(lv_point_t){170,100}; pressed=true; pump(100);
    for (unsigned n=0;n<250;++n) { unsigned t=n%30; point.x=170+(t<=15?t:30-t)*2; pump(20); }
    assert(pet.happy_ms==0 && !pet.happy_latched); pressed=false; pump((unsigned)(RB_MS_WAKE*RB_PLAYBACK_DURATION_SCALE*RB_STYLE_LAZY)+400);
    /* Forehead above the eyes, in real rendered idle/nuzzle poses. */
    point=(lv_point_t){165,70}; pressed=true; pump(100);
    for (unsigned n=0;n<230;++n) { unsigned t=n%30; point.x=165+(t<=15?t:30-t)*2; pump(20); }
    assert(pet.happy_latched); snap("happy.ppm"); pressed=false; pump((unsigned)(RB_MS_WAKE*RB_PLAYBACK_DURATION_SCALE*RB_STYLE_LAZY)+400);
    click(290,212); pump(100); click(240,110); pump(100); assert(pet.lazy); snap("settings.ppm");
    click(290,212); pump(100); snap("lazy.ppm");
    click(290,212); pump(100); click(240,156); pump((unsigned)(RB_MS_REST*RB_PLAYBACK_DURATION_SCALE*RB_STYLE_LAZY)+500);
    assert(pet.state==RB_PET_RESTING); snap("sleep.ppm");
    lv_obj_t *sleep_hint=find_label(lv_screen_active(),"长按左下角恢复");
    assert(sleep_hint && !lv_obj_has_flag(sleep_hint,LV_OBJ_FLAG_HIDDEN));
    unsigned sleep_requests=pet.requests;
    click(145,110); click(145,110); click(145,110);
    click(290,212); assert(!lv_obj_has_flag(menu,LV_OBJ_FLAG_HIDDEN));
    click(240,156); assert(pet.state==RB_PET_RESTING && pet.requests==sleep_requests);
    click(290,212); assert(lv_obj_has_flag(menu,LV_OBJ_FLAG_HIDDEN));
    click(28,212);
    point=(lv_point_t){145,110}; pressed=true; pump(2000); pressed=false; pump(100);
    click(28,212); click(28,212);
    point=(lv_point_t){28,212}; pressed=true; pump(100);
    lv_indev_reset(indev,NULL); pressed=false; pump(100);
    point=(lv_point_t){28,212}; pressed=true; pump(100);
    point=(lv_point_t){90,212}; pump(1000); pressed=false; pump(100);
    pump(120000);
    assert(pet.state==RB_PET_RESTING && pet.action==RB_ANIM_IDLE && pet.requests==sleep_requests);
    assert(lv_obj_has_flag(menu,LV_OBJ_FLAG_HIDDEN));
    uint32_t base_hash=region_hash(80,179,240,200);
    uint32_t head_hash=region_hash(80,40,225,179), z_hash=region_hash(225,40,280,145);
    unsigned head_changes=0,z_changes=0;
    lv_mem_monitor_t sleep_before,sleep_after;
    for (unsigned n=0;n<16;++n) {
        pump(400); bounds(lv_screen_active());
        assert(region_hash(80,179,240,200)==base_hash);
        if (region_hash(80,40,225,179)!=head_hash) ++head_changes;
        if (region_hash(225,40,280,145)!=z_hash) ++z_changes;
        char name[32]; snprintf(name,sizeof(name),"sleep-detail-%02u.ppm",n); snap(name);
    }
    assert(head_changes>3 && z_changes>10);
    /* Scale transforms lazily populate LVGL caches. Warm a complete lazy
       cycle, then compare identical phases over four more full cycles. */
    pump((unsigned)(5460*RB_PLAYBACK_DURATION_SCALE)); lv_mem_monitor(&sleep_before);
    pump((unsigned)(4*5460*RB_PLAYBACK_DURATION_SCALE)); lv_mem_monitor(&sleep_after);
    assert(sleep_before.free_size==sleep_after.free_size);
    /* STOP during sleep freezes the particles as well as the body. */
    rb_pet_stop(&pet,"BACK BUTTON"); pump(100); assert(pet.state==RB_PET_STOPPED);
    uint32_t frozen=region_hash(0,0,320,240); pump(1000);
    assert(region_hash(0,0,320,240)==frozen);
    point=(lv_point_t){38,212}; pressed=true; pump(800); pressed=false; pump((unsigned)(RB_MS_WAKE*RB_PLAYBACK_DURATION_SCALE*RB_STYLE_LAZY)+400);
    assert(pet.state==RB_PET_NORMAL);
    click(290,212); pump(100); click(240,156); pump((unsigned)(RB_MS_REST*RB_PLAYBACK_DURATION_SCALE*RB_STYLE_LAZY)+500);
    assert(pet.state==RB_PET_RESTING);
    click(20,120); assert(pet.state==RB_PET_RESTING);
    click(145,110); pump(100); assert(pet.state==RB_PET_RESTING && pet.action==RB_ANIM_IDLE);
    point=(lv_point_t){28,212}; pressed=true; pump(800); pressed=false; pump(100);
    assert(pet.action==RB_ANIM_WAKE);
    assert(lv_obj_has_flag(sleep_hint,LV_OBJ_FLAG_HIDDEN));
    lv_refr_now(NULL);
    for (int y=45;y<135;++y) for (int x=242;x<278;++x)
        assert(pixels[y*320+x]==(uint16_t)(room[12+(y*320+x)*2]|(room[13+(y*320+x)*2]<<8))); /* Room is restored after Zs disappear. */
    unsigned before_stop=rb_ui_body_updates(); click(38,212); pump(100);
    assert(pet.state==RB_PET_STOPPED); unsigned stopped_updates=rb_ui_body_updates();
    assert(stopped_updates>=before_stop); pump(2000); assert(rb_ui_body_updates()==stopped_updates); snap("stopped.ppm");
    click(38,212); pump((unsigned)(RB_MS_WAKE*RB_PLAYBACK_DURATION_SCALE*RB_STYLE_LAZY)+400); assert(pet.state==RB_PET_NORMAL);
    rb_pet_stop(&pet,"BACK BUTTON"); pump(100);
    click(38,212); assert(pet.state==RB_PET_STOPPED);
    rb_pet_estop(&pet,false); point=(lv_point_t){38,212}; pressed=true; pump(800); pressed=false; pump(100);
    assert(pet.state==RB_PET_STOPPED); rb_pet_estop(&pet,true);
    point=(lv_point_t){38,212}; pressed=true; pump(800); pressed=false; pump((unsigned)(RB_MS_WAKE*RB_PLAYBACK_DURATION_SCALE*RB_STYLE_LAZY)+400);
    assert(pet.state==RB_PET_NORMAL);
    point=(lv_point_t){145,110}; pressed=true; pump(100); assert(pet.contact);
    lv_indev_reset(indev,NULL); assert(!pet.contact); pressed=false; pump(500);
    assert(pet.state!=RB_PET_STOPPED);
    /* The driver's I/O-error cancellation path must leave animation alive
       and accept a fresh contact after release, without a restore button. */
    click(145,110); pump(200); assert(pet.action==RB_ANIM_PROBE);
    unsigned prior_updates=rb_ui_body_updates(); pump(1500);
    assert(pet.state!=RB_PET_STOPPED && rb_ui_body_updates()>prior_updates);
    lv_mem_monitor_t before,after; lv_mem_monitor(&before);
    for (unsigned n=0;n<80;++n) { click(145,110); pump((unsigned)(RB_MS_PROBE*RB_PLAYBACK_DURATION_SCALE*RB_STYLE_LAZY)+100); }
    pump(3000); lv_mem_monitor(&after); assert(after.free_size==before.free_size);
    /* A real fast swipe produces three overhead questions, then clears them.
       Settings and pause hide the effect without leaving stale glyphs. */
    rb_pet_init(&pet,now); rb_ui_create(&pet); pump(100);
    assert(visible_questions(lv_screen_active())==0);
    point=(lv_point_t){100,110}; pressed=true; pump(80);
    for (int x=108;x<=220;x+=8) { point.x=x; pump(20); }
    pressed=false; pump(pet.duration/2); assert(pet.action==RB_ANIM_TILT);
    assert(visible_questions(lv_screen_active())==3); snap("confused.ppm");
    click(290,212); assert(visible_questions(lv_screen_active())==0);
    click(290,212); pump(1600);
    assert(pet.action==RB_ANIM_IDLE && visible_questions(lv_screen_active())==0);
    point=(lv_point_t){100,110}; pressed=true; pump(80);
    for (int x=108;x<=220;x+=8) { point.x=x; pump(20); }
    pressed=false; pump(pet.duration/2); assert(visible_questions(lv_screen_active())==3);
    click(28,212); assert(pet.state==RB_PET_STOPPED);
    assert(visible_questions(lv_screen_active())==0);
    printf("PASS reference UI, RGB565 sequence/cache fallback, anchors, gestures, STOP freeze and stable heap; cache=%zu bytes\n",rb_assets_cache_bytes());
    rb_assets_close(); lv_deinit(); free(blob); free(room); return 0;
}
