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
int main(void)
{
    lv_init();
    FILE *f=fopen(RB_ASSET_FILE,"rb"); assert(f); fseek(f,0,SEEK_END); long bytes=ftell(f); rewind(f);
    uint8_t *blob=malloc((size_t)bytes); assert(blob); assert(fread(blob,1,(size_t)bytes,f)==(size_t)bytes); fclose(f);
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
    point=(lv_point_t){130,110}; pressed=true; pump(100);
    for (unsigned n=0;n<190;++n) { unsigned t=n%60; point.x=130+(t<=30?t:60-t)*2; pump(20); }
    assert(pet.happy_latched); snap("happy.ppm"); pressed=false; pump(2500);
    click(294,20); pump(100); click(70,72); pump(100); assert(pet.lazy); snap("settings.ppm");
    click(70,168); pump(100); snap("lazy.ppm");
    click(294,20); pump(100); click(70,120); pump(4500);
    assert(pet.state==RB_PET_RESTING); snap("sleep.ppm");
    click(20,120); assert(pet.state==RB_PET_RESTING);
    click(145,110); pump(100); assert(pet.action==RB_ANIM_WAKE);
    unsigned before_stop=rb_ui_body_updates(); click(38,20); pump(100);
    assert(pet.state==RB_PET_STOPPED); unsigned stopped_updates=rb_ui_body_updates();
    assert(stopped_updates>=before_stop); pump(2000); assert(rb_ui_body_updates()==stopped_updates); snap("stopped.ppm");
    click(38,20); assert(pet.state==RB_PET_STOPPED);
    rb_pet_estop(&pet,false); point=(lv_point_t){38,20}; pressed=true; pump(800); pressed=false; pump(100);
    assert(pet.state==RB_PET_STOPPED); rb_pet_estop(&pet,true);
    point=(lv_point_t){38,20}; pressed=true; pump(800); pressed=false; pump(2500);
    assert(pet.state==RB_PET_NORMAL);
    point=(lv_point_t){145,110}; pressed=true; pump(100); assert(pet.contact);
    lv_indev_reset(indev,NULL); assert(!pet.contact); pressed=false; pump(500);
    lv_mem_monitor_t before,after; lv_mem_monitor(&before);
    for (unsigned n=0;n<80;++n) { click(145,110); pump(1300); }
    pump(3000); lv_mem_monitor(&after); assert(after.free_size==before.free_size);
    printf("PASS reference UI, RGB565 sequence/cache fallback, anchors, gestures, STOP freeze and stable heap; cache=%zu bytes\n",rb_assets_cache_bytes());
    rb_assets_close(); free(blob); lv_deinit(); return 0;
}
