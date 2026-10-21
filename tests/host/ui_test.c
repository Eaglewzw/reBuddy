#include "pet_ui.h"
#include "lvgl.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static uint16_t buffer[320*40];
static uint16_t pixels[320*240];
static rb_engine_t pet;
static rb_gesture_t gesture;
static uint64_t now=1000;
static lv_point_t point;
static bool pressed;
uint64_t rb_platform_now_ms(void) { return now; }
static void flush(lv_display_t *d,const lv_area_t *area,uint8_t *data)
{
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
        now+=20; lv_tick_inc(20); rb_gesture_tick(&gesture,&pet,now);
        rb_engine_tick(&pet,now); rb_ui_update(); lv_timer_handler();
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
    lv_init(); lv_display_t *d=lv_display_create(320,240); assert(d);
    lv_display_set_color_format(d,LV_COLOR_FORMAT_RGB565);
    lv_display_set_buffers(d,buffer,NULL,sizeof(buffer),LV_DISPLAY_RENDER_MODE_PARTIAL); lv_display_set_flush_cb(d,flush);
    lv_indev_t *indev=lv_indev_create(); lv_indev_set_type(indev,LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(indev,read_touch); lv_indev_set_display(indev,d);
    rb_config_t config=rb_config_default(); assert(rb_engine_init(&pet,&config,now)); rb_gesture_init(&gesture);
    rb_ui_create(&pet,&gesture); pump(200); snap("idle.ppm");
    click(160,100); pump(400); assert(pet.action==RB_LOOK); snap("curious.ppm");
    click(257,90); assert(pet.action==RB_DANCE); snap("happy.ppm");
    click(257,130); assert(pet.style==RB_LAZY); snap("lazy.ppm");
    click(257,170); pump(16000); assert(pet.state==RB_SLEEPING); snap("sleep.ppm");
    click(160,100); assert(pet.state==RB_WAKING);
    click(257,210); assert(pet.state==RB_STOPPED); snap("stopped.ppm");
    click(257,90); assert(pet.state==RB_STOPPED);
    /* Short click must not clear a latched stop. */
    click(257,210); assert(pet.state==RB_STOPPED);
    point=(lv_point_t){257,210}; pressed=true; pump(800); pressed=false; pump(100);
    assert(pet.state==RB_IDLE); assert(pet.action==RB_NONE);
    /* A board I2C read failure resets the pointer, not a successful release. */
    point=(lv_point_t){160,100}; pressed=true; pump(100); assert(gesture.down);
    lv_indev_reset(indev,NULL); assert(!gesture.down && !gesture.tap_pending);
    pressed=false; pump(500); assert(pet.action==RB_NONE);
    point=(lv_point_t){257,90}; pressed=true; pump(100);
    lv_indev_reset(indev,NULL); pressed=false; pump(500);
    assert(pet.action==RB_NONE); /* No phantom HAPPY click on a failed sample. */
    lv_mem_monitor_t before,after; lv_mem_monitor(&before);
    for (unsigned n=0;n<120;++n) { click(257,90); click(257,130); }
    lv_mem_monitor(&after); assert(after.free_size==before.free_size);
    printf("PASS real LVGL 9.5 pointer input, RGB565 partial rendering, six screenshots, no UI heap growth\n");
    lv_deinit(); return 0;
}
