#include "pet_ui.h"
#include "pet_assets.h"
#include "choreo_ui.h"
#include "choreo_motion.h"
#include "choreo_store.h"
#include "lvgl.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <zlib.h>

static uint16_t draw_a[320*40],draw_b[320*40],pixels[320*240];
static rb_pet_t pet;
static uint64_t now=1000;
static lv_point_t point;
static bool pressed,fail_write;
static unsigned pet_events,writes;
static unsigned motor_plays,motor_stops,motor_homes;
static rb_choreo_motion_status_t motor_status;
static bool motor_status_available=true;
static const char *motor_play(const rb_choreo_work_t *work,float *duration)
{ assert(work->count);++motor_plays;*duration=12.5f;motor_status=(rb_choreo_motion_status_t){.state="正在播放",.active=true,.number=10};return NULL; }
static bool motor_read(rb_choreo_motion_status_t *status) {*status=motor_status;return motor_status_available;}
static void motor_stop(void) {++motor_stops;}
static void motor_home(void) {++motor_homes;}
static uint8_t records[20][512];
static size_t lengths[20];
uint64_t rb_platform_now_ms(void) { return now; }
static void sink(void *context,const rb_pet_event_t *event)
{ (void)context;(void)event;++pet_events; }
static rb_choreo_store_result_t memory_read(void *context,unsigned key,uint8_t *data,size_t cap,size_t *length)
{
    (void)context;assert(key<20);
    if (!lengths[key]) return RB_CHOREO_STORE_EMPTY;
    if (lengths[key]>cap) return RB_CHOREO_STORE_CORRUPT;
    memcpy(data,records[key],lengths[key]);*length=lengths[key];return RB_CHOREO_STORE_OK;
}
static rb_choreo_store_result_t memory_write(void *context,unsigned key,const uint8_t *data,size_t length)
{
    (void)context;assert(key<20 && length<=512);
    if (fail_write) return RB_CHOREO_STORE_IO;
    memcpy(records[key],data,length);lengths[key]=length;++writes;return RB_CHOREO_STORE_OK;
}
static void flush(lv_display_t *display,const lv_area_t *area,uint8_t *data)
{
    const uint16_t *source=(const uint16_t *)data;
    for (int y=area->y1;y<=area->y2;++y) for (int x=area->x1;x<=area->x2;++x) {
        assert(x>=0 && x<320 && y>=0 && y<240);pixels[y*320+x]=*source++;
    }
    lv_display_flush_ready(display);
}
static void read_touch(lv_indev_t *input,lv_indev_data_t *data)
{ (void)input;data->point=point;data->state=pressed?LV_INDEV_STATE_PRESSED:LV_INDEV_STATE_RELEASED; }
static void pump(unsigned ms)
{
    for (unsigned t=0;t<ms;t+=20) {
        now+=20;lv_tick_inc(20);rb_pet_tick(&pet,now);rb_ui_update();lv_timer_handler();
    }
}
static void click(int x,int y)
{ point=(lv_point_t){x,y};pressed=true;pump(100);pressed=false;pump(100); }
/* Real pointer motion: never programmatically scroll the widget under test. */
static void swipe(int x,int from,int to)
{
    point=(lv_point_t){x,from};pressed=true;pump(40);
    for(int i=1;i<=12;++i) { point.y=from+(to-from)*i/12;pump(20); }
    pressed=false;pump(400);
}
static void scroll_touch_check(lv_obj_t *list,int top,int bottom)
{
    lv_obj_update_layout(list);
    assert(lv_obj_get_scroll_y(list)==0 && lv_obj_get_scroll_bottom(list)>0);
    /* Begin on blank scrollbar gutter, then on the body of a row button. */
    swipe(309,bottom,top);assert(lv_obj_get_scroll_y(list)>0);
    int before=lv_obj_get_scroll_y(list);swipe(160,bottom,top);
    assert(lv_obj_get_scroll_y(list)>before);
    for(unsigned i=0;i<12 && lv_obj_get_scroll_bottom(list)>0;++i) swipe(160,bottom,top);
    assert(lv_obj_get_scroll_bottom(list)==0);
    lv_area_t viewport,last;lv_obj_get_coords(list,&viewport);
    lv_obj_get_coords(lv_obj_get_child(list,-1),&last);
    assert(last.y1>=viewport.y1 && last.y2<=viewport.y2);
    swipe(309,top,bottom);assert(lv_obj_get_scroll_bottom(list)>0);
    for(unsigned i=0;i<12 && lv_obj_get_scroll_y(list)>0;++i) swipe(160,top,bottom);
    assert(lv_obj_get_scroll_y(list)==0);
}
static lv_obj_t *find(lv_obj_t *parent,const char *text,bool prefix)
{
    if (lv_obj_has_flag(parent,LV_OBJ_FLAG_HIDDEN)) return NULL;
    if (lv_obj_check_type(parent,&lv_label_class)) {
        const char *actual=lv_label_get_text(parent);
        if (prefix?!strncmp(actual,text,strlen(text)):!strcmp(actual,text)) return parent;
    }
    for (unsigned i=0;i<lv_obj_get_child_count(parent);++i) {
        lv_obj_t *found=find(lv_obj_get_child(parent,i),text,prefix);if(found)return found;
    }
    return NULL;
}
static lv_obj_t *text(const char *value)
{ return find(lv_screen_active(),value,false); }
static bool has_prefix(const char *value)
{ return find(lv_screen_active(),value,true)!=NULL; }
static void tap_text(const char *value)
{
    lv_obj_t *label=text(value);
    if (!label) { fprintf(stderr,"Missing UI label: %s\n",value);assert(label); }
    lv_obj_scroll_to_view_recursive(label,LV_ANIM_OFF);lv_obj_update_layout(lv_screen_active());
    lv_area_t area;lv_obj_get_coords(label,&area);
    int x=(area.x1+area.x2)/2,y=(area.y1+area.y2)/2;
    assert(x>=0 && x<320 && y>=0 && y<240);click(x,y);
}
static void rebuild_first(const char *title)
{
    lv_obj_t *label=text(title);assert(label);
    lv_obj_t *row=lv_obj_get_parent(label);
    lv_obj_scroll_to_view_recursive(row,LV_ANIM_OFF);lv_obj_update_layout(lv_screen_active());
    lv_obj_t *rebuild=find(row,"重建",false);assert(rebuild);
    lv_area_t area;lv_obj_get_coords(rebuild,&area);click((area.x1+area.x2)/2,(area.y1+area.y2)/2);
}
static bool disabled(const char *value)
{
    lv_obj_t *label=text(value);assert(label);
    return lv_obj_has_state(lv_obj_get_parent(label),LV_STATE_DISABLED);
}
static void snap(const char *name)
{
    lv_obj_update_layout(lv_screen_active());lv_refr_now(NULL);
    FILE *file=fopen(name,"wb");assert(file);fprintf(file,"P6\n320 240\n255\n");
    for (unsigned i=0;i<320*240;++i) {
        uint16_t value=pixels[i];
        unsigned char rgb[]={((value>>11)&31)*255/31,((value>>5)&63)*255/63,(value&31)*255/31};
        assert(fwrite(rgb,1,3,file)==3);
    }
    fclose(file);
}
static uint32_t picture_hash(void)
{
    lv_refr_now(NULL);uint32_t value=2166136261u;
    for (unsigned y=50;y<188;++y) for(unsigned x=0;x<320;++x)
        value=(value^pixels[y*320+x])*16777619u;
    return value;
}
static uint8_t *load(const char *path,size_t *length)
{
    FILE *file=fopen(path,"rb");assert(file);fseek(file,0,SEEK_END);long size=ftell(file);assert(size>0);
    rewind(file);uint8_t *data=malloc((size_t)size);assert(data);
    assert(fread(data,1,(size_t)size,file)==(size_t)size);fclose(file);*length=(size_t)size;return data;
}
static void enter(void)
{
    click(290,212);assert(text("编排"));click(230,60);
    assert(rb_choreo_ui_active() && pet.suspended && text("我的编排"));
    assert(!pet.contact && !pet.pending_valid);
}
static void add(const char *group,const char *action)
{ tap_text("添加动作");if(group)tap_text(group);tap_text(action); }
static rb_choreo_work_t saved(unsigned slot)
{
    rb_choreo_work_t work;assert(rb_choreo_store_load(slot,&work)==RB_CHOREO_STORE_OK);return work;
}
static void check_dance_room(void)
{
    const lv_image_dsc_t *room=rb_assets_room();assert(room);
    const uint16_t *background=(const uint16_t *)room->data;
    uint16_t *original=malloc(RB_FRAME_BYTES);assert(original);
    for(unsigned id=RB_CHOREO_START;id<=RB_CHOREO_DANCE_PIVOT;++id) {
        const rb_clip_meta_t *clip=&rb_clips[rb_choreo_clip_map[id]];
        for(unsigned tick=0;tick<clip->count;++tick) {
            unsigned frame=clip->frames[tick];const rb_frame_meta_t *meta=NULL;
            const lv_image_dsc_t *raw=rb_assets_frame(frame,&meta);assert(meta==&rb_frames[frame]);
            memcpy(original,raw->data,RB_FRAME_BYTES);
            const lv_image_dsc_t *image=rb_assets_dance_frame(frame,&meta);assert(image && meta==&rb_frames[frame]);
            const uint16_t *screen=(const uint16_t *)image->data;
            assert(!memcmp(original,screen,RB_FRAME_BYTES));
            for(unsigned y=0;y<240;++y)for(unsigned x=0;x<320;++x)
                if(x<40 || x>=280)assert(screen[y*320+x]==background[y*320+x]);
            raw=rb_assets_frame(frame,&meta);assert(!memcmp(original,raw->data,RB_FRAME_BYTES));
            /* Also inspect a rendered midpoint from each whole source clip. */
            if(tick==clip->count/2) {
                char name[64];snprintf(name,sizeof(name),"dance_room_%u.ppm",id);
                FILE *f=fopen(name,"wb");assert(f);fprintf(f,"P6\n320 240\n255\n");
                for(unsigned i=0;i<320*240;++i) {
                    uint16_t v=screen[i];unsigned char rgb[]={((v>>11)&31)*255/31,((v>>5)&63)*255/63,(v&31)*255/31};
                    assert(fwrite(rgb,1,3,f)==3);
                } fclose(f);
            }
        }
    }
    free(original);
    const rb_frame_meta_t *meta=NULL;
    assert(!rb_assets_dance_frame(RB_FRAME_COUNT,&meta));assert(!rb_assets_dance_frame(0,NULL));
}
int main(void)
{
    lv_init();
    rb_choreo_store_backend_t backend={NULL,memory_read,memory_write};
    assert(rb_choreo_store_set_backend(&backend)==RB_CHOREO_STORE_OK);
    assert(rb_choreo_store_init()==RB_CHOREO_STORE_OK);
    size_t bytes,room_bytes;uint8_t *blob=load(RB_ASSET_FILE,&bytes),*room=load(RB_ROOM_FILE,&room_bytes);
    assert(rb_assets_init(blob,bytes,2));assert(rb_assets_set_room(room,room_bytes));

    check_dance_room();
    lv_display_t *display=lv_display_create(320,240);assert(display);
    lv_display_set_color_format(display,LV_COLOR_FORMAT_RGB565);
    lv_display_set_buffers(display,draw_a,draw_b,sizeof(draw_a),LV_DISPLAY_RENDER_MODE_PARTIAL);
    lv_display_set_flush_cb(display,flush);
    lv_indev_t *input=lv_indev_create();lv_indev_set_type(input,LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(input,read_touch);lv_indev_set_display(input,display);
    rb_pet_init(&pet,now);rb_pet_set_sink(&pet,sink,NULL);rb_ui_create(&pet);pump(200);
    enter();
    lv_obj_t *library=lv_obj_get_parent(lv_obj_get_parent(text("1  新建作品")));
    scroll_touch_check(library,65,172);
    assert(text("我的编排") && writes==0 && pet_events==0);
    snap("choreo_library.ppm");
    lv_area_t library_first,library_fourth;
    lv_obj_get_coords(lv_obj_get_child(library,0),&library_first);
    lv_obj_get_coords(lv_obj_get_child(library,3),&library_fourth);
    assert(library_first.y1>=44 && library_fourth.y2<232 && library_fourth.x1>library_first.x2);
    tap_text("1  新建作品");assert(disabled("播放"));
    tap_text("添加动作");
    const char *actions[]={"点头打拍","歪头招手","深蹲回升","前探伸展","探头撒娇",
        "开心欢呼","蛇形波浪","稳定摇摆","空间8字","大幅定点左右上下"};
    for(unsigned i=0;i<10;++i) assert(!disabled(actions[i]));
    const char *removed[]={"开场","收尾","互动","情绪","表演","抬头看你","亲昵轻蹭","疑惑歪头","害羞收拢","安抚展开","开心回应"};
    for(unsigned i=0;i<sizeof(removed)/sizeof(removed[0]);++i) assert(!text(removed[i]));
    lv_obj_t *action_list=lv_obj_get_parent(lv_obj_get_parent(text("点头打拍")));
    scroll_touch_check(action_list,88,175);
    assert(text("添加动作") && writes==0 && pet_events==0);
    /* Four complete cards fit at once; each owns a stable thumbnail. */
    lv_area_t first,third;lv_obj_get_coords(lv_obj_get_child(action_list,0),&first);
    lv_obj_get_coords(lv_obj_get_child(action_list,2),&third);
    assert(first.y1>=44 && third.y2<232);
    const lv_image_dsc_t *pictures[10]={0};
    for(unsigned i=0;i<10;++i) {
        lv_obj_t *card=lv_obj_get_child(action_list,i);
        for(unsigned j=0;j<lv_obj_get_child_count(card);++j) {
            lv_obj_t *child=lv_obj_get_child(card,j);
            if(lv_obj_check_type(child,&lv_image_class))pictures[i]=lv_image_get_src(child);
        }
        assert(pictures[i] && pictures[i]->header.w==96 && pictures[i]->header.h==64);
        for(unsigned j=0;j<i;++j) assert(pictures[j]->data!=pictures[i]->data);
    }
    snap("choreo_performance.ppm");tap_text("点头打拍");
    add(NULL,"稳定摇摆");add(NULL,"空间8字");add(NULL,"＋ 停留积木");
    assert(writes==0 && pet_events==0);
    tap_text("02  稳定摇摆");click(290,112);assert(text("2 次"));
    click(290,65);assert(text("0.3s"));tap_text("前移");assert(disabled("前移"));tap_text("完成");
    tap_text("保存");assert(saved(0).count==4 && saved(0).steps[0].action_id==RB_CHOREO_DANCE_STEADY);
    assert(saved(0).steps[0].repeat==2 && saved(0).steps[0].hold_ms==300);
    tap_text("播放");assert(disabled("机械臂未接入"));tap_text("屏幕预览");pump(100);assert(has_prefix("开场"));
    uint32_t stopped=picture_hash();pump(600);assert(stopped!=picture_hash());snap("choreo_selected_dance.ppm");
    tap_text("停止预览");stopped=picture_hash();pump(2000);assert(stopped==picture_hash());
    rb_choreo_work_t played=saved(0);rb_choreo_plan_t timing;
    assert(rb_choreo_compile(&played,&timing,NULL)==RB_CHOREO_OK);
    tap_text("重播");pump(timing.total_ms-1000);assert(has_prefix("收尾"));pump(1200);assert(has_prefix("已结束"));
    stopped=picture_hash();pump(2000);assert(stopped==picture_hash());
    tap_text("返回编辑");tap_text("返回");
    pump(250);snap("choreo_library_saved.ppm");
    /* Explicit rebuild never overwrites saved work until Save. */
    rebuild_first("1  舞蹈 1 · 4 步");tap_text("确认重建");assert(disabled("播放"));
    tap_text("返回");tap_text("放弃修改");assert(saved(0).count==4);
    /* Touch scrolling can select every imported primitive and persist it. */
    for(unsigned slot=1;slot<10;++slot) {
        char title[64];snprintf(title,sizeof(title),"%u  新建作品",slot+1);tap_text(title);
        add(NULL,actions[slot]);tap_text("保存");
        assert(saved(slot).steps[0].action_id==RB_CHOREO_DANCE_NOD+slot);tap_text("返回");
    }
    tap_text("2  舞蹈 2 · 1 步");
    for(unsigned i=1;i<8;++i)add(NULL,"点头打拍");
    assert(disabled("已满 8 步"));
    lv_obj_t *steps=lv_obj_get_parent(lv_obj_get_parent(text("01  歪头招手")));
    for(unsigned i=0;i<12 && lv_obj_get_scroll_y(steps)>0;++i) swipe(309,94,174);
    scroll_touch_check(steps,94,174);assert(text("已满 8 步") && text("舞蹈 2"));
    tap_text("08  点头打拍");tap_text("删除");
    fail_write=true;tap_text("保存");assert(text("暂时无法完成"));fail_write=false;
    tap_text("返回");tap_text("返回");tap_text("保存并返回");assert(saved(1).count==7);
    tap_text("返回");assert(!rb_choreo_ui_active() && !pet.suspended && pet_events==0);
    /* Only explicit motor Play submits a request. Navigation and preview never do. */
    rb_choreo_motion_backend_t motion_backend={motor_play,motor_read,motor_stop,motor_home};
    rb_choreo_ui_set_motion(&motion_backend);
    enter();tap_text("10  舞蹈 10 · 1 步");tap_text("播放");
    assert(!motor_plays && !disabled("机械臂播放"));tap_text("机械臂播放");
    assert(motor_plays==1 && text("机械臂表演"));
    rb_pet_t old_pet=pet;rb_pet_stop(&pet,"DANCE CONTROL");pump(100);
    assert(!motor_stops); /* Healthy Dance ownership is not a fault. */
    snap("choreo_motor.ppm");
    motor_status_available=false;pump(200);assert(!motor_stops); /* Brief mutex contention. */
    pump(600);assert(motor_stops==1); /* Sustained status loss: one hold request. */
    pump(200);assert(motor_stops==1);motor_status_available=true;pump(100);
    tap_text("返回");assert(text("机械臂表演") && motor_stops==2);
    tap_text("停止保持");assert(motor_stops==3);
    tap_text("回零");assert(motor_homes==1);
    motor_status.active=false;motor_status.finished=true;motor_status.state="已退出";pump(100);
    tap_text("返回");assert(text("舞蹈 10"));tap_text("返回");tap_text("返回");
    pet=old_pet;pet.suspended=false;rb_choreo_ui_set_motion(NULL);
    rb_pet_rest(&pet);pump(RB_MS_REST+200);assert(pet.state==RB_PET_RESTING);
    unsigned before=pet_events;enter();pump(65000);tap_text("返回");
    assert(pet.state==RB_PET_RESTING && pet.action==RB_ANIM_IDLE && pet_events==before);
    click(145,110);assert(pet.state==RB_PET_RESTING && pet_events==before);
    enter();tap_text("1  舞蹈 1 · 4 步");tap_text("播放");tap_text("屏幕预览");pump(1000);
    rb_pet_estop(&pet,false);pump(100);assert(has_prefix("故障停止") && disabled("重播"));
    stopped=picture_hash();pump(1500);assert(stopped==picture_hash());
    tap_text("返回编辑");tap_text("返回");tap_text("返回");
    assert(pet.state==RB_PET_STOPPED && !pet.estop_released && pet_events==before);
    /* Old action IDs stay visible as an old library, never become new dances. */
    for(unsigned key=0;key<2;++key) if(lengths[key]) {
        records[key][57]=33;
        uint32_t crc=(uint32_t)crc32(0,records[key],16);
        crc=(uint32_t)crc32(crc,records[key]+20,lengths[key]-20);
        for(unsigned b=0;b<4;++b)records[key][16+b]=(uint8_t)(crc>>(8*b));
    }
    unsigned before_writes=writes;
    enter();assert(text("1  旧版作品"));tap_text("1  旧版作品");
    assert(text("作品使用旧版动作库。原数据已保留，请返回后选择重建。"));
    assert(writes==before_writes);tap_text("返回");
    rebuild_first("1  旧版作品");tap_text("确认重建");assert(disabled("播放"));
    add(NULL,"点头打拍");assert(writes==before_writes);tap_text("保存");
    assert(saved(0).steps[0].action_id==RB_CHOREO_DANCE_NOD);
    tap_text("返回");tap_text("返回");
    puts("PASS ten-dance-only touch library, all ten selections, repeat/hold/save, eight-step capacity, failure recovery, playback/stop/end/fault and Daily isolation");
    rb_assets_close();lv_deinit();free(blob);free(room);return 0;
}
