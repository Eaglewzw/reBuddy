#include "pet_ui.h"
#include "pet_assets.h"
#include "choreo_ui.h"
#include "choreo_store.h"
#include "lvgl.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static uint16_t draw_a[320*40],draw_b[320*40],pixels[320*240];
static rb_pet_t pet;
static uint64_t now=1000;
static lv_point_t point;
static bool pressed,fail_write;
static unsigned pet_events,writes;
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
static void rebuild_first(void)
{
    lv_obj_t *label=text("1  舞蹈 1 · 4 步");assert(label);
    lv_obj_t *row=lv_obj_get_parent(label);
    lv_obj_scroll_to_view_recursive(row,LV_ANIM_OFF);lv_obj_update_layout(lv_screen_active());
    lv_area_t area;lv_obj_get_coords(row,&area);click(272,(area.y1+area.y2)/2);
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
int main(void)
{
    lv_init();
    rb_choreo_store_backend_t backend={NULL,memory_read,memory_write};
    assert(rb_choreo_store_set_backend(&backend)==RB_CHOREO_STORE_OK);
    assert(rb_choreo_store_init()==RB_CHOREO_STORE_OK);
    size_t bytes,room_bytes;uint8_t *blob=load(RB_ASSET_FILE,&bytes),*room=load(RB_ROOM_FILE,&room_bytes);
    assert(rb_assets_init(blob,bytes,2));assert(rb_assets_set_room(room,room_bytes));
    lv_display_t *display=lv_display_create(320,240);assert(display);
    lv_display_set_color_format(display,LV_COLOR_FORMAT_RGB565);
    lv_display_set_buffers(display,draw_a,draw_b,sizeof(draw_a),LV_DISPLAY_RENDER_MODE_PARTIAL);
    lv_display_set_flush_cb(display,flush);
    lv_indev_t *input=lv_indev_create();lv_indev_set_type(input,LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(input,read_touch);lv_indev_set_display(input,display);
    rb_pet_init(&pet,now);rb_pet_set_sink(&pet,sink,NULL);rb_ui_create(&pet);pump(200);
    enter();snap("choreo_library.ppm");
    tap_text("1  新建作品");assert(text("舞蹈 1") && disabled("预览"));
    tap_text("预览");assert(!text("屏幕预览"));
    add(NULL,"抬头看你");add("情绪","害羞收拢");add(NULL,"＋ 停留积木");
    assert(writes==0 && pet_events==0);
    tap_text("02  害羞收拢");click(290,65);assert(text("0.5s"));
    assert(text("重复（固定）"));click(290,112);assert(text("1 次"));
    tap_text("前移");assert(disabled("前移"));tap_text("完成");
    assert(text("01  害羞收拢") && text("02  抬头看你"));
    tap_text("03  停留");click(184,65);assert(text("0.5s"));
    click(184,65);assert(text("0.5s"));
    for(unsigned i=0;i<5;++i)click(290,65);
    assert(text("3.0s"));click(290,65);assert(text("3.0s"));
    for(unsigned i=0;i<4;++i)click(184,65);
    assert(text("1.0s") && disabled("后移"));
    snap("choreo_params.ppm");tap_text("前移");tap_text("完成");
    tap_text("03  抬头看你");tap_text("删除");assert(!text("03  抬头看你"));
    add("情绪","开心回应");
    tap_text("01  害羞收拢");tap_text("完成");add(NULL,"亲昵轻蹭");
    assert(text("02  亲昵轻蹭") && text("03  停留") && text("04  开心回应"));
    tap_text("添加动作");tap_text("表演");
    const char *dance_actions[]={"点头回应","俏皮摇头","左右摇摆","礼貌鞠躬","夹爪打拍","得意亮相",
        "底盘转拍","侧摆重拍","上下律动","左右交替","连环波浪","舒展收拍"};
    for(unsigned i=0;i<12;++i)assert(!disabled(dance_actions[i]));
    assert(!has_prefix("待素材"));snap("choreo_performance.ppm");tap_text("返回");
    snap("choreo_editor.ppm");
    tap_text("预览");assert(disabled("同步表演 · 待接入"));
    tap_text("同步表演 · 待接入");assert(text("屏幕预览"));
    tap_text("屏幕预览");pump(4000);assert(has_prefix("过渡"));snap("choreo_preview.ppm");
    tap_text("停止预览");assert(has_prefix("已停止"));
    uint32_t stopped=picture_hash();pump(5000);assert(picture_hash()==stopped);
    assert(pet_events==0 && writes==0);snap("choreo_stopped.ppm");tap_text("返回编辑");
    tap_text("节奏");snap("choreo_tempo.ppm");tap_text("快 · 时长 0.8 倍");
    tap_text("保存");assert(writes==1 && disabled("已保存"));
    rb_choreo_work_t work=saved(0);
    assert(work.count==4 && work.tempo==RB_CHOREO_FAST && work.steps[0].action_id==RB_CHOREO_WITHDRAW);
    assert(work.steps[0].hold_ms==500 && work.steps[1].action_id==RB_CHOREO_NUZZLE);
    assert(work.steps[2].action_id==RB_CHOREO_WAIT && work.steps[2].duration_ms==1000);
    assert(work.steps[3].action_id==RB_CHOREO_HAPPY && work.steps[3].hold_ms==0);
    tap_text("04  开心回应");click(290,65);tap_text("完成");tap_text("返回");
    assert(text("修改尚未保存"));snap("choreo_unsaved.ppm");tap_text("继续编辑");
    fail_write=true;tap_text("保存");assert(text("暂时无法完成") && writes==1);
    assert(saved(0).steps[3].hold_ms==0);fail_write=false;tap_text("返回");
    tap_text("返回");tap_text("放弃修改");assert(text("我的编排"));
    tap_text("1  舞蹈 1 · 4 步");assert(disabled("已保存") && !text("停止预览"));
    uint64_t editing_age=pet.now-pet.last_interaction;
    pump(120000);assert(pet.suspended && pet.state==RB_PET_NORMAL && pet.action==RB_ANIM_IDLE && pet_events==0);
    tap_text("预览");tap_text("屏幕预览");pump(21000);
    assert(has_prefix("已结束") && text("重播"));
    stopped=picture_hash();pump(3000);assert(picture_hash()==stopped && pet_events==0);
    snap("choreo_finished.ppm");tap_text("返回编辑");tap_text("返回");tap_text("返回");
    assert(!rb_choreo_ui_active() && !pet.suspended && !pet.contact && !pet.pending_valid);
    assert(pet.state==RB_PET_NORMAL && pet.action==RB_ANIM_IDLE);
    assert(pet.now-pet.last_interaction<editing_age+1000);
    /* Remaining slots, capacity boundaries and explicit unsaved save. */
    enter();tap_text("2  新建作品");assert(text("舞蹈 2"));
    for(unsigned i=0;i<8;++i)add(NULL,"＋ 停留积木");
    assert(disabled("已满 8 步"));tap_text("已满 8 步");assert(!text("添加动作"));
    snap("choreo_capacity.ppm");tap_text("08  停留");tap_text("删除");
    tap_text("返回");tap_text("保存并返回");assert(saved(1).count==7);
    /* Six dedicated dance steps expand into ten clip executions. */
    tap_text("活力律动");assert(text("活力律动"));
    assert(text("基础 24.0s   含过渡 24.0s"));snap("choreo_dance_demo.ppm");
    tap_text("01  底盘转拍");assert(text("重复次数") && text("2 次"));
    click(184,112);assert(text("1 次"));click(184,112);assert(text("1 次"));
    click(290,112);click(290,112);assert(text("3 次"));
    click(290,112);assert(text("3 次"));click(184,112);assert(text("2 次"));
    snap("choreo_repeat.ppm");tap_text("完成");
    tap_text("02  侧摆重拍");assert(text("重复次数") && text("2 次"));
    click(290,112);assert(text("3 次"));click(184,112);assert(text("2 次"));tap_text("完成");
    tap_text("03  上下律动");assert(text("重复次数") && text("2 次"));
    click(290,112);assert(text("3 次"));click(184,112);assert(text("2 次"));tap_text("完成");
    tap_text("06  舒展收拍");assert(text("重复（固定）") && text("1 次"));
    click(290,112);assert(text("1 次"));tap_text("完成");tap_text("保存");
    work=saved(2);assert(work.count==6 && work.tempo==RB_CHOREO_STANDARD);
    rb_choreo_plan_t dance_plan;assert(rb_choreo_compile(&work,&dance_plan,NULL)==RB_CHOREO_OK);
    assert(dance_plan.count==10 && dance_plan.base_ms==24000 && dance_plan.total_ms==24000);
    const uint8_t expected[]={RB_CHOREO_BASE_TURN,RB_CHOREO_BASE_TURN,RB_CHOREO_SIDE_BEAT,RB_CHOREO_SIDE_BEAT,
        RB_CHOREO_BOUNCE,RB_CHOREO_BOUNCE,RB_CHOREO_ALTERNATE,RB_CHOREO_ALTERNATE,RB_CHOREO_WAVE,RB_CHOREO_FINALE};
    for(unsigned i=0;i<10;++i)assert(dance_plan.events[i].action_id==expected[i] && !dance_plan.events[i].transition);
    tap_text("预览");tap_text("屏幕预览");pump(600);
    uint32_t dancing=picture_hash();bool dance_changed=false;
    for(unsigned tick=0;tick<5;++tick) { pump(100);dance_changed|=picture_hash()!=dancing; }
    assert(dance_changed);
    snap("choreo_dance_turn.ppm");pump(4000);snap("choreo_dance_side.ppm");
    pump(8000);snap("choreo_dance_alternate.ppm");pump(4000);snap("choreo_dance_wave.ppm");
    pump(4000);snap("choreo_dance_finale.ppm");pump(4000);assert(has_prefix("已结束") && pet_events==0);
    tap_text("返回编辑");tap_text("返回");
    /* Every performance entry plays its own moving clip. Editing this scratch
       work, including repeat parameters, must not write persistent storage. */
    unsigned before_demos=writes;
    tap_text("4  新建作品");
    for(unsigned i=0;i<12;++i) {
        add("表演",dance_actions[i]);
        char title[64];snprintf(title,sizeof(title),"01  %s",dance_actions[i]);tap_text(title);
        if(i==1 || (i>=6 && i<=10)) {
            assert(text("重复次数"));click(290,112);click(290,112);assert(text("3 次"));
            click(290,112);assert(text("3 次"));click(184,112);click(184,112);assert(text("1 次"));
        } else if(i==3 || i==11) {
            assert(text("重复（固定）"));click(290,112);assert(text("1 次"));
        }
        tap_text("完成");tap_text("预览");tap_text("屏幕预览");
        uint32_t initial=picture_hash();bool changed=false;
        for(unsigned tick=0;tick<7;++tick) { pump(200);changed|=picture_hash()!=initial; }
        assert(changed && text("停止预览") && !text("暂时无法完成"));
        char image[64];snprintf(image,sizeof(image),"choreo_performance_%u.ppm",i+1);snap(image);
        tap_text("停止预览");tap_text("返回编辑");tap_text(title);tap_text("删除");
    }
    tap_text("返回");tap_text("放弃修改");assert(writes==before_demos);
    /* The cat reference demo is separate from works: immediate 12-second
       playback, no NVS writes, explicit replay, and frozen stop/end frames. */
    tap_text("猫步律动");assert(text("停止预览"));
    uint32_t cat_start=picture_hash();pump(600);assert(picture_hash()!=cat_start);
    snap("choreo_cat_preview.ppm");tap_text("停止预览");assert(text("返回列表"));
    stopped=picture_hash();pump(3000);assert(picture_hash()==stopped && writes==before_demos);
    tap_text("重播");assert(text("停止预览"));pump(11200);assert(text("停止预览"));
    pump(1000);assert(has_prefix("已结束") && text("返回列表"));
    stopped=picture_hash();pump(2000);assert(picture_hash()==stopped && writes==before_demos && pet_events==0);
    snap("choreo_cat_finished.ppm");tap_text("返回列表");assert(text("我的编排"));
    /* Create all ten works through scrolling and touch; slot ten survives reload. */
    for(unsigned slot=3;slot<10;++slot) {
        char title[64];snprintf(title,sizeof(title),"%u  新建作品",slot+1);tap_text(title);
        add(NULL,"＋ 停留积木");tap_text("保存");
        work=saved(slot);assert(work.work_id==slot+1 && work.count==1 && work.steps[0].action_id==RB_CHOREO_WAIT);
        tap_text("返回");
    }
    tap_text("10  舞蹈 10 · 1 步");assert(text("舞蹈 10") && disabled("已保存"));
    snap("choreo_slot10.ppm");tap_text("01  停留");click(290,65);tap_text("完成");tap_text("保存");
    assert(saved(9).steps[0].duration_ms==1500);tap_text("返回");
    tap_text("10  舞蹈 10 · 1 步");tap_text("01  停留");assert(text("1.5s"));
    tap_text("完成");tap_text("返回");
    tap_text("活力律动");assert(text("暂时无法完成"));tap_text("返回");
    rebuild_first();assert(text("重建作品"));tap_text("返回");assert(saved(0).count==4);
    rebuild_first();tap_text("确认重建");assert(disabled("预览"));
    tap_text("返回");tap_text("放弃修改");assert(saved(0).count==4);tap_text("返回");
    assert(!rb_choreo_ui_active() && pet_events==0);
    /* Saved resting state survives a long visit; ordinary screen contact
       cannot become wake or replay an edited sequence after returning. */
    rb_pet_rest(&pet);pump(RB_MS_REST+200);assert(pet.state==RB_PET_RESTING);
    unsigned before=pet_events;enter();pump(65000);tap_text("返回");
    assert(pet.state==RB_PET_RESTING && pet.action==RB_ANIM_IDLE && pet_events==before);
    click(145,110);assert(pet.state==RB_PET_RESTING && pet_events==before);
    /* A fault in preview cancels playback and survives exiting the editor. */
    enter();tap_text("1  舞蹈 1 · 4 步");tap_text("预览");tap_text("屏幕预览");pump(1000);
    rb_pet_estop(&pet,false);pump(100);assert(has_prefix("故障停止") && disabled("重播"));
    stopped=picture_hash();pump(1500);assert(stopped==picture_hash());
    tap_text("返回编辑");tap_text("返回");tap_text("返回");
    assert(pet.state==RB_PET_STOPPED && !pet.estop_released && pet_events==before);
    rb_pet_estop(&pet,true);pump(100);assert(pet.state==RB_PET_STOPPED);
    printf("PASS real-touch choreography editing, boundaries, ten-slot persistence, dance repeats and 24-second preview, corruption-safe save failure, twelve performance clips and independent cat showcase, preview stop/end/fault, daily isolation and clock restoration; writes=%u\n",writes);
    rb_assets_close();lv_deinit();free(blob);free(room);return 0;
}
