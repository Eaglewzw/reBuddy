#include "choreo_ui.h"
#include "choreo_store.h"
#include "pet_assets.h"
#include "rb_pet.h"
#include <stdio.h>
#include <string.h>

extern const lv_font_t rb_choreo_font,rb_choreo_small_font;
enum page { LIBRARY,EDITOR,ADD,PARAMS,TEMPO,UNSAVED,REPLACE,PLAY_CHOICE,PREVIEW,ERROR_PAGE };
enum command { NONE,BACK,OPEN,REBUILD,SAMPLE,CONFIRM_REPLACE,SAVE,SAVE_EXIT,DISCARD,
    CONTINUE,ADD_PAGE,GROUP,ADD_ACTION,SELECT_STEP,MOVE_UP,MOVE_DOWN,DELETE_STEP,
    HOLD_LESS,HOLD_MORE,REPEAT_LESS,REPEAT_MORE,TEMPO_PAGE,SET_TEMPO,PLAY_PAGE,PLAY,STOP };
static struct {
    lv_obj_t *root,*image,*progress,*stop,*bar;
    void (*on_exit)(void *); void *context;
    enum page page,error_return;
    enum command command;
    int arg,selected,slot,group;
    uint64_t now;
    bool dirty,faulted;
    rb_choreo_work_t work;
    rb_choreo_plan_t plan;
    rb_choreo_player_t player;
    unsigned frame;
    char notice[128];
} ui;
static const char *const tempos[]={"慢","标准","快"};
static void draw(void);
static lv_obj_t *panel(lv_obj_t *parent,int x,int y,int w,int h,uint32_t color,int radius)
{
    lv_obj_t *o=lv_obj_create(parent); lv_obj_remove_style_all(o);
    lv_obj_set_pos(o,x,y); lv_obj_set_size(o,w,h);
    lv_obj_set_style_bg_color(o,lv_color_hex(color),0); lv_obj_set_style_bg_opa(o,LV_OPA_COVER,0);
    lv_obj_set_style_radius(o,radius,0);
    lv_obj_remove_flag(o,LV_OBJ_FLAG_SCROLLABLE|LV_OBJ_FLAG_CLICKABLE); return o;
}
static lv_obj_t *label(lv_obj_t *parent,int x,int y,int w,const char *value,bool small)
{
    lv_obj_t *o=lv_label_create(parent); lv_label_set_text(o,value);
    lv_obj_set_pos(o,x,y); lv_obj_set_width(o,w);
    lv_obj_set_style_text_font(o,small?&rb_choreo_small_font:&rb_choreo_font,0);
    lv_obj_set_style_text_color(o,lv_color_hex(0x333b2d),0);
    lv_obj_remove_flag(o,LV_OBJ_FLAG_CLICKABLE); return o;
}
static void event(lv_event_t *e)
{
    if (ui.command!=NONE) return;
    uintptr_t value=(uintptr_t)lv_event_get_user_data(e);
    ui.command=(enum command)(value&255u); ui.arg=(int)(value>>8);
    lv_indev_t *input=lv_indev_active(); if (input) lv_indev_wait_release(input);
}
static lv_obj_t *button(lv_obj_t *parent,int x,int y,int w,int h,const char *value,
                        enum command cmd,int arg,bool enabled,uint32_t color)
{
    lv_obj_t *o=panel(parent,x,y,w,h,color,9);
    lv_obj_add_flag(o,LV_OBJ_FLAG_CLICKABLE|LV_OBJ_FLAG_PRESS_LOCK);
    lv_obj_set_style_bg_color(o,lv_color_hex(0xcbd8a4),LV_STATE_PRESSED);
    lv_obj_set_style_opa(o,LV_OPA_50,LV_STATE_DISABLED);
    if (!enabled) lv_obj_add_state(o,LV_STATE_DISABLED);
    lv_obj_t *l=label(o,0,0,w-8,value,false);
    lv_obj_set_style_text_align(l,LV_TEXT_ALIGN_CENTER,0); lv_obj_center(l);
    lv_obj_add_event_cb(o,event,LV_EVENT_CLICKED,(void *)((uintptr_t)cmd|((uintptr_t)arg<<8)));
    return o;
}
static void header(const char *title,bool save)
{
    button(ui.root,8,5,46,34,"返回",BACK,0,true,0xe8e8da);
    label(ui.root,64,13,save?174:248,title,false);
    if (save) button(ui.root,252,5,60,34,ui.dirty?"保存":"已保存",SAVE,0,ui.dirty,0xd8ef79);
}
static const char *store_error(rb_choreo_store_result_t status)
{
    switch (status) {
    case RB_CHOREO_STORE_VERSION:return "作品版本不兼容，已保留原数据。";
    case RB_CHOREO_STORE_CORRUPT:return "作品数据损坏，无法打开。";
    case RB_CHOREO_STORE_INVALID:return "作品参数无效，请检查后重试。";
    default:return "存储暂不可用。修改仍保留在本页，请稍后重试。";
    }
}
static void error_page(const char *message,enum page return_to)
{
    snprintf(ui.notice,sizeof(ui.notice),"%s",message);
    ui.error_return=return_to; ui.page=ERROR_PAGE;
}
static bool save(void)
{
    rb_choreo_store_result_t result=rb_choreo_store_save((unsigned)ui.slot,&ui.work);
    if (result!=RB_CHOREO_STORE_OK) { error_page(store_error(result),EDITOR); return false; }
    ui.dirty=false; return true;
}
static void fresh_work(void)
{
    rb_choreo_work_init(&ui.work,(unsigned)ui.slot);
    ui.selected=-1; ui.dirty=true; ui.page=EDITOR;
}
static void close_page(void)
{
    void (*callback)(void *)=ui.on_exit; void *context=ui.context;
    lv_obj_delete(ui.root); ui.root=NULL; ui.command=NONE;
    if (callback) callback(context);
}
static void back(void)
{
    switch (ui.page) {
    case LIBRARY:close_page();break;
    case EDITOR:ui.page=ui.dirty?UNSAVED:LIBRARY;break;
    case ADD:case PARAMS:case TEMPO:case PLAY_CHOICE:ui.page=EDITOR;break;
    case PREVIEW:rb_choreo_player_stop(&ui.player,ui.now);ui.page=EDITOR;break;
    case UNSAVED:ui.page=EDITOR;break;
    case REPLACE:ui.page=LIBRARY;break;
    case ERROR_PAGE:ui.page=ui.error_return;break;
    }
}
static void handle(enum command command,int arg)
{
    rb_choreo_step_t *step=(ui.selected>=0 && ui.selected<ui.work.count)?&ui.work.steps[ui.selected]:NULL;
    switch (command) {
    case BACK:back();break;
    case OPEN: {
        ui.slot=arg;
        rb_choreo_store_result_t result=rb_choreo_store_load((unsigned)arg,&ui.work);
        if (result==RB_CHOREO_STORE_EMPTY) fresh_work();
        else if (result==RB_CHOREO_STORE_OK) { ui.selected=-1;ui.dirty=false;ui.page=EDITOR; }
        else error_page(store_error(result),LIBRARY);
        break;
    }
    case REBUILD:ui.slot=arg;ui.page=REPLACE;break;
    case CONFIRM_REPLACE:fresh_work();break;
    case SAMPLE: {
        bool found=false;
        for (unsigned slot=0;slot<RB_CHOREO_SLOTS;++slot) {
            rb_choreo_work_t stored;
            if (rb_choreo_store_load(slot,&stored)==RB_CHOREO_STORE_EMPTY) {
                ui.slot=(int)slot; found=true; break;
            }
        }
        if (!found) { error_page("作品已满（10条），请选择已有作品重建。",LIBRARY);break; }
        fresh_work(); rb_choreo_work_demo(&ui.work,(unsigned)ui.slot);
        break;
    }
    case SAVE:(void)save();break;
    case SAVE_EXIT:if (save()) ui.page=LIBRARY;break;
    case DISCARD:ui.dirty=false;ui.page=LIBRARY;break;
    case CONTINUE:ui.page=EDITOR;break;
    case ADD_PAGE:ui.page=ADD;ui.group=RB_CHOREO_INTERACTION;break;
    case GROUP:ui.group=arg;break;
    case ADD_ACTION: {
        rb_choreo_step_t item={.action_id=arg,.repeat=1,.duration_ms=arg==RB_CHOREO_WAIT?1000:0};
        int insertion=ui.selected<0?ui.work.count:ui.selected+1;
        if (rb_choreo_add_after(&ui.work,ui.selected,&item)==RB_CHOREO_OK) {
            ui.selected=insertion;ui.dirty=true;ui.page=EDITOR;
        }
        break;
    }
    case SELECT_STEP:ui.selected=arg;ui.page=PARAMS;break;
    case MOVE_UP:case MOVE_DOWN: {
        int direction=command==MOVE_UP?-1:1;
        if (step && rb_choreo_move(&ui.work,(unsigned)ui.selected,direction)==RB_CHOREO_OK) {
            ui.selected+=direction;ui.dirty=true;
        }
        break;
    }
    case DELETE_STEP:
        if (step && rb_choreo_delete(&ui.work,(unsigned)ui.selected)==RB_CHOREO_OK) {
            ui.selected=-1;ui.dirty=true;ui.page=EDITOR;
        }
        break;
    case HOLD_LESS:case HOLD_MORE:
        if (step) {
            uint16_t *value=step->action_id==RB_CHOREO_WAIT?&step->duration_ms:&step->hold_ms;
            int next=*value+(command==HOLD_MORE?(int)RB_CHOREO_TIME_STEP_MS:-(int)RB_CHOREO_TIME_STEP_MS);
            int minimum=step->action_id==RB_CHOREO_WAIT?RB_CHOREO_WAIT_MIN_MS:0;
            int maximum=step->action_id==RB_CHOREO_WAIT?RB_CHOREO_WAIT_MAX_MS:RB_CHOREO_HOLD_MAX_MS;
            if (next>=minimum && next<=maximum) { *value=(uint16_t)next;ui.dirty=true; }
        }
        break;
    case REPEAT_LESS:case REPEAT_MORE:
        if (step) {
            const rb_choreo_action_t *action=rb_choreo_action(step->action_id);
            int next=step->repeat+(command==REPEAT_MORE?1:-1);
            if (action && next>=1 && next<=action->repeat_max) { step->repeat=(uint8_t)next;ui.dirty=true; }
        }
        break;
    case TEMPO_PAGE:ui.page=TEMPO;break;
    case SET_TEMPO:if (arg<RB_CHOREO_TEMPO_COUNT) { ui.dirty|=ui.work.tempo!=arg;ui.work.tempo=arg;ui.page=EDITOR; }break;
    case PLAY_PAGE:ui.page=PLAY_CHOICE;break;
    case PLAY: {
        uint8_t bad=RB_CHOREO_NO_STEP;
        rb_choreo_status_t result=rb_choreo_compile(&ui.work,&ui.plan,&bad);
        if (result!=RB_CHOREO_OK) {
            char message[100];
            if (bad!=RB_CHOREO_NO_STEP) snprintf(message,sizeof(message),"第 %u 步不可预览，请检查动作与参数。",bad+1);
            else snprintf(message,sizeof(message),"请先添加可用动作，再开始预览。");
            error_page(message,EDITOR);break;
        }
        if (ui.faulted) { error_page("当前已停止，请退出编排后恢复。",EDITOR);break; }
        (void)rb_choreo_player_start(&ui.player,&ui.plan,ui.now);
        ui.frame=UINT32_MAX;ui.page=PREVIEW;break;
    }
    case STOP:rb_choreo_player_stop(&ui.player,ui.now);break;
    default:break;
    }
}
static lv_obj_t *list(int y,int h)
{
    lv_obj_t *o=panel(ui.root,8,y,304,h,0xf7f5e9,0);
    /* The list must receive a drag beginning between cards or on its gutter. */
    lv_obj_add_flag(o,LV_OBJ_FLAG_SCROLLABLE|LV_OBJ_FLAG_CLICKABLE);
    lv_obj_remove_flag(o,LV_OBJ_FLAG_SCROLL_CHAIN_HOR|LV_OBJ_FLAG_SCROLL_CHAIN_VER);
    lv_obj_set_style_width(o,4,LV_PART_SCROLLBAR);
    lv_obj_set_style_bg_color(o,lv_color_hex(0x8a9c63),LV_PART_SCROLLBAR);
    lv_obj_set_style_bg_opa(o,LV_OPA_COVER,LV_PART_SCROLLBAR);
    lv_obj_set_style_radius(o,2,LV_PART_SCROLLBAR);
    lv_obj_set_style_pad_right(o,0,LV_PART_SCROLLBAR);
    lv_obj_set_scroll_dir(o,LV_DIR_VER);lv_obj_set_scrollbar_mode(o,LV_SCROLLBAR_MODE_AUTO);
    return o;
}
static void draw_library(void)
{
    header("我的编排",false);
    label(ui.root,242,15,70,"最多10条",true);
    lv_obj_t *items=list(48,140);
    for (unsigned i=0;i<RB_CHOREO_SLOTS;++i) {
        rb_choreo_work_t work; char value[80];
        rb_choreo_store_result_t result=rb_choreo_store_load(i,&work);
        bool saved=result==RB_CHOREO_STORE_OK;
        bool rebuild=saved || result==RB_CHOREO_STORE_CORRUPT;
        if (saved) snprintf(value,sizeof(value),"%u  %s · %u 步",i+1,work.name,work.count);
        else snprintf(value,sizeof(value),"%u  %s",i+1,result==RB_CHOREO_STORE_EMPTY?"新建作品":"作品无法读取");
        button(items,0,(int)i*48,rebuild?224:296,40,value,OPEN,(int)i,true,0xffffff);
        if (rebuild) button(items,232,(int)i*48,64,40,"重建",REBUILD,(int)i,true,0xe8e8da);
    }
    label(ui.root,12,204,296,"共10个编排 · 上下滑动查看",true);
}
static void draw_editor(void)
{
    header(ui.work.name,true);
    char value[100]; rb_choreo_plan_t plan;
    bool valid=rb_choreo_compile(&ui.work,&plan,NULL)==RB_CHOREO_OK;
    snprintf(value,sizeof(value),"%u/%u 步 · %s · 屏幕时长估算",
        ui.work.count,RB_CHOREO_MAX_STEPS,tempos[ui.work.tempo]);
    label(ui.root,10,43,300,value,true);
    snprintf(value,sizeof(value),"总时长 %.1fs（含开场收尾）",
        valid?plan.total_ms/1000.0:0);
    label(ui.root,10,60,300,value,true);
    lv_obj_t *items=list(82,108);
    if (!ui.work.count) {
        label(items,16,24,270,"把喜欢的动作排成一段小表演",false);
        label(items,16,55,270,"从下方「添加」开始",true);
    }
    for (unsigned i=0;i<ui.work.count;++i) {
        const rb_choreo_action_t *action=rb_choreo_action(ui.work.steps[i].action_id);
        lv_obj_t *card=button(items,0,(int)i*55,296,49,"",SELECT_STEP,(int)i,true,
            ui.selected==(int)i?0xe4efb7:0xffffff);
        snprintf(value,sizeof(value),"%02u  %s",i+1,action?action->name:"未知动作");
        label(card,10,4,278,value,false);
        rb_choreo_step_t step=ui.work.steps[i];
        if (step.action_id==RB_CHOREO_WAIT) snprintf(value,sizeof(value),"保持当前姿态 %.1f 秒 · 点击调整",step.duration_ms/1000.0);
        else snprintf(value,sizeof(value),"%u 次 · 结束停留 %.1f 秒 · 点击调整",step.repeat,step.hold_ms/1000.0);
        label(card,10,27,278,value,true);
        if (ui.selected==(int)i) lv_obj_scroll_to_view(card,LV_ANIM_OFF);
    }
    button(ui.root,8,198,104,34,ui.work.count<RB_CHOREO_MAX_STEPS?"添加动作":"已满 8 步",ADD_PAGE,0,ui.work.count<RB_CHOREO_MAX_STEPS,0xd8ef79);
    button(ui.root,120,198,72,34,"节奏",TEMPO_PAGE,0,true,0xe8e8da);
    button(ui.root,200,198,112,34,"预览",PLAY_PAGE,0,valid,0xe4edbf);
}
static void draw_add(void)
{
    header("添加动作",false);
    label(ui.root,12,48,296,"共10组舞蹈 · 上下滑动查看",true);
    lv_obj_t *items=list(72,118); int row=0;
    for (unsigned i=0;i<rb_choreo_action_count;++i) {
        const rb_choreo_action_t *action=&rb_choreo_actions[i];
        if (action->group!=RB_CHOREO_PERFORMANCE) continue;
        char value[80];snprintf(value,sizeof(value),"%s%s",action->name,action->available?"":" · 待素材");
        button(items,0,row++*44,296,38,value,ADD_ACTION,action->action_id,action->available,0xffffff);
    }
    button(ui.root,8,198,304,34,"＋ 停留积木",ADD_ACTION,RB_CHOREO_WAIT,true,0xe4edbf);
}
static void draw_params(void)
{
    const rb_choreo_step_t *step=&ui.work.steps[ui.selected];
    const rb_choreo_action_t *action=rb_choreo_action(step->action_id);
    header(action->name,false);
    bool wait=step->action_id==RB_CHOREO_WAIT;
    unsigned value=wait?step->duration_ms:step->hold_ms;
    unsigned minimum=wait?RB_CHOREO_WAIT_MIN_MS:0;
    unsigned maximum=wait?RB_CHOREO_WAIT_MAX_MS:RB_CHOREO_HOLD_MAX_MS;
    label(ui.root,12,61,145,wait?"停留时长":"结束后停留",false);
    button(ui.root,164,48,40,38,"-",HOLD_LESS,0,value>minimum,0xe8e8da);
    char number[24];snprintf(number,sizeof(number),"%.1fs",value/1000.0);
    label(ui.root,212,61,58,number,false);
    button(ui.root,272,48,40,38,"+",HOLD_MORE,0,value<maximum,0xe4edbf);
    label(ui.root,12,106,145,action->repeat_max==1?"重复（固定）":"重复次数",false);
    button(ui.root,164,93,40,38,"-",REPEAT_LESS,0,step->repeat>1,0xe8e8da);
    snprintf(number,sizeof(number),"%u 次",step->repeat);label(ui.root,212,106,58,number,false);
    button(ui.root,272,93,40,38,"+",REPEAT_MORE,0,step->repeat<action->repeat_max,0xe4edbf);
    button(ui.root,8,146,96,38,"前移",MOVE_UP,0,ui.selected>0,0xe8e8da);
    button(ui.root,112,146,96,38,"后移",MOVE_DOWN,0,ui.selected+1<ui.work.count,0xe8e8da);
    button(ui.root,216,146,96,38,"删除",DELETE_STEP,0,true,0xf1dcd0);
    button(ui.root,8,198,304,34,"完成",BACK,0,true,0xd8ef79);
}
static void draw(void)
{
    if (!ui.root) return;
    lv_obj_clean(ui.root);ui.image=NULL;ui.progress=NULL;ui.stop=NULL;ui.bar=NULL;
    switch (ui.page) {
    case LIBRARY:draw_library();break;
    case EDITOR:draw_editor();break;
    case ADD:draw_add();break;
    case PARAMS:draw_params();break;
    case TEMPO: {
        header("屏幕节奏",false);
        const char *names[]={"慢 · 时长 1.5 倍","标准 · 原始时长","快 · 时长 0.8 倍"};
        for (int i=0;i<3;++i) button(ui.root,8,48+i*46,304,38,names[i],SET_TEMPO,i,true,ui.work.tempo==i?0xd8ef79:0xffffff);
        label(ui.root,12,197,296,"仅调整屏幕预览，实体节奏待标定",true);break;
    }
    case UNSAVED:
        header("修改尚未保存",false);
        button(ui.root,16,61,288,42,"保存并返回",SAVE_EXIT,0,true,0xd8ef79);
        button(ui.root,16,115,288,42,"放弃修改",DISCARD,0,true,0xf1dcd0);
        button(ui.root,16,169,288,42,"继续编辑",CONTINUE,0,true,0xffffff);break;
    case REPLACE:
        header("重建作品",false);
        label(ui.root,20,65,280,"重建后将从空白作品开始。点击保存时，才会替换原作品。",false);
        button(ui.root,16,150,288,42,"确认重建",CONFIRM_REPLACE,0,true,0xf1dcd0);
        break;
    case PLAY_CHOICE:
        header("预览与表演",false);
        button(ui.root,16,59,288,46,"屏幕预览",PLAY,0,!ui.faulted,0xd8ef79);
        button(ui.root,16,121,288,46,"同步表演 · 待接入",NONE,0,false,0xe8e8da);
        label(ui.root,20,186,280,"当前只播放屏幕动画\n机械臂动作与标定后续接入",true);break;
    case PREVIEW:
        ui.image=lv_image_create(ui.root);lv_obj_set_pos(ui.image,0,0);
        lv_obj_remove_flag(ui.image,LV_OBJ_FLAG_CLICKABLE);
        ui.progress=label(ui.root,0,0,320,"",true);
        lv_obj_set_style_bg_color(ui.progress,lv_color_hex(0xfffdf3),0);
        lv_obj_set_style_bg_opa(ui.progress,LV_OPA_90,0);
        lv_obj_set_style_pad_all(ui.progress,4,0);
        panel(ui.root,0,192,320,48,0xf7f5e9,0);
        if (ui.player.running) ui.stop=button(ui.root,8,199,104,34,"停止预览",STOP,0,true,0xf1c6ad);
        else {
            button(ui.root,8,199,104,34,"返回编辑",BACK,0,true,0xe8e8da);
            button(ui.root,120,199,72,34,"重播",PLAY,0,!ui.faulted,0xd8ef79);
        }
        label(ui.root,ui.player.running?125:204,207,ui.player.running?190:108,"仅屏幕预览",true);
        ui.bar=panel(ui.root,0,188,1,4,0x96b833,0);
        ui.frame=UINT32_MAX;break;
    case ERROR_PAGE:
        header("暂时无法完成",false);
        label(ui.root,20,65,280,ui.notice,false);
        button(ui.root,16,182,288,42,"返回",BACK,0,true,0xe4edbf);break;
    }
}
void rb_choreo_ui_open(lv_obj_t *parent,void (*on_exit)(void *),void *context,uint64_t now)
{
    if (ui.root) return;
    memset(&ui,0,sizeof(ui));ui.on_exit=on_exit;ui.context=context;ui.now=now;ui.selected=-1;
    ui.root=panel(parent,0,0,320,240,0xf7f5e9,0);
    lv_obj_add_flag(ui.root,LV_OBJ_FLAG_CLICKABLE);
    ui.page=LIBRARY;draw();
}
bool rb_choreo_ui_active(void) { return ui.root!=NULL; }
void rb_choreo_ui_fault(void)
{
    if (!ui.root || ui.faulted) return;
    ui.faulted=true;
    if (ui.player.running) { rb_choreo_player_stop(&ui.player,ui.now);draw(); }
}
void rb_choreo_ui_update(uint64_t now)
{
    if (!ui.root) return;
    ui.now=now;
    if (ui.command!=NONE) {
        enum command command=ui.command;int arg=ui.arg;ui.command=NONE;
        handle(command,arg);if (!ui.root) return;draw();
    }
    if (ui.page!=PREVIEW) return;
    bool running=ui.player.running;
    rb_choreo_player_tick(&ui.player,now);
    if (running && !ui.player.running) draw();
    const rb_choreo_sample_t *sample=&ui.player.sample;
    unsigned origin=sample->action_id==RB_ANIM_RECOVER?RB_POSE_FOLDED:RB_POSE_IDLE;
    unsigned clip_index=sample->action_id<RB_ANIM_COUNT?rb_clip_map[origin][sample->action_id]:
        sample->action_id<sizeof(rb_choreo_clip_map)/sizeof(rb_choreo_clip_map[0])?rb_choreo_clip_map[sample->action_id]:UINT8_MAX;
    if (clip_index==UINT8_MAX) {
        rb_choreo_player_stop(&ui.player,now);
        error_page("当前动作缺少动画，预览已停止。",EDITOR);draw();return;
    }
    const rb_clip_meta_t *clip=&rb_clips[clip_index];
    unsigned n=0;
    while (n+1<clip->count && clip->at[n+1]<=sample->clip_time_ms) ++n;
    unsigned frame=clip->frames[n];
    if (frame!=ui.frame) {
        const rb_frame_meta_t *meta=NULL;
        const lv_image_dsc_t *image=rb_assets_dance_frame(frame,&meta);
        if (!image || meta!=&rb_frames[frame]) {
            rb_choreo_player_stop(&ui.player,now);
            error_page("动画资源读取失败，预览已停止。",EDITOR);draw();return;
        }
        lv_image_set_src(ui.image,image);lv_obj_invalidate(ui.image);ui.frame=frame;
    }
    char value[128];
    const rb_choreo_action_t *action=rb_choreo_action(sample->action_id);
    const char *state=ui.faulted?"故障停止":!ui.player.running?(sample->done?"已结束":"已停止"):sample->transition?(sample->action_id==RB_CHOREO_START?"开场":"收尾"):sample->holding?"停留":"预览";
    unsigned steps=ui.work.count;
    snprintf(value,sizeof(value),"%s · %u/%u · %s  %.1f/%.1fs",state,
        sample->step_index==RB_CHOREO_NO_STEP?steps:(unsigned)sample->step_index+1u,steps,
        action?action->name:"待机",ui.player.elapsed_ms/1000.0,ui.plan.total_ms/1000.0);
    if(sample->step_index==RB_CHOREO_NO_STEP)
        snprintf(value,sizeof(value),"%s · %s  %.1f/%.1fs",state,
            ui.player.running?"自动播放":action->name,ui.player.elapsed_ms/1000.0,ui.plan.total_ms/1000.0);
    if (strcmp(lv_label_get_text(ui.progress),value)) lv_label_set_text(ui.progress,value);
    uint32_t elapsed=ui.player.elapsed_ms>ui.plan.total_ms?ui.plan.total_ms:ui.player.elapsed_ms;
    unsigned width=ui.plan.total_ms?(unsigned)(UINT64_C(320)*elapsed/ui.plan.total_ms):0;
    lv_obj_set_width(ui.bar,(int)(width?width:1));
}
