#include <assert.h>
#include "../../main/app/app_dance.c"
#include "../../main/app/app_arm.c"
static rb_dc_status_t board_status;
static unsigned plays,homes,pauses,daily_requests,tasks,sync_calls;
static uint64_t clock_ms=1000;
static uint8_t selected[32];static unsigned selected_count;static float selected_speed;
static rb_choreo_motion_backend_t registered;
void rb_choreo_ui_set_motion(const rb_choreo_motion_backend_t *backend) {registered=*backend;}
const char *esp_err_to_name(esp_err_t e) {(void)e;return "mock";}
uint32_t esp_random(void) {return 42;}
int xTaskCreate(void (*fn)(void *),const char *name,unsigned stack,void *arg,unsigned priority,TaskHandle_t *handle)
{(void)fn;(void)name;(void)stack;(void)arg;(void)priority;(void)handle;++tasks;return pdPASS;}
void vTaskDelay(TickType_t t) {(void)t;}
esp_err_t rb_board_read_enter(bool *pressed) {*pressed=false;return ESP_OK;}
uint64_t rb_platform_now_ms(void) {return clock_ms;}
void rb_ui_arm_status(const char *s,bool active) {(void)s;(void)active;}
void rb_ui_set_arm_control(void (*fn)(void)) {(void)fn;}
esp_err_t rb_arm_daily_start(void) {return ESP_OK;}
esp_err_t rb_arm_daily_status(rb_dc_status_t *s) {*s=board_status;return ESP_OK;}
void rb_arm_daily_home(void) {++homes;}
void rb_arm_daily_pause(void) {++pauses;}
void rb_arm_daily_heartbeat(uint64_t t) {(void)t;}
esp_err_t rb_arm_daily_request(unsigned action,unsigned request)
{++daily_requests;board_status.action=action;board_status.request_id=request;return ESP_OK;}
const char *rb_daily_control_state_name(rb_dc_state_t state) {(void)state;return "mock";}
bool rb_daily_sync_screen(rb_pet_t *p,const rb_dc_status_t *s) {(void)p;(void)s;++sync_calls;return false;}
esp_err_t rb_arm_dance_request(const uint8_t *numbers,unsigned count,float speed,unsigned request,float *seconds)
{
    static rb_dance_plan_t plan;assert(rb_dance_compile(&plan,numbers,count,speed));
    ++plays;memcpy(selected,numbers,count);selected_count=count;selected_speed=speed;*seconds=plan.duration_s;
    board_status=(rb_dc_status_t){.state=RB_DC_PLAY,.action=RB_DANCE_CONTROL_ACTION,.enabled=true,.request_id=request};return ESP_OK;
}
static void input(const char *s) {char line[256];snprintf(line,sizeof(line),"%s",s);command(line);}
int main(void)
{
    rb_pet_t p;rb_app_arm_init(&p);rb_pet_init_sleep(&p,clock_ms);rb_app_arm_start_input();
    assert(tasks==2 && !plays && !daily_requests);
    assert(registered.play && registered.status && registered.stop && registered.home);
    input("dance random");assert(plays==1 && selected_count==5);
    uint8_t repeat[5];memcpy(repeat,selected,5);input("dance random 5 42");assert(!memcmp(repeat,selected,5));
    input("dance speed 50");input("dance play 01 08 07");
    assert(selected_count==3 && selected[0]==1 && selected[1]==8 && selected[2]==7 && selected_speed==.5f);
    unsigned before=plays;
    input("dance random 0");input("dance random 11");input("dance play 00");input("dance play 01 nope");
    input("dance random 5 -1");input("dance random 5 4294967296");input("dance play");assert(plays==before);
    p.suspended=true;rb_app_arm_tick(clock_ms);assert(!homes && !sync_calls && p.state==RB_PET_STOPPED);
    rb_pet_event_t e={.action_id=RB_ANIM_WAKE,.request_id=7};rb_app_arm_action(NULL,&e);assert(!daily_requests);
    input("dance stop");input("dance home");assert(pauses==1 && homes==1);
    board_status.state=RB_DC_OFF;board_status.enabled=false;board_status.disable_confirmed=true;
    rb_app_arm_tick(clock_ms);assert(p.state==RB_PET_RESTING);
    rb_pet_t saved=p;clock_ms+=20;rb_app_arm_tick(clock_ms);assert(!memcmp(&p,&saved,sizeof(p)));
    rb_app_arm_action(NULL,&e);assert(daily_requests==1 && board_status.action==RB_DAILY_WAKE);
    rb_choreo_work_t work;rb_choreo_work_init(&work,0);
    work.count=2;work.steps[0]=(rb_choreo_step_t){.action_id=RB_CHOREO_DANCE_STEADY,.repeat=2};
    work.steps[1]=(rb_choreo_step_t){.action_id=RB_CHOREO_DANCE_NOD,.repeat=1};
    float duration;assert(!registered.play(&work,&duration));
    assert(selected_count==3 && selected[0]==8 && selected[1]==8 && selected[2]==1 && selected_speed==1.f);
    rb_choreo_motion_status_t status;assert(registered.status(&status) && status.active);
    board_status.state=RB_DC_HOLD;assert(registered.status(&status) && status.finished && !status.active);
    work.tempo=RB_CHOREO_SLOW;assert(!registered.play(&work,&duration) && selected_speed==.5f);
    before=plays;work.tempo=RB_CHOREO_FAST;assert(registered.play(&work,&duration));
    work.tempo=RB_CHOREO_STANDARD;work.steps[0].hold_ms=300;assert(registered.play(&work,&duration));
    work.steps[0]=(rb_choreo_step_t){.action_id=RB_CHOREO_WAIT,.repeat=1,.duration_ms=600};
    assert(registered.play(&work,&duration) && plays==before);
    puts("PASS serial random/manual commands, no startup playback, Daily exclusion and one-time screen reset");return 0;
}
