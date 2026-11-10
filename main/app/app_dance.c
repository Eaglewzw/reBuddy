/* Touch and serial submissions. Motor I/O remains in the shared Daily task. */
#include "app_dance.h"
#include "choreo_motion.h"
#include "sdkconfig.h"
#ifdef CONFIG_REBUDDY_ARM_DANCE_CONTROL
#include "rb_arm_daily.h"
#include "esp_random.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <stdatomic.h>
static float speed=1.f;
static atomic_uint request_id=0x80000000u;
static unsigned screen_request;
static unsigned next_request(void) {return atomic_fetch_add(&request_id,1)+1;}
static const char *screen_play(const rb_choreo_work_t *work,float *seconds)
{
    if(rb_choreo_validate(work,NULL)!=RB_CHOREO_OK || !work->count)return "请先添加有效动作。";
    if(work->tempo==RB_CHOREO_FAST)return "实机暂不支持快节奏，请选择慢或标准。";
    uint8_t numbers[RB_DANCE_MAX_STEPS];unsigned count=0;
    for(unsigned i=0;i<work->count;++i) {
        const rb_choreo_step_t *step=&work->steps[i];
        if(step->action_id<RB_CHOREO_DANCE_NOD || step->action_id>RB_CHOREO_DANCE_PIVOT || step->hold_ms)
            return "实机连续编舞暂不支持停留，请移除停留积木并将结束停留设为零。";
        for(unsigned r=0;r<step->repeat;++r) {
            if(count==RB_DANCE_MAX_STEPS)return "动作次数太多，请减少重复。";
            numbers[count++]=(uint8_t)(step->action_id-RB_CHOREO_DANCE_NOD+1);
        }
    }
    unsigned request=next_request();
    esp_err_t result=rb_arm_dance_request(numbers,count,work->tempo==RB_CHOREO_SLOW?.5f:1.f,request,seconds);
    if(result!=ESP_OK)return "机械臂忙碌或处于保护状态，请先回零，再重试。";
    screen_request=request;return NULL;
}
static bool screen_status(rb_choreo_motion_status_t *out)
{
    rb_dc_status_t s;
    if(rb_arm_daily_status(&s)!=ESP_OK)return false;
    *out=(rb_choreo_motion_status_t){.state="正在准备",.error=s.error,
        .elapsed_s=s.dance_seconds,.phase=s.phase,.number=s.dance_number,
        .bridge=s.dance_transition,.active=true};
    if(s.request_id!=screen_request) {
        out->state="控制任务已切换";out->active=false;out->finished=true;return true;
    }
    switch(s.state) {
    case RB_DC_PLAY:out->state="正在播放";break;
    case RB_DC_ENTRY:case RB_DC_ENTRY_SETTLE:out->state="正在前往待机";break;
    case RB_DC_HOLD:out->state="播放完成 · 保持待机";out->active=false;out->finished=true;break;
    case RB_DC_PROTECTIVE_HOLD:out->state="已停止 · 保护保持";out->active=false;break;
    case RB_DC_HOME:case RB_DC_HOME_SETTLE:out->state="正在回零";break;
    case RB_DC_OFF:out->state="已退出";out->active=false;out->finished=true;break;
    case RB_DC_FAULT:out->state="控制故障";out->active=false;break;
    case RB_DC_BRAKING:case RB_DC_HOLD_RAMP:out->state="正在制动保持";break;
    default:break;
    }
    return true;
}
static const rb_choreo_motion_backend_t screen_backend={screen_play,screen_status,rb_arm_daily_pause,rb_arm_daily_home};
static bool integer(const char *s,uint32_t *out)
{
    if(!s || !*s || *s=='-')return false;
    char *end;errno=0;unsigned long long n=strtoull(s,&end,10);
    if(errno || *end || n>UINT32_MAX)return false;
    *out=(uint32_t)n;return true;
}
static void help(void)
{
    printf("\nDance: dance play 01 08 07 | dance random [1..10] [seed]\n"
           "dance speed 50|100 | dance status | dance stop | dance home\n"
           "No boot motion. Completion holds idle; HOME returns to zero. Ctrl+C holds.\n");
}
static void command(char *line)
{
    char *words[36],*save=NULL;unsigned argc=0;
    for(char *p=strtok_r(line," \t",&save);p;p=strtok_r(NULL," \t",&save)) {
        if(argc==36) {puts("Too many arguments");return;}words[argc++]=p;
    }
    if(!argc)return;
    if(strcmp(words[0],"dance") || argc<2) {help();return;}
    const char *cmd=words[1];rb_dc_status_t status;
    if(!strcmp(cmd,"stop") && argc==2) {rb_arm_daily_pause();puts("Protective hold requested");return;}
    if(!strcmp(cmd,"home") && argc==2) {rb_arm_daily_home();puts("Zero return requested");return;}
    if(!strcmp(cmd,"status") && argc==2) {
        if(rb_arm_daily_status(&status)!=ESP_OK) {puts("Controller unavailable");return;}
        printf("%s enabled=%d action=%u dance=%02u bridge=%d t=%.2fs phase=%.1f%% error=%s\n",
               rb_daily_control_state_name(status.state),status.enabled,status.action,status.dance_number,
               status.dance_transition,(double)status.dance_seconds,(double)status.phase*100,status.error?status.error:"none");return;
    }
    if(!strcmp(cmd,"speed") && argc==3) {
        uint32_t value;
        if(!integer(words[2],&value) || (value!=50 && value!=100)) {puts("Use speed 50 or 100");return;}
        speed=value/100.f;printf("Next sequence speed: %lu%%\n",(unsigned long)value);return;
    }
    uint8_t numbers[RB_DANCE_MAX_STEPS];unsigned count=0;
    if(!strcmp(cmd,"random") && argc<=4) {
        uint32_t n=5,seed=esp_random();
        if((argc>=3 && !integer(words[2],&n)) || (argc==4 && !integer(words[3],&seed)) ||
           !rb_dance_random(numbers,n,seed)) {puts("Use dance random 1..10 [seed]");return;}
        count=n;printf("Seed: %lu (board XorShift32)\n",(unsigned long)seed);
    } else if(!strcmp(cmd,"play") && argc>=3 && argc<=34) {
        count=argc-2;
        for(unsigned i=0;i<count;++i) {
            uint32_t n;if(!integer(words[i+2],&n) || n<1 || n>10) {puts("Actions: 01..10");return;}
            numbers[i]=(uint8_t)n;
        }
    } else {help();return;}
    printf("Sequence:");for(unsigned i=0;i<count;++i)printf(" %02u",numbers[i]);putchar('\n');fflush(stdout);
    float duration=0;
    esp_err_t result=rb_arm_dance_request(numbers,count,speed,next_request(),&duration);
    if(result==ESP_OK)printf("Accepted %.2fs at %.0f%%; completion holds. dance stop / dance home\n",(double)duration,(double)speed*100);
    else printf("Rejected: %s; stop/home any active motion first\n",esp_err_to_name(result));
}
static void worker(void *unused)
{
    (void)unused;char line[256];unsigned used=0;bool overflow=false;help();
    for(;;) {
        int c=getchar();
        if(c==EOF) {clearerr(stdin);vTaskDelay(pdMS_TO_TICKS(20));continue;}
        if(c==3) {rb_arm_daily_pause();used=0;overflow=false;puts("Protective hold requested");continue;}
        if(c=='\r' || c=='\n') {
            if(overflow)puts("Command too long");else if(used) {line[used]=0;command(line);}
            used=0;overflow=false;
        } else if(c==8 || c==127) {if(used)--used;}
        else if(c>=32 && c<127 && !overflow) {
            if(used+1<sizeof(line))line[used++]=(char)c;else overflow=true;
        }
    }
}
void rb_app_dance_console_start(void)
{
    rb_choreo_ui_set_motion(&screen_backend);
    if(xTaskCreate(worker,"dance_console",6144,NULL,2,NULL)!=pdPASS)
        ESP_LOGE("rb_dance","Serial command task unavailable");
}
#else
void rb_app_dance_console_start(void) {rb_choreo_ui_set_motion(NULL);}
#endif
