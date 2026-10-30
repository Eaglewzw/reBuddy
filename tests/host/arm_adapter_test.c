// SPDX-License-Identifier: Apache-2.0
#include <assert.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "rb_arm.h"
#include "rb_board.h"
#include "can_console_bsp.h"
#include "can_console_can.h"
#include "can_console_motor_client.h"
#include "can_console_motor_runtime.h"
#include "can_console_arm_motion.h"

/* Only these initialization/read functions are linked. Adding an admin,
 * target, scan or control-tick call to the adapter fails this executable. */
static unsigned calls, fail_at, bound, starts, reads;
static bool standby=true;
static int scenario;
static bool fail_model, fail_pins;
static can_console_motor_state_t motors[7];
static can_console_motor_bus_state_t bus={.started=true,.status=CAN_CONSOLE_MOTOR_BUS_ACTIVE};
static const can_console_bsp_can_pins_t pins={5,3};
static esp_err_t step(void) { return ++calls==fail_at?ESP_FAIL:ESP_OK; }
int64_t esp_timer_get_time(void) { return 1000000; }
esp_err_t rb_board_can1_standby(bool value)
{
    esp_err_t e=step();
    if(e==ESP_OK)standby=value;
    return e;
}
bool can_console_ik_select_model(can_console_ik_model_t m)
{ assert(m==CAN_CONSOLE_IK_MODEL_REBOTARM_RS);return !fail_model; }
const can_console_bsp_can_pins_t *can_console_bsp_get_can_pins(uint8_t ch)
{ assert(ch==0); return fail_pins?NULL:&pins; }
void can_console_motor_config_set_defaults(can_console_motor_config_t *c)
{ memset(c,0,sizeof(*c));c->apply_default_id_map_at_boot=true; }
esp_err_t can_console_motor_init(const can_console_motor_config_t *c)
{ assert(c->max_motors==7 && !c->apply_default_id_map_at_boot && c->control_period_ms==20);return step(); }
esp_err_t can_console_motor_add_bus(const can_console_motor_bus_config_t *c,int *ch)
{
    assert(c->mode==CAN_CONSOLE_CAN_MODE_LISTEN_ONLY && c->bitrate==1000000);
    assert(c->data_bitrate==0 && c->controller_id==0 && c->tx_gpio==5 && c->rx_gpio==3);
    *ch=0;return step();
}
esp_err_t can_console_motor_bind(uint8_t id,const char *vendor,const char *model,int ch)
{
    assert(id==++bound && ch==0 && !strcmp(vendor,"robstride"));
    assert(!strcmp(model,id<=3?"rs-06":"rs-00"));return step();
}
void can_console_arm_motion_config_set_defaults(can_console_arm_motion_config_t *c)
{ memset(c,0,sizeof(*c)); }
esp_err_t can_console_arm_motion_init(const can_console_arm_motion_config_t *c)
{
    assert(c->sample_period_ms==20);
    for(unsigned i=0;i<6;++i)assert(c->motor_ids[i]==i+1);
    return step();
}
esp_err_t can_console_arm_motion_start(void) { return step(); }
esp_err_t can_console_motor_start_bus(int ch)
{ assert(ch==0 && standby); ++starts;return step(); }
esp_err_t can_console_motor_get_bus_state(int ch,int32_t window,can_console_motor_bus_state_t *out)
{ assert(ch==0 && window==100);if(scenario==1)return ESP_FAIL; *out=bus;return ESP_OK; }
esp_err_t can_console_motor_get_state(uint8_t id,can_console_motor_state_t *out)
{ assert(id>=1 && id<=7);++reads; *out=motors[id-1];return scenario==2?ESP_FAIL:ESP_OK; }
static void valid_feedback(void)
{
    for(unsigned i=0;i<7;++i) {
        motors[i]=(can_console_motor_state_t){.motor_id=i+1,.channel=0,.online=true,
            .last_feedback_us=990000,.position=.1f*i,.velocity=.02f,.torque=.3f};
        strcpy(motors[i].vendor_name,"robstride");
    }
}
int main(int argc,char **argv)
{
    assert(argc==2);
    rb_arm_status_t status;
    assert(rb_arm_get_status(NULL)==ESP_ERR_INVALID_ARG);
    assert(rb_arm_get_status(&status)==ESP_ERR_INVALID_STATE);
    assert(status.state==RB_ARM_UNINITIALIZED && isnan(status.position_rad[6]));
    bool monitor=strcmp(argv[1],"standby")!=0;
    if(!strncmp(argv[1],"fail-",5))fail_at=(unsigned)atoi(argv[1]+5);
    /* Init has 14 fallible calls; index 15 exercises successful boot. */
    fail_model=!strcmp(argv[1],"fail-model");
    fail_pins=!strcmp(argv[1],"fail-pins");
    esp_err_t error=rb_arm_init(monitor);
    if(fail_model || fail_pins) {
        assert(error==ESP_ERR_INVALID_STATE && standby);
        assert(rb_arm_get_status(&status)==ESP_ERR_INVALID_STATE && status.state==RB_ARM_FAULT);
        assert(rb_arm_init(false)==ESP_ERR_INVALID_STATE);
        return 0;
    }
    if(fail_at && fail_at<=14) {
        assert(error==ESP_FAIL && standby);
        assert(rb_arm_get_status(&status)==ESP_FAIL && status.state==RB_ARM_FAULT);
        assert(status.feedback_valid_mask==0 && isnan(status.position_rad[0]));
    } else {
        assert(error==ESP_OK && bound==7);
        assert(starts==(unsigned)monitor && standby==!monitor);
        assert(rb_arm_get_status(&status)==ESP_OK && status.planner_ready);
        assert(status.state==(monitor?RB_ARM_MONITORING:RB_ARM_STANDBY));
        if(!monitor)assert(reads==0 && !status.bus_started);
    }
    unsigned previous=calls;
    assert(rb_arm_init(false)==ESP_ERR_INVALID_STATE && previous==calls);
    if(strcmp(argv[1],"feedback"))return 0;
    valid_feedback();
    assert(rb_arm_get_status(&status)==ESP_OK && status.feedback_valid_mask==0x7f);
    assert(fabsf(status.position_rad[6]-.6f)<1e-6f);
    motors[0].last_feedback_us=899999; // stale
    motors[1].last_feedback_us=1000001; // future
    motors[2].position=NAN;
    motors[3].channel=1;
    motors[4].online=false;
    motors[5].last_feedback_us=0;
    motors[6].last_feedback_us=900000; // inclusive age boundary
    motors[6].fault_raw=8;
    assert(rb_arm_get_status(&status)==ESP_OK && status.feedback_valid_mask==0x40);
    assert(status.fault[6]==8 && isnan(status.position_rad[0]));
    valid_feedback();motors[0].velocity=INFINITY;motors[1].torque=NAN;
    strcpy(motors[2].vendor_name,"damiao");motors[3].motor_id=7;
    assert(rb_arm_get_status(&status)==ESP_OK && status.feedback_valid_mask==0x70);
    bus.status=CAN_CONSOLE_MOTOR_BUS_OFF;
    assert(rb_arm_get_status(&status)==ESP_OK && !status.bus_healthy && !status.feedback_valid_mask);
    assert(isnan(status.position_rad[6]));
    bus.status=CAN_CONSOLE_MOTOR_BUS_ACTIVE;scenario=1;
    assert(rb_arm_get_status(&status)==ESP_FAIL && status.error==ESP_FAIL);
    scenario=2;assert(rb_arm_get_status(&status)==ESP_OK && !status.feedback_valid_mask);
    return 0;
}
