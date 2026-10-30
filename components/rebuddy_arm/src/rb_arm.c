// SPDX-License-Identifier: Apache-2.0
#include "rb_arm.h"
#include <math.h>
#include <string.h>
#include "rb_board.h"
#include "can_console_bsp.h"
#include "can_console_can.h"
#include "can_console_motor_runtime.h"
#include "can_console_motor_client.h"
#include "can_console_arm_motion.h"
#include "esp_timer.h"

static rb_arm_state_t state;
static esp_err_t failure;
static int channel = -1;
static bool planner_ready;

static esp_err_t fail(esp_err_t error)
{
    state=RB_ARM_FAULT;
    failure=error;
    /* No active-mode controller is created. Quarantine a partial bus start
       behind the transceiver standby pin; upstream has no runtime deinit. */
    (void)rb_board_can1_standby(true);
    return error;
}

esp_err_t rb_arm_init(bool monitor)
{
    if (state!=RB_ARM_UNINITIALIZED) return ESP_ERR_INVALID_STATE;
    state=RB_ARM_FAULT; /* No retries into partly initialized singletons. */
    esp_err_t error=rb_board_can1_standby(true);
    if (error!=ESP_OK) return fail(error);
    if (!can_console_ik_select_model(CAN_CONSOLE_IK_MODEL_REBOTARM_RS))
        return fail(ESP_ERR_INVALID_STATE);
    can_console_motor_config_t motor;
    can_console_motor_config_set_defaults(&motor);
    motor.max_motors=RB_ARM_CHANNELS;
    motor.apply_default_id_map_at_boot=false;
    motor.control_period_ms=20;
    error=can_console_motor_init(&motor);
    if (error!=ESP_OK) return fail(error);
    const can_console_bsp_can_pins_t *pins=can_console_bsp_get_can_pins(0);
    if (!pins) return fail(ESP_ERR_INVALID_STATE);
    const can_console_motor_bus_config_t bus={
        .tx_gpio=pins->tx_gpio, .rx_gpio=pins->rx_gpio, .controller_id=0,
        .bitrate=1000000, .data_bitrate=0, .mode=CAN_CONSOLE_CAN_MODE_LISTEN_ONLY
    };
    error=can_console_motor_add_bus(&bus,&channel);
    if (error!=ESP_OK) return fail(error);
    for (unsigned id=1;id<=RB_ARM_CHANNELS;++id) {
        error=can_console_motor_bind(id,"robstride",id<=3?"rs-06":"rs-00",channel);
        if (error!=ESP_OK) return fail(error);
    }
    can_console_arm_motion_config_t motion;
    can_console_arm_motion_config_set_defaults(&motion);
    motion.sample_period_ms=20;
    for (unsigned i=0;i<RB_ARM_JOINTS;++i) motion.motor_ids[i]=i+1;
    error=can_console_arm_motion_init(&motion);
    if (error!=ESP_OK) return fail(error);
    error=can_console_arm_motion_start();
    if (error!=ESP_OK) return fail(error);
    planner_ready=true;
    if (monitor) {
        error=can_console_motor_start_bus(channel);
        if (error!=ESP_OK) return fail(error);
        error=rb_board_can1_standby(false);
        if (error!=ESP_OK) return fail(error);
    }
    state=monitor?RB_ARM_MONITORING:RB_ARM_STANDBY;
    return ESP_OK;
}

esp_err_t rb_arm_get_status(rb_arm_status_t *out)
{
    if (!out) return ESP_ERR_INVALID_ARG;
    memset(out,0,sizeof(*out));
    out->state=state;out->error=failure;out->planner_ready=planner_ready;
    const int64_t now=esp_timer_get_time();
    out->timestamp_us=now>0?(uint64_t)now:0;
    for (unsigned i=0;i<RB_ARM_CHANNELS;++i)
        out->position_rad[i]=out->velocity_rad_s[i]=out->torque_nm[i]=NAN;
    if (state==RB_ARM_UNINITIALIZED) return ESP_ERR_INVALID_STATE;
    if (state==RB_ARM_FAULT) return failure;
    if (state!=RB_ARM_MONITORING) return ESP_OK;
    can_console_motor_bus_state_t bus;
    esp_err_t error=can_console_motor_get_bus_state(channel,100,&bus);
    if (error!=ESP_OK) { out->error=error;return error; }
    out->bus_started=bus.started;
    out->bus_healthy=bus.started && (bus.status==CAN_CONSOLE_MOTOR_BUS_ACTIVE ||
                                    bus.status==CAN_CONSOLE_MOTOR_BUS_WARNING);
    if (!out->bus_healthy) return ESP_OK;
    for (unsigned i=0;i<RB_ARM_CHANNELS;++i) {
        can_console_motor_state_t m;
        if (can_console_motor_get_state(i+1,&m)!=ESP_OK) continue;
        out->fault[i]=m.fault_raw;
        if (m.motor_id!=i+1 || m.channel!=channel || !m.online ||
            strcmp(m.vendor_name,"robstride")!=0 ||
            m.last_feedback_us<=0 || m.last_feedback_us>now ||
            now-m.last_feedback_us>RB_ARM_FEEDBACK_MAX_AGE_US ||
            !isfinite(m.position) || !isfinite(m.velocity) || !isfinite(m.torque)) continue;
        out->feedback_valid_mask|=(uint8_t)(1u<<i);
        out->position_rad[i]=m.position;
        out->velocity_rad_s[i]=m.velocity;
        out->torque_nm[i]=m.torque;
    }
    return ESP_OK;
}
