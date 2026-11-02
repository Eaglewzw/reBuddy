#pragma once
#include "rb_daily.h"
#include "can_console_arm_gravity.h"
#include "can_console_motor_client.h"
/* Serialized by one application task. Motor service owns protocol and TX. */
typedef enum { RB_DC_OFF, RB_DC_PRECHECK, RB_DC_MODE, RB_DC_ENABLE, RB_DC_RAMP,
    RB_DC_ENTRY, RB_DC_ENTRY_SETTLE, RB_DC_PLAY, RB_DC_SETTLE, RB_DC_HOLD,
    RB_DC_ZERO_HOLD, RB_DC_PROTECTIVE_HOLD, RB_DC_HOME, RB_DC_HOME_SETTLE,
    RB_DC_DISABLING, RB_DC_FAULT, RB_DC_BRAKING, RB_DC_HOLD_RAMP } rb_dc_state_t;
typedef struct {
    bool valid,torque_checked;
    rb_dc_state_t source_state;
    uint64_t at_ms;
    float measured[7],target[7],velocity[7],estimated_torque[7];
} rb_dc_hold_snapshot_t;
typedef struct {
    rb_dc_state_t state;
    unsigned action,request_id,completed_id;
    float phase;
    bool enabled,disable_confirmed;
    const char *error;
    const char *stop_error;
    unsigned bad_motor;
    /* Latched before protective hold replaces the failed target with measured
     * position. Reading it later must not erase the original arrival error. */
    rb_dc_hold_snapshot_t hold;
} rb_dc_status_t;
typedef struct {
    rb_dc_status_t status;
    can_console_arm_gravity_t gravity;
    can_console_motor_state_t motors[7];
    int channel;
    bool initialized,owned,pending,disable_sent,home_requested,pause_requested;
    uint32_t sequence,fence;
    uint64_t now,changed,last_tick,sent,still_since,report_at,ui_at,enable_at;
    float elapsed,queued_elapsed,duration;
    float anchor[7],target[7],from[7],to[7],lower[7],upper[7];
    float previous_q[7],estimated_v[7];
    float estimated_torque[7];
    float brake_kd[7];
    uint64_t recovery_at,previous_sample_ms[7];
    int64_t previous_feedback[7];
} rb_daily_control_t;
esp_err_t rb_daily_control_init(rb_daily_control_t *c,int channel);
esp_err_t rb_daily_control_request(rb_daily_control_t *c,unsigned action,unsigned request_id);
void rb_daily_control_home(rb_daily_control_t *c);
void rb_daily_control_pause(rb_daily_control_t *c);
void rb_daily_control_tick(rb_daily_control_t *c,uint64_t now_ms);
/* UI must heartbeat even while screen idle. Loss freezes into protective hold. */
void rb_daily_control_heartbeat(rb_daily_control_t *c,uint64_t now_ms);
const char *rb_daily_control_state_name(rb_dc_state_t state);
