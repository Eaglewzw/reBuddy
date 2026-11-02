#pragma once
#include <stdbool.h>
#include <stdint.h>

/* Pet IDs stay stable (4/5 are retired); SLEEP is a resting-state binding,
 * not a new touch action. GIF file order is deliberately a separate field. */
enum { RB_DAILY_IDLE=0, RB_DAILY_PROBE=1, RB_DAILY_NUZZLE=2, RB_DAILY_TILT=3,
       RB_DAILY_HAPPY=6, RB_DAILY_REST=7, RB_DAILY_WAKE=8, RB_DAILY_SLEEP=9 };
enum { RB_DAILY_IDLE_ENABLED, RB_DAILY_ZERO_ENABLED, RB_DAILY_ZERO_DISABLED };
typedef struct { float begin_s,end_s; float coefficient[7][6]; } rb_daily_piece_t;
typedef struct { float motion_s,clip_ms; } rb_daily_sync_t;
typedef struct {
    uint8_t id,gif_index,end_state;
    bool loop;
    const char *name,*gif_file;
    float duration_s;
    uint32_t duration_ms;
    const rb_daily_piece_t *pieces;
    uint16_t piece_count;
    const rb_daily_sync_t *sync;
    uint8_t sync_count;
    uint16_t clip_duration_ms;
    float anchor_rad[7];
} rb_daily_action_t;
typedef struct {
    float kp[7],kd[7],torque_budget_nm[7];
    float position_speed,position_acceleration,exit_speed,exit_acceleration;
    float idle_tolerance_deg,gripper_tolerance_deg,zero_tolerance_deg;
    uint32_t control_period_ms,feedback_max_age_ms;
} rb_daily_policy_t;
extern const rb_daily_policy_t rb_daily_policy;
extern const char rb_daily_calibration_sha256[],rb_daily_urdf_sha256[];
extern const rb_daily_action_t rb_daily_actions[8];
const rb_daily_action_t *rb_daily_find(unsigned id);
const rb_daily_action_t *rb_daily_for_screen(unsigned pet_action,unsigned pose);
/* Pure offline plan evaluation: radians, rad/s, rad/s^2. Never sends CAN,
 * enables a motor, or treats the planned endpoint as measured arrival. */
bool rb_daily_sample(unsigned id,float seconds,float q[7],float v[7],float a[7]);
/* One shared plan phase drives the authored GIF clock. */
uint16_t rb_daily_clip_time(unsigned id,float phase);
