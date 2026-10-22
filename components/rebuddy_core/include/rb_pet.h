#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "rb_pet_config.h"

typedef enum { RB_PET_NORMAL, RB_PET_FOLDED, RB_PET_RESTING, RB_PET_STOPPED } rb_pet_state_t;
typedef enum { RB_POSE_IDLE, RB_POSE_FOLDED, RB_POSE_REST } rb_pet_pose_t;
typedef enum { RB_ANIM_IDLE, RB_ANIM_PROBE, RB_ANIM_NUZZLE, RB_ANIM_TILT,
    RB_ANIM_WITHDRAW, RB_ANIM_RECOVER, RB_ANIM_HAPPY, RB_ANIM_REST, RB_ANIM_WAKE, RB_ANIM_COUNT } rb_pet_action_t;
typedef enum { RB_INPUT_TAP, RB_INPUT_STROKE, RB_INPUT_SWIPE, RB_INPUT_POKE,
    RB_INPUT_IDLE, RB_INPUT_WAKE, RB_INPUT_RESET, RB_INPUT_SETTINGS } rb_pet_source_t;
typedef struct {
    rb_pet_action_t action_id;
    uint32_t request_id;
    rb_pet_source_t source;
    rb_pet_state_t target_state;
    bool lazy;
    uint64_t timestamp_ms;
} rb_pet_event_t;
typedef void (*rb_pet_sink_t)(void *,const rb_pet_event_t *);
typedef struct { uint64_t at; uint16_t duration, claw_duration; } rb_valid_interval_t;
typedef struct {
    rb_pet_state_t state;
    rb_pet_pose_t pose, origin;
    rb_pet_action_t action, pending;
    rb_pet_source_t pending_source;
    bool lazy, active_lazy, pending_valid, returning, estop_released;
    int direction, active_direction, pending_direction;
    uint64_t now, started, duration, return_started, return_duration, last_interaction, nuzzle_hold;
    float phase, return_phase;
    uint32_t requests, completed;
    unsigned nuzzle_cycles;
    rb_pet_event_t last_event;
    rb_pet_sink_t sink;
    void *sink_context;
    const char *stop_reason;
    bool raw_down, contact, blocked, consumed, swiped, stroke;
    int raw_x,raw_y,x,y,start_x,start_y,previous_raw_x,previous_raw_y;
    uint64_t raw_changed, down_at, sampled, last_motion, last_gentle, release_at;
    float path, max_offset, speed, candidate_path;
    unsigned fast_ms, moving_ms, candidate_ms, soothe_ms;
    unsigned claw_moving_ms;
    int16_t claw_region[4][2];
    struct { int x,y; uint64_t at; } positions[8];
    unsigned position_count;
    rb_valid_interval_t candidate[64];
    unsigned candidate_count;
    uint64_t taps[3];
    unsigned tap_count;
    rb_valid_interval_t happy_history[512];
    unsigned happy_head,happy_count,happy_ms;
    bool happy_latched;
    uint64_t happy_cooldown_until;
    bool blink;
    uint64_t next_blink,blink_until;
    uint32_t random;
} rb_pet_t;

void rb_pet_init(rb_pet_t *p,uint64_t now);
void rb_pet_set_sink(rb_pet_t *p,rb_pet_sink_t sink,void *context);
void rb_pet_input(rb_pet_t *p,bool down,int x,int y,uint64_t now);
/* Convex claw polygon in screen coordinates, supplied by the displayed pose. */
void rb_pet_set_claw_region(rb_pet_t *p,const int16_t points[4][2]);
void rb_pet_cancel(rb_pet_t *p);
void rb_pet_tick(rb_pet_t *p,uint64_t now);
void rb_pet_stop(rb_pet_t *p,const char *reason);
bool rb_pet_reset(rb_pet_t *p,uint64_t now);
void rb_pet_estop(rb_pet_t *p,bool released);
void rb_pet_rest(rb_pet_t *p);
/* Explicit wake request; ordinary contact never wakes a resting pet. */
void rb_pet_wake(rb_pet_t *p);
const char *rb_pet_action_name(rb_pet_action_t action);
