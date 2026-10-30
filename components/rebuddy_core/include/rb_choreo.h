#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define RB_CHOREO_SCHEMA_VERSION 1u
#define RB_CHOREO_MAX_STEPS 8u
#define RB_CHOREO_SLOTS 10u
#define RB_CHOREO_NAME_BYTES 32u
#define RB_CHOREO_MAX_EVENTS 40u
#define RB_CHOREO_HOLD_MAX_MS 3000u
#define RB_CHOREO_WAIT_MIN_MS 300u
#define RB_CHOREO_WAIT_MAX_MS 3000u
/* Storage accepts legacy 500ms multiples; new UI edits use half a 100 BPM beat. */
#define RB_CHOREO_TIME_STEP_MS 100u
#define RB_CHOREO_HALF_BEAT_MS 300u
#define RB_CHOREO_NO_STEP UINT8_MAX

typedef enum { RB_CHOREO_SLOW, RB_CHOREO_STANDARD, RB_CHOREO_FAST, RB_CHOREO_TEMPO_COUNT } rb_choreo_tempo_t;
/* IDs 1..6 match rb_pet_action_t; 7/8 remain exclusively rest/wake. */
typedef enum {
    RB_CHOREO_PROBE=1, RB_CHOREO_NUZZLE=2, RB_CHOREO_TILT=3,
    RB_CHOREO_WITHDRAW=4, RB_CHOREO_RECOVER=5, RB_CHOREO_HAPPY=6,
    RB_CHOREO_WAIT=15,
    /* Retired dance IDs 9..14 and 16..22 are reserved, never aliased. */
    RB_CHOREO_GROOVE_BOUNCE=23,
    RB_CHOREO_GROOVE_ROCK=24,
    RB_CHOREO_GROOVE_SIDE=25,
    RB_CHOREO_DANCE_POINT=26,
    RB_CHOREO_ARM_WAVE=27,
    RB_CHOREO_DANCE_ROLL=28,
    RB_CHOREO_WRIST_ISOLATION=29,
    RB_CHOREO_DANCE_TUT=30,
    RB_CHOREO_DANCE_HIT=31,
    RB_CHOREO_DIME_STOP=32,
    /* User-selected 2D library; retired 23..32 are never reused. */
    RB_CHOREO_2D_NOD=33,
    RB_CHOREO_2D_LIFT=34,
    RB_CHOREO_2D_DIP=35,
    RB_CHOREO_2D_REACH=36,
    RB_CHOREO_2D_BASE_TURN=37,
    RB_CHOREO_2D_SWAY=38,
    RB_CHOREO_2D_ELBOW=39,
    RB_CHOREO_2D_WRIST=40,
    RB_CHOREO_2D_GRIP=41,
    RB_CHOREO_2D_WAVE=42,
    RB_CHOREO_START=43, RB_CHOREO_END=44
} rb_choreo_action_id_t;
typedef enum { RB_CHOREO_INTERACTION, RB_CHOREO_EMOTION, RB_CHOREO_PERFORMANCE, RB_CHOREO_CONTROL } rb_choreo_group_t;
typedef enum { RB_CHOREO_POSE_IDLE, RB_CHOREO_POSE_FOLDED } rb_choreo_pose_t;
typedef enum {
    RB_CHOREO_OK, RB_CHOREO_INVALID_ARGUMENT, RB_CHOREO_BAD_VERSION,
    RB_CHOREO_BAD_WORK_ID, RB_CHOREO_BAD_NAME, RB_CHOREO_BAD_TEMPO,
    RB_CHOREO_TOO_MANY_STEPS, RB_CHOREO_EMPTY, RB_CHOREO_UNKNOWN_ACTION,
    RB_CHOREO_UNAVAILABLE, RB_CHOREO_BAD_REPEAT, RB_CHOREO_BAD_HOLD,
    RB_CHOREO_BAD_DURATION, RB_CHOREO_BAD_INDEX, RB_CHOREO_TOO_MANY_EVENTS
} rb_choreo_status_t;
typedef struct {
    uint8_t action_id, repeat;
    uint16_t hold_ms, duration_ms;
} rb_choreo_step_t;
typedef struct {
    uint16_t schema_version;
    uint8_t work_id;
    char name[RB_CHOREO_NAME_BYTES];
    uint8_t tempo, count;
    rb_choreo_step_t steps[RB_CHOREO_MAX_STEPS];
} rb_choreo_work_t;
typedef struct {
    uint8_t action_id;
    const char *name;
    uint8_t group, repeat_max;
    bool available;
    uint16_t duration_ms;
    uint8_t entry_pose, exit_pose;
} rb_choreo_action_t;
extern const rb_choreo_action_t rb_choreo_actions[];
extern const size_t rb_choreo_action_count;
const rb_choreo_action_t *rb_choreo_action(unsigned action_id);
void rb_choreo_work_init(rb_choreo_work_t *work,unsigned slot);
void rb_choreo_work_demo(rb_choreo_work_t *work,unsigned slot);
/* Empty works are valid for editing/storage, but cannot compile or play. */
rb_choreo_status_t rb_choreo_validate(const rb_choreo_work_t *work,uint8_t *bad_step);
rb_choreo_status_t rb_choreo_add_after(rb_choreo_work_t *work,int selected,const rb_choreo_step_t *step);
rb_choreo_status_t rb_choreo_delete(rb_choreo_work_t *work,unsigned index);
rb_choreo_status_t rb_choreo_move(rb_choreo_work_t *work,unsigned index,int direction);
const char *rb_choreo_error_name(rb_choreo_status_t status);

typedef struct {
    uint8_t action_id, step_index;
    bool transition, holding;
    uint32_t start_ms, duration_ms;
    uint16_t source_duration_ms, clip_time_ms;
} rb_choreo_event_t;
typedef struct {
    uint8_t count;
    uint32_t base_ms, total_ms;
    rb_choreo_event_t events[RB_CHOREO_MAX_EVENTS];
} rb_choreo_plan_t;
typedef struct {
    uint8_t action_id, step_index;
    uint16_t clip_time_ms;
    bool holding, transition, done;
    uint16_t phase_q16; /* Shared normalized authored phase, never joint angles. */
} rb_choreo_sample_t;
uint16_t rb_choreo_beat_ms(unsigned tempo);
rb_choreo_status_t rb_choreo_compile(const rb_choreo_work_t *work,rb_choreo_plan_t *plan,uint8_t *bad_step);
/* clip_time_ms is unscaled GIF time. Holds/WAIT retain the last decoded pose.
   This module is a visual preview only and never emits motor commands. */
bool rb_choreo_sample(const rb_choreo_plan_t *plan,uint32_t elapsed_ms,rb_choreo_sample_t *sample);
typedef struct {
    const rb_choreo_plan_t *plan;
    uint64_t started_ms;
    uint32_t elapsed_ms;
    bool running;
    rb_choreo_sample_t sample;
} rb_choreo_player_t;
bool rb_choreo_player_start(rb_choreo_player_t *player,const rb_choreo_plan_t *plan,uint64_t now_ms);
void rb_choreo_player_tick(rb_choreo_player_t *player,uint64_t now_ms);
void rb_choreo_player_stop(rb_choreo_player_t *player,uint64_t now_ms);
