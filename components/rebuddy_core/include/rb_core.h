#ifndef RB_CORE_H
#define RB_CORE_H
#include <stdbool.h>
#include <stdint.h>
#define RB_JOINTS 7
#define RB_MAX_FRAMES 8
typedef enum { RB_LOOK, RB_NUZZLE, RB_SHY, RB_DANCE, RB_PUZZLED, RB_DOZE, RB_ACTION_COUNT, RB_NONE = 255 } rb_action_id_t;
typedef enum { RB_IDLE, RB_PLAYING, RB_DROWSY, RB_SLEEPING, RB_WAKING, RB_STOPPED } rb_state_t;
typedef enum { RB_LIVELY, RB_LAZY } rb_style_t;
typedef enum { RB_FACE_IDLE, RB_FACE_HAPPY, RB_FACE_CURIOUS, RB_FACE_RELAXED, RB_FACE_SHY, RB_FACE_PUZZLED, RB_FACE_DROWSY, RB_FACE_SLEEP, RB_FACE_STOP } rb_face_t;
typedef enum { RB_TAP, RB_STROKE, RB_MULTI_TAP, RB_ZIGZAG, RB_HAPPY, RB_REST, RB_WAKE } rb_event_t;
typedef enum { RB_REGION_NEAR, RB_REGION_HEAD, RB_REGION_BODY, RB_REGION_CLAW } rb_region_t;
/* Screen-only expression state: never added to physical joint targets. */
typedef struct {
    bool contact, stroking, blink;
    rb_region_t region;
    rb_face_t face;
    float comfort, excitement, disturbance, gaze_x, gaze_y, target_x, target_y;
    float lean, lift, openness;
    uint64_t contact_ms, release_ms, last_poke_ms, puzzled_until, startled_until;
    uint64_t stroke_until, rough_until;
    uint64_t next_blink_ms, blink_until, next_look_ms, celebrate_until, celebrate_after;
    uint32_t random;
    unsigned pokes;
} rb_life_t;
typedef struct { uint32_t at_ms; float offset_deg[RB_JOINTS]; rb_face_t face; } rb_keyframe_t;
typedef struct { const char *name; unsigned count; rb_keyframe_t frames[RB_MAX_FRAMES]; } rb_action_t;
extern const rb_action_t rb_actions[RB_ACTION_COUNT];
typedef struct {
    uint32_t drowsy_ms, sleep_ms;
    float max_speed_deg_s, max_offset_deg;
} rb_config_t;
typedef struct {
    rb_config_t config;
    rb_state_t state;
    rb_style_t style, active_style;
    rb_face_t face;
    rb_action_id_t action;
    uint64_t now_ms, last_input_ms, started_ms;
    float pose[RB_JOINTS], origin[RB_JOINTS];
    unsigned actions_started, actions_completed;
    const char *stop_reason;
    rb_life_t life;
} rb_engine_t;
void rb_life_tick(rb_engine_t *e, uint64_t now, uint64_t elapsed);
rb_config_t rb_config_default(void);
bool rb_engine_init(rb_engine_t *e, const rb_config_t *config, uint64_t now);
bool rb_engine_action(rb_engine_t *e, rb_action_id_t action, uint64_t now);
bool rb_engine_event(rb_engine_t *e, rb_event_t event, uint64_t now);
void rb_engine_tick(rb_engine_t *e, uint64_t now);
void rb_engine_style(rb_engine_t *e, rb_style_t style);
void rb_engine_stop(rb_engine_t *e, const char *reason);
bool rb_engine_reset(rb_engine_t *e, uint64_t now);
const char *rb_state_name(rb_state_t state);

typedef enum { RB_TOUCH_DOWN, RB_TOUCH_MOVE, RB_TOUCH_UP, RB_TOUCH_CANCEL } rb_touch_phase_t;
typedef struct {
    bool down;
    int x, y, anchor_x, direction, reversals;
    unsigned path;
    uint64_t started_ms, last_sample_ms;
} rb_gesture_t;
void rb_gesture_init(rb_gesture_t *g);
void rb_gesture_touch(rb_gesture_t *g, rb_engine_t *e, rb_touch_phase_t phase, int x, int y, uint64_t now);
void rb_gesture_touch_region(rb_gesture_t *g, rb_engine_t *e, rb_touch_phase_t phase,
                             int x, int y, rb_region_t region, uint64_t now);
void rb_gesture_cancel(rb_gesture_t *g, rb_engine_t *e);
void rb_gesture_tick(rb_gesture_t *g, rb_engine_t *e, uint64_t now);
#endif
