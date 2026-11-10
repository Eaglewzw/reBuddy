#pragma once
#include <stdbool.h>
#include <stdint.h>
#define RB_DANCE_COUNT 10u
#define RB_DANCE_MAX_STEPS 32u
#define RB_DANCE_MAX_PARTS (2u*RB_DANCE_MAX_STEPS-1u)
#define RB_DANCE_CONTROL_ACTION 100u
typedef struct { float begin_s,end_s,c[7][6]; } rb_dance_piece_t;
typedef struct { float duration_s; const rb_dance_piece_t *pieces; uint16_t count; } rb_dance_action_t;
typedef struct { float left_trim_s,right_trim_s,duration_s; } rb_dance_join_t;
typedef struct { float begin_s,end_s,source_s; uint8_t number,bridge; } rb_dance_part_t;
typedef struct {
    uint8_t count,part_count,numbers[RB_DANCE_MAX_STEPS];
    float duration_s,speed,nominal_s;
    rb_dance_part_t parts[RB_DANCE_MAX_PARTS];
    float bridges[RB_DANCE_MAX_STEPS-1][7][6];
} rb_dance_plan_t;
extern const rb_dance_action_t rb_dance_actions[RB_DANCE_COUNT];
extern const rb_dance_join_t rb_dance_joins[RB_DANCE_COUNT][RB_DANCE_COUNT];
extern const char rb_dance_calibration_sha256[],rb_dance_urdf_sha256[];
bool rb_dance_action_sample(unsigned number,float t,float q[7],float v[7],float a[7]);
/* Whole trajectory only: no motor I/O, internal settling or per-action clock. */
bool rb_dance_compile(rb_dance_plan_t *out,const uint8_t *numbers,unsigned count,float speed);
bool rb_dance_sample(const rb_dance_plan_t *plan,float t,float q[7],float v[7],float a[7],unsigned *number,bool *bridge);
bool rb_dance_random(uint8_t *numbers,unsigned count,uint32_t seed);
