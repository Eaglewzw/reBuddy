#pragma once
#include <stdbool.h>
#include <stdint.h>

/* Compact calibrated data, not bitmap-derived angles. Units: 0.01 degrees.
 * On-disk encoding must be explicit little endian, never a struct dump.
 * 32 keys x (u16 time + 6 x i16 angles) = 448 bytes before record metadata.
 * No catalog is calibrated or connected to motors in this firmware. */
#define RB_MOTION_JOINTS 6u
#define RB_MOTION_MAX_KEYS 32u
typedef struct { uint16_t at_ms; int16_t angle[RB_MOTION_JOINTS]; } rb_motion_key_t;
typedef struct {
    int16_t minimum, maximum;
    uint16_t velocity, acceleration; /* angle units/s and angle units/s^2 */
} rb_motion_limit_t;
/* Same zero-velocity/zero-acceleration endpoint curve for planned bridges.
 * The backend must first confirm its actual start pose and a collision-checked
 * route. Bounds here only cover joint travel, speed and acceleration. */
float rb_motion_ease(float phase);
bool rb_motion_bridge_duration(const int16_t from[RB_MOTION_JOINTS],
    const int16_t to[RB_MOTION_JOINTS], const rb_motion_limit_t limits[RB_MOTION_JOINTS],
    uint32_t minimum_ms, uint32_t *duration_ms);
bool rb_motion_bridge_sample(const int16_t from[RB_MOTION_JOINTS],
    const int16_t to[RB_MOTION_JOINTS], uint32_t elapsed_ms,uint32_t duration_ms,
    float output[RB_MOTION_JOINTS]);
