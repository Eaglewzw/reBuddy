#pragma once
#include "can_console_arm_motion.h"

typedef struct {
    float position[6], velocity[6];
    double duration_s;
    uint32_t intervals, step_ms, next;
} arm_stop_t;

esp_err_t arm_stop_prepare(const float position[6], const float velocity[6],
    const arm_joint_path_limits_t *limits, const float lower[6], const float upper[6],
    uint32_t step_ms, arm_stop_t *stop);
void arm_stop_sample(const arm_stop_t *stop, uint32_t index, can_console_arm_motion_sample_t *point);
