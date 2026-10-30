/* Local PCHIP interpolation and whole-path preflight. No allocation or IK. */
#pragma once
#include "can_console_arm_motion.h"

typedef struct {
    double coefficient[CAN_CONSOLE_ARM_MOTION_DOF][4]; /* a*u^3+b*u^2+c*u+d */
    double duration_s;
} arm_joint_path_segment_t;

typedef struct {
    arm_joint_path_view_t path;
    double effective_speed;
    double duration_s;
    uint32_t total_points;
    uint32_t sample_period_ms;
} arm_joint_path_plan_t;

/* NULL limits preserves original timing; position/time preflight still runs.
 * keep_running is an optional cancellation check, not a data-source callback. */
esp_err_t arm_joint_path_prepare(const arm_joint_path_view_t *path,
    const arm_joint_path_limits_t *limits, const float lower[6], const float upper[6],
    uint32_t period_ms, float speed, bool (*keep_running)(void *), void *context,
    arm_joint_path_plan_t *plan);
void arm_joint_path_make_segment(const arm_joint_path_view_t *path, uint32_t index,
    arm_joint_path_segment_t *segment);
void arm_joint_path_evaluate(const arm_joint_path_segment_t *segment, double u,
    double speed, can_console_arm_motion_sample_t *sample);
