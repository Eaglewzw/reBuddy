/*
 * SPDX-FileCopyrightText: 2026 XIAO-C5-CAN
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "can_console_ik.h"
#include "can_console_arm_teach.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CAN_CONSOLE_ARM_MOTION_DOF 6U
#define CAN_CONSOLE_ARM_MOTION_BUFFER_CAPACITY 64U

typedef enum {
    CAN_CONSOLE_ARM_MOTION_IDLE = 0,
    CAN_CONSOLE_ARM_MOTION_PLANNING,
    CAN_CONSOLE_ARM_MOTION_RUNNING,
    CAN_CONSOLE_ARM_MOTION_COMPLETE,
    CAN_CONSOLE_ARM_MOTION_ERROR,
} can_console_arm_motion_state_t;

typedef enum {
    CAN_CONSOLE_ARM_MOTION_POSE = 0,
    CAN_CONSOLE_ARM_MOTION_JOINT,
    CAN_CONSOLE_ARM_MOTION_JOINT_PATH,
    CAN_CONSOLE_ARM_MOTION_STOP,
} can_console_arm_motion_kind_t;

typedef enum {
    CAN_CONSOLE_ARM_MOTION_ERROR_NONE = 0,
    CAN_CONSOLE_ARM_MOTION_ERROR_PLANNING,
    CAN_CONSOLE_ARM_MOTION_ERROR_EXECUTION,
} can_console_arm_motion_error_source_t;

typedef enum {
    CAN_CONSOLE_ARM_PLAN_OK = 0,
    CAN_CONSOLE_ARM_PLAN_IK_FAILED,
    CAN_CONSOLE_ARM_PLAN_DISCONTINUITY,
    CAN_CONSOLE_ARM_PLAN_POINT_LIMIT,
    CAN_CONSOLE_ARM_PLAN_SPEED_LIMIT,
    CAN_CONSOLE_ARM_PLAN_NO_MEMORY,
    CAN_CONSOLE_ARM_PLAN_INVALID_JOINTS,
} can_console_arm_plan_reason_t;

/** Pose/joint preflight diagnostics. point and joint are one-based; zero means
 * unavailable. IK residual is valid only for IK_FAILED. duration_s is the
 * attempted/required path duration, not time already spent moving. */
typedef struct {
    can_console_arm_plan_reason_t reason;
    uint32_t point, points;
    uint8_t joint;
    float duration_s;
    can_console_ik_result_t ik;
} can_console_arm_plan_diagnostic_t;

/** Borrowed, immutable recording; no whole-path copy is made. */
typedef struct {
    const arm_teach_sample_t *samples;
    uint32_t count;
} arm_joint_path_view_t;

/** Explicit calibrated limits, rad/s and rad/s^2. No acceleration defaults.
 * The planner also caps velocity by the configured/requested motion vlim. */
typedef struct {
    float velocity[CAN_CONSOLE_ARM_MOTION_DOF];
    float acceleration[CAN_CONSOLE_ARM_MOTION_DOF];
} arm_joint_path_limits_t;

typedef struct {
    float position[CAN_CONSOLE_ARM_MOTION_DOF];
    /** Nominal path-time derivative, scaled by effective_speed. Not a claim
     * about wall-clock velocity during DEFER. Zero for legacy pose/joint plans. */
    float reference_velocity[CAN_CONSOLE_ARM_MOTION_DOF];
    float reference_time_s; /* Recording source time or elapsed point-to-point plan time. */
} can_console_arm_motion_sample_t;

/**
 * Configuration for the planner/executor boundary.
 *
 * motor_ids maps joint indices 0..5 to the CAN command IDs of the arm.
 * velocity_limits are the per-joint limits the planner must honour: a requested
 * duration is stretched until every commanded joint step stays inside them,
 * except recorded paths explicitly requested with NULL limits.
 * Reading the arm feedback, and mapping it to joint order, stays with the
 * application.
 */
typedef struct {
    uint8_t motor_ids[CAN_CONSOLE_ARM_MOTION_DOF];
    float velocity_limits[CAN_CONSOLE_ARM_MOTION_DOF];
    uint32_t sample_period_ms;
    uint32_t planner_stack_size;
    uint32_t planner_priority;
    can_console_ik_options_t ik_options;
} can_console_arm_motion_config_t;

typedef struct {
    can_console_arm_motion_state_t state;
    /** Points the last accepted plan is made of. */
    uint32_t total_points;
    /** Duration the last accepted plan actually uses. */
    float duration_s;
    uint32_t produced_points;
    uint32_t consumed_points;
    uint32_t buffered_points;
    /** Shallowest the buffer got while a plan was still running. */
    uint32_t buffer_low_water;
    /** Ticks that found the buffer empty after the executor had been fed. */
    uint32_t underruns;
    /** Inverse-kinematics solves since init, and their cost in microseconds. */
    uint32_t ik_solves;
    uint32_t ik_average_us;
    uint32_t ik_max_us;
    bool plan_complete;
    /** Includes a cancelled planner still reading its request. */
    bool planner_busy;
    can_console_arm_motion_kind_t kind;
    can_console_arm_motion_error_source_t error_source;
    float requested_speed;
    float effective_speed;
    float consumed_reference_time_s;
    esp_err_t last_error;
    can_console_arm_plan_diagnostic_t planning;
} can_console_arm_motion_status_t;

typedef enum {
    CAN_CONSOLE_ARM_MOTION_SAMPLE_ERROR = -1,
    CAN_CONSOLE_ARM_MOTION_SAMPLE_DEFER = 0,
    CAN_CONSOLE_ARM_MOTION_SAMPLE_APPLIED = 1,
} can_console_arm_motion_sample_result_t;

/** Apply, defer, or reject one complete, time-aligned six-joint sample. */
typedef can_console_arm_motion_sample_result_t
(*can_console_arm_motion_apply_sample_fn)(
    const uint8_t motor_ids[CAN_CONSOLE_ARM_MOTION_DOF],
    const float position[CAN_CONSOLE_ARM_MOTION_DOF],
    const float velocity_limits[CAN_CONSOLE_ARM_MOTION_DOF],
    void *context);

/** Extended consumer for joint paths; same APPLIED/DEFER/ERROR contract. */
typedef can_console_arm_motion_sample_result_t
(*can_console_arm_motion_apply_reference_fn)(
    const uint8_t motor_ids[CAN_CONSOLE_ARM_MOTION_DOF],
    const can_console_arm_motion_sample_t *sample,
    const float velocity_limits[CAN_CONSOLE_ARM_MOTION_DOF], void *context);

void can_console_arm_motion_config_set_defaults(
    can_console_arm_motion_config_t *config);

/** Initialize the singleton motion planner and its point buffer. */
esp_err_t can_console_arm_motion_init(
    const can_console_arm_motion_config_t *config);

/** Start the low-priority planner task. */
esp_err_t can_console_arm_motion_start(void);

/** Update the uniform joint velocity limit while no trajectory is active. */
esp_err_t can_console_arm_motion_set_velocity_limit(float velocity_limit);
/** Read the current per-joint motion vlim, also used by position-mode playback. */
esp_err_t can_console_arm_motion_get_velocity_limits(float velocity_limits[6]);

/**
 * Request a point-to-point Cartesian move.
 *
 * initial_q is normally assembled from the arm feedback. The planner samples a
 * minimum-jerk Cartesian pose path, solves IK once per sample using the previous
 * solution as the next seed, and streams the resulting joint targets to the
 * executor.
 *
 * duration_s is a lower bound, not a promise: the planner stretches it until the
 * solved joint path honours the configured velocity limits, and aligns it to the
 * sample grid so the last point lands exactly on the target. The executor may
 * stretch wall-clock time further when the application defers a sample.
 * can_console_arm_motion_get_status() reports the duration actually used.
 */
esp_err_t can_console_arm_motion_plan_pose(
    const float initial_q[CAN_CONSOLE_ARM_MOTION_DOF],
    const float target_position[CAN_CONSOLE_IK_POSITION_DIM],
    const float target_rotation[CAN_CONSOLE_IK_ROTATION_ELEMENTS],
    float duration_s);

/** Pose request with copied, per-request IK precision and joint velocity caps.
 * Caps are also bounded by the configured planner limits. Does not mutate the
 * console defaults. Same full-path validation and asynchronous execution. */
esp_err_t can_console_arm_motion_plan_pose_with_limits(
    const float initial_q[6], const float target_position[3],
    const float target_rotation[9], float duration_s,
    const float velocity_limits[6], const can_console_ik_options_t *ik_options);

/** Request a minimum-jerk joint-space move, used for safe homing. */
esp_err_t can_console_arm_motion_plan_joints(
    const float initial_q[CAN_CONSOLE_ARM_MOTION_DOF],
    const float target_q[CAN_CONSOLE_ARM_MOTION_DOF],
    float duration_s);

/** Queue a read-only six-joint recording for complete preflight then streaming.
 * speed is 0.1..1; pass 0.5 for half-speed. With limits, effective speed may
 * be lower. NULL limits keeps the recording's timing (aligned to one sample
 * period), with position/time validation but no velocity/acceleration retiming.
 * Requires a quiescent planner and empty queue. Metadata/profile/calibration,
 * measured start, feedback and stop remain application responsibilities.
 * Samples are in the selected IK model coordinates. They must remain immutable
 * and alive until the worker is quiescent, even after cancel or execution error.
 * Preflight errors publish ERROR_PLANNING without releasing any points.
 * This API alone does not provide a safe real-arm playback lifecycle. */
esp_err_t can_console_arm_motion_plan_joint_path(
    const arm_joint_path_view_t *path, const arm_joint_path_limits_t *limits,
    float speed);

/** Same joint planner with model range and continuous v/a checks. */
esp_err_t can_console_arm_motion_plan_joints_limited(const float initial_q[6],
    const float target_q[6], float duration_s, const arm_joint_path_limits_t *limits);
/** Replace future points with a constant-deceleration segment from a fresh,
 * delivery-confirmed reference. step_ms is the verified execution cadence,
 * at least the configured planner period. Caller owns downstream handoff.
 * No path allocation; an old worker may still be retiring after return. */
esp_err_t can_console_arm_motion_plan_stop(const float position[6], const float velocity[6],
    const arm_joint_path_limits_t *limits, uint32_t step_ms);

/** Cancel the current plan and discard all buffered points. */
void can_console_arm_motion_cancel(void);

/** Cancel and wait for the planner to leave its current solve and free its path.
 * Caller must prevent new plans and must not hold locks needed by the planner.
 * Success is a quiescence barrier for changing the shared IK model. */
esp_err_t can_console_arm_motion_cancel_and_wait(uint32_t timeout_ms);

/**
 * Consume one precomputed joint target without sending it.
 * Returns false when the buffer is empty.
 */
bool can_console_arm_motion_pop_joint_target(
    float position[CAN_CONSOLE_ARM_MOTION_DOF]);

/**
 * Offer one target to the application. APPLIED consumes it, DEFER keeps it at
 * the head of the queue for the next tick, and ERROR aborts the plan.
 * Call this from the fixed-period CAN control task.
 */
bool can_console_arm_motion_tick(
    can_console_arm_motion_apply_sample_fn apply_sample, void *context);

bool can_console_arm_motion_tick_reference(
    can_console_arm_motion_apply_reference_fn apply_reference, void *context);

esp_err_t can_console_arm_motion_get_status(
    can_console_arm_motion_status_t *status);

#ifdef __cplusplus
}
#endif
