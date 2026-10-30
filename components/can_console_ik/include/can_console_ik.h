/*
 * SPDX-FileCopyrightText: 2026 XIAO-C5-CAN
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CAN_CONSOLE_IK_DOF                  6U
#define CAN_CONSOLE_IK_POSITION_DIM        3U
#define CAN_CONSOLE_IK_ROTATION_ELEMENTS   9U
/** Preserve small encoder zero offsets in measured starts; targets keep model limits. */
#define CAN_CONSOLE_IK_SEED_LIMIT_TOLERANCE_RAD 0.02f

/** Kinematic description matching the physical reBotArm variant. */
typedef enum {
    CAN_CONSOLE_IK_MODEL_REBOTARM_RS = 0,
    CAN_CONSOLE_IK_MODEL_REBOTARM_DM = 1,
} can_console_ik_model_t;

/** Solver options for the B601 six-axis arm. Angles are in radians, distances in metres. */
typedef struct {
    uint16_t max_iterations;
    float damping;
    float orientation_weight;
    float max_step_rad;
    float position_tolerance_m;
    float orientation_tolerance_rad;
} can_console_ik_options_t;

/** Result information returned by the inverse-kinematics solver. */
typedef struct {
    bool converged;
    uint16_t iterations;
    float position_error_m;
    float orientation_error_rad;
} can_console_ik_result_t;

/** Immutable geometry owned by the component; metres and radians. */
typedef struct {
    float origin_xyz[3], origin_rpy[3], axis[3];
    float lower, upper;
} can_console_ik_joint_model_t;
const can_console_ik_joint_model_t *can_console_ik_joint_model(can_console_ik_model_t model, unsigned joint);
bool can_console_ik_tool_origin(can_console_ik_model_t model, float xyz[3], float rpy[3]);
bool can_console_ik_get_joint_limits_for_model(can_console_ik_model_t model, float lower[6], float upper[6]);

/** Set the default model. Each call snapshots the model; explicit-model APIs
 * are unaffected by selection, and cached geometry is immutable after init.
 * Higher-level multi-call control sequences must still exclude model changes. */
bool can_console_ik_select_model(can_console_ik_model_t model);

/** Return the selected physical arm model. */
can_console_ik_model_t can_console_ik_get_model(void);

/** Copy hard joint ranges from the selected model, in model joint coordinates.
 * Like FK, caller excludes concurrent model selection. No seed tolerance/clamp. */
bool can_console_ik_get_joint_limits(float lower[CAN_CONSOLE_IK_DOF],
                                     float upper[CAN_CONSOLE_IK_DOF]);

/** Return a stable, human-readable name for a model. */
const char *can_console_ik_model_name(can_console_ik_model_t model);

/** Initialize the cached B601 kinematic model and trigonometric lookup table. */
void can_console_ik_init(void);

/** Fill solver options with the component defaults. */
void can_console_ik_default_options(can_console_ik_options_t *options);

/** Compute end-effector position and row-major 3x3 rotation from six joint angles. */
void can_console_ik_forward(
    const float q[CAN_CONSOLE_IK_DOF],
    float position[CAN_CONSOLE_IK_POSITION_DIM],
    float rotation[CAN_CONSOLE_IK_ROTATION_ELEMENTS]);

/** Base-frame chain geometry; link rotations are after each joint rotation. */
typedef struct {
    float joint_position[CAN_CONSOLE_IK_DOF][3];
    float joint_axis[CAN_CONSOLE_IK_DOF][3];
    float link_rotation[CAN_CONSOLE_IK_DOF][9];
    float end_position[3];
    float end_rotation[9];
} can_console_ik_chain_t;

/** Same FK as forward(), with intermediate frames for statics; no IK solve. */
bool can_console_ik_forward_chain(const float q[CAN_CONSOLE_IK_DOF],
                                  can_console_ik_chain_t *chain);

/** Explicit-model operations share the same implementation as default APIs. */
bool can_console_ik_forward_chain_for_model(can_console_ik_model_t model,
    const float q[6], can_console_ik_chain_t *chain);
bool can_console_ik_solve_for_model(can_console_ik_model_t model,
    const float position[3], const float rotation[9], const float seed[6],
    const can_console_ik_options_t *options, float solution[6], can_console_ik_result_t *result);

/**
 * Solve for six joint angles reaching the target pose.
 *
 * Returns true when both position and orientation tolerances are reached. If
 * result is non-NULL, it also contains the final error and iteration count.
 */
bool can_console_ik_solve(
    const float target_position[CAN_CONSOLE_IK_POSITION_DIM],
    const float target_rotation[CAN_CONSOLE_IK_ROTATION_ELEMENTS],
    const float initial_q[CAN_CONSOLE_IK_DOF],
    const can_console_ik_options_t *options,
    float solution_q[CAN_CONSOLE_IK_DOF],
    can_console_ik_result_t *result);

/** Run the built-in B601 solver benchmark and log its results. */
void can_console_ik_benchmark(void);

#ifdef __cplusplus
}
#endif
