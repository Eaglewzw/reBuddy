      
/*
 * SPDX-FileCopyrightText: 2026 XIAO-C5-CAN
 * SPDX-License-Identifier: Apache-2.0
 */

#include "can_console_ik.h"
#include "can_console_pose_math.h"

#include <math.h>
#include <string.h>
#include <stdatomic.h>
#ifdef ESP_PLATFORM
#include <sys/lock.h>
#endif

#include "esp_log.h"
#include "esp_timer.h"

#define CAN_CONSOLE_IK_PI              3.14159265358979323846f
#define CAN_CONSOLE_IK_TWO_PI          (2.0f * CAN_CONSOLE_IK_PI)
#define CAN_CONSOLE_IK_EPSILON         1.0e-7f

#ifndef CAN_CONSOLE_IK_USE_TRIG_LUT
#define CAN_CONSOLE_IK_USE_TRIG_LUT 0
#endif

#define CAN_CONSOLE_IK_TRIG_LUT_SIZE   1024

static const char *TAG = "can_console_ik";

typedef struct {
    float r[3][3];
    float p[3];
} rigid_transform_t;

typedef can_console_ik_joint_model_t joint_model_t;

typedef struct {
    const char *name;
    joint_model_t joints[CAN_CONSOLE_IK_DOF];
    float end_xyz[3];
    float end_rpy[3];
} arm_model_t;

static const arm_model_t k_arm_models[] = {
    [CAN_CONSOLE_IK_MODEL_REBOTARM_RS] = {
        .name = "reBotArm RS (00-arm-rs_asm-v3)",
        .joints = {
            {{-0.00034283f, -0.00098683f, 0.075f}, {0.0f, 0.0f, 0.0f},
             {0.0f, 0.0f, -1.0f}, -2.8f, 2.8f},
            {{0.020343f, 0.027237f, 0.070f}, {-1.5708f, 0.0f, 0.0f},
             {0.0f, 0.0f, 1.0f}, 0.0f, 3.14f},
            {{-0.236f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f},
             {0.0f, 0.0f, -1.0f}, 0.0f, 3.490658504f},
            {{0.228f, -0.072746f, 0.0045f}, {0.0f, 0.0f, 0.0f},
             {0.0f, 0.0f, -1.0f}, -1.57f, 1.57f},
            {{0.087f, -0.048f, -0.03075f}, {-1.5708f, 0.0f, 0.0f},
             {0.0f, 0.0f, -1.0f}, -1.57f, 1.57f},
            {{0.0365f, 0.0f, 0.048f}, {0.0f, 1.5708f, 0.0f},
             {0.0f, 0.0f, -1.0f}, -3.14f, 3.14f},
        },
        .end_xyz = {0.0f, 0.0f, 0.16621f},
        .end_rpy = {3.1416f, -1.5708f, 0.0f},
    },
    [CAN_CONSOLE_IK_MODEL_REBOTARM_DM] = {
        .name = "reBotArm DM (ReBot_Arm_DM)",
        .joints = {
            {{-0.00008416f, 0.0f, 0.08465f}, {0.0f, 0.0f, 0.0f},
             {0.0f, 0.0f, 1.0f}, -2.8f, 2.8f},
            {{0.020084f, 0.031625f, 0.05555f}, {-1.5708f, 0.0f, 0.0f},
             {0.0f, 0.0f, -1.0f}, -3.14f, 0.0f},
            {{-0.264f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f},
             {0.0f, 0.0f, 1.0f}, -3.14f, 0.0f},
            {{0.2426f, -0.054f, -0.001625f}, {0.0f, 0.0f, 0.0f},
             {0.0f, 0.0f, 1.0f}, -1.87f, 1.57f},
            {{0.078308f, -0.0375f, -0.03f}, {-1.5708f, 0.0f, 0.0f},
             {0.0f, 0.0f, 1.0f}, -1.57f, 1.57f},
            {{0.028008f, 0.0f, 0.04f}, {0.0f, 1.5708f, 0.0f},
             {0.0f, 0.0f, 1.0f}, -3.14f, 3.14f},
        },
        .end_xyz = {0.0f, 0.0f, 0.15539f},
        .end_rpy = {0.0f, -1.5708f, 3.1415f},
    },
};

static rigid_transform_t s_joint_origins[2][CAN_CONSOLE_IK_DOF];
static rigid_transform_t s_end_origin[2];
static _Atomic can_console_ik_model_t s_selected_model = CAN_CONSOLE_IK_MODEL_REBOTARM_RS;
static atomic_bool s_initialized;
#ifdef ESP_PLATFORM
static _lock_t s_init_lock;
#else
static atomic_flag s_init_lock = ATOMIC_FLAG_INIT;
#endif
static float s_reach_bound[2];

static const arm_model_t *active_model(void)
{
    return &k_arm_models[s_selected_model];
}

static bool valid_model(can_console_ik_model_t model)
{
    return model == CAN_CONSOLE_IK_MODEL_REBOTARM_RS || model == CAN_CONSOLE_IK_MODEL_REBOTARM_DM;
}

const can_console_ik_joint_model_t *can_console_ik_joint_model(can_console_ik_model_t model, unsigned joint)
{
    return valid_model(model) && joint < CAN_CONSOLE_IK_DOF ? &k_arm_models[model].joints[joint] : NULL;
}

bool can_console_ik_tool_origin(can_console_ik_model_t model, float xyz[3], float rpy[3])
{
    if (!valid_model(model) || !xyz || !rpy) return false;
    memcpy(xyz, k_arm_models[model].end_xyz, sizeof(k_arm_models[model].end_xyz));
    memcpy(rpy, k_arm_models[model].end_rpy, sizeof(k_arm_models[model].end_rpy));
    return true;
}

bool can_console_ik_get_joint_limits_for_model(can_console_ik_model_t model, float lower[6], float upper[6])
{
    if (!valid_model(model) || !lower || !upper) return false;
    for (unsigned i = 0; i < CAN_CONSOLE_IK_DOF; ++i) {
        lower[i] = k_arm_models[model].joints[i].lower;
        upper[i] = k_arm_models[model].joints[i].upper;
    }
    return true;
}

bool can_console_ik_get_joint_limits(float lower[6], float upper[6])
{
    return can_console_ik_get_joint_limits_for_model(s_selected_model, lower, upper);
}

#if CAN_CONSOLE_IK_USE_TRIG_LUT
static float s_sin_lut[CAN_CONSOLE_IK_TRIG_LUT_SIZE + 1];
static float s_cos_lut[CAN_CONSOLE_IK_TRIG_LUT_SIZE + 1];
#endif

static float clampf_local(float value, float lower, float upper)
{
    if (value < lower) {
        return lower;
    }
    if (value > upper) {
        return upper;
    }
    return value;
}

static void rigid_identity(rigid_transform_t *out)
{
    memset(out, 0, sizeof(*out));
    out->r[0][0] = 1.0f;
    out->r[1][1] = 1.0f;
    out->r[2][2] = 1.0f;
}

static void rigid_compose(const rigid_transform_t *a,
                          const rigid_transform_t *b,
                          rigid_transform_t *out)
{
    rigid_transform_t temp;

    for (int row = 0; row < 3; ++row) {
        temp.p[row] = a->p[row];
        for (int k = 0; k < 3; ++k) {
            temp.p[row] += a->r[row][k] * b->p[k];
        }
        for (int col = 0; col < 3; ++col) {
            temp.r[row][col] = 0.0f;
            for (int k = 0; k < 3; ++k) {
                temp.r[row][col] += a->r[row][k] * b->r[k][col];
            }
        }
    }
    *out = temp;
}

static void make_origin(const float xyz[3], const float rpy[3],
                        rigid_transform_t *out)
{
    float rotation[CAN_CONSOLE_POSE_ROTATION_ELEMENTS];
    can_console_pose_rpy_to_rotation(rpy[0], rpy[1], rpy[2], rotation);
    memcpy(out->r, rotation, sizeof(rotation));
    memcpy(out->p, xyz, sizeof(out->p));
}

#if CAN_CONSOLE_IK_USE_TRIG_LUT
static void init_trig_lut(void)
{
    const float step = CAN_CONSOLE_IK_TWO_PI /
                       (float)CAN_CONSOLE_IK_TRIG_LUT_SIZE;

    for (int i = 0; i <= CAN_CONSOLE_IK_TRIG_LUT_SIZE; ++i) {
        const float angle = -CAN_CONSOLE_IK_PI + (float)i * step;
        __builtin_sincosf(angle, &s_sin_lut[i], &s_cos_lut[i]);
    }
}

static void fast_sincosf(float angle, float *sin_value, float *cos_value)
{
    /* IK joint limits already keep this on the fast path. */
    if (angle < -CAN_CONSOLE_IK_PI || angle > CAN_CONSOLE_IK_PI) {
        angle = fmodf(angle + CAN_CONSOLE_IK_PI, CAN_CONSOLE_IK_TWO_PI);
        if (angle < 0.0f) {
            angle += CAN_CONSOLE_IK_TWO_PI;
        }
        angle -= CAN_CONSOLE_IK_PI;
    }

    const float scaled = (angle + CAN_CONSOLE_IK_PI) *
                         ((float)CAN_CONSOLE_IK_TRIG_LUT_SIZE /
                          CAN_CONSOLE_IK_TWO_PI);
    int index = (int)scaled;
    float fraction = scaled - (float)index;

    if (index >= CAN_CONSOLE_IK_TRIG_LUT_SIZE) {
        index = CAN_CONSOLE_IK_TRIG_LUT_SIZE - 1;
        fraction = 1.0f;
    }

    *sin_value = s_sin_lut[index] +
                 fraction * (s_sin_lut[index + 1] - s_sin_lut[index]);
    *cos_value = s_cos_lut[index] +
                 fraction * (s_cos_lut[index + 1] - s_cos_lut[index]);
}
#endif

/* All B601 joints rotate around their local +Z or -Z axis. */
static void rotate_local_z(rigid_transform_t *frame, float angle)
{
    float s;
    float c;
#if CAN_CONSOLE_IK_USE_TRIG_LUT
    fast_sincosf(angle, &s, &c);
#else
    __builtin_sincosf(angle, &s, &c);
#endif

    for (int row = 0; row < 3; ++row) {
        const float col0 = frame->r[row][0];
        const float col1 = frame->r[row][1];
        frame->r[row][0] = c * col0 + s * col1;
        frame->r[row][1] = -s * col0 + c * col1;
    }
}

static void cross3(const float a[3], const float b[3], float out[3])
{
    out[0] = a[1] * b[2] - a[2] * b[1];
    out[1] = a[2] * b[0] - a[0] * b[2];
    out[2] = a[0] * b[1] - a[1] * b[0];
}

static float norm3(const float v[3])
{
    return sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

static float norm3_squared(const float v[3])
{
    return v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
}

static void compute_kinematics(
    can_console_ik_model_t model,
    const float q[CAN_CONSOLE_IK_DOF],
    float position[CAN_CONSOLE_IK_POSITION_DIM],
    float rotation[CAN_CONSOLE_IK_ROTATION_ELEMENTS],
    float jacobian[CAN_CONSOLE_IK_DOF][CAN_CONSOLE_IK_DOF],
    can_console_ik_chain_t *chain)
{
    rigid_transform_t transform;
    rigid_identity(&transform);

    float joint_position[CAN_CONSOLE_IK_DOF][3];
    float joint_axis[CAN_CONSOLE_IK_DOF][3];

    for (int i = 0; i < CAN_CONSOLE_IK_DOF; ++i) {
        rigid_transform_t joint_frame;
        rigid_compose(&transform, &s_joint_origins[model][i], &joint_frame);

        joint_position[i][0] = joint_frame.p[0];
        joint_position[i][1] = joint_frame.p[1];
        joint_position[i][2] = joint_frame.p[2];

        const float axis_sign = k_arm_models[model].joints[i].axis[2];
        for (int row = 0; row < 3; ++row) {
            joint_axis[i][row] = axis_sign * joint_frame.r[row][2];
        }

        transform = joint_frame;
        rotate_local_z(&transform, axis_sign * q[i]);
        if (chain != NULL) {
            memcpy(chain->joint_position[i], joint_position[i], sizeof(joint_position[i]));
            memcpy(chain->joint_axis[i], joint_axis[i], sizeof(joint_axis[i]));
            memcpy(chain->link_rotation[i], transform.r, sizeof(transform.r));
        }
    }

    rigid_transform_t end_frame;
    rigid_compose(&transform, &s_end_origin[model], &end_frame);
    memcpy(position, end_frame.p, sizeof(end_frame.p));

    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            rotation[row * 3 + col] = end_frame.r[row][col];
        }
    }

    if (jacobian == NULL) {
        return;
    }

    for (int i = 0; i < CAN_CONSOLE_IK_DOF; ++i) {
        float end_minus_joint[3] = {
            position[0] - joint_position[i][0],
            position[1] - joint_position[i][1],
            position[2] - joint_position[i][2],
        };
        float linear[3];
        cross3(joint_axis[i], end_minus_joint, linear);

        for (int row = 0; row < 3; ++row) {
            jacobian[row][i] = linear[row];
            jacobian[row + 3][i] = joint_axis[i][row];
        }
    }
}

static void pose_error(
    const float target_position[CAN_CONSOLE_IK_POSITION_DIM],
    const float target_rotation[CAN_CONSOLE_IK_ROTATION_ELEMENTS],
    const float current_position[CAN_CONSOLE_IK_POSITION_DIM],
    const float current_rotation[CAN_CONSOLE_IK_ROTATION_ELEMENTS],
    float error[CAN_CONSOLE_IK_DOF])
{
    error[0] = target_position[0] - current_position[0];
    error[1] = target_position[1] - current_position[1];
    error[2] = target_position[2] - current_position[2];

    for (int component = 0; component < 3; ++component) {
        const int row_a = (component + 1) % 3;
        const int row_b = (component + 2) % 3;
        float value = 0.0f;
        for (int col = 0; col < 3; ++col) {
            value += current_rotation[row_a * 3 + col] *
                         target_rotation[row_b * 3 + col] -
                     current_rotation[row_b * 3 + col] *
                         target_rotation[row_a * 3 + col];
        }
        error[component + 3] = 0.5f * value;
    }

    float dot = 0.0f;
    for (int i = 0; i < 9; ++i) {
        dot += target_rotation[i] * current_rotation[i];
    }
    if (dot > 2.98f) {
        /* theta/sin(theta), small-angle series. No sqrt/trig on the tracking
         * fast path. The trace guard excludes the other zero at theta=pi. */
        const float sine_squared = norm3_squared(&error[3]);
        const float scale = 1.0f + sine_squared *
            (1.0f / 6.0f + sine_squared * (3.0f / 40.0f));
        for (int i = 3; i < 6; ++i) error[i] *= scale;
        return;
    }

    /* Spatial rotation log of R_target R_current^T. Quaternion extraction is
     * stable near pi, where the antisymmetric part alone loses the axis. */
    float relative[9];
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            float value = 0.0f;
            for (int k = 0; k < 3; ++k) {
                value += target_rotation[row * 3 + k] *
                         current_rotation[col * 3 + k];
            }
            relative[row * 3 + col] = value;
        }
    }
    float quaternion[4];
    can_console_pose_rotation_to_quaternion(relative, quaternion);
    const float sine_half = norm3(&quaternion[1]);
    const float sign = quaternion[0] < 0.0f ? -1.0f : 1.0f;
    const float scale = sine_half > 1.0e-7f ?
        sign * 2.0f * atan2f(sine_half, fabsf(quaternion[0])) / sine_half : 2.0f;
    for (int i = 0; i < 3; ++i) error[i + 3] = scale * quaternion[i + 1];
}

/* Solve the damped symmetric positive-definite normal equation with LDL^T. */
static bool solve_normal_equation(
    const float matrix[CAN_CONSOLE_IK_DOF][CAN_CONSOLE_IK_DOF + 1],
    float solution[CAN_CONSOLE_IK_DOF])
{
    float lower[CAN_CONSOLE_IK_DOF][CAN_CONSOLE_IK_DOF] = {{0}};
    float diagonal[CAN_CONSOLE_IK_DOF];
    float inverse_diagonal[CAN_CONSOLE_IK_DOF];
    float forward[CAN_CONSOLE_IK_DOF];

    for (int row = 0; row < CAN_CONSOLE_IK_DOF; ++row) {
        lower[row][row] = 1.0f;

        for (int col = 0; col < row; ++col) {
            float value = matrix[row][col];
            for (int k = 0; k < col; ++k) {
                value -= lower[row][k] * diagonal[k] * lower[col][k];
            }
            lower[row][col] = value * inverse_diagonal[col];
        }

        float pivot = matrix[row][row];
        for (int k = 0; k < row; ++k) {
            pivot -= lower[row][k] * lower[row][k] * diagonal[k];
        }
        if (pivot <= CAN_CONSOLE_IK_EPSILON) {
            return false;
        }
        diagonal[row] = pivot;
        inverse_diagonal[row] = 1.0f / pivot;
    }

    for (int row = 0; row < CAN_CONSOLE_IK_DOF; ++row) {
        float value = matrix[row][CAN_CONSOLE_IK_DOF];
        for (int col = 0; col < row; ++col) {
            value -= lower[row][col] * forward[col];
        }
        forward[row] = value;
    }

    for (int row = 0; row < CAN_CONSOLE_IK_DOF; ++row) {
        forward[row] *= inverse_diagonal[row];
    }

    for (int row = CAN_CONSOLE_IK_DOF - 1; row >= 0; --row) {
        float value = forward[row];
        for (int col = row + 1; col < CAN_CONSOLE_IK_DOF; ++col) {
            value -= lower[col][row] * solution[col];
        }
        solution[row] = value;
    }

    return true;
}

static bool solve_damped_step(
    const float jacobian[CAN_CONSOLE_IK_DOF][CAN_CONSOLE_IK_DOF],
    const float error[CAN_CONSOLE_IK_DOF],
    float damping,
    float orientation_weight,
    float delta_q[CAN_CONSOLE_IK_DOF])
{
    float normal[CAN_CONSOLE_IK_DOF][CAN_CONSOLE_IK_DOF + 1] = {{0}};
    const float orientation_weight_squared =
        orientation_weight * orientation_weight;
    const float damping_squared = damping * damping;

    for (int row = 0; row < CAN_CONSOLE_IK_DOF; ++row) {
        for (int col = row; col < CAN_CONSOLE_IK_DOF; ++col) {
            float value = 0.0f;
            for (int k = 0; k < 3; ++k) {
                value += jacobian[k][row] * jacobian[k][col];
            }
            for (int k = 3; k < CAN_CONSOLE_IK_DOF; ++k) {
                value += orientation_weight_squared *
                         jacobian[k][row] * jacobian[k][col];
            }
            normal[row][col] = value;
            normal[col][row] = value;
        }

        for (int k = 0; k < 3; ++k) {
            normal[row][CAN_CONSOLE_IK_DOF] += jacobian[k][row] * error[k];
        }
        for (int k = 3; k < CAN_CONSOLE_IK_DOF; ++k) {
            normal[row][CAN_CONSOLE_IK_DOF] += orientation_weight_squared *
                                               jacobian[k][row] * error[k];
        }
        normal[row][row] += damping_squared;
    }

    return solve_normal_equation(normal, delta_q);
}

static void init_model_frames(can_console_ik_model_t model)
{
    const arm_model_t *description = &k_arm_models[model];
    s_reach_bound[model] = norm3(description->end_xyz);
    for (int i = 0; i < CAN_CONSOLE_IK_DOF; ++i) {
        s_reach_bound[model] += norm3(description->joints[i].origin_xyz);
        make_origin(description->joints[i].origin_xyz, description->joints[i].origin_rpy,
                    &s_joint_origins[model][i]);
    }
    make_origin(description->end_xyz, description->end_rpy, &s_end_origin[model]);
}

void can_console_ik_init(void)
{
    if (atomic_load_explicit(&s_initialized, memory_order_acquire)) return;
    /* A task may preempt first initialization on the single-core target.
     * Use a blocking libc lock there, never spin behind a lower-priority task. */
#ifdef ESP_PLATFORM
    _lock_acquire(&s_init_lock);
#else
    while (atomic_flag_test_and_set_explicit(&s_init_lock, memory_order_acquire)) {}
#endif
    if (!atomic_load_explicit(&s_initialized, memory_order_relaxed)) {
        init_model_frames(CAN_CONSOLE_IK_MODEL_REBOTARM_RS);
        init_model_frames(CAN_CONSOLE_IK_MODEL_REBOTARM_DM);
#if CAN_CONSOLE_IK_USE_TRIG_LUT
        init_trig_lut();
#endif
        atomic_store_explicit(&s_initialized, true, memory_order_release);
    }
#ifdef ESP_PLATFORM
    _lock_release(&s_init_lock);
#else
    atomic_flag_clear_explicit(&s_init_lock, memory_order_release);
#endif
}

bool can_console_ik_select_model(can_console_ik_model_t model)
{
    if (!valid_model(model)) return false;
    can_console_ik_init();
    s_selected_model = model;
    return true;
}

can_console_ik_model_t can_console_ik_get_model(void)
{
    return s_selected_model;
}

const char *can_console_ik_model_name(can_console_ik_model_t model)
{
    if (model < CAN_CONSOLE_IK_MODEL_REBOTARM_RS ||
        model > CAN_CONSOLE_IK_MODEL_REBOTARM_DM) {
        return "unknown";
    }
    return k_arm_models[model].name;
}

void can_console_ik_default_options(can_console_ik_options_t *options)
{
    if (options == NULL) {
        return;
    }
    options->max_iterations = 50;
    options->damping = 0.03f;
    options->orientation_weight = 0.35f;
    options->max_step_rad = 0.20f;
    options->position_tolerance_m = 1.0e-4f;
    options->orientation_tolerance_rad = 1.0e-3f;
}

void can_console_ik_forward(
    const float q[CAN_CONSOLE_IK_DOF],
    float position[CAN_CONSOLE_IK_POSITION_DIM],
    float rotation[CAN_CONSOLE_IK_ROTATION_ELEMENTS])
{
    can_console_ik_init();
    compute_kinematics(s_selected_model, q, position, rotation, NULL, NULL);
}

bool can_console_ik_forward_chain_for_model(can_console_ik_model_t model, const float q[CAN_CONSOLE_IK_DOF],
                                  can_console_ik_chain_t *chain)
{
    if (!valid_model(model) || q == NULL || chain == NULL) {
        return false;
    }
    for (size_t i = 0; i < CAN_CONSOLE_IK_DOF; ++i) {
        if (!isfinite(q[i])) {
            return false;
        }
    }
    can_console_ik_init();
    compute_kinematics(model, q, chain->end_position, chain->end_rotation, NULL, chain);
    return true;
}

bool can_console_ik_solve_for_model(
    can_console_ik_model_t model,
    const float target_position[CAN_CONSOLE_IK_POSITION_DIM],
    const float target_rotation[CAN_CONSOLE_IK_ROTATION_ELEMENTS],
    const float initial_q[CAN_CONSOLE_IK_DOF],
    const can_console_ik_options_t *options,
    float solution_q[CAN_CONSOLE_IK_DOF],
    can_console_ik_result_t *result)
{
    can_console_ik_options_t local_options;
    float q[CAN_CONSOLE_IK_DOF];
    float lower_bound[CAN_CONSOLE_IK_DOF];
    float upper_bound[CAN_CONSOLE_IK_DOF];
    bool converged = false;
    uint16_t iterations = 0;
    if (result != NULL) {
        *result = (can_console_ik_result_t){
            .position_error_m = INFINITY, .orientation_error_rad = INFINITY,
        };
    }

    if (!valid_model(model) || target_position == NULL || target_rotation == NULL || initial_q == NULL ||
        solution_q == NULL) {
        return false;
    }

    can_console_ik_init();
    if (options == NULL) {
        can_console_ik_default_options(&local_options);
        options = &local_options;
    }
    for (int i = 0; i < 3; ++i) {
        if (!isfinite(target_position[i])) return false;
    }
    for (int i = 0; i < 9; ++i) {
        if (!isfinite(target_rotation[i])) return false;
    }
    for (int i = 0; i < CAN_CONSOLE_IK_DOF; ++i) {
        if (!isfinite(initial_q[i])) return false;
    }
    if (!isfinite(options->damping) || options->damping < 0.0f ||
        !isfinite(options->orientation_weight) || options->orientation_weight < 0.0f ||
        !isfinite(options->max_step_rad) || options->max_step_rad <= 0.0f ||
        !isfinite(options->position_tolerance_m) || options->position_tolerance_m < 0.0f ||
        !isfinite(options->orientation_tolerance_rad) || options->orientation_tolerance_rad < 0.0f) {
        return false;
    }
    memcpy(q, initial_q, sizeof(q));
    for (int i = 0; i < CAN_CONSOLE_IK_DOF; ++i) {
        const joint_model_t *joint = &k_arm_models[model].joints[i];
        lower_bound[i] = joint->lower;
        upper_bound[i] = joint->upper;
        if (q[i] < joint->lower) {
            if (joint->lower - q[i] <=
                CAN_CONSOLE_IK_SEED_LIMIT_TOLERANCE_RAD) {
                lower_bound[i] = q[i];
            } else {
                q[i] = joint->lower;
            }
        } else if (q[i] > joint->upper) {
            if (q[i] - joint->upper <=
                CAN_CONSOLE_IK_SEED_LIMIT_TOLERANCE_RAD) {
                upper_bound[i] = q[i];
            } else {
                q[i] = joint->upper;
            }
        }
    }

    const float position_tolerance_squared =
        options->position_tolerance_m * options->position_tolerance_m;
    const float orientation_tolerance_squared =
        options->orientation_tolerance_rad * options->orientation_tolerance_rad;

    float best_q[CAN_CONSOLE_IK_DOF];
    float best_error[CAN_CONSOLE_IK_DOF] = {0};
    float best_cost = INFINITY;
    float progress_cost = INFINITY;
    unsigned stalled = 0U;
    const float weight_squared = options->orientation_weight * options->orientation_weight;
    /* Triangle inequality: a cheap conservative exclusion, not a workspace
     * or joint-limit feasibility test. Padding covers float/LUT roundoff. */
    const float reach = s_reach_bound[model] + options->position_tolerance_m + 1.0e-4f;
    const bool outside_reach = norm3_squared(target_position) > reach * reach;

    for (;;) {
        float current_position[CAN_CONSOLE_IK_POSITION_DIM];
        float current_rotation[CAN_CONSOLE_IK_ROTATION_ELEMENTS];
        float jacobian[CAN_CONSOLE_IK_DOF][CAN_CONSOLE_IK_DOF];
        float error[CAN_CONSOLE_IK_DOF];
        float delta_q[CAN_CONSOLE_IK_DOF];

        const bool can_step = iterations < options->max_iterations && !outside_reach;
        compute_kinematics(model, q, current_position, current_rotation,
                           can_step ? jacobian : NULL, NULL);
        pose_error(target_position, target_rotation, current_position,
                   current_rotation, error);
        const float position_squared = norm3_squared(error);
        const float orientation_squared = norm3_squared(&error[3]);
        const float cost = position_squared + weight_squared * orientation_squared;
        if (!isfinite(cost)) break;
        if (cost < best_cost) {
            best_cost = cost;
            memcpy(best_q, q, sizeof(q));
            memcpy(best_error, error, sizeof(error));
        }
        if (position_squared <= position_tolerance_squared &&
            orientation_squared <= orientation_tolerance_squared) {
            /* Both tolerances take precedence over the weighted objective. */
            memcpy(best_q, q, sizeof(q));
            memcpy(best_error, error, sizeof(error));
            converged = true;
            if (iterations < options->max_iterations) ++iterations;
            break;
        }
        if (!can_step) break;
        if (best_cost < progress_cost * (1.0f - 1.0e-4f)) {
            progress_cost = best_cost;
            stalled = 0U;
        } else if (++stalled >= 16U) {
            break;
        }
        if (!solve_damped_step(jacobian, error, options->damping,
                               options->orientation_weight, delta_q)) {
            break;
        }
        float largest = 0.0f;
        for (int i = 0; i < CAN_CONSOLE_IK_DOF; ++i) {
            if (!isfinite(delta_q[i])) goto finished;
            largest = fmaxf(largest, fabsf(delta_q[i]));
        }
        const float scale = largest > options->max_step_rad ?
            options->max_step_rad / largest : 1.0f;
        float actual_step = 0.0f;
        for (int i = 0; i < CAN_CONSOLE_IK_DOF; ++i) {
            const float next = clampf_local(q[i] + scale * delta_q[i],
                                            lower_bound[i], upper_bound[i]);
            actual_step = fmaxf(actual_step, fabsf(next - q[i]));
            q[i] = next;
        }
        ++iterations;
        if (actual_step == 0.0f) break;
    }

finished:
    if (!isfinite(best_cost)) return false;
    memcpy(solution_q, best_q, sizeof(best_q));
    if (result != NULL) {
        result->converged = converged;
        result->iterations = iterations;
        result->position_error_m = norm3(best_error);
        result->orientation_error_rad = norm3(&best_error[3]);
    }
    return converged;
}

bool can_console_ik_forward_chain(const float q[6], can_console_ik_chain_t *chain)
{
    return can_console_ik_forward_chain_for_model(s_selected_model, q, chain);
}

bool can_console_ik_solve(const float position[3], const float rotation[9],
                          const float seed[6], const can_console_ik_options_t *options,
                          float solution[6], can_console_ik_result_t *result)
{
    return can_console_ik_solve_for_model(s_selected_model, position, rotation, seed, options, solution, result);
}

static float rotation_distance_rad(
    const float a[CAN_CONSOLE_IK_ROTATION_ELEMENTS],
    const float b[CAN_CONSOLE_IK_ROTATION_ELEMENTS])
{
    float element_sum = 0.0f;
    for (int i = 0; i < CAN_CONSOLE_IK_ROTATION_ELEMENTS; ++i) {
        element_sum += a[i] * b[i];
    }
    const float cosine = clampf_local(0.5f * (element_sum - 1.0f), -1.0f, 1.0f);
    return acosf(cosine);
}

static void benchmark_case(const char *name,
                           const float target_q[CAN_CONSOLE_IK_DOF],
                           const float initial_q[CAN_CONSOLE_IK_DOF],
                           const can_console_ik_options_t *options)
{
    const int sample_count = 100;
    float target_position[CAN_CONSOLE_IK_POSITION_DIM];
    float target_rotation[CAN_CONSOLE_IK_ROTATION_ELEMENTS];
    float initial_position[CAN_CONSOLE_IK_POSITION_DIM];
    float initial_rotation[CAN_CONSOLE_IK_ROTATION_ELEMENTS];
    can_console_ik_result_t result;
    float solution[CAN_CONSOLE_IK_DOF];

    can_console_ik_forward(target_q, target_position, target_rotation);
    can_console_ik_forward(initial_q, initial_position, initial_rotation);

    float position_delta[CAN_CONSOLE_IK_POSITION_DIM] = {
        target_position[0] - initial_position[0],
        target_position[1] - initial_position[1],
        target_position[2] - initial_position[2],
    };
    const float initial_position_delta_m = norm3(position_delta);
    const float initial_orientation_delta_rad =
        rotation_distance_rad(initial_rotation, target_rotation);

    ESP_LOGI(TAG, "[%s] start delta: position=%.1f mm, orientation=%.1f deg",
             name, initial_position_delta_m * 1000.0f,
             initial_orientation_delta_rad *
                 (180.0f / CAN_CONSOLE_IK_PI));

    for (int i = 0; i < 3; ++i) {
        can_console_ik_solve(target_position, target_rotation, initial_q, options,
                             solution, &result);
    }

    int64_t total_us = 0;
    int64_t min_us = INT64_MAX;
    int64_t max_us = 0;
    uint32_t total_iterations = 0;
    uint16_t min_iterations = UINT16_MAX;
    uint16_t max_iterations = 0;
    int converged_count = 0;

    for (int i = 0; i < sample_count; ++i) {
        const int64_t start_us = esp_timer_get_time();
        const bool converged = can_console_ik_solve(
            target_position, target_rotation, initial_q, options, solution, &result);
        const int64_t elapsed_us = esp_timer_get_time() - start_us;
        total_us += elapsed_us;
        total_iterations += result.iterations;
        if (elapsed_us < min_us) {
            min_us = elapsed_us;
        }
        if (elapsed_us > max_us) {
            max_us = elapsed_us;
        }
        if (result.iterations < min_iterations) {
            min_iterations = result.iterations;
        }
        if (result.iterations > max_iterations) {
            max_iterations = result.iterations;
        }
        if (converged) {
            ++converged_count;
        }
    }

    ESP_LOGI(TAG,
             "[%s] samples=%d, converged=%d/%d, "
             "time min/avg/max=%lld/%lld/%lld us, "
             "iterations min/avg/max=%u/%u/%u",
             name, sample_count, converged_count, sample_count, min_us,
             total_us / sample_count, max_us, (unsigned int)min_iterations,
             (unsigned int)(total_iterations / sample_count),
             (unsigned int)max_iterations);
    ESP_LOGI(TAG,
             "[%s] solution: [%.5f, %.5f, %.5f, %.5f, %.5f, %.5f], "
             "pos_err=%.6f m, ori_err=%.6f rad",
             name, solution[0], solution[1], solution[2], solution[3], solution[4],
             solution[5], result.position_error_m, result.orientation_error_rad);
}

void can_console_ik_benchmark(void)
{
    static const float target_q[CAN_CONSOLE_IK_DOF] = {
        -0.30f, 1.00f, 1.20f, -0.20f, -0.30f, -0.40f,
    };
    static const struct {
        const char *name;
        float initial_q[CAN_CONSOLE_IK_DOF];
    } cases[] = {
        {"near", {-0.25f, 0.95f, 1.15f, -0.15f, -0.25f, -0.35f}},
        {"medium", {0.00f, 0.70f, 0.80f, 0.00f, 0.00f, 0.00f}},
        {"large", {0.30f, 0.40f, 0.40f, 0.20f, 0.30f, 0.40f}},
        {"very_large", {0.60f, 0.10f, 0.00f, 0.40f, 0.60f, 0.80f}},
        {"different_branch", {-1.20f, 2.40f, 2.60f, -1.00f, 1.00f, -1.50f}},
    };
    can_console_ik_options_t options;

    can_console_ik_default_options(&options);

    ESP_LOGI(TAG, "kinematic model: %s", active_model()->name);

#if CAN_CONSOLE_IK_USE_TRIG_LUT
    ESP_LOGI(TAG, "trig mode: %d-point LUT with linear interpolation",
             CAN_CONSOLE_IK_TRIG_LUT_SIZE);
#else
    ESP_LOGI(TAG, "trig mode: libm sincosf");
#endif

    for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i) {
        benchmark_case(cases[i].name, target_q, cases[i].initial_q, &options);
    }
}

    