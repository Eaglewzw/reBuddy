/*
 * SPDX-FileCopyrightText: 2026 XIAO-C5-CAN
 * SPDX-License-Identifier: Apache-2.0
 */

#include "can_console_arm_motion.h"
#include "can_console_pose_math.h"

#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#include "arm_trajectory.h"
#include "arm_joint_path.h"
#include "arm_stop.h"
#include "esp_log.h"
#include "esp_timer.h"
#ifdef ESP_PLATFORM
#include "esp_heap_caps.h"
#endif
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#define ARM_MOTION_DEFAULT_SAMPLE_PERIOD_MS 20U
#define ARM_MOTION_DEFAULT_STACK_SIZE       6144U
#define ARM_MOTION_DEFAULT_PRIORITY        4U
#define ARM_MOTION_DEFAULT_VELOCITY_LIMIT  1.0f
#define ARM_MOTION_MAX_POINTS               16384U
#define ARM_MOTION_INTERNAL_MAX_POINTS      4096U

/*
 * Once every step is divided by the profile's own step, a smooth joint path
 * keeps a neighbour ratio near one, so a single step this far above both of its
 * neighbours is a solver branch jump. A longer duration only slows a jump down,
 * it cannot remove the discontinuity, so such a plan is rejected instead.
 */
#define ARM_MOTION_JUMP_STEP_RATIO          8.0f

/*
 * Retiming recomputes the duration the velocity limits need, and the resampled
 * path lands within a fraction of a percent of the limit. Aiming one percent
 * inside it settles in a single pass.
 */
#define ARM_MOTION_RETIME_HEADROOM          1.01f

/*
 * The extra attempt absorbs the change in path resolution between two durations.
 */
#define ARM_MOTION_MAX_RETIMES              2U

static const char *TAG = "can_console_arm_motion";

static uint32_t path_point_limit(void)
{
#ifdef ESP_PLATFORM
    /* Keep the larger path budget out of devices without PSRAM. */
    if (heap_caps_get_total_size(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT) == 0) {
        return ARM_MOTION_INTERNAL_MAX_POINTS;
    }
#endif
    return ARM_MOTION_MAX_POINTS;
}

/* Full paths are not DMA buffers. Keep them out of the internal heap needed
 * by control tasks, even when SPIRAM_USE_MALLOC is disabled. If external RAM
 * is registered but exhausted, fail rather than consume the control reserve. */
static void *path_alloc(size_t bytes)
{
#ifdef ESP_PLATFORM
    const unsigned external = MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT;
    const unsigned internal = MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT;
    const bool use_external = heap_caps_get_total_size(external) > 0;
    void *buffer = heap_caps_malloc(bytes, use_external ? external : internal);
    if (buffer == NULL) {
        ESP_LOGE(TAG, "path allocation failed: bytes=%u heap=%s internal largest=%u PSRAM largest=%u",
                 (unsigned)bytes, use_external ? "PSRAM" : "internal",
                 (unsigned)heap_caps_get_largest_free_block(internal),
                 (unsigned)heap_caps_get_largest_free_block(external));
    }
    return buffer;
#else
    return malloc(bytes);
#endif
}

static void path_free(void *buffer)
{
#ifdef ESP_PLATFORM
    heap_caps_free(buffer);
#else
    free(buffer);
#endif
}

typedef can_console_arm_motion_sample_t motion_point_t;
typedef struct {
    float position[CAN_CONSOLE_ARM_MOTION_DOF];
} joint_position_t;

typedef struct {
    can_console_arm_motion_kind_t kind;
    float initial_q[CAN_CONSOLE_ARM_MOTION_DOF];
    float target_q[CAN_CONSOLE_ARM_MOTION_DOF];
    float target_position[CAN_CONSOLE_IK_POSITION_DIM];
    float target_rotation[CAN_CONSOLE_IK_ROTATION_ELEMENTS];
    float duration_s;
    arm_joint_path_view_t path;
    arm_joint_path_limits_t path_limits;
    float speed;
    bool original_timing;
    bool limited_joints;
    bool configured_pose;
    float pose_velocity_limits[6];
    can_console_ik_options_t pose_ik_options;
    uint32_t generation;
} motion_request_t;

/** Joint path accepted by the velocity-limit and branch checks. */
typedef struct {
    joint_position_t *points;
    uint32_t total_points;
    float duration_s;
    uint32_t ik_solves;
    uint32_t ik_total_us;
    uint32_t ik_max_us;
    can_console_arm_plan_diagnostic_t diagnostic;
} solved_plan_t;

typedef struct {
    bool initialized;
    bool planner_busy;
    bool started;
    bool request_pending;
    bool plan_complete;
    uint32_t generation;
    uint32_t produced_points;
    uint32_t consumed_points;
    uint32_t total_points;
    uint32_t buffer_low_water;
    uint32_t underruns;
    uint32_t ik_solves;
    uint64_t ik_total_us;
    uint32_t ik_max_us;
    float duration_s;
    float requested_speed;
    float effective_speed;
    float consumed_reference_time_s;
    can_console_arm_motion_kind_t kind;
    can_console_arm_motion_error_source_t error_source;
    can_console_arm_plan_diagnostic_t planning;
    arm_stop_t stop;
    bool stop_active;
    motion_request_t request;
    can_console_arm_motion_config_t config;
    can_console_arm_motion_state_t state;
    esp_err_t last_error;
    QueueHandle_t buffer;
    SemaphoreHandle_t lock;
    TaskHandle_t planner_task;
} motion_context_t;

static motion_context_t s_motion;

static bool finite_array(const float *values, size_t count)
{
    if (values == NULL) {
        return false;
    }
    for (size_t i = 0U; i < count; ++i) {
        if (!isfinite(values[i])) {
            return false;
        }
    }
    return true;
}

static void interpolate_pose(
    const float start_position[CAN_CONSOLE_IK_POSITION_DIM],
    const float start_rotation[CAN_CONSOLE_IK_ROTATION_ELEMENTS],
    const float target_position[CAN_CONSOLE_IK_POSITION_DIM],
    const float target_rotation[CAN_CONSOLE_IK_ROTATION_ELEMENTS],
    float s,
    float position[CAN_CONSOLE_IK_POSITION_DIM],
    float rotation[CAN_CONSOLE_IK_ROTATION_ELEMENTS])
{
    for (size_t i = 0U; i < CAN_CONSOLE_IK_POSITION_DIM; ++i) {
        position[i] = start_position[i] +
                      (target_position[i] - start_position[i]) * s;
    }

    float start_q[4];
    float target_q[4];
    float interpolated_q[4];
    can_console_pose_rotation_to_quaternion(start_rotation, start_q);
    can_console_pose_rotation_to_quaternion(target_rotation, target_q);
    can_console_pose_quaternion_slerp(start_q, target_q, s, interpolated_q);
    can_console_pose_quaternion_to_rotation(interpolated_q, rotation);
}

/* One point of the Cartesian path, starting at the arm's measured pose. */
static void sample_path(const motion_request_t *request,
                        const float start_position[CAN_CONSOLE_IK_POSITION_DIM],
                        const float start_rotation[CAN_CONSOLE_IK_ROTATION_ELEMENTS],
                        float amount,
                        float position[CAN_CONSOLE_IK_POSITION_DIM],
                        float rotation[CAN_CONSOLE_IK_ROTATION_ELEMENTS])
{
    const float s = arm_trajectory_profile(amount);
    interpolate_pose(start_position, start_rotation, request->target_position,
                     request->target_rotation, s, position, rotation);
}

/** Caller holds the lock. Skips 0 so a wrapped generation never aliases a fresh one. */
static void bump_generation(void)
{
    if (++s_motion.generation == 0U) {
        s_motion.generation = 1U;
    }
}

/* The generation stops matching as soon as plan_pose or cancel bumps it. */
static bool generation_is_current(uint32_t generation)
{
    bool current;
    xSemaphoreTake(s_motion.lock, portMAX_DELAY);
    current = s_motion.generation == generation;
    xSemaphoreGive(s_motion.lock);
    return current;
}

static bool push_point(uint32_t generation, const motion_point_t *point)
{
    for (;;) {
        xSemaphoreTake(s_motion.lock, portMAX_DELAY);
        const bool current = s_motion.generation == generation;
        BaseType_t pushed = pdFALSE;
        if (current) {
            pushed = xQueueSend(s_motion.buffer, point, 0);
        }
        if (!current) {
            xSemaphoreGive(s_motion.lock);
            return false;
        }
        if (pushed == pdTRUE) {
            /* Publish under the same generation check as the queue insertion.
             * Cancel/new request cannot steal an old worker's counters. */
            s_motion.produced_points++;
            if (s_motion.state == CAN_CONSOLE_ARM_MOTION_PLANNING) {
                s_motion.state = CAN_CONSOLE_ARM_MOTION_RUNNING;
            }
            xSemaphoreGive(s_motion.lock);
            return true;
        }
        xSemaphoreGive(s_motion.lock);
        /* Must block for at least one tick: pdMS_TO_TICKS(1) rounds to zero
         * ticks at CONFIG_FREERTOS_HZ=100, degenerating this backpressure wait
         * into a yield spin that starves the idle task and trips the task
         * watchdog (whose ISR-context dump then stalls CAN RX). */
        vTaskDelay(1);
    }
}

/* Caller holds motion lock. Stops share the same queue and consumer. */
static void fill_stop_point(void)
{
    if (!s_motion.stop_active || uxQueueMessagesWaiting(s_motion.buffer) != 0U) return;
    motion_point_t point;
    arm_stop_sample(&s_motion.stop, s_motion.stop.next, &point);
    point.reference_time_s = s_motion.consumed_reference_time_s;
    (void)xQueueSend(s_motion.buffer, &point, 0);
    ++s_motion.produced_points;
    if (s_motion.stop.next++ == s_motion.stop.intervals) {
        s_motion.stop_active = false;
        s_motion.plan_complete = true;
    }
}

/* Caller holds motion lock. COMPLETE only leaves a non-ERROR motion; this is
 * the one completion transition, shared by the consumer and producer sides. */
static void motion_complete_if_drained_locked(void)
{
    if (s_motion.plan_complete &&
        s_motion.state != CAN_CONSOLE_ARM_MOTION_ERROR &&
        uxQueueMessagesWaiting(s_motion.buffer) == 0U) {
        s_motion.state = CAN_CONSOLE_ARM_MOTION_COMPLETE;
    }
}

/* Caller holds motion lock. Bookkeeping shared by every point consumer. */
static void motion_note_consumed_locked(const motion_point_t *point)
{
    ++s_motion.consumed_points;
    s_motion.consumed_reference_time_s = point->reference_time_s;
    if (!s_motion.plan_complete) {
        const uint32_t depth = (uint32_t)uxQueueMessagesWaiting(s_motion.buffer);
        if (depth < s_motion.buffer_low_water) {
            s_motion.buffer_low_water = depth;
        }
    }
    motion_complete_if_drained_locked();
}

static bool take_point(motion_point_t *point)
{
    bool taken;
    xSemaphoreTake(s_motion.lock, portMAX_DELAY);
    fill_stop_point();
    taken = xQueueReceive(s_motion.buffer, point, 0) == pdTRUE;
    if (taken) {
        motion_note_consumed_locked(point);
    } else if (s_motion.state == CAN_CONSOLE_ARM_MOTION_RUNNING &&
               !s_motion.plan_complete && s_motion.consumed_points > 0U) {
        s_motion.underruns++;
    }
    xSemaphoreGive(s_motion.lock);
    return taken;
}

/*
 * Solve the whole path, seeding every solve with the previous solution so the
 * branch stays continuous along the path.
 */
static esp_err_t solve_path(const motion_request_t *request,
                            joint_position_t *points,
                            uint32_t total_points,
                            solved_plan_t *plan)
{
    float start_position[CAN_CONSOLE_IK_POSITION_DIM];
    float start_rotation[CAN_CONSOLE_IK_ROTATION_ELEMENTS];
    float q_seed[CAN_CONSOLE_ARM_MOTION_DOF];

    if (request->kind == CAN_CONSOLE_ARM_MOTION_JOINT) {
        memset(start_position, 0, sizeof(start_position));
        memset(start_rotation, 0, sizeof(start_rotation));
    } else {
        can_console_ik_forward(request->initial_q, start_position, start_rotation);
    }
    memcpy(q_seed, request->initial_q, sizeof(q_seed));

    for (uint32_t i = 0U; i < total_points; ++i) {
        if (!generation_is_current(request->generation)) {
            return ESP_ERR_INVALID_STATE;
        }

        float solved_q[CAN_CONSOLE_ARM_MOTION_DOF];
        if (request->kind == CAN_CONSOLE_ARM_MOTION_JOINT) {
            const float s = arm_trajectory_profile(
                (float)i / (float)(total_points - 1U));
            for (size_t joint = 0U; joint < CAN_CONSOLE_ARM_MOTION_DOF; ++joint) {
                solved_q[joint] = request->initial_q[joint] +
                                  (request->target_q[joint] -
                                   request->initial_q[joint]) * s;
            }
        } else {
            float sample_position[CAN_CONSOLE_IK_POSITION_DIM];
            float sample_rotation[CAN_CONSOLE_IK_ROTATION_ELEMENTS];
            can_console_ik_result_t result = {0};
            sample_path(request, start_position, start_rotation,
                        (float)i / (float)(total_points - 1U),
                        sample_position, sample_rotation);
            const int64_t start_us = esp_timer_get_time();
            const bool solved = can_console_ik_solve(
                sample_position, sample_rotation, q_seed,
                request->configured_pose ? &request->pose_ik_options : &s_motion.config.ik_options, solved_q, &result);
            const uint32_t elapsed_us =
                (uint32_t)(esp_timer_get_time() - start_us);
            plan->ik_solves++;
            plan->ik_total_us += elapsed_us;
            if (elapsed_us > plan->ik_max_us) {
                plan->ik_max_us = elapsed_us;
            }

            if (!solved) {
                plan->diagnostic.reason=CAN_CONSOLE_ARM_PLAN_IK_FAILED;
                plan->diagnostic.point=i+1U;plan->diagnostic.points=total_points;
                plan->diagnostic.ik=result;
                ESP_LOGE(TAG,
                         "inverse kinematics failed at point %u/%u: "
                         "position error %.6f m, orientation error %.6f rad",
                         (unsigned)i, (unsigned)total_points,
                         result.position_error_m, result.orientation_error_rad);
                return ESP_FAIL;
            }
        }

        memcpy(points[i].position, solved_q, sizeof(solved_q));
        memcpy(q_seed, solved_q, sizeof(q_seed));
    }
    return ESP_OK;
}

/*
 * Largest per-sample joint step along a solved path, and whether any single step
 * is a solver branch jump.
 *
 * Dividing every step by the profile's own step takes the timing profile out of
 * the comparison, so what remains between two neighbouring steps is the joint
 * path itself. A smooth path keeps that near one, so a step eight times its
 * neighbours means the solver changed branch - or that the path grazes a
 * kinematic singularity, which compresses motion the same way and is refused
 * just as well.
 *
 * The step also has to be a real motion, not just an odd-looking one. Near the
 * ends of the path the profile's step is thousands of times smaller than at its
 * peak, so dividing by it magnifies the last bits of the solver's own precision
 * until a fraction of a milliradian looks like a spike. Requiring the step to
 * exceed the velocity limit as well keeps that noise out while still catching
 * any jump that would command a violent motion.
 */
static void measure_path(const joint_position_t *points,
                         uint32_t total_points,
                         const float step_limit[CAN_CONSOLE_ARM_MOTION_DOF],
                         float max_step[CAN_CONSOLE_ARM_MOTION_DOF],
                         bool *branch_jump, can_console_arm_plan_diagnostic_t *diagnostic)
{
    const uint32_t intervals = total_points - 1U;
    const float du = 1.0f / (float)intervals;

    for (size_t i = 0U; i < CAN_CONSOLE_ARM_MOTION_DOF; ++i) {
        max_step[i] = 0.0f;
    }
    for (uint32_t k = 0U; k < intervals; ++k) {
        for (size_t i = 0U; i < CAN_CONSOLE_ARM_MOTION_DOF; ++i) {
            const float step =
                fabsf(points[k + 1U].position[i] - points[k].position[i]);
            if (step > max_step[i]) {
                max_step[i] = step;
            }
        }
    }

    *branch_jump = false;
    for (uint32_t k = 1U; k + 1U < intervals; ++k) {
        const float previous_profile =
            arm_trajectory_profile((float)k * du) -
            arm_trajectory_profile((float)(k - 1U) * du);
        const float current_profile =
            arm_trajectory_profile((float)(k + 1U) * du) -
            arm_trajectory_profile((float)k * du);
        const float next_profile =
            arm_trajectory_profile((float)(k + 2U) * du) -
            arm_trajectory_profile((float)(k + 1U) * du);

        for (size_t i = 0U; i < CAN_CONSOLE_ARM_MOTION_DOF; ++i) {
            const float current =
                fabsf(points[k + 1U].position[i] - points[k].position[i]);
            if (current <= step_limit[i]) {
                continue;
            }
            const float previous =
                fabsf(points[k].position[i] - points[k - 1U].position[i]) /
                previous_profile;
            const float next =
                fabsf(points[k + 2U].position[i] - points[k + 1U].position[i]) /
                next_profile;
            const float neighbours = fmaxf(previous, next);
            const float normalized = current / current_profile;
            if (normalized > ARM_MOTION_JUMP_STEP_RATIO * neighbours) {
                ESP_LOGE(TAG,
                         "point %u/%u joint %u: step %.5f rad is above the %.5f "
                         "rad velocity limit and %.1f x its neighbours",
                         (unsigned)(k + 1U), (unsigned)intervals, (unsigned)(i + 1U),
                         (double)current, (double)step_limit[i],
                         (double)(neighbours > 0.0f ? normalized / neighbours
                                                    : normalized));
                diagnostic->reason=CAN_CONSOLE_ARM_PLAN_DISCONTINUITY;
                diagnostic->point=k+2U;diagnostic->points=total_points;diagnostic->joint=i+1U;
                *branch_jump = true;
                return;
            }
        }
    }
}

/*
 * Duration the velocity limits need for the solved path. A longer duration
 * resamples the same path more finely, so the step, and with it the joint speed
 * max_step[i] / step_limit[i] * velocity_limits[i], falls as 1/duration.
 */
static float required_duration(
    const float max_step[CAN_CONSOLE_ARM_MOTION_DOF],
    const float step_limit[CAN_CONSOLE_ARM_MOTION_DOF],
    float duration_s)
{
    float ratio = 1.0f;
    for (size_t i = 0U; i < CAN_CONSOLE_ARM_MOTION_DOF; ++i) {
        const float joint_ratio = max_step[i] / step_limit[i];
        if (joint_ratio > ratio) {
            ratio = joint_ratio;
        }
    }
    return duration_s * ratio;
}

/*
 * Turn one request into an accepted joint path, stretching the duration until
 * the velocity limits hold. On success plan->points is heap allocated and the
 * caller owns it; the inverse-kinematics cost is reported either way.
 */
static esp_err_t solve_plan(const motion_request_t *request, solved_plan_t *plan)
{
    const float sample_period_s =
        (float)s_motion.config.sample_period_ms / 1000.0f;
    float step_limit[CAN_CONSOLE_ARM_MOTION_DOF];
    for (size_t i = 0U; i < CAN_CONSOLE_ARM_MOTION_DOF; ++i) {
        step_limit[i] = (request->configured_pose ? request->pose_velocity_limits[i] : s_motion.config.velocity_limits[i]) * sample_period_s;
    }
    float minimum_duration = request->duration_s;
    if (request->limited_joints) {
        float lower[6], upper[6];
        if (!can_console_ik_get_joint_limits(lower, upper)) return ESP_ERR_INVALID_ARG;
        for (unsigned j=0;j<6;++j) {
            /* Match IK's measured-seed tolerance without moving the first point
             * away from the actual encoder reading. Targets remain in range. */
            if (request->initial_q[j]<lower[j]-CAN_CONSOLE_IK_SEED_LIMIT_TOLERANCE_RAD ||
                request->initial_q[j]>upper[j]+CAN_CONSOLE_IK_SEED_LIMIT_TOLERANCE_RAD ||
                request->target_q[j]<lower[j] || request->target_q[j]>upper[j]) {
                plan->diagnostic.reason=CAN_CONSOLE_ARM_PLAN_INVALID_JOINTS;
                plan->diagnostic.joint=j+1U;return ESP_ERR_INVALID_ARG;
            }
            const float delta=fabsf(request->target_q[j]-request->initial_q[j]);
            const float velocity=fminf(request->path_limits.velocity[j],s_motion.config.velocity_limits[j]);
            minimum_duration=fmaxf(minimum_duration,1.875f*delta/velocity);
            minimum_duration=fmaxf(minimum_duration,sqrtf(5.773503f*delta/request->path_limits.acceleration[j]));
        }
    }
    float duration_s = arm_trajectory_align_duration(minimum_duration,
                                                     sample_period_s);

    const uint32_t point_limit = path_point_limit();
    for (uint32_t retime = 0U; retime <= ARM_MOTION_MAX_RETIMES; ++retime) {
        /* Bound the float before conversion: huge finite UI targets must not
         * overflow lroundf or wrap the allocation/sample count. */
        plan->diagnostic.duration_s=duration_s;
        if (!isfinite(duration_s) || duration_s > (point_limit-1U)*sample_period_s) {
            plan->diagnostic.reason=CAN_CONSOLE_ARM_PLAN_POINT_LIMIT;
            ESP_LOGE(TAG,"path duration %.3f s exceeds %u-point budget (%.3f s)",
                (double)duration_s,(unsigned)point_limit,
                (double)((point_limit-1U)*sample_period_s));
            return ESP_ERR_INVALID_ARG;
        }
        const uint32_t total_points =
            (uint32_t)lroundf(duration_s / sample_period_s) + 1U;
        plan->diagnostic.points=total_points;
        if (total_points > point_limit) {
            plan->diagnostic.reason=CAN_CONSOLE_ARM_PLAN_POINT_LIMIT;
            ESP_LOGE(TAG, "%.2f s needs %u points, above the %u point limit",
                     duration_s, (unsigned)total_points,
                     (unsigned)point_limit);
            return ESP_ERR_INVALID_ARG;
        }

        joint_position_t *points = path_alloc(total_points * sizeof(joint_position_t));
        if (points == NULL) {
            plan->diagnostic.reason=CAN_CONSOLE_ARM_PLAN_NO_MEMORY;
            ESP_LOGE(TAG, "no memory for %u points", (unsigned)total_points);
            return ESP_ERR_NO_MEM;
        }

        esp_err_t err = solve_path(request, points, total_points, plan);
        if (err != ESP_OK) {
            path_free(points);
            return err;
        }

        float max_step[CAN_CONSOLE_ARM_MOTION_DOF];
        bool branch_jump = false;
        measure_path(points, total_points, step_limit, max_step, &branch_jump, &plan->diagnostic);
        if (branch_jump && request->configured_pose &&
            request->kind == CAN_CONSOLE_ARM_MOTION_POSE) {
            /* A loose endpoint tolerance can accept the same seed for several
             * tiny Cartesian samples, then jump on the first sample outside
             * that tolerance. Re-solve once at finer sampling precision; never
             * bypass the velocity or branch checks on the replacement path. */
            motion_request_t refined = *request;
            refined.pose_ik_options.position_tolerance_m =
                fminf(refined.pose_ik_options.position_tolerance_m, 5e-6f);
            refined.pose_ik_options.orientation_tolerance_rad =
                fminf(refined.pose_ik_options.orientation_tolerance_rad, 8.717797887e-5f);
            if (refined.pose_ik_options.position_tolerance_m < request->pose_ik_options.position_tolerance_m ||
                refined.pose_ik_options.orientation_tolerance_rad < request->pose_ik_options.orientation_tolerance_rad) {
                ESP_LOGI(TAG,"recheck path discontinuity with finer intermediate IK precision");
                plan->diagnostic.reason=CAN_CONSOLE_ARM_PLAN_OK;
                plan->diagnostic.point=plan->diagnostic.joint=0;
                err=solve_path(&refined,points,total_points,plan);
                if(err!=ESP_OK) { path_free(points);return err; }
                measure_path(points,total_points,step_limit,max_step,&branch_jump,&plan->diagnostic);
            }
        }
        if (branch_jump) {
            path_free(points);
            return ESP_ERR_INVALID_STATE;
        }

        const float needed_s = required_duration(max_step, step_limit, duration_s);
        if (needed_s <= duration_s) {
            plan->points = points;
            plan->total_points = total_points;
            plan->duration_s = duration_s;
            return ESP_OK;
        }
        path_free(points);

        if (retime == ARM_MOTION_MAX_RETIMES) {
            plan->diagnostic.reason=CAN_CONSOLE_ARM_PLAN_SPEED_LIMIT;
            plan->diagnostic.duration_s=needed_s;
            ESP_LOGE(TAG, "joint velocity limits still need %.2f s for %.2f s",
                     needed_s, duration_s);
            return ESP_ERR_INVALID_ARG;
        }
        duration_s = arm_trajectory_align_duration(
            needed_s * ARM_MOTION_RETIME_HEADROOM, sample_period_s);
        ESP_LOGD(TAG, "retimed to %.2f s for the joint velocity limits",
                 duration_s);
    }
    return ESP_ERR_INVALID_ARG;
}

static void publish_solve(uint32_t generation, esp_err_t err,
                          const solved_plan_t *plan)
{
    xSemaphoreTake(s_motion.lock, portMAX_DELAY);
    if (s_motion.generation == generation) {
        s_motion.planning=plan->diagnostic;
        s_motion.total_points = plan->total_points;
        s_motion.duration_s = plan->duration_s;
        s_motion.ik_solves += plan->ik_solves;
        s_motion.ik_total_us += plan->ik_total_us;
        if (plan->ik_max_us > s_motion.ik_max_us) {
            s_motion.ik_max_us = plan->ik_max_us;
        }
        if (err != ESP_OK) {
            s_motion.last_error = err;
            s_motion.error_source = CAN_CONSOLE_ARM_MOTION_ERROR_PLANNING;
            s_motion.state = CAN_CONSOLE_ARM_MOTION_ERROR;
        }
    }
    xSemaphoreGive(s_motion.lock);
}

/* Called once per request, after the points have been handed to the buffer. */
static void publish_complete(uint32_t generation)
{
    xSemaphoreTake(s_motion.lock, portMAX_DELAY);
    if (s_motion.generation == generation) {
        s_motion.plan_complete = true;
        motion_complete_if_drained_locked();
    }
    xSemaphoreGive(s_motion.lock);
}

static bool path_request_current(void *context)
{
    const motion_request_t *request = context;
    return generation_is_current(request->generation);
}

static void stream_joint_path(const motion_request_t *request)
{
    float lower[CAN_CONSOLE_ARM_MOTION_DOF], upper[CAN_CONSOLE_ARM_MOTION_DOF];
    arm_joint_path_plan_t path_plan = {0};
    esp_err_t error = ESP_ERR_INVALID_ARG;
    if (can_console_ik_get_joint_limits(lower, upper)) {
        error = arm_joint_path_prepare(&request->path, request->original_timing ? NULL : &request->path_limits, lower, upper,
            s_motion.config.sample_period_ms, request->speed, path_request_current,
            (void *)request, &path_plan);
    }
    const solved_plan_t summary = {
        .total_points = path_plan.total_points, .duration_s = (float)path_plan.duration_s,
    };
    publish_solve(request->generation, error, &summary);
    if (error != ESP_OK) return;
    xSemaphoreTake(s_motion.lock, portMAX_DELAY);
    if (s_motion.generation == request->generation) {
        s_motion.effective_speed = (float)path_plan.effective_speed;
    }
    xSemaphoreGive(s_motion.lock);

    /* Only one local segment and one outgoing point; no resampled path array.
     * A double time cursor avoids drift even for very slow, long recordings. */
    uint32_t segment_index = 0U;
    arm_joint_path_segment_t segment;
    arm_joint_path_make_segment(&request->path, segment_index, &segment);
    const uint32_t final_ms = request->path.samples[request->path.count - 1U].time_ms;
    for (uint32_t i = 0U; i < path_plan.total_points; ++i) {
        if (!generation_is_current(request->generation)) return;
        const bool last = i + 1U == path_plan.total_points;
        const double time_ms = last ? final_ms :
            (double)i * path_plan.sample_period_ms * path_plan.effective_speed;
        while (segment_index + 2U < request->path.count &&
               time_ms > request->path.samples[segment_index + 1U].time_ms) {
            ++segment_index;
            arm_joint_path_make_segment(&request->path, segment_index, &segment);
        }
        motion_point_t point = {0};
        const double u = (time_ms - request->path.samples[segment_index].time_ms) /
            (segment.duration_s * 1000.0);
        arm_joint_path_evaluate(&segment, u, path_plan.effective_speed, &point);
        if (i == 0U || last) {
            const uint32_t raw_index = last ? request->path.count - 1U : 0U;
            for (size_t j = 0U; j < CAN_CONSOLE_ARM_MOTION_DOF; ++j) {
                point.position[j] = request->path.samples[raw_index].position_q[j] / ARM_TEACH_POSITION_SCALE;
                point.reference_velocity[j] = 0.0f;
            }
        }
        point.reference_time_s = (float)(time_ms * .001);
        if (!push_point(request->generation, &point)) return;
    }
}

static void run_request(const motion_request_t *request)
{
    if (request->kind == CAN_CONSOLE_ARM_MOTION_JOINT_PATH) {
        stream_joint_path(request);
    } else {
        solved_plan_t plan = {0};
        const esp_err_t err = solve_plan(request, &plan);
        publish_solve(request->generation, err, &plan);
        if (err == ESP_OK) {
            for (uint32_t i = 0U; i < plan.total_points; ++i) {
                motion_point_t point = {0};
                point.reference_time_s=(float)i*s_motion.config.sample_period_ms*.001f;
                memcpy(point.position, plan.points[i].position, sizeof(point.position));
                if (request->limited_joints) {
                    const float u=(float)i/(float)(plan.total_points-1U);
                    for (unsigned j=0;j<6;++j) point.reference_velocity[j]=
                        (request->target_q[j]-request->initial_q[j])*30*u*u*(1-u)*(1-u)/plan.duration_s;
                }
                if (!push_point(request->generation, &point)) break;
            }
        }
        path_free(plan.points);
    }
    publish_complete(request->generation);
    xSemaphoreTake(s_motion.lock, portMAX_DELAY);
    s_motion.planner_busy = false;
    xSemaphoreGive(s_motion.lock);
}

static bool take_request(motion_request_t *request)
{
    bool pending;
    xSemaphoreTake(s_motion.lock, portMAX_DELAY);
    pending = s_motion.request_pending;
    if (pending) {
        s_motion.planner_busy = true;
        *request = s_motion.request;
        s_motion.request_pending = false;
        s_motion.plan_complete = false;
        s_motion.state = CAN_CONSOLE_ARM_MOTION_PLANNING;
        s_motion.last_error = ESP_OK;
    }
    xSemaphoreGive(s_motion.lock);
    return pending;
}

static void planner_task(void *arg)
{
    (void)arg;

    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        motion_request_t request;
        while (take_request(&request)) {
            run_request(&request);
        }
    }
}

void can_console_arm_motion_config_set_defaults(
    can_console_arm_motion_config_t *config)
{
    if (config == NULL) {
        return;
    }

    memset(config, 0, sizeof(*config));
    for (size_t i = 0U; i < CAN_CONSOLE_ARM_MOTION_DOF; ++i) {
        config->motor_ids[i] = (uint8_t)(i + 1U);
        config->velocity_limits[i] = ARM_MOTION_DEFAULT_VELOCITY_LIMIT;
    }
    config->sample_period_ms = ARM_MOTION_DEFAULT_SAMPLE_PERIOD_MS;
    config->planner_stack_size = ARM_MOTION_DEFAULT_STACK_SIZE;
    config->planner_priority = ARM_MOTION_DEFAULT_PRIORITY;
    can_console_ik_default_options(&config->ik_options);
}

esp_err_t can_console_arm_motion_init(
    const can_console_arm_motion_config_t *config)
{
    if (s_motion.initialized || config == NULL ||
        config->sample_period_ms == 0U || config->sample_period_ms > 1000U ||
        config->planner_stack_size == 0U || config->planner_priority == 0U) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!finite_array(config->velocity_limits,
                      CAN_CONSOLE_ARM_MOTION_DOF)) {
        return ESP_ERR_INVALID_ARG;
    }
    for (size_t i = 0U; i < CAN_CONSOLE_ARM_MOTION_DOF; ++i) {
        if (config->motor_ids[i] == 0U || config->velocity_limits[i] <= 0.0f) {
            return ESP_ERR_INVALID_ARG;
        }
    }

    memset(&s_motion, 0, sizeof(s_motion));
    s_motion.config = *config;
    s_motion.state = CAN_CONSOLE_ARM_MOTION_IDLE;
    s_motion.last_error = ESP_OK;
    s_motion.buffer_low_water = CAN_CONSOLE_ARM_MOTION_BUFFER_CAPACITY;
    s_motion.buffer = xQueueCreate(CAN_CONSOLE_ARM_MOTION_BUFFER_CAPACITY,
                                   sizeof(motion_point_t));
    s_motion.lock = xSemaphoreCreateMutex();
    if (s_motion.buffer == NULL || s_motion.lock == NULL) {
        if (s_motion.buffer != NULL) {
            vQueueDelete(s_motion.buffer);
        }
        if (s_motion.lock != NULL) {
            vSemaphoreDelete(s_motion.lock);
        }
        memset(&s_motion, 0, sizeof(s_motion));
        return ESP_ERR_NO_MEM;
    }

    s_motion.initialized = true;
    return ESP_OK;
}

esp_err_t can_console_arm_motion_start(void)
{
    if (!s_motion.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (s_motion.started) {
        return ESP_OK;
    }

    if (xTaskCreate(planner_task, "arm_ik_planner",
                    s_motion.config.planner_stack_size, NULL,
                    (UBaseType_t)s_motion.config.planner_priority,
                    &s_motion.planner_task) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    s_motion.started = true;
    return ESP_OK;
}

esp_err_t can_console_arm_motion_set_velocity_limit(float velocity_limit)
{
    if (!s_motion.initialized || !isfinite(velocity_limit) ||
        velocity_limit <= 0.0f) {
        return ESP_ERR_INVALID_ARG;
    }

    xSemaphoreTake(s_motion.lock, portMAX_DELAY);
    if (s_motion.state == CAN_CONSOLE_ARM_MOTION_PLANNING ||
        s_motion.state == CAN_CONSOLE_ARM_MOTION_RUNNING) {
        xSemaphoreGive(s_motion.lock);
        return ESP_ERR_INVALID_STATE;
    }
    for (size_t i = 0U; i < CAN_CONSOLE_ARM_MOTION_DOF; ++i) {
        s_motion.config.velocity_limits[i] = velocity_limit;
    }
    xSemaphoreGive(s_motion.lock);
    return ESP_OK;
}

esp_err_t can_console_arm_motion_get_velocity_limits(float velocity_limits[6])
{
    if(!s_motion.initialized || velocity_limits==NULL)return ESP_ERR_INVALID_ARG;
    xSemaphoreTake(s_motion.lock,portMAX_DELAY);
    memcpy(velocity_limits,s_motion.config.velocity_limits,sizeof(s_motion.config.velocity_limits));
    xSemaphoreGive(s_motion.lock);
    return ESP_OK;
}

/* Hand an accepted request to the planner task. Takes the request by value copy. */
static esp_err_t submit_request(const motion_request_t *request)
{
    xSemaphoreTake(s_motion.lock, portMAX_DELAY);
    if ((request->kind == CAN_CONSOLE_ARM_MOTION_JOINT_PATH || request->limited_joints || request->configured_pose) &&
        (s_motion.planner_busy || s_motion.request_pending ||
         s_motion.state == CAN_CONSOLE_ARM_MOTION_PLANNING ||
         s_motion.state == CAN_CONSOLE_ARM_MOTION_RUNNING ||
         uxQueueMessagesWaiting(s_motion.buffer) != 0U)) {
        xSemaphoreGive(s_motion.lock);
        return ESP_ERR_INVALID_STATE;
    }
    bump_generation();
    s_motion.stop_active = false;
    s_motion.request = *request;
    if (request->configured_pose) for (unsigned j=0;j<6;++j)
        s_motion.request.pose_velocity_limits[j]=fminf(request->pose_velocity_limits[j],s_motion.config.velocity_limits[j]);
    if ((request->kind == CAN_CONSOLE_ARM_MOTION_JOINT_PATH && !request->original_timing) || request->limited_joints) {
        for (size_t j = 0U; j < CAN_CONSOLE_ARM_MOTION_DOF; ++j) {
            s_motion.request.path_limits.velocity[j] = fminf(
                request->path_limits.velocity[j], s_motion.config.velocity_limits[j]);
        }
    }
    s_motion.request.generation = s_motion.generation;
    s_motion.request_pending = true;
    s_motion.plan_complete = false;
    s_motion.produced_points = 0U;
    s_motion.consumed_points = 0U;
    s_motion.total_points = 0U;
    s_motion.duration_s = 0.0f;
    s_motion.requested_speed = request->speed;
    s_motion.effective_speed = 0.0f;
    s_motion.consumed_reference_time_s = 0.0f;
    s_motion.kind = request->kind;
    s_motion.error_source = CAN_CONSOLE_ARM_MOTION_ERROR_NONE;
    s_motion.planning=(can_console_arm_plan_diagnostic_t){0};
    s_motion.buffer_low_water = CAN_CONSOLE_ARM_MOTION_BUFFER_CAPACITY;
    s_motion.underruns = 0U;
    s_motion.last_error = ESP_OK;
    s_motion.state = CAN_CONSOLE_ARM_MOTION_PLANNING;
    xQueueReset(s_motion.buffer);
    xSemaphoreGive(s_motion.lock);

    xTaskNotifyGive(s_motion.planner_task);
    return ESP_OK;
}

esp_err_t can_console_arm_motion_plan_pose(
    const float initial_q[CAN_CONSOLE_ARM_MOTION_DOF],
    const float target_position[CAN_CONSOLE_IK_POSITION_DIM],
    const float target_rotation[CAN_CONSOLE_IK_ROTATION_ELEMENTS],
    float duration_s)
{
    if (!s_motion.initialized || !s_motion.started ||
        initial_q == NULL || target_position == NULL || target_rotation == NULL ||
        !finite_array(initial_q, CAN_CONSOLE_ARM_MOTION_DOF) ||
        !finite_array(target_position, CAN_CONSOLE_IK_POSITION_DIM) ||
        !finite_array(target_rotation, CAN_CONSOLE_IK_ROTATION_ELEMENTS) ||
        !isfinite(duration_s) || duration_s <= 0.0f) {
        return ESP_ERR_INVALID_ARG;
    }

    motion_request_t request = {0};
    memcpy(request.initial_q, initial_q, sizeof(request.initial_q));
    memcpy(request.target_position, target_position,
           sizeof(request.target_position));
    memcpy(request.target_rotation, target_rotation,
           sizeof(request.target_rotation));
    request.duration_s = duration_s;
    return submit_request(&request);
}

esp_err_t can_console_arm_motion_plan_pose_with_limits(
    const float initial_q[6], const float target_position[3],
    const float target_rotation[9], float duration_s,
    const float velocity_limits[6], const can_console_ik_options_t *options)
{
    if (!s_motion.initialized || !s_motion.started || !initial_q ||
        !target_position || !target_rotation || !velocity_limits || !options ||
        !finite_array(initial_q,6) || !finite_array(target_position,3) ||
        !finite_array(target_rotation,9) || !finite_array(velocity_limits,6) ||
        !isfinite(duration_s) || duration_s<=0 || !options->max_iterations ||
        !isfinite(options->damping) || options->damping<0 ||
        !isfinite(options->orientation_weight) || options->orientation_weight<0 ||
        !isfinite(options->max_step_rad) || options->max_step_rad<=0 ||
        !isfinite(options->position_tolerance_m) || options->position_tolerance_m<0 ||
        !isfinite(options->orientation_tolerance_rad) || options->orientation_tolerance_rad<0)
        return ESP_ERR_INVALID_ARG;
    for(unsigned j=0;j<6;++j)if(velocity_limits[j]<=0)return ESP_ERR_INVALID_ARG;
    motion_request_t request={0};
    request.configured_pose=true;request.pose_ik_options=*options;
    memcpy(request.initial_q,initial_q,sizeof(request.initial_q));
    memcpy(request.target_position,target_position,sizeof(request.target_position));
    memcpy(request.target_rotation,target_rotation,sizeof(request.target_rotation));
    memcpy(request.pose_velocity_limits,velocity_limits,sizeof(request.pose_velocity_limits));
    request.duration_s=duration_s;
    return submit_request(&request);
}

esp_err_t can_console_arm_motion_plan_joints(
    const float initial_q[CAN_CONSOLE_ARM_MOTION_DOF],
    const float target_q[CAN_CONSOLE_ARM_MOTION_DOF],
    float duration_s)
{
    if (!s_motion.initialized || !s_motion.started || initial_q == NULL ||
        target_q == NULL || !finite_array(initial_q, CAN_CONSOLE_ARM_MOTION_DOF) ||
        !finite_array(target_q, CAN_CONSOLE_ARM_MOTION_DOF) ||
        !isfinite(duration_s) || duration_s <= 0.0f) {
        return ESP_ERR_INVALID_ARG;
    }

    motion_request_t request = {0};
    request.kind = CAN_CONSOLE_ARM_MOTION_JOINT;
    memcpy(request.initial_q, initial_q, sizeof(request.initial_q));
    memcpy(request.target_q, target_q, sizeof(request.target_q));
    request.duration_s = duration_s;
    return submit_request(&request);
}

esp_err_t can_console_arm_motion_plan_joint_path(const arm_joint_path_view_t *path,
    const arm_joint_path_limits_t *limits, float speed)
{
    if (!s_motion.initialized || !s_motion.started || path == NULL ||
        path->samples == NULL || path->count < 2U || path->count > ARM_TEACH_MAX_SAMPLES ||
        !isfinite(speed) || speed < .1f || speed > 1.0f ||
        (limits != NULL && (!finite_array(limits->velocity, CAN_CONSOLE_ARM_MOTION_DOF) ||
        !finite_array(limits->acceleration, CAN_CONSOLE_ARM_MOTION_DOF)))) return ESP_ERR_INVALID_ARG;
    for (size_t j = 0U; limits != NULL && j < CAN_CONSOLE_ARM_MOTION_DOF; ++j) {
        if (limits->velocity[j] <= 0.0f || limits->acceleration[j] <= 0.0f) return ESP_ERR_INVALID_ARG;
    }
    motion_request_t request = {
        .kind = CAN_CONSOLE_ARM_MOTION_JOINT_PATH, .path = *path,
        .original_timing = limits == NULL, .speed = speed,
    };
    if (limits != NULL) request.path_limits = *limits;
    return submit_request(&request);
}

esp_err_t can_console_arm_motion_plan_joints_limited(const float initial_q[6],
    const float target_q[6], float duration_s, const arm_joint_path_limits_t *limits)
{
    if (!s_motion.initialized || !s_motion.started || !finite_array(initial_q,6) ||
        !finite_array(target_q,6) || !isfinite(duration_s) || duration_s<=0 || limits==NULL ||
        !finite_array(limits->velocity,6) || !finite_array(limits->acceleration,6)) return ESP_ERR_INVALID_ARG;
    for(unsigned j=0;j<6;++j) if(limits->velocity[j]<=0 || limits->acceleration[j]<=0) return ESP_ERR_INVALID_ARG;
    motion_request_t request={.kind=CAN_CONSOLE_ARM_MOTION_JOINT,.limited_joints=true,
        .duration_s=duration_s,.path_limits=*limits};
    memcpy(request.initial_q,initial_q,sizeof(request.initial_q));
    memcpy(request.target_q,target_q,sizeof(request.target_q));
    return submit_request(&request);
}

esp_err_t can_console_arm_motion_plan_stop(const float position[6],const float velocity[6],
    const arm_joint_path_limits_t *limits,uint32_t step_ms)
{
    if(!s_motion.initialized || step_ms<s_motion.config.sample_period_ms || limits==NULL ||
        !finite_array(position,6) || !finite_array(velocity,6) ||
        !finite_array(limits->velocity,6)) return ESP_ERR_INVALID_ARG;
    float lower[6],upper[6];
    if(!can_console_ik_get_joint_limits(lower,upper)) return ESP_ERR_INVALID_ARG;
    /* A recovery seek can still be inside its small measured-seed offset.
     * Permit a stop there, without extending the range beyond that start. */
    for(unsigned j=0;j<6;++j) {
        if(position[j]<lower[j] && lower[j]-position[j]<=CAN_CONSOLE_IK_SEED_LIMIT_TOLERANCE_RAD)
            lower[j]=position[j];
        if(position[j]>upper[j] && position[j]-upper[j]<=CAN_CONSOLE_IK_SEED_LIMIT_TOLERANCE_RAD)
            upper[j]=position[j];
    }
    arm_stop_t stop;
    arm_joint_path_limits_t capped=*limits;
    xSemaphoreTake(s_motion.lock,portMAX_DELAY);
    for(unsigned j=0;j<6;++j)capped.velocity[j]=fminf(capped.velocity[j],s_motion.config.velocity_limits[j]);
    xSemaphoreGive(s_motion.lock);
    const esp_err_t error=arm_stop_prepare(position,velocity,&capped,lower,upper,step_ms,&stop);
    if(error!=ESP_OK)return error;
    xSemaphoreTake(s_motion.lock,portMAX_DELAY);
    bump_generation();s_motion.request_pending=false;xQueueReset(s_motion.buffer);
    s_motion.stop=stop;s_motion.stop_active=true;s_motion.plan_complete=false;
    s_motion.state=CAN_CONSOLE_ARM_MOTION_RUNNING;s_motion.kind=CAN_CONSOLE_ARM_MOTION_STOP;
    s_motion.request.path_limits=capped;s_motion.request.limited_joints=false;
    s_motion.total_points=stop.intervals;s_motion.produced_points=s_motion.consumed_points=0;
    s_motion.duration_s=(float)stop.duration_s;s_motion.last_error=ESP_OK;
    s_motion.error_source=CAN_CONSOLE_ARM_MOTION_ERROR_NONE;
    xSemaphoreGive(s_motion.lock);
    return ESP_OK;
}

void can_console_arm_motion_cancel(void)
{
    if (!s_motion.initialized) {
        return;
    }

    xSemaphoreTake(s_motion.lock, portMAX_DELAY);
    bump_generation();
    s_motion.request_pending = false;
    s_motion.stop_active = false;
    s_motion.plan_complete = false;
    s_motion.state = CAN_CONSOLE_ARM_MOTION_IDLE;
    s_motion.last_error = ESP_OK;
    s_motion.error_source = CAN_CONSOLE_ARM_MOTION_ERROR_NONE;
    s_motion.planning=(can_console_arm_plan_diagnostic_t){0};
    xQueueReset(s_motion.buffer);
    xSemaphoreGive(s_motion.lock);
}

esp_err_t can_console_arm_motion_cancel_and_wait(uint32_t timeout_ms)
{
    if (!s_motion.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    can_console_arm_motion_cancel();
    const int64_t deadline = esp_timer_get_time() + (int64_t)timeout_ms * 1000;
    for (;;) {
        xSemaphoreTake(s_motion.lock, portMAX_DELAY);
        const bool busy = s_motion.planner_busy;
        xSemaphoreGive(s_motion.lock);
        if (!busy) {
            return ESP_OK;
        }
        if (esp_timer_get_time() >= deadline) {
            return ESP_ERR_TIMEOUT;
        }
        vTaskDelay(1);
    }
}

bool can_console_arm_motion_pop_joint_target(
    float position[CAN_CONSOLE_ARM_MOTION_DOF])
{
    if (!s_motion.initialized || position == NULL) {
        return false;
    }

    motion_point_t point;
    if (!take_point(&point)) {
        return false;
    }
    memcpy(position, point.position, sizeof(point.position));
    return true;
}

bool can_console_arm_motion_tick_reference(
    can_console_arm_motion_apply_reference_fn apply_reference, void *context)
{
    if (!s_motion.initialized || apply_reference == NULL) {
        return false;
    }

    xSemaphoreTake(s_motion.lock, portMAX_DELAY);
    motion_point_t point;
    fill_stop_point();
    if (xQueuePeek(s_motion.buffer, &point, 0) != pdTRUE) {
        if (s_motion.state == CAN_CONSOLE_ARM_MOTION_RUNNING &&
            !s_motion.plan_complete && s_motion.consumed_points > 0U) {
            s_motion.underruns++;
        }
        xSemaphoreGive(s_motion.lock);
        return false;
    }

    const can_console_arm_motion_sample_result_t result = apply_reference(
        s_motion.config.motor_ids, &point,
        ((s_motion.kind == CAN_CONSOLE_ARM_MOTION_JOINT_PATH && !s_motion.request.original_timing) ||
         s_motion.kind == CAN_CONSOLE_ARM_MOTION_STOP || s_motion.request.limited_joints)
            ? s_motion.request.path_limits.velocity : s_motion.request.configured_pose
            ? s_motion.request.pose_velocity_limits : s_motion.config.velocity_limits, context);
    if (result == CAN_CONSOLE_ARM_MOTION_SAMPLE_DEFER) {
        xSemaphoreGive(s_motion.lock);
        return false;
    }
    if (result == CAN_CONSOLE_ARM_MOTION_SAMPLE_ERROR) {
        bump_generation();
        s_motion.request_pending = false;
        s_motion.stop_active = false;
        s_motion.plan_complete = true;
        xQueueReset(s_motion.buffer);
        s_motion.last_error = ESP_FAIL;
        s_motion.error_source = CAN_CONSOLE_ARM_MOTION_ERROR_EXECUTION;
        s_motion.state = CAN_CONSOLE_ARM_MOTION_ERROR;
        xSemaphoreGive(s_motion.lock);
        return false;
    }

    (void)xQueueReceive(s_motion.buffer, &point, 0);
    motion_note_consumed_locked(&point);
    xSemaphoreGive(s_motion.lock);
    return true;
}

typedef struct {
    can_console_arm_motion_apply_sample_fn apply;
    void *context;
} legacy_consumer_t;

static can_console_arm_motion_sample_result_t apply_legacy(
    const uint8_t ids[CAN_CONSOLE_ARM_MOTION_DOF], const can_console_arm_motion_sample_t *point,
    const float limits[CAN_CONSOLE_ARM_MOTION_DOF], void *context)
{
    const legacy_consumer_t *consumer = context;
    return consumer->apply(ids, point->position, limits, consumer->context);
}

bool can_console_arm_motion_tick(can_console_arm_motion_apply_sample_fn apply_sample,
    void *context)
{
    if (apply_sample == NULL) return false;
    legacy_consumer_t consumer = {.apply = apply_sample, .context = context};
    return can_console_arm_motion_tick_reference(apply_legacy, &consumer);
}

esp_err_t can_console_arm_motion_get_status(
    can_console_arm_motion_status_t *status)
{
    if (!s_motion.initialized || status == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    xSemaphoreTake(s_motion.lock, portMAX_DELAY);
    status->state = s_motion.state;
    status->total_points = s_motion.total_points;
    status->duration_s = s_motion.duration_s;
    status->produced_points = s_motion.produced_points;
    status->consumed_points = s_motion.consumed_points;
    status->buffered_points = (uint32_t)uxQueueMessagesWaiting(s_motion.buffer);
    status->buffer_low_water = s_motion.buffer_low_water;
    status->underruns = s_motion.underruns;
    status->ik_solves = s_motion.ik_solves;
    status->ik_average_us = s_motion.ik_solves > 0U
                                ? (uint32_t)(s_motion.ik_total_us /
                                             s_motion.ik_solves)
                                : 0U;
    status->ik_max_us = s_motion.ik_max_us;
    status->plan_complete = s_motion.plan_complete;
    status->planner_busy = s_motion.planner_busy || s_motion.request_pending;
    status->kind = s_motion.kind;
    status->error_source = s_motion.error_source;
    status->planning=s_motion.planning;
    status->requested_speed = s_motion.requested_speed;
    status->effective_speed = s_motion.effective_speed;
    status->consumed_reference_time_s = s_motion.consumed_reference_time_s;
    status->last_error = s_motion.last_error;
    xSemaphoreGive(s_motion.lock);
    return ESP_OK;
}
