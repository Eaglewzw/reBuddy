#include "arm_joint_path.h"

#include <math.h>
#include <stddef.h>

static double position(const arm_joint_path_view_t *path, uint32_t i, size_t joint)
{
    /* Match the public float joint coordinate representation at hard bounds. */
    return path->samples[i].position_q[joint] / ARM_TEACH_POSITION_SCALE;
}

static double slope(const arm_joint_path_view_t *path, uint32_t i, size_t joint)
{
    if (i == 0U || i + 1U == path->count) return 0.0;
    const double before = (path->samples[i].time_ms - path->samples[i - 1U].time_ms) * .001;
    const double after = (path->samples[i + 1U].time_ms - path->samples[i].time_ms) * .001;
    const double previous = (position(path, i, joint) - position(path, i - 1U, joint)) / before;
    const double next = (position(path, i + 1U, joint) - position(path, i, joint)) / after;
    /* Flat segments and reversals have zero tangent. The weighted harmonic
     * mean limits interior tangents without erasing recorded pauses. */
    if (previous * next <= 0.0) return 0.0;
    const double w1 = 2.0 * after + before, w2 = after + 2.0 * before;
    return (w1 + w2) / (w1 / previous + w2 / next);
}

void arm_joint_path_make_segment(const arm_joint_path_view_t *path, uint32_t i,
    arm_joint_path_segment_t *segment)
{
    segment->duration_s = (path->samples[i + 1U].time_ms - path->samples[i].time_ms) * .001;
    for (size_t joint = 0U; joint < CAN_CONSOLE_ARM_MOTION_DOF; ++joint) {
        const double q0 = position(path, i, joint), q1 = position(path, i + 1U, joint);
        const double c = slope(path, i, joint) * segment->duration_s;
        const double end = slope(path, i + 1U, joint) * segment->duration_s;
        double *k = segment->coefficient[joint];
        k[0] = 2.0 * (q0 - q1) + c + end;
        k[1] = 3.0 * (q1 - q0) - 2.0 * c - end;
        k[2] = c;
        k[3] = q0;
    }
}

static double value(const double k[4], double u)
{
    return ((k[0] * u + k[1]) * u + k[2]) * u + k[3];
}

static double derivative(const double k[4], double u)
{
    return (3.0 * k[0] * u + 2.0 * k[1]) * u + k[2];
}

void arm_joint_path_evaluate(const arm_joint_path_segment_t *segment, double u,
    double speed, can_console_arm_motion_sample_t *sample)
{
    for (size_t joint = 0U; joint < CAN_CONSOLE_ARM_MOTION_DOF; ++joint) {
        const double *k = segment->coefficient[joint];
        sample->position[joint] = (float)value(k, u);
        sample->reference_velocity[joint] = (float)(derivative(k, u) / segment->duration_s * speed);
    }
}

static bool inside(double q, float lower, float upper)
{
    /* Compare in the same precision as the model and outgoing positions. */
    return isfinite(q) && (float)q >= lower && (float)q <= upper;
}

static bool segment_bounds(const double k[4], float lower, float upper)
{
    /* Position extrema are roots of the quadratic derivative, not samples on
     * a coarse grid. Endpoints were already checked in the raw-sample pass. */
    const double a = 3.0 * k[0], b = 2.0 * k[1], c = k[2];
    if (a == 0.0) {
        const double u = b != 0.0 ? -c / b : -1.0;
        return !(u > 0.0 && u < 1.0) || inside(value(k, u), lower, upper);
    }
    const double discriminant = b * b - 4.0 * a * c;
    if (discriminant < 0.0) return true;
    /* Stable quadratic roots, including the zero root at a stopped endpoint. */
    const double q = -.5 * (b + copysign(sqrt(discriminant), b));
    const double u1 = q / a, u2 = q != 0.0 ? c / q : 0.0;
    return (!(u1 > 0.0 && u1 < 1.0) || inside(value(k, u1), lower, upper)) &&
           (!(u2 > 0.0 && u2 < 1.0) || inside(value(k, u2), lower, upper));
}

esp_err_t arm_joint_path_prepare(const arm_joint_path_view_t *path,
    const arm_joint_path_limits_t *limits, const float lower[6], const float upper[6],
    uint32_t period_ms, float speed, bool (*keep_running)(void *), void *context,
    arm_joint_path_plan_t *plan)
{
    if (path == NULL || path->samples == NULL || path->count < 2U ||
        path->count > ARM_TEACH_MAX_SAMPLES || lower == NULL ||
        upper == NULL || plan == NULL || period_ms == 0U || !isfinite(speed) ||
        speed < .1f || speed > 1.0f) return ESP_ERR_INVALID_ARG;
    for (size_t j = 0U; j < CAN_CONSOLE_ARM_MOTION_DOF; ++j) {
        if (!isfinite(lower[j]) || !isfinite(upper[j]) || lower[j] > upper[j] ||
            (limits != NULL && (!isfinite(limits->velocity[j]) || limits->velocity[j] <= 0.0f ||
            !isfinite(limits->acceleration[j]) || limits->acceleration[j] <= 0.0f)))
            return ESP_ERR_INVALID_ARG;
    }
    /* Validate each stored sample once before any neighbour-based arithmetic. */
    for (uint32_t i = 0U; i < path->count; ++i) {
        if (keep_running != NULL && !keep_running(context)) return ESP_ERR_INVALID_STATE;
        const uint32_t time = path->samples[i].time_ms;
        if (time > ARM_TEACH_MAX_DURATION_MS || (i == 0U && time != 0U) ||
            (i != 0U && (time <= path->samples[i - 1U].time_ms ||
                        time - path->samples[i - 1U].time_ms > ARM_TEACH_MAX_GAP_MS)))
            return ESP_ERR_INVALID_ARG;
        for (size_t j = 0U; j < CAN_CONSOLE_ARM_MOTION_DOF; ++j) {
            if (!inside(position(path, i, j), lower[j], upper[j])) return ESP_ERR_INVALID_ARG;
        }
    }
    double effective = speed;
    for (uint32_t i = 0U; i + 1U < path->count; ++i) {
        if (keep_running != NULL && !keep_running(context)) return ESP_ERR_INVALID_STATE;
        arm_joint_path_segment_t segment;
        arm_joint_path_make_segment(path, i, &segment);
        for (size_t j = 0U; j < CAN_CONSOLE_ARM_MOTION_DOF; ++j) {
            const double *k = segment.coefficient[j], h = segment.duration_s;
            if (!segment_bounds(k, lower[j], upper[j])) return ESP_ERR_INVALID_ARG;
            /* Original-time playback keeps the position preflight, without
             * stretching the whole recording to satisfy probe v/a budgets. */
            if (limits == NULL) continue;
            double velocity = fmax(fabs(derivative(k, 0.0)), fabs(derivative(k, 1.0))) / h;
            const double u = k[0] != 0.0 ? -k[1] / (3.0 * k[0]) : -1.0;
            if (u > 0.0 && u < 1.0) velocity = fmax(velocity, fabs(derivative(k, u)) / h);
            const double acceleration = fmax(fabs(2.0 * k[1]), fabs(6.0 * k[0] + 2.0 * k[1])) / (h * h);
            if (velocity > 0.0) effective = fmin(effective, limits->velocity[j] / velocity);
            if (acceleration > 0.0) effective = fmin(effective, sqrt(limits->acceleration[j] / acceleration));
        }
    }
    const double original_ms = path->samples[path->count - 1U].time_ms;
    const double intervals = ceil(original_ms / (period_ms * effective));
    if (!isfinite(intervals) || intervals < 1.0 || intervals > UINT32_MAX - 1.0)
        return ESP_ERR_INVALID_ARG;
    *plan = (arm_joint_path_plan_t) {
        .path = *path, .effective_speed = original_ms / (intervals * period_ms),
        .duration_s = intervals * period_ms * .001,
        .total_points = (uint32_t)intervals + 1U, .sample_period_ms = period_ms,
    };
    return ESP_OK;
}
