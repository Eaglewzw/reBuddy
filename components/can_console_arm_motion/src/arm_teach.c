#include "can_console_arm_teach.h"

#include <math.h>
#include <stddef.h>

_Static_assert(sizeof(arm_teach_sample_t) == 20, "update the RAM sample budget");

esp_err_t can_console_arm_teach_begin(arm_teach_t *teach,
    arm_teach_sample_t *buffer, uint32_t capacity,
    const arm_teach_metadata_t *metadata)
{
    if (teach == NULL || buffer == NULL || metadata == NULL || capacity == 0U ||
        capacity > ARM_TEACH_MAX_SAMPLES || metadata->arm_dof != ARM_TEACH_DOF ||
        metadata->channel_mask != ARM_TEACH_CHANNELS_GRIPPER ||
        (metadata->model_id != ARM_TEACH_MODEL_RS && metadata->model_id != ARM_TEACH_MODEL_DM)) {
        return ESP_ERR_INVALID_ARG;
    }
    if(!metadata->gripper_motor_id || !metadata->gripper_calibration_id)return ESP_ERR_INVALID_ARG;
    for(unsigned j=0;j<ARM_TEACH_DOF;++j)
        if(metadata->motor_ids[j]==metadata->gripper_motor_id)return ESP_ERR_INVALID_ARG;
    if (teach->recording || teach->trajectory.metadata.sample_count != 0U) {
        return ESP_ERR_INVALID_STATE;
    }
    *teach = (arm_teach_t) {
        .trajectory = {.samples = buffer, .capacity = capacity, .metadata = *metadata},
        .recording = true,
    };
    teach->trajectory.metadata.sample_count = 0U;
    teach->trajectory.metadata.sample_format = ARM_TEACH_SAMPLE_FORMAT_GRIPPER;
    teach->trajectory.metadata.duration_ms = 0U;
    teach->trajectory.metadata.end_reason = ARM_TEACH_END_NONE;
    return ESP_OK;
}

void can_console_arm_teach_stop(arm_teach_t *teach, arm_teach_end_reason_t reason)
{
    if (teach->recording) {
        teach->recording = false;
        teach->trajectory.metadata.end_reason = reason;
    }
}

static esp_err_t abort_sampling(arm_teach_t *teach, arm_teach_end_reason_t reason)
{
    can_console_arm_teach_stop(teach, reason);
    teach->last_error = ESP_ERR_INVALID_ARG;
    return teach->last_error;
}

esp_err_t can_console_arm_teach_append(arm_teach_t *teach,
    const float position[ARM_TEACH_DOF], float gripper_opening, int64_t now_us,
    uint32_t feedback_age_ms, uint32_t feedback_skew_ms)
{
    if (teach == NULL || position == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!teach->recording) {
        return ESP_ERR_INVALID_STATE;
    }
    arm_teach_metadata_t *meta = &teach->trajectory.metadata;
    if (now_us < 0 || (meta->sample_count != 0U && now_us < teach->last_sample_us)) {
        return abort_sampling(teach, ARM_TEACH_END_TIME);
    }
    if (feedback_age_ms > teach->max_feedback_age_ms) teach->max_feedback_age_ms = feedback_age_ms;
    if (feedback_skew_ms > teach->max_feedback_skew_ms) teach->max_feedback_skew_ms = feedback_skew_ms;

    arm_teach_sample_t sample = {0};
    if (meta->sample_count != 0U) {
        const int64_t gap_us = now_us - teach->last_sample_us;
        const int64_t gap_ms = gap_us / 1000;
        if (gap_ms > teach->max_sample_gap_ms) {
            teach->max_sample_gap_ms = gap_ms > UINT32_MAX ? UINT32_MAX : (uint32_t)gap_ms;
        }
        if (gap_us > (int64_t)ARM_TEACH_MAX_GAP_MS * 1000) {
            return abort_sampling(teach, ARM_TEACH_END_SAMPLE_GAP);
        }
        const int64_t elapsed_us = now_us - teach->start_us;
        if (elapsed_us > (int64_t)ARM_TEACH_MAX_DURATION_MS * 1000) {
            can_console_arm_teach_stop(teach, ARM_TEACH_END_DURATION);
            return ESP_OK;
        }
        sample.time_ms = (uint32_t)(elapsed_us / 1000);
        if (sample.time_ms == meta->duration_ms) return ESP_OK;
    }
    for (size_t i = 0U; i < ARM_TEACH_DOF; ++i) {
        const float q = roundf(position[i] * ARM_TEACH_POSITION_SCALE);
        if (!isfinite(q) || q < INT16_MIN || q > INT16_MAX) {
            teach->bad_joint = (uint8_t)(i + 1U);
            return abort_sampling(teach, ARM_TEACH_END_ENCODING);
        }
        sample.position_q[i] = (int16_t)q;
    }
    if(!isfinite(gripper_opening) || gripper_opening<0 || gripper_opening>1) {
        teach->bad_joint=7;
        return abort_sampling(teach,ARM_TEACH_END_ENCODING);
    }
    sample.gripper_q=(uint16_t)lroundf(gripper_opening*65535.0f);
    teach->trajectory.samples[meta->sample_count++] = sample;
    if (meta->sample_count == 1U) teach->start_us = now_us;
    teach->last_sample_us = now_us;
    meta->duration_ms = sample.time_ms;
    if (sample.time_ms == ARM_TEACH_MAX_DURATION_MS) {
        can_console_arm_teach_stop(teach, ARM_TEACH_END_DURATION);
    } else if (meta->sample_count == teach->trajectory.capacity) {
        can_console_arm_teach_stop(teach, ARM_TEACH_END_CAPACITY);
    }
    return ESP_OK;
}

const char *can_console_arm_teach_end_reason_name(arm_teach_end_reason_t reason)
{
    static const char *const names[] = {
        "none", "user", "duration", "capacity", "gravity_off", "disable", "estop",
        "control_fault", "sample_gap", "encoding", "time",
    };
    return (unsigned)reason < sizeof(names) / sizeof(names[0]) ? names[reason] : "unknown";
}
