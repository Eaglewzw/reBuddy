#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ARM_TEACH_DOF 6U
#define ARM_TEACH_MAX_SAMPLES 30001U
#define ARM_TEACH_MAX_DURATION_MS 600000U
#define ARM_TEACH_SAMPLE_PERIOD_MS 20U
#define ARM_TEACH_MAX_GAP_MS 100U
#define ARM_TEACH_POSITION_SCALE 10000.0f
#define ARM_TEACH_SAMPLE_FORMAT_GRIPPER 2U
#define ARM_TEACH_CHANNELS_GRIPPER 0x007FU

/* Stable trajectory IDs; deliberately independent of the IK model enum. */
typedef enum {
    ARM_TEACH_MODEL_UNKNOWN = 0,
    ARM_TEACH_MODEL_RS = 1,
    ARM_TEACH_MODEL_DM = 2,
} arm_teach_model_t;

typedef enum {
    ARM_TEACH_END_NONE = 0,
    ARM_TEACH_END_USER,
    ARM_TEACH_END_DURATION,
    ARM_TEACH_END_CAPACITY,
    ARM_TEACH_END_GRAVITY_OFF,
    ARM_TEACH_END_DISABLE,
    ARM_TEACH_END_ESTOP,
    ARM_TEACH_END_CONTROL_FAULT,
    ARM_TEACH_END_SAMPLE_GAP,
    ARM_TEACH_END_ENCODING,
    ARM_TEACH_END_TIME,
} arm_teach_end_reason_t;

typedef struct {
    uint32_t time_ms;
    int16_t position_q[ARM_TEACH_DOF]; /* 0.0001 rad per unit. */
    uint16_t gripper_q; /* Normalized opening, 0..65535; required in every recording. */
} arm_teach_sample_t;

typedef struct {
    uint8_t model_id;
    uint8_t sample_format; /* 2: six axes plus uint16 opening. */
    uint8_t arm_dof;
    uint16_t channel_mask;
    uint8_t motor_ids[ARM_TEACH_DOF];
    uint8_t gripper_motor_id;
    uint32_t gripper_calibration_id;
    uint16_t sample_period_ms;
    uint32_t coordinate_version;
    /* 0 means no persistent calibration identity yet. */
    uint32_t calibration_id;
    uint32_t sample_count;
    uint32_t duration_ms;
    arm_teach_end_reason_t end_reason;
} arm_teach_metadata_t;

typedef struct {
    arm_teach_metadata_t metadata;
    arm_teach_sample_t *samples;
    uint32_t capacity;
} arm_teach_trajectory_t;

/* Caller serializes access and owns the buffer. No allocation or hardware I/O. */
typedef struct {
    arm_teach_trajectory_t trajectory;
    bool recording;
    int64_t start_us;
    int64_t last_sample_us;
    uint32_t max_sample_gap_ms;
    uint32_t max_feedback_age_ms;
    uint32_t max_feedback_skew_ms;
    esp_err_t last_error;
    uint8_t bad_joint; /* 1..6 for joints, 7 for gripper; 0 when unspecified. */
} arm_teach_t;

esp_err_t can_console_arm_teach_begin(arm_teach_t *teach,
    arm_teach_sample_t *buffer, uint32_t capacity,
    const arm_teach_metadata_t *metadata);
/* now_us is the monotonic snapshot time, not a sample index. Encoding/time
 * errors end sampling and retain the prefix. Sub-millisecond duplicates skip.
 * Every snapshot includes six joint angles and measured gripper opening (0..1). */
esp_err_t can_console_arm_teach_append(arm_teach_t *teach,
    const float position[ARM_TEACH_DOF], float gripper_opening, int64_t now_us,
    uint32_t feedback_age_ms, uint32_t feedback_skew_ms);
void can_console_arm_teach_stop(arm_teach_t *teach, arm_teach_end_reason_t reason);
const char *can_console_arm_teach_end_reason_name(arm_teach_end_reason_t reason);

#ifdef __cplusplus
}
#endif
