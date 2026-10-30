#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "can_console_arm_teach.h"

static const arm_teach_metadata_t metadata = {
    .model_id = ARM_TEACH_MODEL_RS, .arm_dof = 6, .channel_mask = 0x7f, .gripper_motor_id=7, .gripper_calibration_id=1,
    .motor_ids = {1,2,3,4,5,6}, .sample_period_ms = 20, .coordinate_version = 1,
};

int main(void)
{
    arm_teach_sample_t *buffer = calloc(ARM_TEACH_MAX_SAMPLES, sizeof(*buffer));
    assert(buffer != NULL && sizeof(*buffer) == 20);
    float q[6] = {-3.14f, 3.14f, -.00015f, .00015f, -1.23456f, 1.23456f};
    for (unsigned model = ARM_TEACH_MODEL_RS; model <= ARM_TEACH_MODEL_DM; ++model) {
        arm_teach_t teach = {0};
        arm_teach_metadata_t meta = metadata; meta.model_id = model;
        assert(can_console_arm_teach_begin(&teach, buffer, 30001, &meta) == ESP_OK);
        for (unsigned i = 0; i < 30001; ++i) {
            assert(can_console_arm_teach_append(&teach, q, .5f, 1234567LL + i * 20000LL, 7, 4) == ESP_OK);
            assert(buffer[i].time_ms == i * 20 && buffer[i].gripper_q == 32768);
            for (unsigned joint = 0; joint < 6; ++joint) assert(fabsf(buffer[i].position_q[joint] * .0001f - q[joint]) <= .000051f);
        }
        assert(!teach.recording && teach.trajectory.metadata.sample_count == 30001);
        assert(teach.trajectory.metadata.sample_format == ARM_TEACH_SAMPLE_FORMAT_GRIPPER);
        assert(teach.trajectory.metadata.duration_ms == 600000 && teach.trajectory.metadata.model_id == model);
        assert(teach.trajectory.metadata.end_reason == ARM_TEACH_END_DURATION);
        assert(teach.max_sample_gap_ms == 20 && teach.max_feedback_age_ms == 7 && teach.max_feedback_skew_ms == 4);
        can_console_arm_teach_stop(&teach, ARM_TEACH_END_USER);
        assert(teach.trajectory.metadata.end_reason == ARM_TEACH_END_DURATION);
        assert(can_console_arm_teach_begin(&teach, buffer, 30001, &meta) == ESP_ERR_INVALID_STATE);
    }
    arm_teach_t teach = {0};
    arm_teach_metadata_t invalid_metadata=metadata;
    invalid_metadata.channel_mask=0x3f;
    assert(can_console_arm_teach_begin(&teach,buffer,20,&invalid_metadata)==ESP_ERR_INVALID_ARG);
    invalid_metadata=metadata;invalid_metadata.gripper_motor_id=6;
    assert(can_console_arm_teach_begin(&teach,buffer,20,&invalid_metadata)==ESP_ERR_INVALID_ARG);
    invalid_metadata=metadata;invalid_metadata.gripper_calibration_id=0;
    assert(can_console_arm_teach_begin(&teach,buffer,20,&invalid_metadata)==ESP_ERR_INVALID_ARG);
    memset(buffer, 0x5a, 3 * sizeof(*buffer));
    arm_teach_sample_t guard = buffer[2];
    assert(can_console_arm_teach_begin(&teach, buffer, 2, &metadata) == ESP_OK);
    assert(can_console_arm_teach_append(&teach, q, .5f, 1000, 0, 0) == ESP_OK);
    assert(can_console_arm_teach_append(&teach, q, .5f, 1500, 0, 0) == ESP_OK);
    assert(teach.trajectory.metadata.sample_count == 1);
    assert(can_console_arm_teach_append(&teach, q, .5f, 23345, 3, 2) == ESP_OK);
    assert(buffer[1].time_ms == 22 && teach.trajectory.metadata.end_reason == ARM_TEACH_END_CAPACITY);
    assert(can_console_arm_teach_append(&teach, q, .5f, 40000, 0, 0) == ESP_ERR_INVALID_STATE);
    assert(memcmp(&guard, &buffer[2], sizeof(guard)) == 0);
    const float invalid[] = {NAN, INFINITY, -INFINITY, 3.2768f, -3.2769f, 1e30f};
    for (unsigned i = 0; i < sizeof(invalid) / sizeof(invalid[0]); ++i) {
        teach = (arm_teach_t){0};
        assert(can_console_arm_teach_begin(&teach, buffer, 20, &metadata) == ESP_OK);
        q[2] = 0; assert(can_console_arm_teach_append(&teach, q, .5f, 20000, 0, 0) == ESP_OK);
        q[2] = invalid[i];
        assert(can_console_arm_teach_append(&teach, q, .5f, 40000, 0, 0) == ESP_ERR_INVALID_ARG);
        assert(!teach.recording && teach.trajectory.metadata.sample_count == 1 && teach.bad_joint == 3);
        assert(teach.trajectory.metadata.end_reason == ARM_TEACH_END_ENCODING);
    }
    q[2] = 0;
    teach = (arm_teach_t){0};
    assert(can_console_arm_teach_begin(&teach, buffer, 20, &metadata) == ESP_OK);
    assert(can_console_arm_teach_append(&teach, q, .5f, 20000, 0, 0) == ESP_OK);
    assert(can_console_arm_teach_append(&teach, q, .5f, 19999, 0, 0) == ESP_ERR_INVALID_ARG);
    assert(teach.trajectory.metadata.end_reason == ARM_TEACH_END_TIME);
    teach = (arm_teach_t){0};
    assert(can_console_arm_teach_begin(&teach, buffer, 20, &metadata) == ESP_OK);
    assert(can_console_arm_teach_append(&teach, q, .5f, 20000, 0, 0) == ESP_OK);
    assert(can_console_arm_teach_append(&teach, q, .5f, 120001, 0, 0) == ESP_ERR_INVALID_ARG);
    assert(teach.trajectory.metadata.end_reason == ARM_TEACH_END_SAMPLE_GAP && teach.trajectory.metadata.sample_count == 1);
    const float invalid_openings[]={NAN,INFINITY,-INFINITY,-.01f,1.01f};
    for(unsigned i=0;i<sizeof(invalid_openings)/sizeof(invalid_openings[0]);++i) {
        teach=(arm_teach_t){0};
        assert(can_console_arm_teach_begin(&teach,buffer,20,&metadata)==ESP_OK);
        assert(can_console_arm_teach_append(&teach,q,.5f,20000,0,0)==ESP_OK);
        assert(can_console_arm_teach_append(&teach,q,invalid_openings[i],40000,0,0)==ESP_ERR_INVALID_ARG);
        assert(!teach.recording && teach.bad_joint==7 && teach.trajectory.metadata.sample_count==1);
        assert(buffer[0].gripper_q==32768 && teach.trajectory.metadata.end_reason==ARM_TEACH_END_ENCODING);
    }
    free(buffer);
    puts("teach passed: RS/DM ten-minute capacity, quantization, timestamps, boundaries and retained error prefixes");
}
