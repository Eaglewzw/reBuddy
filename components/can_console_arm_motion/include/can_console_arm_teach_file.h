#pragma once
#include <stddef.h>
#include "can_console_arm_teach.h"
#define ARM_TEACH_FILE_HEADER_SIZE 64U
#define ARM_TEACH_FILE_SAMPLE_SIZE 18U
#define ARM_TEACH_FILE_MAX_SIZE (ARM_TEACH_FILE_HEADER_SIZE + ARM_TEACH_MAX_SAMPLES * ARM_TEACH_FILE_SAMPLE_SIZE)
/* TCH1 v2 only: six axes plus gripper. CRC bytes 60..63 are zeroed. */
uint32_t arm_teach_crc32(uint32_t state, const void *data, size_t size);
esp_err_t arm_teach_file_header(const arm_teach_metadata_t *metadata, uint8_t out[64]);
esp_err_t arm_teach_file_parse_header(const uint8_t in[64], arm_teach_metadata_t *metadata, uint32_t *crc);
/* Fixed 18-byte samples; callers own buffers of at least SAMPLE_SIZE bytes. */
void arm_teach_file_encode_sample(const arm_teach_sample_t *sample, uint8_t out[ARM_TEACH_FILE_SAMPLE_SIZE]);
void arm_teach_file_decode_sample(const uint8_t in[ARM_TEACH_FILE_SAMPLE_SIZE], arm_teach_sample_t *sample);
esp_err_t arm_teach_file_check_sample(const arm_teach_metadata_t *metadata, uint32_t index,
    const arm_teach_sample_t *sample, uint32_t previous_ms);
