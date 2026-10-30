#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t motor_id;
    const char *vendor;
    const char *model;
    float closed_position, open_position; /* Motor radians; order sets direction. */
    float mit_kp, mit_kd;
    float teach_kd; /* Teaching: kp=0, v_des=0, tau_ff=0. */
    float velocity_limit; /* Motor rad/s, enforced by manual reference ramps. */
    float start_tolerance; /* Motor radians. */
    uint32_t calibration_id; /* Bump whenever mapping/zero calibration changes. */
} arm_gripper_profile_t;

bool arm_gripper_profile_valid(const arm_gripper_profile_t *profile);
esp_err_t arm_gripper_position(const arm_gripper_profile_t *profile, float opening, float *position);
/* Reject travel outside the calibrated span (apart from configured endpoint tolerance).
 * Valid near-endpoint feedback is clamped only after validation. */
esp_err_t arm_gripper_opening(const arm_gripper_profile_t *profile, float position, float *opening);
/* Pure preview: caller advances its reference only after successful enqueue. */
float arm_gripper_step(float position, float target, float velocity, float dt_s);

#ifdef __cplusplus
}
#endif
