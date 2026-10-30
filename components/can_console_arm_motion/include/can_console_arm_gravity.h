#pragma once

#include "can_console_ik.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** q_model[i] = direction[i] * (q_motor[i] - zero_offset[i]). */
typedef struct {
    can_console_ik_model_t model;
    float gravity_base[3];
    float direction[CAN_CONSOLE_IK_DOF];
    float zero_offset[CAN_CONSOLE_IK_DOF];
    float torque_scale[CAN_CONSOLE_IK_DOF];
    /** Extra payload only; the built-in URDF gripper is already included. */
    float payload_mass;
    /** Payload COM in the IK end-effector frame, in metres. */
    float payload_com[3];
} can_console_arm_gravity_config_t;

/** Caller-owned context. No tasks, locks, queues or heap allocations. */
typedef struct {
    can_console_arm_gravity_config_t config;
    bool initialized;
} can_console_arm_gravity_t;

/** Defaults: selected IK model, vertical gravity, identity mapping, no payload. */
void can_console_arm_gravity_config_set_defaults(can_console_arm_gravity_config_t *config);

/** Initialize a fixed-model context; independent of subsequent default model selection. */
esp_err_t can_console_arm_gravity_init(can_console_arm_gravity_t *context,
                                      const can_console_arm_gravity_config_t *config);

/**
 * Compute gravity-balancing OUTPUT-SHAFT torques in N*m from actual motor angles.
 * No clipping, gain changes, mode changes or motor writes. Output is only
 * updated on success. Caller must enforce feedback freshness and torque limits.
 * Read-only after init; separate output arrays may be used by concurrent callers.
 */
esp_err_t can_console_arm_gravity_compute(const can_console_arm_gravity_t *context,
                                         const float q_motor[CAN_CONSOLE_IK_DOF],
                                         float torque[CAN_CONSOLE_IK_DOF]);

#ifdef __cplusplus
}
#endif
