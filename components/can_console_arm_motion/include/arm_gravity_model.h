#pragma once

#include "can_console_ik.h"

typedef struct {
    float mass;
    float com[3];
} arm_gravity_body_t;

typedef struct {
    arm_gravity_body_t links[CAN_CONSOLE_IK_DOF];
    /** Entire tool tree merged into link6 at zero finger opening. */
    arm_gravity_body_t tool;
} arm_gravity_model_t;

const arm_gravity_model_t *arm_gravity_model_get(can_console_ik_model_t model);
