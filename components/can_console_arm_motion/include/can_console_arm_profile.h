/*
 * SPDX-FileCopyrightText: 2026 XIAO-C5-CAN
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stdint.h>
#include <stddef.h>

#include "can_console_ik.h"
#include "can_console_arm_motion.h"
#include "can_console_arm_gripper.h"

#define ARM_HARDWARE_PROFILE_DOF 6U

typedef struct {
    uint8_t motor_id;
    const char *model;
    float mit_kp;
    float mit_kd;
    float gravity_torque_scale; /**< Per-joint model torque calibration. */
} arm_joint_profile_t;

/* Shared configuration schema; concrete hardware instances belong to the app.
 * Identity (model/motor IDs below, coordinate/calibration here) matches data.
 * position_mode selects the wire protocol, recorded_timing selects scheduling.
 * Probe limits and start tolerances apply to motion; physical test evidence
 * belongs in the validation documentation. */
typedef struct {
    arm_joint_path_limits_t limits;
    float start_tolerance[6];
    uint32_t calibration_id;
    uint32_t coordinate_version;
    /** Playback reference groups use vendor POS_VEL (vlim-bounded) instead
     *  of MIT; MIT does not encode vlim or guarantee total output torque.
     * Recording stays MIT+gravity. */
    bool position_mode;
    /** Replay by recorded timestamps, without commissioning v/a caps.
     * Ordinary seek/home use motion vlim; commissioning probes use limits above. */
    bool recorded_timing;
} arm_playback_calibration_t;

typedef struct {
    const char *name;
    const char *vendor;
    can_console_ik_model_t ik_model;
    arm_joint_profile_t joints[ARM_HARDWARE_PROFILE_DOF];
    arm_gripper_profile_t gripper; /* Separate from the six kinematic joints. */
    /** Basic compliant gravity-control gains from the matching Python profile. */
    float gravity_kp;
    float gravity_kd;
    /** NULL when no playback configuration is available. */
    const arm_playback_calibration_t *playback;
} arm_hardware_profile_t;


