/*
 * SPDX-FileCopyrightText: 2026 XIAO-C5-CAN
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#define CAN_CONSOLE_POSE_ROTATION_ELEMENTS 9U
#define CAN_CONSOLE_POSE_QUATERNION_ELEMENTS 4U

/**
 * Build a row-major rotation matrix using
 * R = Rz(yaw) * Ry(pitch) * Rx(roll).
 */
void can_console_pose_rpy_to_rotation(
    float roll, float pitch, float yaw,
    float rotation[CAN_CONSOLE_POSE_ROTATION_ELEMENTS]);

/** Convert rotation to roll/pitch/yaw radians; yaw is zero at gimbal lock. */
void can_console_pose_rotation_to_rpy(const float rotation[9], float rpy[3]);

/** Convert a row-major 3x3 rotation matrix to a [w, x, y, z] quaternion. */
void can_console_pose_rotation_to_quaternion(
    const float rotation[CAN_CONSOLE_POSE_ROTATION_ELEMENTS],
    float quaternion[CAN_CONSOLE_POSE_QUATERNION_ELEMENTS]);

/** Convert a [w, x, y, z] quaternion to a row-major 3x3 rotation matrix. */
void can_console_pose_quaternion_to_rotation(
    const float quaternion[CAN_CONSOLE_POSE_QUATERNION_ELEMENTS],
    float rotation[CAN_CONSOLE_POSE_ROTATION_ELEMENTS]);

/** Interpolate two normalized [w, x, y, z] quaternions on the shortest arc. */
void can_console_pose_quaternion_slerp(
    const float from[CAN_CONSOLE_POSE_QUATERNION_ELEMENTS],
    const float to[CAN_CONSOLE_POSE_QUATERNION_ELEMENTS], float amount,
    float result[CAN_CONSOLE_POSE_QUATERNION_ELEMENTS]);

#ifdef __cplusplus
}
#endif
