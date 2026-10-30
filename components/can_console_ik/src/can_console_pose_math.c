/*
 * SPDX-FileCopyrightText: 2026 XIAO-C5-CAN
 * SPDX-License-Identifier: Apache-2.0
 */

#include "can_console_pose_math.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define CAN_CONSOLE_POSE_EPSILON 1.0e-6f

static float clampf_local(float value, float low, float high)
{
    if (value < low) {
        return low;
    }
    if (value > high) {
        return high;
    }
    return value;
}

void can_console_pose_rpy_to_rotation(
    float roll, float pitch, float yaw,
    float rotation[CAN_CONSOLE_POSE_ROTATION_ELEMENTS])
{
    const float cr = cosf(roll);
    const float sr = sinf(roll);
    const float cp = cosf(pitch);
    const float sp = sinf(pitch);
    const float cy = cosf(yaw);
    const float sy = sinf(yaw);

    rotation[0] = cy * cp;
    rotation[1] = cy * sp * sr - sy * cr;
    rotation[2] = cy * sp * cr + sy * sr;
    rotation[3] = sy * cp;
    rotation[4] = sy * sp * sr + cy * cr;
    rotation[5] = sy * sp * cr - cy * sr;
    rotation[6] = -sp;
    rotation[7] = cp * sr;
    rotation[8] = cp * cr;
}

void can_console_pose_rotation_to_quaternion(
    const float rotation[CAN_CONSOLE_POSE_ROTATION_ELEMENTS],
    float quaternion[CAN_CONSOLE_POSE_QUATERNION_ELEMENTS])
{
    const float trace = rotation[0] + rotation[4] + rotation[8];
    if (trace > 0.0f) {
        const float s = sqrtf(trace + 1.0f) * 2.0f;
        quaternion[0] = 0.25f * s;
        quaternion[1] = (rotation[7] - rotation[5]) / s;
        quaternion[2] = (rotation[2] - rotation[6]) / s;
        quaternion[3] = (rotation[3] - rotation[1]) / s;
    } else if (rotation[0] > rotation[4] && rotation[0] > rotation[8]) {
        const float s = sqrtf(fmaxf(1.0f + rotation[0] - rotation[4] -
                                        rotation[8],
                                    0.0f)) *
                        2.0f;
        quaternion[0] = (rotation[7] - rotation[5]) / s;
        quaternion[1] = 0.25f * s;
        quaternion[2] = (rotation[1] + rotation[3]) / s;
        quaternion[3] = (rotation[2] + rotation[6]) / s;
    } else if (rotation[4] > rotation[8]) {
        const float s = sqrtf(fmaxf(1.0f + rotation[4] - rotation[0] -
                                        rotation[8],
                                    0.0f)) *
                        2.0f;
        quaternion[0] = (rotation[2] - rotation[6]) / s;
        quaternion[1] = (rotation[1] + rotation[3]) / s;
        quaternion[2] = 0.25f * s;
        quaternion[3] = (rotation[5] + rotation[7]) / s;
    } else {
        const float s = sqrtf(fmaxf(1.0f + rotation[8] - rotation[0] -
                                        rotation[4],
                                    0.0f)) *
                        2.0f;
        quaternion[0] = (rotation[3] - rotation[1]) / s;
        quaternion[1] = (rotation[2] + rotation[6]) / s;
        quaternion[2] = (rotation[5] + rotation[7]) / s;
        quaternion[3] = 0.25f * s;
    }

    const float norm =
        sqrtf(quaternion[0] * quaternion[0] +
              quaternion[1] * quaternion[1] +
              quaternion[2] * quaternion[2] +
              quaternion[3] * quaternion[3]);
    if (norm > CAN_CONSOLE_POSE_EPSILON && isfinite(norm)) {
        for (size_t i = 0U; i < CAN_CONSOLE_POSE_QUATERNION_ELEMENTS; ++i) {
            quaternion[i] /= norm;
        }
    } else {
        quaternion[0] = 1.0f;
        quaternion[1] = 0.0f;
        quaternion[2] = 0.0f;
        quaternion[3] = 0.0f;
    }
}

void can_console_pose_quaternion_to_rotation(
    const float quaternion[CAN_CONSOLE_POSE_QUATERNION_ELEMENTS],
    float rotation[CAN_CONSOLE_POSE_ROTATION_ELEMENTS])
{
    const float w = quaternion[0];
    const float x = quaternion[1];
    const float y = quaternion[2];
    const float z = quaternion[3];
    const float xx = x * x;
    const float yy = y * y;
    const float zz = z * z;
    const float xy = x * y;
    const float xz = x * z;
    const float yz = y * z;
    const float wx = w * x;
    const float wy = w * y;
    const float wz = w * z;

    rotation[0] = 1.0f - 2.0f * (yy + zz);
    rotation[1] = 2.0f * (xy - wz);
    rotation[2] = 2.0f * (xz + wy);
    rotation[3] = 2.0f * (xy + wz);
    rotation[4] = 1.0f - 2.0f * (xx + zz);
    rotation[5] = 2.0f * (yz - wx);
    rotation[6] = 2.0f * (xz - wy);
    rotation[7] = 2.0f * (yz + wx);
    rotation[8] = 1.0f - 2.0f * (xx + yy);
}

void can_console_pose_quaternion_slerp(
    const float from[CAN_CONSOLE_POSE_QUATERNION_ELEMENTS],
    const float to[CAN_CONSOLE_POSE_QUATERNION_ELEMENTS], float amount,
    float result[CAN_CONSOLE_POSE_QUATERNION_ELEMENTS])
{
    float target[CAN_CONSOLE_POSE_QUATERNION_ELEMENTS];
    memcpy(target, to, sizeof(target));
    float dot = from[0] * target[0] + from[1] * target[1] +
                from[2] * target[2] + from[3] * target[3];
    if (dot < 0.0f) {
        dot = -dot;
        for (size_t i = 0U; i < CAN_CONSOLE_POSE_QUATERNION_ELEMENTS; ++i) {
            target[i] = -target[i];
        }
    }

    if (dot > 0.9995f) {
        for (size_t i = 0U; i < CAN_CONSOLE_POSE_QUATERNION_ELEMENTS; ++i) {
            result[i] = from[i] + amount * (target[i] - from[i]);
        }
    } else {
        const float angle = acosf(clampf_local(dot, -1.0f, 1.0f));
        const float sin_angle = sinf(angle);
        const float weight_from = sinf((1.0f - amount) * angle) / sin_angle;
        const float weight_to = sinf(amount * angle) / sin_angle;
        for (size_t i = 0U; i < CAN_CONSOLE_POSE_QUATERNION_ELEMENTS; ++i) {
            result[i] = weight_from * from[i] + weight_to * target[i];
        }
    }

    const float norm = sqrtf(result[0] * result[0] + result[1] * result[1] +
                             result[2] * result[2] + result[3] * result[3]);
    if (norm > CAN_CONSOLE_POSE_EPSILON && isfinite(norm)) {
        for (size_t i = 0U; i < CAN_CONSOLE_POSE_QUATERNION_ELEMENTS; ++i) {
            result[i] /= norm;
        }
    }
}

void can_console_pose_rotation_to_rpy(const float r[9], float rpy[3])
{
    float cp = hypotf(r[0], r[3]);
    rpy[1] = atan2f(-r[6], cp);
    if (cp > 1e-6f) {
        rpy[0] = atan2f(r[7], r[8]);
        rpy[2] = atan2f(r[3], r[0]);
    } else {
        rpy[0] = atan2f(-r[5], r[4]);
        rpy[2] = 0;
    }
}
