/*
 * SPDX-FileCopyrightText: 2026 XIAO-C5-CAN
 * SPDX-License-Identifier: Apache-2.0
 */

#include "arm_trajectory.h"

#include <math.h>

static float clamp_unit(float value)
{
    if (value < 0.0f) {
        return 0.0f;
    }
    if (value > 1.0f) {
        return 1.0f;
    }
    return value;
}

float arm_trajectory_profile(float u)
{
    const float value = clamp_unit(u);
    return value * value * value *
           (10.0f - 15.0f * value + 6.0f * value * value);
}

float arm_trajectory_align_duration(float duration_s, float sample_period_s)
{
    if (!(sample_period_s > 0.0f)) {
        return duration_s;
    }
    if (!(duration_s > 0.0f)) {
        return sample_period_s;
    }
    /* Multiplying an integer period count and dividing it back can land one
     * float ULP above that integer. Do not add an unintended extra sample. */
    return fmaxf(1.0f, ceilf(nextafterf(duration_s / sample_period_s, -INFINITY))) * sample_period_s;
}
