/* Internal time parameterization for Cartesian arm motion. */
#pragma once

float arm_trajectory_profile(float u);
float arm_trajectory_align_duration(float duration_s, float sample_period_s);
