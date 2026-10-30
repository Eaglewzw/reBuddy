#include "arm_stop.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

esp_err_t arm_stop_prepare(const float position[6], const float velocity[6],
    const arm_joint_path_limits_t *limits, const float lower[6], const float upper[6],
    uint32_t step_ms, arm_stop_t *stop)
{
    if (position == NULL || velocity == NULL || limits == NULL || lower == NULL ||
        upper == NULL || stop == NULL || step_ms == 0) return ESP_ERR_INVALID_ARG;
    double duration = step_ms * .001;
    for (unsigned j=0;j<6;++j) {
        if (!isfinite(position[j]) || !isfinite(velocity[j]) || !isfinite(limits->velocity[j]) ||
            !isfinite(limits->acceleration[j]) || limits->velocity[j]<=0 || limits->acceleration[j]<=0 ||
            !isfinite(lower[j]) || !isfinite(upper[j]) || position[j]<lower[j] || position[j]>upper[j] ||
            fabsf(velocity[j])>limits->velocity[j]) return ESP_ERR_INVALID_ARG;
        duration=fmax(duration,fabs((double)velocity[j])/limits->acceleration[j]);
    }
    const double intervals=ceil(duration/(step_ms*.001));
    if (!isfinite(intervals) || intervals > UINT32_MAX-1.0) return ESP_ERR_INVALID_ARG;
    duration=intervals*step_ms*.001;
    for(unsigned j=0;j<6;++j) {
        const double end=position[j]+.5*velocity[j]*duration;
        if ((float)end<lower[j] || (float)end>upper[j] || !isfinite(end)) return ESP_ERR_INVALID_ARG;
    }
    *stop=(arm_stop_t){.duration_s=duration,.intervals=(uint32_t)intervals,.step_ms=step_ms,.next=1};
    memcpy(stop->position,position,sizeof(stop->position));
    memcpy(stop->velocity,velocity,sizeof(stop->velocity));
    return ESP_OK;
}

void arm_stop_sample(const arm_stop_t *stop,uint32_t index,can_console_arm_motion_sample_t *point)
{
    const double t=index==stop->intervals?stop->duration_s:index*stop->step_ms*.001;
    *point=(can_console_arm_motion_sample_t){0};
    for(unsigned j=0;j<6;++j) {
        point->position[j]=(float)(stop->position[j]+stop->velocity[j]*t*(1-.5*t/stop->duration_s));
        point->reference_velocity[j]=index==stop->intervals?0:(float)(stop->velocity[j]*(1-t/stop->duration_s));
    }
}
