#include "rb_motion.h"
#include <math.h>
#include <stddef.h>

float rb_motion_ease(float phase)
{
    if (phase<=0) return 0;
    if (phase>=1) return 1;
    return phase*phase*phase*(10+phase*(-15+6*phase));
}
bool rb_motion_bridge_duration(const int16_t from[RB_MOTION_JOINTS],
    const int16_t to[RB_MOTION_JOINTS],const rb_motion_limit_t limits[RB_MOTION_JOINTS],
    uint32_t minimum_ms,uint32_t *duration_ms)
{
    if (!from || !to || !limits || !duration_ms) return false;
    double seconds=minimum_ms/1000.0;
    for(unsigned i=0;i<RB_MOTION_JOINTS;++i) {
        if (limits[i].minimum>limits[i].maximum || !limits[i].velocity || !limits[i].acceleration ||
            from[i]<limits[i].minimum || from[i]>limits[i].maximum ||
            to[i]<limits[i].minimum || to[i]>limits[i].maximum) return false;
        double distance=fabs((double)to[i]-from[i]);
        /* Analytic maxima of the quintic: |q'|=1.875, |q''|=10/sqrt(3). */
        seconds=fmax(seconds,1.875*distance/limits[i].velocity);
        seconds=fmax(seconds,sqrt((10/sqrt(3.0))*distance/limits[i].acceleration));
    }
    double ms=ceil(seconds*1000);
    if (ms>UINT32_MAX) return false;
    *duration_ms=ms<1?1:(uint32_t)ms;
    return true;
}
bool rb_motion_bridge_sample(const int16_t from[RB_MOTION_JOINTS],
    const int16_t to[RB_MOTION_JOINTS],uint32_t elapsed_ms,uint32_t duration_ms,
    float output[RB_MOTION_JOINTS])
{
    if(!from || !to || !output || !duration_ms) return false;
    float phase=elapsed_ms>=duration_ms?1:rb_motion_ease((float)elapsed_ms/duration_ms);
    for(unsigned i=0;i<RB_MOTION_JOINTS;++i)
        output[i]=from[i]+((float)to[i]-from[i])*phase;
    return true;
}
