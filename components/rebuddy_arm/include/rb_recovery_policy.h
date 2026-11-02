#pragma once
#include <math.h>
/* Recovery keeps gravity unchanged and limits damping gains explicitly. This
 * is not a clamp on the ordinary trajectory command or a larger torque cap. */
static inline float rb_recovery_kd(float gravity,float velocity,float kd,float cap)
{
    float estimate=gravity-kd*velocity;
    if(fabsf(estimate)<=cap)return kd;
    if(fabsf(velocity)<1e-8f)return 0;
    float bounded=(gravity-copysignf(cap*.98f,estimate))/velocity;
    return fmaxf(0,fminf(kd,bounded));
}
static inline float rb_filtered_velocity(float previous,float delta,float dt)
{ return previous+dt/(.04f+dt)*(delta/dt-previous); }
