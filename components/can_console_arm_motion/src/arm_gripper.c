#include "can_console_arm_gripper.h"
#include <math.h>

bool arm_gripper_profile_valid(const arm_gripper_profile_t *p)
{
    return p && p->motor_id && p->vendor && p->model && p->calibration_id &&
        isfinite(p->closed_position) && isfinite(p->open_position) &&
        fabsf(p->open_position-p->closed_position)>1e-4f &&
        isfinite(p->mit_kp) && p->mit_kp>0 && isfinite(p->mit_kd) && p->mit_kd>0 &&
        isfinite(p->teach_kd) && p->teach_kd>0 &&
        isfinite(p->velocity_limit) && p->velocity_limit>0 &&
        isfinite(p->start_tolerance) && p->start_tolerance>0;
}

esp_err_t arm_gripper_position(const arm_gripper_profile_t *p,float opening,float *position)
{
    if(!position || !arm_gripper_profile_valid(p) || !isfinite(opening) || opening<0 || opening>1)
        return ESP_ERR_INVALID_ARG;
    *position=p->closed_position+opening*(p->open_position-p->closed_position);
    return ESP_OK;
}

esp_err_t arm_gripper_opening(const arm_gripper_profile_t *p,float position,float *opening)
{
    if(!opening || !arm_gripper_profile_valid(p) || !isfinite(position) ||
        position<fminf(p->closed_position,p->open_position)-p->start_tolerance ||
        position>fmaxf(p->closed_position,p->open_position)+p->start_tolerance)
        return ESP_ERR_INVALID_ARG;
    *opening=fminf(1,fmaxf(0,(position-p->closed_position)/(p->open_position-p->closed_position)));
    return ESP_OK;
}

float arm_gripper_step(float position,float target,float velocity,float dt_s)
{
    const float step=velocity*fmaxf(0,dt_s);
    return fminf(position+step,fmaxf(position-step,target));
}
