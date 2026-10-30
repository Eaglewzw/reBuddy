#include "can_console_arm_gravity.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#include "arm_gravity_model.h"

void can_console_arm_gravity_config_set_defaults(can_console_arm_gravity_config_t *config)
{
    if (config == NULL) {
        return;
    }
    memset(config, 0, sizeof(*config));
    config->model = can_console_ik_get_model();
    config->gravity_base[2] = -9.81f;
    for (size_t i = 0; i < CAN_CONSOLE_IK_DOF; ++i) {
        config->direction[i] = 1.0f;
        config->torque_scale[i] = 1.0f;
    }
}

esp_err_t can_console_arm_gravity_init(can_console_arm_gravity_t *context,
                                      const can_console_arm_gravity_config_t *config)
{
    if (context == NULL || config == NULL || arm_gravity_model_get(config->model) == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    for (size_t i = 0; i < 3; ++i) {
        if (!isfinite(config->gravity_base[i]) || !isfinite(config->payload_com[i])) {
            return ESP_ERR_INVALID_ARG;
        }
    }
    for (size_t i = 0; i < CAN_CONSOLE_IK_DOF; ++i) {
        if ((config->direction[i] != 1.0f && config->direction[i] != -1.0f) ||
            !isfinite(config->zero_offset[i]) || !isfinite(config->torque_scale[i]) ||
            config->torque_scale[i] < 0.0f) {
            return ESP_ERR_INVALID_ARG;
        }
    }
    if (!isfinite(config->payload_mass) || config->payload_mass < 0.0f) {
        return ESP_ERR_INVALID_ARG;
    }
    can_console_ik_init();
    context->config = *config;
    context->initialized = true;
    return ESP_OK;
}

static void transform_com(const float position[3], const float rotation[9],
                          const float local[3], float world[3])
{
    for (size_t r = 0; r < 3; ++r) {
        world[r] = position[r];
        for (size_t c = 0; c < 3; ++c) {
            world[r] += rotation[3 * r + c] * local[c];
        }
    }
}

static void add_body(const can_console_ik_chain_t *chain, size_t upstream_count,
                     float mass, const float com[3], const float gravity[3],
                     float torque[CAN_CONSOLE_IK_DOF])
{
    const float force[3] = {mass * gravity[0], mass * gravity[1], mass * gravity[2]};
    for (size_t i = 0; i < upstream_count; ++i) {
        const float *p = chain->joint_position[i];
        const float r[3] = {com[0] - p[0], com[1] - p[1], com[2] - p[2]};
        const float moment[3] = {
            r[1] * force[2] - r[2] * force[1],
            r[2] * force[0] - r[0] * force[2],
            r[0] * force[1] - r[1] * force[0],
        };
        for (size_t c = 0; c < 3; ++c) {
            torque[i] -= chain->joint_axis[i][c] * moment[c];
        }
    }
}

esp_err_t can_console_arm_gravity_compute(const can_console_arm_gravity_t *context,
                                         const float q_motor[CAN_CONSOLE_IK_DOF],
                                         float torque[CAN_CONSOLE_IK_DOF])
{
    if (context == NULL || q_motor == NULL || torque == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!context->initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    const can_console_arm_gravity_config_t *cfg = &context->config;
    const arm_gravity_model_t *model = arm_gravity_model_get(cfg->model);
    if (model == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    float q[CAN_CONSOLE_IK_DOF];
    for (size_t i = 0; i < CAN_CONSOLE_IK_DOF; ++i) {
        q[i] = cfg->direction[i] * (q_motor[i] - cfg->zero_offset[i]);
    }
    can_console_ik_chain_t chain;
    if (!can_console_ik_forward_chain_for_model(cfg->model, q, &chain)) {
        return ESP_ERR_INVALID_ARG;
    }
    float result[CAN_CONSOLE_IK_DOF] = {0};
    float com[3];
    for (size_t i = 0; i < CAN_CONSOLE_IK_DOF; ++i) {
        transform_com(chain.joint_position[i], chain.link_rotation[i], model->links[i].com, com);
        add_body(&chain, i + 1U, model->links[i].mass, com, cfg->gravity_base, result);
    }
    const size_t last = CAN_CONSOLE_IK_DOF - 1U;
    transform_com(chain.joint_position[last], chain.link_rotation[last], model->tool.com, com);
    add_body(&chain, CAN_CONSOLE_IK_DOF, model->tool.mass, com, cfg->gravity_base, result);
    transform_com(chain.end_position, chain.end_rotation, cfg->payload_com, com);
    add_body(&chain, CAN_CONSOLE_IK_DOF, cfg->payload_mass, com, cfg->gravity_base, result);
    for (size_t i = 0; i < CAN_CONSOLE_IK_DOF; ++i) {
        result[i] *= cfg->direction[i] * cfg->torque_scale[i];
        if (!isfinite(result[i])) {
            return ESP_FAIL;
        }
    }
    memcpy(torque, result, sizeof(result));
    return ESP_OK;
}
