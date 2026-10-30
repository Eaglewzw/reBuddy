#include "app_config.h"

static app_config_t s_cfg;
static bool s_set = false;

/* 内置弱默认：未注入时的兜底。集成方应调 app_config_set() 覆盖。 */
static const app_config_t s_default = {
    .control_period_ms = 5,
    .min_tx_interval_ms = 0,
    .offline_timeout_ms = 1000,
    .log_period_ms = 1000,
    .max_motors = 7,
    .default_kp = 0.5f,
    .default_kd = 0.08f,
    .default_motor_id = 1,
    .default_feedback_id = 0x11,
    .apply_default_id_map_at_boot = false,
    .bulk_op_gap_ms = 2,
    .robstride_default_model = "rs-00",
    .damiao_default_model = NULL,
};

void app_config_set(const app_config_t *cfg)
{
    if (cfg != NULL) {
        s_cfg = *cfg;
        s_set = true;
    } else {
        s_set = false;
    }
}

const app_config_t *app_config_get(void)
{
    return s_set ? &s_cfg : &s_default;
}

const app_config_t *app_config_get_defaults(void)
{
    return &s_default;
}
