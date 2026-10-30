#include "safety_manager.h"

#include "mb_port.h"

#include "config/app_config.h"
#include "core/motor_manager.h"

void safety_manager_tick(void)
{
    const app_config_t *cfg = app_config_get();
    if (cfg == NULL || cfg->offline_timeout_ms <= 0) {
        return;
    }

    motor_manager_mark_offline_by_timeout(mb_now_us(), cfg->offline_timeout_ms);
}

