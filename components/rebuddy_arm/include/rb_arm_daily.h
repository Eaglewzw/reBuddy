#pragma once
#include "rb_daily_control.h"
/* Native task facade. Init starts transport/control task, never enables. */
esp_err_t rb_arm_daily_start(void);
esp_err_t rb_arm_daily_request(unsigned action,unsigned request);
esp_err_t rb_arm_daily_status(rb_dc_status_t *out);
void rb_arm_daily_home(void);
void rb_arm_daily_pause(void);
void rb_arm_daily_heartbeat(uint64_t now);
