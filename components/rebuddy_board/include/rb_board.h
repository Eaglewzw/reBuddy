// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "esp_err.h"
#include "rb_board_input.h"
/* One-shot startup. A failure is fatal to the preview, not silently ignored. */
esp_err_t rb_board_init(void);
/* Startup-only, serialized with board init. Changes CAN1 P1.0 only; CAN2,
 * display and touch outputs retain their levels. Does not initialize I2C. */
esp_err_t rb_board_can1_standby(bool standby);
/* PCA9535 P0.6, active low, matching esp32c5_ui_console's enter_pin.
 * Read-only, on the existing board I2C bus; caller supplies debounce. */
esp_err_t rb_board_read_enter(bool *pressed);
bool rb_board_display_lock(uint32_t timeout_ms);
void rb_board_display_unlock(void);
