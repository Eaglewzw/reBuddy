// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "esp_err.h"
#include "rb_board_input.h"
/* One-shot startup. A failure is fatal to the preview, not silently ignored. */
esp_err_t rb_board_init(void);
bool rb_board_estop_released(void);
bool rb_board_read_input(rb_board_input_t *event);
bool rb_board_display_lock(uint32_t timeout_ms);
void rb_board_display_unlock(void);
