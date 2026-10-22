// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <stdbool.h>
#include <stdint.h>

typedef struct { int8_t dial; bool enter, back, fault; } rb_board_input_t;
typedef struct {
    bool initialized;
    uint16_t candidate, stable;
    uint64_t changed_ms;
} rb_keys_t;
/* Active-low PCA9535 inputs. Suppress held keys at startup/reconnection. */
rb_board_input_t rb_keys_sample(rb_keys_t *keys, uint16_t levels, bool valid, uint64_t now_ms);
/* Native GT911 portrait coordinates -> fixed 320x240 landscape. */
bool rb_touch_map(uint16_t raw_x, uint16_t raw_y, int16_t *x, int16_t *y);
typedef struct { bool active, blocked; uint8_t id; } rb_primary_touch_t;
/* Index of original contact; -1 released, -2 cancelled until all contacts lift. */
int rb_primary_touch_select(rb_primary_touch_t *tracker, const uint8_t *ids, unsigned count, bool valid);
