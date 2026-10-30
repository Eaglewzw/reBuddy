#pragma once
#include <stddef.h>
#include <stdint.h>

/* control_loop.h — service-owner periodic transmit implementation. */

#ifdef __cplusplus
extern "C" {
#endif

void control_loop_process_tick(float dt_s);
void control_loop_reference_ids(const uint8_t *ids, size_t count);

#ifdef __cplusplus
}
#endif

