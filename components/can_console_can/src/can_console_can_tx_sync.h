/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include <stdbool.h>
#include "esp_twai.h"

/* The TX task owns its frame until TWAI reports hardware/queue idle. */
esp_err_t can_console_can_wait_tx_idle(twai_node_handle_t node, const volatile bool *running,
                                     const volatile bool *completed);
