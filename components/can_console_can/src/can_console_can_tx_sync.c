/* SPDX-License-Identifier: Apache-2.0 */
#include "can_console_can_tx_sync.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

esp_err_t can_console_can_wait_tx_idle(twai_node_handle_t node, const volatile bool *running,
                                     const volatile bool *completed)
{
    bool bus_off_seen = false;
    while (*running) {
        /* on_tx_done notifies this task directly from ISR. IDF's blocking
         * wait_all_done instead waits for an event-group update dispatched by
         * the low-priority FreeRTOS timer daemon, which UI rendering can delay.
         * A short fallback also handles bus-off/recovery without a TX callback. */
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10));
        if (!*running) break;
        esp_err_t result = twai_node_transmit_wait_all_done(node, 0);
        if (result == ESP_ERR_INVALID_STATE) bus_off_seen = true;
        /* A deferred idle bit from a PREVIOUS transfer may be delivered while
         * this frame is busy. Require its own TX callback before reusing it.
         * After bus-off the single in-flight frame may instead be discarded by
         * recovery; the driver's recovered-idle result releases that buffer. */
        if (result == ESP_OK && !*completed && !bus_off_seen) continue;
        if (result != ESP_ERR_TIMEOUT && result != ESP_ERR_INVALID_STATE) return result;
    }
    return ESP_ERR_INVALID_STATE;
}
