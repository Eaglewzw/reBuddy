// SPDX-License-Identifier: Apache-2.0
#include "rb_memory.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_psram.h"
#include <stdlib.h>

void *rb_memory_lvgl_pool(size_t bytes)
{
    static void *pool;
    static size_t capacity;
    if (pool) return bytes <= capacity ? pool : NULL;
    if (!esp_psram_is_initialized()) {
        ESP_LOGE("rb_memory", "This board profile requires PSRAM");
        abort();
    }
    pool = heap_caps_calloc(1, bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!pool) abort();
    capacity = bytes;
    return pool;
}

void rb_memory_log(void)
{
    ESP_LOGI("rb_memory", "Internal free/min/largest=%u/%u/%u; PSRAM free=%u total=%u min=%u largest=%u",
        (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
        (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
        (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
        (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM),
        (unsigned)esp_psram_get_size(),
        (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_SPIRAM),
        (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM));
}
