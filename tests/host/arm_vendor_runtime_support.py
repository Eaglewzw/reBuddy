"""Temporary IDF headers for pthread-based lifecycle checks; no hardware I/O."""
from pathlib import Path


def write_headers(folder: Path):
    headers = {
        "esp_err.h": """#pragma once
typedef int esp_err_t;
#define ESP_OK 0
#define ESP_FAIL -1
#define ESP_ERR_NO_MEM 0x101
#define ESP_ERR_INVALID_ARG 0x102
#define ESP_ERR_INVALID_STATE 0x103
#define ESP_ERR_NOT_FOUND 0x105
#define ESP_ERR_NOT_SUPPORTED 0x106
#define ESP_ERR_TIMEOUT 0x107
const char *esp_err_to_name(esp_err_t);
""",
        "esp_log.h": """#pragma once
static inline void host_log(const char *tag, const char *format, ...) { (void)tag; (void)format; }
#define ESP_LOGI(...) host_log(__VA_ARGS__)
#define ESP_LOGW(...) host_log(__VA_ARGS__)
#define ESP_LOGE(...) host_log(__VA_ARGS__)
#define ESP_LOGD(...) host_log(__VA_ARGS__)
""",
        "esp_timer.h": "#pragma once\n#include <stdint.h>\nint64_t esp_timer_get_time(void);\n",
        "driver/gpio.h": "#pragma once\ntypedef int gpio_num_t;\n",
        "driver/twai.h": """#pragma once
#include <stdint.h>
typedef struct { uint32_t identifier; uint8_t data_length_code; uint8_t data[8];
 unsigned extd:1; unsigned rtr:1; unsigned ss:1; unsigned self:1; unsigned dlc_non_comp:1;
} twai_message_t;
""",
        "freertos/FreeRTOS.h": """#pragma once
#include <stdbool.h>
#include <stdint.h>
typedef int BaseType_t;
typedef unsigned UBaseType_t;
typedef uint32_t TickType_t;
#define pdTRUE 1
#define pdFALSE 0
#define pdPASS 1
#define portMAX_DELAY UINT32_MAX
#define pdMS_TO_TICKS(ms) (ms)
""",
        "freertos/queue.h": """#pragma once
#include "FreeRTOS.h"
typedef struct host_queue *QueueHandle_t;
QueueHandle_t xQueueCreate(unsigned capacity, unsigned item_size);
int xQueueSend(QueueHandle_t, const void *, TickType_t);
int xQueueReceive(QueueHandle_t, void *, TickType_t);
int xQueuePeek(QueueHandle_t, void *, TickType_t);
int xQueueOverwrite(QueueHandle_t, const void *);
unsigned uxQueueMessagesWaiting(QueueHandle_t);
void xQueueReset(QueueHandle_t);
void vQueueDelete(QueueHandle_t);
""",
        "freertos/semphr.h": """#pragma once
#include "queue.h"
typedef QueueHandle_t SemaphoreHandle_t;
SemaphoreHandle_t xSemaphoreCreateMutex(void);
SemaphoreHandle_t xSemaphoreCreateBinary(void);
#define xSemaphoreTake(s, ticks) xQueueReceive(s, 0, ticks)
#define xSemaphoreGive(s) xQueueSend(s, 0, 0)
#define vSemaphoreDelete(s) vQueueDelete(s)
""",
        "freertos/task.h": """#pragma once
#include "FreeRTOS.h"
typedef void *TaskHandle_t;
int xTaskCreate(void (*fn)(void *), const char *, unsigned, void *, unsigned, TaskHandle_t *);
void vTaskDelay(TickType_t);
static inline void xTaskNotifyGive(TaskHandle_t t) { (void)t; }
static inline unsigned ulTaskNotifyTake(int clear, TickType_t t) { (void)clear; (void)t; return 0; }
void host_hold_tasks(void);
void host_release_tasks(void);
""",
    }
    for name, contents in headers.items():
        path = folder / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(contents)
