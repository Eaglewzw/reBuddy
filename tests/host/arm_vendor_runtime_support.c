/* Small pthread queue/semaphore stand-ins; tasks run only when explicitly released. */
#include <assert.h>
#include <pthread.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_err.h"

struct host_queue {
    pthread_mutex_t lock;
    pthread_cond_t changed;
    unsigned capacity, item_size, head, count;
    unsigned char data[];
};

int64_t esp_timer_get_time(void)
{
    struct timespec now; clock_gettime(CLOCK_MONOTONIC, &now);
    return (int64_t)now.tv_sec * 1000000 + now.tv_nsec / 1000;
}
const char *esp_err_to_name(esp_err_t e) { (void)e; return "host error"; }
QueueHandle_t xQueueCreate(unsigned capacity, unsigned item_size)
{
    QueueHandle_t q = calloc(1, sizeof(*q) + capacity * item_size);
    assert(q); q->capacity = capacity; q->item_size = item_size;
    pthread_mutex_init(&q->lock, NULL); pthread_cond_init(&q->changed, NULL);
    return q;
}
static bool wait_changed(QueueHandle_t q, TickType_t ticks)
{
    if (ticks == 0) return false;
    if (ticks == portMAX_DELAY) return pthread_cond_wait(&q->changed, &q->lock) == 0;
    struct timespec until; clock_gettime(CLOCK_REALTIME, &until);
    until.tv_sec += ticks / 1000; until.tv_nsec += (long)(ticks % 1000) * 1000000;
    until.tv_sec += until.tv_nsec / 1000000000; until.tv_nsec %= 1000000000;
    return pthread_cond_timedwait(&q->changed, &q->lock, &until) == 0;
}
int xQueueSend(QueueHandle_t q, const void *item, TickType_t ticks)
{
    pthread_mutex_lock(&q->lock);
    while (q->count == q->capacity) {
        if (!wait_changed(q, ticks)) { pthread_mutex_unlock(&q->lock); return pdFALSE; }
    }
    if (q->item_size) memcpy(q->data + ((q->head + q->count) % q->capacity) * q->item_size, item, q->item_size);
    ++q->count; pthread_cond_broadcast(&q->changed); pthread_mutex_unlock(&q->lock); return pdTRUE;
}
static int receive(QueueHandle_t q, void *item, TickType_t ticks, bool pop)
{
    pthread_mutex_lock(&q->lock);
    while (!q->count) {
        if (!wait_changed(q, ticks)) { pthread_mutex_unlock(&q->lock); return pdFALSE; }
    }
    if (q->item_size) memcpy(item, q->data + q->head * q->item_size, q->item_size);
    if (pop) { q->head = (q->head + 1) % q->capacity; --q->count; }
    pthread_cond_broadcast(&q->changed); pthread_mutex_unlock(&q->lock); return pdTRUE;
}
int xQueueReceive(QueueHandle_t q, void *item, TickType_t ticks) { return receive(q, item, ticks, true); }
int xQueuePeek(QueueHandle_t q, void *item, TickType_t ticks) { return receive(q, item, ticks, false); }
int xQueueOverwrite(QueueHandle_t q, const void *item)
{
    assert(q->capacity == 1); pthread_mutex_lock(&q->lock);
    if (q->item_size) memcpy(q->data, item, q->item_size);
    q->head = 0; q->count = 1; pthread_cond_broadcast(&q->changed); pthread_mutex_unlock(&q->lock); return pdTRUE;
}
unsigned uxQueueMessagesWaiting(QueueHandle_t q)
{ pthread_mutex_lock(&q->lock); unsigned n = q->count; pthread_mutex_unlock(&q->lock); return n; }
void xQueueReset(QueueHandle_t q)
{ pthread_mutex_lock(&q->lock); q->head = q->count = 0; pthread_cond_broadcast(&q->changed); pthread_mutex_unlock(&q->lock); }
void vQueueDelete(QueueHandle_t q)
{ pthread_cond_destroy(&q->changed); pthread_mutex_destroy(&q->lock); free(q); }
SemaphoreHandle_t xSemaphoreCreateBinary(void) { return xQueueCreate(1, 0); }
SemaphoreHandle_t xSemaphoreCreateMutex(void)
{ SemaphoreHandle_t s = xSemaphoreCreateBinary(); xSemaphoreGive(s); return s; }

static pthread_mutex_t task_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t task_changed = PTHREAD_COND_INITIALIZER;
static bool tasks_released = true;
void host_hold_tasks(void) { pthread_mutex_lock(&task_lock); tasks_released = false; pthread_mutex_unlock(&task_lock); }
void host_release_tasks(void)
{ pthread_mutex_lock(&task_lock); tasks_released = true; pthread_cond_broadcast(&task_changed); pthread_mutex_unlock(&task_lock); }
struct task_start { void (*fn)(void *); void *arg; };
static void *start_task(void *data)
{
    struct task_start start = *(struct task_start *)data; free(data);
    pthread_mutex_lock(&task_lock);
    while (!tasks_released) pthread_cond_wait(&task_changed, &task_lock);
    pthread_mutex_unlock(&task_lock); start.fn(start.arg); return NULL;
}
int xTaskCreate(void (*fn)(void *), const char *name, unsigned stack, void *arg, unsigned priority, TaskHandle_t *out)
{
    (void)name; (void)stack; (void)priority;
    struct task_start *start = malloc(sizeof(*start)); assert(start); *start = (struct task_start){fn, arg};
    pthread_t thread; assert(pthread_create(&thread, NULL, start_task, start) == 0); pthread_detach(thread);
    if (out) *out = (void *)1;
    return pdPASS;
}
void vTaskDelay(TickType_t ticks)
{ struct timespec delay = {.tv_sec = ticks / 1000, .tv_nsec = (long)(ticks % 1000) * 1000000}; nanosleep(&delay, NULL); }
