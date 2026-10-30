#pragma once

/*
 * mb_port.h — OS 汇聚头(单目标 ESP32-C5)。
 *
 * 时间/日志经本头汇聚;mutex/任务直接假定 FreeRTOS(ESP-IDF 提供)。
 * 单目标后不再有 port/ 分层,本头是唯一入口。
 */

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 单调时间戳,单位微秒。映射到 ESP-IDF 的 esp_timer_get_time。 */
static inline int64_t mb_now_us(void)
{
    return esp_timer_get_time();
}

/* ---- 最小等待 ---- */

/* 毫秒→tick 换算注入点:生产用 pdMS_TO_TICKS(目标 100 Hz、截断);确定性
 * 测试注入自定义换算以覆盖换算边界。 */
typedef TickType_t mb_ms_to_ticks_fn_t(uint32_t ms);

static inline TickType_t mb_ms_to_ticks(uint32_t ms)
{
    return pdMS_TO_TICKS(ms);
}

/* 确定性测试注入时钟/睡眠/换算的核心。契约:仅在 now_us() ≥ deadline 时
 * 返回;每次 sleep 参数 ≥1;传入期限不被缩短。入参应为配置量级(配置
 * 校验限制 bulk gap ≤60s,距 ms→tick 换算的溢出阈值仍有量级余量)。 */
static inline void mb_sleep_min_core(int64_t deadline_us,
    int64_t (*now_us)(void), void (*sleep_ticks)(TickType_t),
    mb_ms_to_ticks_fn_t *ms_to_ticks)
{
    for (;;) {
        const int64_t remain = deadline_us - now_us();
        if (remain <= 0) {
            return;
        }
        const uint32_t ms = (uint32_t)((remain + 999) / 1000);
        TickType_t ticks = ms_to_ticks(ms);
        sleep_ticks(ticks > 0 ? ticks : 1);
    }
}

/* 阻塞至少 us 微秒(单调时钟)。只保证下界:唤醒与调度不受控,不构成
 * 等待上界;实现软件操作间隔契约(如 bulk_op_gap、扫描探针节拍),不
 * 构成总线帧间隔或设备处理时间的保证。亚 tick 总线节拍归发送侧
 * (esp_rom_delay_us 忙等),分工见组件 README。 */
static inline void mb_sleep_min_us(int64_t us)
{
    mb_sleep_min_core(mb_now_us() + us, mb_now_us, vTaskDelay, mb_ms_to_ticks);
}

/* 日志宏:直接转发到 esp_log。TAG 由各 .c 文件内定义。 */
#define MB_LOGI(tag, fmt, ...) ESP_LOGI(tag, fmt, ##__VA_ARGS__)
#define MB_LOGW(tag, fmt, ...) ESP_LOGW(tag, fmt, ##__VA_ARGS__)
#define MB_LOGE(tag, fmt, ...) ESP_LOGE(tag, fmt, ##__VA_ARGS__)

#ifdef __cplusplus
}
#endif

