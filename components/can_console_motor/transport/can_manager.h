#pragma once

/*
 * can_manager.h — 发送策略层(单目标 ESP32-C5,直调 ESP-IDF TWAI)。
 *
 * 在 twai 之上提供:每通道独立的发送背压、扫描任务
 * + RX 回调转发。上层(control_loop、frame_router、app)只经本头接触"发送"与"扫描"。
 *
 * 多实例:每条 CAN 总线对应一个 can_channel_t(0..CAN_MAX_CHANNELS-1),
 * 各通道状态完全隔离。ESP32-C5 有两路
 * TWAI 控制器,经 controller_id(0/1)分别装载。
 */

#include <stdbool.h>
#include <stdint.h>

#include "can_console_can.h"
#include "can_console_motor_session.h"
#include "driver/twai.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 通道索引(小整数,便于存进 motor_state_t)。 */
typedef int can_channel_t;

#define CAN_MAX_CHANNELS 2

/* 用户 RX 回调:带通道索引,使上层识别帧来自哪条总线。 */
typedef void (*can_rx_callback_t)(can_channel_t ch, const twai_message_t *msg);

/* 登记一路 CAN:调 twai_driver_install_v2 装驱动,返回通道索引(<0 失败)。
 * controller_id 选 TWAI 控制器(C5 两路:0/1)。必须在 start 之前、motor 绑定
 * channel 之前调。 */
can_channel_t can_manager_add_channel(int tx_gpio, int rx_gpio, int controller_id, int bitrate, int data_bitrate, int mode);

/* 启动一路:twai_start + 建 RX 任务与扫描任务。必须在 set_rx_callback 之后调。 */
esp_err_t can_manager_start_channel(can_channel_t ch);

/* per-channel 注入上层 RX 处理回调。必须在 start_channel 之前调。 */
void can_manager_set_rx_callback(can_channel_t ch, can_rx_callback_t cb);

/* 发送一帧:检查状态和队列水位后交给 can_console_can。 */
esp_err_t can_manager_send(can_channel_t ch, const twai_message_t *msg, int timeout_ms);

esp_err_t can_manager_send_for_client(can_channel_t ch,const twai_message_t *msg,
    int timeout_ms,can_motor_client_t client);

/* 扫描:让该通道扫描任务在 ~2s 内对 [min_id,max_id] 中属于本通道的电机
 * 逐个发 build_scan_request(仅扫 m.channel==ch 的 id)。 */
void can_manager_trigger_scan(can_channel_t ch, int min_id, int max_id);

/* Stop an active scan as soon as its caller has a verdict. */
void can_manager_stop_scan(can_channel_t ch);

/* "近期是否收到过帧"探针:供看门狗区分单电机掉线与总线静默。 */
bool can_manager_recent_rx(can_channel_t ch, int32_t within_ms);

/* Driver counters for diagnosing a silent bus without exposing the CAN handle. */
esp_err_t can_manager_get_status(can_channel_t ch,
                                 can_console_can_status_t *out_status);
esp_err_t can_manager_tx_fence_begin(can_channel_t ch, uint32_t *token);
esp_err_t can_manager_tx_fence_poll(can_channel_t ch, uint32_t token,
    can_console_can_tx_fence_t *receipt);

#ifdef __cplusplus
}
#endif
