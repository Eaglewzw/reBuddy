#pragma once

/*
 * app_config.h — 运行时配置 + 协议常量（芯片无关）。
 *
 * 已与 Kconfig/sdkconfig 解耦：集成方启动时调 app_config_set() 注入配置。
 * GPIO/波特率/控制器 id 属 CAN 板级配置，见 can_manager_add_channel（transport/can_manager.h）。
 * WiFi 等属 app/bsp，不在此处。
 */

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>   /* NULL */

#include "can_console_motor_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- 主机 CAN 协议基址（host ingress 用，可选）---- */
#define MOTORBRIDGE_HOST_ADMIN_ID        0x050u

#define MOTORBRIDGE_HOST_MIT_BASE_ID     0x100u
#define MOTORBRIDGE_HOST_POS_VEL_BASE_ID 0x140u
#define MOTORBRIDGE_HOST_VEL_BASE_ID     0x180u
#define MOTORBRIDGE_HOST_FORCE_POS_BASE_ID 0x1C0u

/* ---- 电机 id 范围 ---- */
#define MOTORBRIDGE_MIN_MOTOR_ID 0
/* Compile-time upper bound for per-motor static tables (s_motors, vendor
 * mode/verify state, frame-router time stamps).  Integrators driving a small
 * fixed id set can override this on the command line / via CMake to shrink
 * the static arrays and free SRAM for the heap; full 128-motor support is
 * the default.  Must stay >= the largest id the app ever probes. */
#ifndef MOTORBRIDGE_MAX_MOTOR_ID
#define MOTORBRIDGE_MAX_MOTOR_ID 127
#endif

/* ---- 运行时配置 ---- */
typedef struct {
    int   control_period_ms;     /* 控制周期 */
    int   min_tx_interval_ms;    /* 单电机最小发送间隔（节流） */
    int   offline_timeout_ms;    /* 反馈超时→离线 */
    int   log_period_ms;
    int   max_motors;
    float default_kp;
    float default_kd;
    uint8_t default_motor_id;
    uint8_t default_feedback_id;
    bool  apply_default_id_map_at_boot;
    /* 范围管理操作(submit_admin_range)中,前一电机的命令处理返回后、
     * 开始下一电机前的最小墙钟间隔(ms)。软件操作间隔:帧异步排队发送,
     * 不构成总线帧间隔或设备处理时间保证。完整等待配置值、不静默缩短,
     * 按 FreeRTOS tick 量化(默认 100 Hz 下 10 ms);上界 60000 由
     * can_console_motor_init 显式拒绝。镜像 motorbridge
     * DEFAULT_BULK_OP_GAP_MS(7289487 / fee4cad)。0 = 不延迟。 */
    int   bulk_op_gap_ms;
    /* Robstride 默认型号,用于厂商绑定时查 PvT 量程。host 无线上型号探测,
     * 型号是配置值(默认 "rs-00")。镜像 motorbridge faba15f。 */
    const char *robstride_default_model;
    /* Damiao 默认型号,用于厂商绑定时查 PvT 量程。NULL 保持历史 4340
     * 默认量程,集成方可设置为 "4310" 等具体型号。 */
    const char *damiao_default_model;
    /* 0 keeps component defaults: service 6, CAN I/O 5, discovery 6. */
    uint8_t service_task_priority, can_task_priority, scan_task_priority;
} app_config_t;

/* 注入配置（启动期调用一次）。传 NULL 则使用内置弱默认。 */
void app_config_set(const app_config_t *cfg);

/* 取当前配置指针。未调用 set 时返回弱默认。 */
const app_config_t *app_config_get(void);

/* Internal source of startup defaults for the public runtime facade. */
const app_config_t *app_config_get_defaults(void);

#ifdef __cplusplus
}
#endif
