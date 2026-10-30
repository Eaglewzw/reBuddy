#pragma once

/*
 * motor_manager.h — 电机状态/命令缓存枢纽（芯片无关）。
 *
 * 一块按 id 索引的数组 + 互斥锁；五类方法操作之。
 * 向上层暴露控制接口（set_*）与状态读取（get_state）；
 * 向 control_loop 暴露遍历接口（for_each）。
 */

#include <stdbool.h>
#include <stdint.h>

#include "config/app_config.h"
#include "transport/can_manager.h"   /* can_channel_t */

struct motor_vendor_ops_s;
typedef struct motor_vendor_ops_s motor_vendor_ops_t;

#ifdef __cplusplus
extern "C" {
#endif

/* 命令载荷（沿用历史命名，实为通用 MIT/POS/VEL/FORCE_POS 载荷） */
typedef struct {
    float pos;
    float vel;
    float tau;
    float vlim;
    float ratio;
    float acc;
    /* Incremented for every host control command.  Parameter-based vendors
     * use it to send one transaction per command instead of retriggering a
     * profile on every control-loop tick. */
    uint32_t seq;
} damiao_cmd_t;

/* 电机参数（限幅 + 增益） */
typedef struct {
    float kp;
    float kd;
    float p_min;
    float p_max;
    float v_min;
    float v_max;
    float t_min;
    float t_max;
    /* MIT 编码的增益上限,按型号查表(robstride rs-03/04/06 用 5000/100,
     * 其余 500/5)。damiao 等固定 500/5 的厂商保持默认值即可。镜像
     * motorbridge motor.rs:167-171。<=0 时 build_control_frame 回退 500/5。 */
    float kp_max;
    float kd_max;
} motor_params_t;

/* Common status bits used by vendor feedback decoders. The low byte keeps
 * the full RobStride operation-status flags; the high bits preserve the
 * asynchronous fault summary separately from telemetry flags. */
#define MOTOR_STATUS_MODE_MASK          0x0003U
#define MOTOR_STATUS_UNCALIBRATED       0x0004U
#define MOTOR_STATUS_STALL              0x0008U
#define MOTOR_STATUS_MAG_ENCODER_FAULT  0x0010U
#define MOTOR_STATUS_OVERTEMPERATURE     0x0020U
#define MOTOR_STATUS_OVERCURRENT         0x0040U
#define MOTOR_STATUS_UNDERVOLTAGE        0x0080U
#define MOTOR_STATUS_FAULT               0x0100U
#define MOTOR_STATUS_WARNING             0x0200U

#define MOTOR_DEFAULT_HOST_ID            0xFFU

/* 单颗电机完整状态 */
typedef struct {
    uint8_t id;
    bool online;
    bool enabled;
    motor_mode_t mode;
    /* Snapshot maintained by the single motor-service owner. */
    bool mode_ready;
    can_channel_t channel;     /* 该电机挂哪条 CAN 总线(TX 据此发,RX 回调带 channel 区分来源) */
    damiao_cmd_t cmd;
    float position;
    float speed;
    float torque;
    float t_mos;
    float t_rotor;
    /* Common status layout. Bits 0..7 carry vendor telemetry; bits 8..9
     * carry the asynchronous fault/warning summary. */
    uint16_t status;
    int64_t last_seen_us;
    /* Timestamp of the last decoded telemetry/status frame.  Discovery and
     * version replies update last_seen_us too, but do not contain position. */
    int64_t last_feedback_us;
    /* RobStride response/host ID learned from a ping or status frame. */
    uint8_t host_id;
    bool host_id_valid;
    motor_params_t params;
    const motor_vendor_ops_t *vendor;
    /* Robstride fault 上报(comm 21)原始字 + 时间戳。镜像 motorbridge 9908d0f。
     * 0 = 未见过故障 / 已清除。 */
    uint32_t fault_raw;
    uint32_t warning_raw;
    int64_t fault_seen_us;
    /* Robstride 固件版本(4 字节) + 时间戳。镜像 motorbridge 5e54fe1。 */
    uint8_t fw_version[4];
    int64_t fw_seen_us;
} motor_state_t;

void motor_manager_init(int max_motors);
int  motor_manager_count(void);

bool motor_manager_set_vendor(uint8_t id, const char *vendor_name);
/* Service-owner-only, with a validated disabled slot. */
void motor_manager_reset_session(uint8_t id);
bool motor_manager_set_channel(uint8_t id, can_channel_t channel);

bool motor_manager_set_mode(uint8_t id, motor_mode_t mode);
bool motor_manager_set_enabled(uint8_t id, bool enabled);

bool motor_manager_set_mit_cmd(uint8_t id, float pos, float vel, float tau);
bool motor_manager_set_pos_vel_cmd(uint8_t id, float pos, float vlim);
bool motor_manager_set_pos_csp_cmd(uint8_t id, float pos, float vlim);
bool motor_manager_set_pos_pp_cmd(uint8_t id, float pos, float vlim, float acc);
bool motor_manager_set_vel_cmd(uint8_t id, float vel);
bool motor_manager_set_force_pos_cmd(uint8_t id, float pos, float vlim, float ratio);
bool motor_manager_set_gains(uint8_t id, float kp, float kd);
/* 按型号名设量程(运行时覆盖,经厂商 resolve_model 钩子查表)。
 * 返回 false 表示型号未知或厂商无型号表。镜像 motorbridge faba15f。 */
bool motor_manager_set_model(uint8_t id, const char *model);

bool motor_manager_get_state(uint8_t id, motor_state_t *out);
void motor_manager_update_feedback(uint8_t id, float pos, float speed, float torque,
                                   float t_mos, float t_rotor, uint16_t status);
void motor_manager_mark_seen(uint8_t id);
void motor_manager_update_host_id(uint8_t id, uint8_t host_id);

/* 记录 Robstride fault 上报(comm 21)。置 fault/status 位 + fault_seen_us 时间戳。
 * 镜像 motorbridge 9908d0f。 */
void motor_manager_update_fault(uint8_t id, uint32_t fault_raw, uint32_t warning_raw);
/* 记录 Robstride 固件版本应答。镜像 motorbridge 5e54fe1。 */
void motor_manager_update_fw_version(uint8_t id, const uint8_t version[4]);

void motor_manager_mark_offline_by_timeout(int64_t now_us, int32_t timeout_ms);
void motor_manager_estop_all(void);
void motor_manager_clear_online_all(void);

/* visitor：持锁遍历，对每个电机调 fn。发送/安全/查询共用。 */
void motor_manager_for_each(void (*fn)(motor_state_t *m, void *ctx), void *ctx);

#ifdef __cplusplus
}
#endif
