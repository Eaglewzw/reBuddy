#pragma once
#include "can_console_arm_profile.h"
#include "can_console_motor_client.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum { ARM_PLAYBACK_IDLE, ARM_PLAYBACK_PREPARING, ARM_PLAYBACK_ACQUIRING,
    ARM_PLAYBACK_RUNNING, ARM_PLAYBACK_STOPPING, ARM_PLAYBACK_DRAINING,
    ARM_PLAYBACK_RELEASING, ARM_PLAYBACK_CANCELLING } arm_playback_phase_t;
typedef enum { ARM_PLAYBACK_NONE, ARM_PLAYBACK_COMPLETED, ARM_PLAYBACK_USER_STOP,
    ARM_PLAYBACK_DISABLE, ARM_PLAYBACK_ESTOP, ARM_PLAYBACK_PLANNING_ERROR,
    ARM_PLAYBACK_CONTROL_FAULT } arm_playback_reason_t;
typedef struct {
    float position[6], velocity[6];
    float gripper_position;
    bool gripper_valid;
    uint32_t age_ms, skew_ms;
} arm_playback_feedback_t;

/* Caller-serialized executor state (zero-initialize before first use).
 * No task, lock, allocation or fault latch is added by this component.
 * Uses the existing singleton planner and motor reference API; the caller
 * excludes competing plans/targets/admin/gains and keeps profile/trajectory
 * data alive until retirement. The application supplies gravity torque.
 * After a tick error, the caller aborts and disables/ESTOPs the motor group;
 * retire() waits for planner quiescence and motor ownership cancellation.
 * Idle means delivery ownership retired, not measured physical arrival. */
typedef struct {
    arm_playback_phase_t phase;
    arm_playback_reason_t reason;
    const arm_hardware_profile_t *profile;
    bool gravity_enabled;
    float torque[6], queued_torque[6], initial_torque[6];
    bool home, seek, follow, stop_requested, stop_planned, pending, acquired, motion_accepted;
    bool gripper_owned, gripper_hold;
    const arm_teach_sample_t *gripper_samples; /* Borrowed until executor retirement. */
    uint32_t gripper_sample_count;
    float gripper_start, gripper_first, gripper_confirmed, gripper_queued;
    float gripper_seek_duration, gripper_stop_position;
    unsigned pending_kind; /* initial=1, point=2, heartbeat=3, release=4 */
    uint32_t sequence, step_ms;
    int64_t begun_us, submitted_us, confirmed_us, last_delivery_us, empty_us;
    float execution_velocity[6];
    can_console_arm_motion_sample_t confirmed, queued;
    uint32_t min_delivery_us, max_delivery_us;
    esp_err_t last_error;
    uint32_t confirmed_points;
    bool probe;
    bool position_mode; /* POS_VEL reference group instead of MIT */
    bool recorded_timing, planned_timing, due_ready;
    float speed, drive_velocity_limit[6];
    int64_t timeline_us;
    can_console_arm_motion_sample_t due;
    uint32_t skipped_points;
    int64_t ended_us, stop_requested_us;
    float measured[6], stop_origin[6], max_stop_travel[6];
    float max_tracking[6], max_measured_velocity[6];
    uint32_t max_ack_us, deferrals;
    uint32_t max_feedback_age_ms, max_feedback_skew_ms;
    unsigned feedback_fault, feedback_motor, feedback_status;
    uint32_t feedback_fault_age_ms;
} arm_playback_t;

/* App-owned scratch, immutable until planner/reference retirement. Never aliases
 * the user's recording. These are commissioning budgets, not a calibration. */
typedef struct {
    arm_hardware_profile_t profile;
    arm_playback_calibration_t budgets;
    arm_teach_sample_t samples[2];
} arm_playback_probe_t;

esp_err_t arm_playback_begin(arm_playback_t *playback,const arm_teach_trajectory_t *trajectory,
    const arm_hardware_profile_t *profile,const arm_playback_feedback_t *feedback,bool seek,float speed);
esp_err_t arm_playback_begin_follow(arm_playback_t *playback,const arm_teach_trajectory_t *trajectory,
    const arm_hardware_profile_t *profile,const arm_playback_feedback_t *feedback,float speed);
/* Continue a completed play from its last confirmed reference, using the
 * existing joint planner and the same delivery/stop/release state machine.
 * No measured-arrival wait. Profile and borrowed trajectory lifetime remain
 * caller-owned until retirement; callers serialize all API calls. */
esp_err_t arm_playback_begin_home(arm_playback_t *playback,
    const arm_playback_feedback_t *feedback, float duration_s);
esp_err_t arm_playback_begin_probe(arm_playback_t *playback,arm_playback_probe_t *scratch,
    const arm_hardware_profile_t *profile,const arm_playback_feedback_t *feedback,
    bool seek,unsigned joint,float delta,float velocity,float acceleration,bool position_mode);
esp_err_t arm_playback_tick(arm_playback_t *playback,const arm_playback_feedback_t *feedback);
void arm_playback_stop(arm_playback_t *playback);
void arm_playback_abort(arm_playback_t *playback,arm_playback_reason_t reason,esp_err_t error);
void arm_playback_retire(arm_playback_t *playback);
const char *arm_playback_phase_name(arm_playback_phase_t phase);

#ifdef __cplusplus
}
#endif
