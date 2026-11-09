#include "app_diagnostics.h"
#include "pet_ui.h"
#include "pet_assets.h"
#include "rb_board.h"
#include "rb_arm.h"
#include "rb_daily.h"
#include "rb_arm_daily.h"
#include "rb_memory.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_flash.h"
#include "esp_partition.h"

static void arm_log(void)
{
    rb_arm_status_t status;
    esp_err_t error=rb_arm_get_status(&status);
    ESP_LOGI("rb_arm","state=%u planner=%d CAN_started=%d CAN_healthy=%d feedback=0x%02x error=%s",
             (unsigned)status.state,status.planner_ready,status.bus_started,status.bus_healthy,
             status.feedback_valid_mask,esp_err_to_name(error));
#ifdef CONFIG_REBUDDY_ARM_DAILY_CONTROL
    /* Runs on the low-priority application task, outside the control mutex.
     * Keep a late-attached serial monitor useful even without a state change. */
    rb_dc_status_t daily;
    if(rb_arm_daily_status(&daily)==ESP_OK) {
        const rb_daily_action_t *action=rb_daily_find(daily.action);
        ESP_LOGI("rb_daily","snapshot state=%s action=%s request=%u completed=%u enabled=%d disable_confirmed=%d motor=%u detail=%s stop_detail=%s",
            rb_daily_control_state_name(daily.state),action?action->name:"unknown",
            daily.request_id,daily.completed_id,daily.enabled,daily.disable_confirmed,
            daily.bad_motor,daily.error?daily.error:"ok",daily.stop_error?daily.stop_error:"none");
        if(daily.error && daily.hold.valid) {
            const rb_dc_hold_snapshot_t *h=&daily.hold;
            ESP_LOGW("rb_daily","hold trigger at=%llu ms from=%s torque_checked=%d; following values are latched, not live",
                (unsigned long long)h->at_ms,rb_daily_control_state_name(h->source_state),h->torque_checked);
            for(unsigned j=0;j<7;++j)
                ESP_LOGW("rb_daily","trigger J%u measured=%.3f target=%.3f error=%.3f deg velocity=%.3f deg/s estimated_torque=%.3f budget=%.3f Nm",
                    j+1,h->measured[j]*57.2957795f,h->target[j]*57.2957795f,
                    (h->measured[j]-h->target[j])*57.2957795f,h->velocity[j]*57.2957795f,
                    h->estimated_torque[j],rb_daily_policy.torque_budget_nm[j]);
        }
    }
#endif
}
void rb_app_log_animation(lv_timer_t *timer)
{
    (void)timer;
    static unsigned previous;
    static uint64_t last;
    uint64_t now=rb_platform_now_ms(); unsigned updates=rb_ui_body_updates();
    if (last && now>last) ESP_LOGI("rb_animation","body_changes=%.1f/s feedback_to_ui=%llums cache=%u degraded=%d",
        (updates-previous)*1000.0/(now-last),(unsigned long long)rb_ui_last_feedback_ms(),
        (unsigned)rb_assets_cache_bytes(),rb_assets_degraded());
    previous=updates; last=now;
    rb_asset_stats_t stats=rb_assets_stats();
    ESP_LOGI("rb_decode","frames=%lu mean=%.2fms max=%.2fms failures=%lu",
        (unsigned long)stats.frames,stats.frames?stats.total_us/1000.0/stats.frames:0,
        stats.max_us/1000.0,(unsigned long)stats.failures);
}
void rb_app_log_startup(size_t animation_bytes)
{
}

void rb_app_log_health(void)
{
    rb_memory_log();
    arm_log();
}
