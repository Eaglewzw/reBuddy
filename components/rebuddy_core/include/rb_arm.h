#ifndef RB_ARM_H
#define RB_ARM_H
#include "rb_core.h"
/* Adapter contract: nonblocking, latest target only, feedback age <=100 ms.
 * send() means accepted, never proof of arrival. stop() must cancel queued travel
 * and request a controller-defined safe hold. Software STOP is not a power cut. */
typedef struct {
    bool (*feedback)(void *context, float degrees[RB_JOINTS], uint64_t *stamp_ms);
    bool (*send)(void *context, const float degrees[RB_JOINTS]);
    void (*stop)(void *context);
    void *context;
} rb_arm_driver_t;
typedef struct {
    bool calibrated;
    float neutral[RB_JOINTS], lower[RB_JOINTS], upper[RB_JOINTS];
} rb_arm_calibration_t;
typedef struct {
    const rb_arm_driver_t *driver;
    rb_arm_calibration_t calibration;
    bool enabled;
    float last_target[RB_JOINTS];
    uint64_t sent_ms;
} rb_arm_t;
bool rb_arm_enable(rb_arm_t *arm, const rb_arm_driver_t *driver, const rb_arm_calibration_t *calibration, uint64_t now);
bool rb_arm_update(rb_arm_t *arm, rb_engine_t *engine, uint64_t now, bool estop_released);
void rb_arm_stop(rb_arm_t *arm);
#endif
