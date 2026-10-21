#include "rb_arm.h"
#include <math.h>
#include <string.h>
bool rb_arm_enable(rb_arm_t *a, const rb_arm_driver_t *d, const rb_arm_calibration_t *c, uint64_t now)
{
    if (!a || a->enabled || !d || !c || !c->calibrated || !d->feedback || !d->send || !d->stop) return false;
    float measured[RB_JOINTS]; uint64_t stamp=0;
    if (!d->feedback(d->context,measured,&stamp) || stamp>now || now-stamp>100) return false;
    for (unsigned j=0;j<RB_JOINTS;++j)
        if (!isfinite(c->neutral[j]) || !isfinite(c->lower[j]) || !isfinite(c->upper[j]) ||
            c->lower[j]>=c->upper[j] || c->neutral[j]<c->lower[j] || c->neutral[j]>c->upper[j] ||
            !isfinite(measured[j]) || fabsf(measured[j]-c->neutral[j])>.5f) return false;
    a->driver=d; a->calibration=*c; a->enabled=true; a->sent_ms=now;
    memcpy(a->last_target,measured,sizeof(measured)); return true;
}
void rb_arm_stop(rb_arm_t *a)
{ if (a && a->enabled) { a->enabled=false; a->driver->stop(a->driver->context); } }
bool rb_arm_update(rb_arm_t *a, rb_engine_t *e, uint64_t now, bool estop_released)
{
    if (!a || !e) return false;
    if (!estop_released) rb_engine_stop(e,"E-STOP");
    if (e->state==RB_STOPPED) { rb_arm_stop(a); return false; }
    if (!a->enabled) return true; /* Explicit preview mode: no physical output. */
    if (now<a->sent_ms || now-a->sent_ms>100) { rb_engine_stop(e,"ARM UPDATE TIMEOUT"); rb_arm_stop(a); return false; }
    float actual[RB_JOINTS], target[RB_JOINTS]; uint64_t stamp=0;
    if (!a->driver->feedback(a->driver->context,actual,&stamp) || stamp>now || now-stamp>100) {
        rb_engine_stop(e,"ARM FEEDBACK STALE"); rb_arm_stop(a); return false;
    }
    for (unsigned j=0;j<RB_JOINTS;++j) {
        target[j]=a->calibration.neutral[j]+e->pose[j];
        if (!isfinite(actual[j]) || !isfinite(target[j]) || actual[j]<a->calibration.lower[j] ||
            actual[j]>a->calibration.upper[j] || target[j]<a->calibration.lower[j] ||
            target[j]>a->calibration.upper[j] || fabsf(actual[j]-a->last_target[j])>3 ||
            fabsf(target[j]-a->last_target[j])>e->config.max_speed_deg_s*(now-a->sent_ms)*.001f+.001f) {
            rb_engine_stop(e,"ARM LIMIT / FOLLOWING ERROR"); rb_arm_stop(a); return false;
        }
    }
    if (!a->driver->send(a->driver->context,target)) {
        rb_engine_stop(e,"ARM SEND FAILED"); rb_arm_stop(a); return false;
    }
    memcpy(a->last_target,target,sizeof(target)); a->sent_ms=now; return true;
}
