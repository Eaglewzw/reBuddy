"""Playback-only time scaling; nominal plans and their checks remain unchanged."""
import math
import numpy as np


def checked_trial_speed(value):
    speed=float(value)
    if not math.isfinite(speed) or not 0.1 <= speed <= 1.:
        raise ValueError('定点动作试播速度需在 0.1～1.0 之间')
    return speed


class ScaledSegment:
    def __init__(self, source, speed):
        self.source=source;self.speed=checked_trial_speed(speed)
        self.duration=source.duration/self.speed
        self.peak_velocity=source.peak_velocity*self.speed
        self.peak_acceleration=source.peak_acceleration*self.speed**2
        if hasattr(source,'hold_interval_s'):
            self.hold_interval_s=tuple(t/self.speed for t in source.hold_interval_s)

    def __getattr__(self, name):
        return getattr(self.source,name)

    def at(self, stamp):
        q,v=self.source.at(float(np.clip(stamp,0,self.duration))*self.speed)
        return q,v*self.speed

    def acceleration_at(self, stamp):
        return self.source.acceleration_at(float(np.clip(stamp,0,self.duration))*self.speed)*self.speed**2

    def jerk_at(self, stamp):
        return self.source.jerk_at(float(np.clip(stamp,0,self.duration))*self.speed)*self.speed**3


class ScaledPlan(ScaledSegment):
    def __init__(self, source, speed):
        super().__init__(source,speed)
        self.stages=[(label,ScaledSegment(curve,self.speed)) for label,curve in source.stages]
        self.ends=np.cumsum([curve.duration for _,curve in self.stages])

    def jerk_at(self, stamp):
        stamp=float(np.clip(stamp,0,self.duration))
        index=min(len(self.stages)-1,int(np.searchsorted(self.ends,stamp,side='right')))
        return self.stages[index][1].jerk_at(stamp-(self.ends[index-1] if index else 0.))


def pivot_code_for(plans):
    # Identify the pivot by its trajectory metadata rather than a menu number.
    return next((code for code,plan in plans.items() if hasattr(plan,'default_trial_speed')),
                None)


def trial_speed_for(plans, requested=None):
    return checked_trial_speed(requested if requested is not None else
                               getattr(plans.get(pivot_code_for(plans)),'default_trial_speed',.3))


def apply_trial_speed(plans, speed=None):
    speed=trial_speed_for(plans,speed)
    result=dict(plans)
    code=pivot_code_for(plans)
    if code is not None:result[code]=ScaledPlan(result[code],speed)
    return result
