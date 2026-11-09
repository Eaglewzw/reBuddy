"""Runtime plans for the current arm, gripper and fixed-point motions."""
import math
import numpy as np
import rs_dance_motion as base

def _c2(plan):
    for (_,left),(_,right) in zip(plan.stages,plan.stages[1:]):
        for a,b in ((left.at(left.duration)[0],right.at(0)[0]),
                    (left.at(left.duration)[1],right.at(0)[1]),
                    (left.acceleration_at(left.duration),right.acceleration_at(0))):
            if np.max(abs(a-b))>1e-7:raise ValueError('showcase stage seam is not C2')
    for stamp in (0,plan.duration):
        if max(np.max(abs(plan.at(stamp)[1])),np.max(abs(plan.acceleration_at(stamp))))>1e-7:
            raise ValueError('showcase endpoints must stop smoothly')


def fast_wide_gripper_cycle(idle):
    return [dict(kind='gripper_wide_pulse',label='wide_two_second_pulse',duration_s=2.,
                 base_rad=np.asarray(idle).tolist(),amplitude_rad=math.radians(90.))]


class CompositeSegment:
    """An arm phrase with two validated gripper cycles on the same clock."""
    def __init__(self,spec):
        self.body=base.DancePlan(spec['body_stages'])
        self.pulse=base.DancePlan(spec['pulse_stages'],gripper_motion=True)
        self.duration=float(spec['duration_s'])
        self.pulse_starts_s=np.asarray(spec['pulse_starts_s'],float)
        if (abs(self.duration-self.body.duration)>1e-9 or len(self.pulse_starts_s)!=2
            or not np.all(np.isfinite(self.pulse_starts_s))
            or self.pulse_starts_s[0]<0 or self.pulse_starts_s[-1]+self.pulse.duration>self.duration
            or np.any(np.diff(self.pulse_starts_s)<self.pulse.duration)):
            raise ValueError('invalid gripper pulse schedule')
        expected=fast_wide_gripper_cycle(self.body.start)
        if spec['pulse_stages']!=expected:
            raise ValueError('showcase gripper differs from its validated pulse profile')
        _c2(self.body);_c2(self.pulse)
        self.start,self.end=self.body.start.copy(),self.body.end.copy()
        self.lower=np.min([c.lower for _,c in self.body.stages],axis=0)
        self.upper=np.max([c.upper for _,c in self.body.stages],axis=0)
        self.lower[6]=min(c.lower[6] for _,c in self.pulse.stages)
        self.upper[6]=max(c.upper[6] for _,c in self.pulse.stages)
        self.peak_velocity=self.body.peak_velocity.copy();self.peak_velocity[6]=self.pulse.peak_velocity[6]
        self.peak_acceleration=self.body.peak_acceleration.copy();self.peak_acceleration[6]=self.pulse.peak_acceleration[6]
        self.velocity_limit=1.;self.acceleration_limit=2.
        self.gripper_velocity_limit=self.pulse.gripper_velocity_limit
        self.gripper_acceleration_limit=self.pulse.gripper_acceleration_limit
        self.gripper_following_limit=base.GRIPPER_FOLLOWING_LIMIT
        self.gripper_motion=True

    def _pulse_at(self,t):
        for start in self.pulse_starts_s:
            if start<=t<=start+self.pulse.duration:return float(t-start)
        return None

    def at(self,t):
        q,v=self.body.at(t);q=q.copy();v=v.copy();stamp=self._pulse_at(t)
        if stamp is not None:q[6],v[6]=self.pulse.at(stamp)[0][6],self.pulse.at(stamp)[1][6]
        return q,v

    def acceleration_at(self,t):
        a=self.body.acceleration_at(t).copy();stamp=self._pulse_at(t)
        if stamp is not None:a[6]=self.pulse.acceleration_at(stamp)[6]
        return a


class DancePlan(base.DancePlan):
    def __init__(self,stages,*,bookend=False,gripper_motion=False):
        kind=stages[0].get('kind') if len(stages)==1 else None
        if kind in ('body_gripper_fast_wide_overlay','virtual_pivot_joined'):
            if bookend:raise ValueError('bookend must use Daily positioning')
            if kind=='body_gripper_fast_wide_overlay':
                if not gripper_motion:raise ValueError('overlay requires gripper motion')
                curve=CompositeSegment(stages[0])
            else:
                if gripper_motion:raise ValueError('pivot must keep gripper closed')
                from rs_dance_pivot_join import JoinedSwaySegment
                curve=JoinedSwaySegment(stages[0]);self.default_trial_speed=curve.default_trial_speed
            self.stages=[(stages[0]['label'],curve)]
            self.ends=np.array([curve.duration]);self.duration=curve.duration
            for attr in ('start','end','peak_velocity','peak_acceleration','velocity_limit','acceleration_limit',
                         'gripper_velocity_limit','gripper_acceleration_limit','gripper_following_limit','gripper_motion'):
                if hasattr(curve,attr):setattr(self,attr,getattr(curve,attr))
        else:super().__init__(stages,bookend=bookend,gripper_motion=gripper_motion)
        _c2(self)
