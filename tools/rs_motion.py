"""Shared offline seven-axis trajectory interpolation and time scaling.

No CAN connection or motion commands; used by CSV and daily preparation.
"""
import math

import numpy as np
from scipy.interpolate import make_interp_spline, PPoly
from rs_record_playback import LOWER, UPPER

VMAX = .12
AMAX = .25


class Curve:
    """C2 entry/exit, analytic per-axis extrema; retime, never clamp joint values."""
    def __init__(self,t,q,minimum_duration=0,measurement_margin=False,initial_position_margin=0.,
                 velocity_limit=VMAX,acceleration_limit=AMAX):
        t=np.asarray(t,float);q=np.asarray(q,float)
        if q.shape!=(len(t),7) or t[0]!=0 or np.any(np.diff(t)<=0) or not np.all(np.isfinite(q)):
            raise ValueError('invalid curve')
        if not math.isfinite(initial_position_margin) or initial_position_margin<0:
            raise ValueError('invalid initial position allowance')
        if not all(math.isfinite(x) and x>0 for x in (velocity_limit,acceleration_limit)):
            raise ValueError('invalid curve dynamic limits')
        self.velocity_limit=velocity_limit;self.acceleration_limit=acceleration_limit
        if initial_position_margin:
            if (len(t)!=2 or np.any(q[0]<LOWER-initial_position_margin)
                    or np.any(q[0]>UPPER+initial_position_margin)
                    or np.any(q[-1]<LOWER) or np.any(q[-1]>UPPER)):
                raise ValueError('initial feedback or destination outside joint bounds')
        zero=np.zeros(7)
        self.spline=make_interp_spline(t,q,k=5,bc_type=([(1,zero),(2,zero)],[(1,zero),(2,zero)]))
        vmax=amax=0.;self.lower=[];self.upper=[]
        margin=math.radians(.1) if measurement_margin else 1e-8
        for j in range(7):
            p=PPoly.from_spline((self.spline.t,self.spline.c[:,j],self.spline.k))
            for derivative in range(3):
                candidates=np.r_[t,p.derivative(derivative+1).roots(extrapolate=False)]
                candidates=candidates[np.isfinite(candidates)&(candidates>=t[0])&(candidates<=t[-1])]
                values=p(candidates,nu=derivative)
                if not np.all(np.isfinite(values)):raise ValueError('invalid curve extrema')
                if derivative==0:
                    lo=float(min(values));hi=float(max(values));self.lower.append(lo);self.upper.append(hi)
                    lower=min(LOWER[j],q[0,j]) if initial_position_margin else LOWER[j]-margin
                    upper=max(UPPER[j],q[0,j]) if initial_position_margin else UPPER[j]+margin
                    if lo<lower-1e-8 or hi>upper+1e-8:
                        raise ValueError(f'curve exceeds motor {j+1} bounds: {math.degrees(lo):.3f}..{math.degrees(hi):.3f}deg')
                elif derivative==1:vmax=max(vmax,float(max(abs(values))))
                else:amax=max(amax,float(max(abs(values))))
        self.scale=min(1.,velocity_limit/max(vmax,1e-12),math.sqrt(acceleration_limit/max(amax,1e-12)),t[-1]/max(minimum_duration,1e-12))
        self.duration=float(t[-1]/self.scale)
        self.start=q[0].copy();self.end=q[-1].copy()
        self.peak_velocity=vmax*self.scale;self.peak_acceleration=amax*self.scale**2

    def at(self,t):
        x=np.clip(t,0,self.duration)*self.scale
        return self.spline(x),self.spline(x,nu=1)*self.scale

    def reverse_at(self,t):
        q,v=self.at(self.duration-t)
        return q,-v

