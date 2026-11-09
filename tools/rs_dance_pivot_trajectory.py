"""Checked dense motion chunks and exact holds for fixed-TCP playback."""
from __future__ import annotations
import math
import numpy as np
from scipy.interpolate import PPoly
import rs_dance_design as base
from rs_dance_pivot_primitives import (
    _config
)

class _DenseCurve:
    def __init__(self,spec):
        self.t_s=np.asarray(spec['t_s'],float);self.q_rad=np.asarray(spec['q_rad'],float)
        if not 4<=len(self.t_s)<=5001:raise ValueError('invalid dense pivot sample count')
        self.curve=base.Curve(self.t_s,self.q_rad,velocity_limit=1.,acceleration_limit=2.)
        if self.curve.scale<1-1e-8:raise ValueError('stored pivot timing exceeds dynamic budget')
        self.duration=float(self.t_s[-1]);self.start=self.q_rad[0].copy();self.end=self.q_rad[-1].copy()
        self.lower=np.asarray(self.curve.lower);self.upper=np.asarray(self.curve.upper)
        self.peak_velocity=np.zeros(7);self.peak_acceleration=np.zeros(7)
        for joint in range(7):
            p=PPoly.from_spline((self.curve.spline.t,self.curve.spline.c[:,joint],5))
            for order,out in ((1,self.peak_velocity),(2,self.peak_acceleration)):
                points=np.r_[self.t_s,p.derivative(order+1).roots(extrapolate=False)]
                points=points[np.isfinite(points)&(points>=0)&(points<=self.duration)]
                out[joint]=float(np.max(abs(p(points,nu=order))))
    def at(self,t):
        t=float(np.clip(t,0,self.duration));return self.curve.spline(t),self.curve.spline(t,nu=1)
    def acceleration_at(self,t):return self.curve.spline(float(np.clip(t,0,self.duration)),nu=2)
    def jerk_at(self,t):return self.curve.spline(float(np.clip(t,0,self.duration)),nu=3)


class PivotSegment:
    """Dense C4 motion chunks joined C2 to an exact two-beat hold."""
    def __init__(self,spec):
        self.config=_config(spec['config']);self.amplitude_scale=float(spec['amplitude_scale'])
        if self.amplitude_scale not in self.config['amplitude_scales']:raise ValueError('unlisted pivot amplitude scale')
        self.velocity_limit=1.;self.acceleration_limit=2.;self.gripper_velocity_limit=1.;self.gripper_acceleration_limit=2.
        self.gripper_motion=False;self.stages=[]
        chunks=spec['chunks']
        if len(chunks)!=3 or [c['kind'] for c in chunks]!=['dense_curve','hold','dense_curve']:raise ValueError('invalid pivot motion/hold layout')
        for chunk in chunks:
            if chunk['kind']=='hold':
                curve=base.PoseSegment(dict(start_rad=chunk['q_rad'],end_rad=chunk['q_rad'],duration_s=chunk['duration_s']))
            else:curve=_DenseCurve(chunk)
            if self.stages:
                previous=self.stages[-1][1]
                if max(np.max(abs(previous.end-curve.start)),np.max(abs(previous.at(previous.duration)[1]-curve.at(0)[1])),
                       np.max(abs(previous.acceleration_at(previous.duration)-curve.acceleration_at(0))))>1e-7:raise ValueError('pivot chunk seam is not C2')
            self.stages.append((chunk['kind'],curve))
        self.ends=np.cumsum([c.duration for _,c in self.stages]);self.duration=float(self.ends[-1])
        if abs(self.duration-spec['duration_s'])>1e-8:raise ValueError('pivot duration mismatch')
        expected=np.cumsum(self.config['beat_sections'])[[2,3,4]]/self.config['beats']
        if not np.allclose(self.ends/self.duration,expected,rtol=0,atol=1e-9):raise ValueError('pivot hold must match its configured beat section')
        self.start=self.stages[0][1].start.copy();self.end=self.stages[-1][1].end.copy()
        self.lower=np.min([c.lower for _,c in self.stages],axis=0);self.upper=np.max([c.upper for _,c in self.stages],axis=0)
        self.peak_velocity=np.max([c.peak_velocity for _,c in self.stages],axis=0)
        self.peak_acceleration=np.max([c.peak_acceleration for _,c in self.stages],axis=0)
        self.hold_interval_s=(float(self.ends[0]),float(self.ends[1]))
        if np.max(abs(np.r_[self.lower[6],self.upper[6]]))>1e-10:raise ValueError('pivot must keep gripper closed')
        if np.max(self.peak_velocity)>1+1e-8 or np.max(self.peak_acceleration)>2+1e-8:raise ValueError('pivot exceeds arm dynamics')
        for t in (0,self.duration):
            if max(np.max(abs(self.at(t)[1])),np.max(abs(self.acceleration_at(t))))>1e-7:raise ValueError('pivot endpoint is not C2 stationary')
    def _chunk(self,t):
        t=float(np.clip(t,0,self.duration));i=min(2,int(np.searchsorted(self.ends,t,side='right')))
        return self.stages[i][1],t-(self.ends[i-1] if i else 0.)
    def at(self,t):c,t=self._chunk(t);return c.at(t)
    def acceleration_at(self,t):c,t=self._chunk(t);return c.acceleration_at(t)
    def jerk_at(self,t):
        c,t=self._chunk(t);return c.jerk_at(t) if hasattr(c,'jerk_at') else np.zeros(7)


