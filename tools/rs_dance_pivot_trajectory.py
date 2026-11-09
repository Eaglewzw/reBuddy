"""Dense spline chunks for the current fixed-point action."""
import numpy as np
from scipy.interpolate import PPoly
import rs_dance_motion as base

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

