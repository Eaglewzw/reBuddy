"""Monotone C2 time reparameterization of an already checked joint path.

Position follows the original spline exactly; only its clock changes. No CAN I/O.
"""
import numpy as np
from scipy.interpolate import CubicSpline, BPoly, PPoly


def author_tilt_clock(source):
    """Conservative offline clock for the current TILT budget and path.

    Widen high-demand regions before smoothing so acceleration peaks keep time.
    The resulting clock still has to pass TimeWarpedCurve and trial validation.
    """
    from scipy.ndimage import maximum_filter1d, gaussian_filter1d
    x=np.linspace(0,source.duration,220)
    _,v=source.at(x)
    a=source.spline(x*source.scale,nu=2)*source.scale**2
    weight=np.maximum.reduce([np.full(len(x),.35),
        np.max(abs(v),axis=1)/min(1.1,source.velocity_limit*.92),
        np.sqrt(np.max(abs(a),axis=1)/min(4.5,source.acceleration_limit*.82))])
    weight=gaussian_filter1d(maximum_filter1d(weight,size=17,mode='nearest'),sigma=4,mode='nearest')*1.015
    return dict(schema='rebuddy.time_warp.v1',source_s=x.tolist(),dt_ds=weight.tolist())


class TimeWarpedCurve:
    def __init__(self, source, profile):
        if profile.get('schema')!='rebuddy.time_warp.v1':
            raise ValueError('invalid time-warp profile')
        x=np.asarray(profile['source_s'],float);w=np.asarray(profile['dt_ds'],float)
        if (x.ndim!=1 or not 4<=len(x)<=1024 or w.shape!=x.shape
                or not np.all(np.isfinite(x)) or not np.all(np.isfinite(w))
                or abs(x[0])>1e-10 or abs(x[-1]-source.duration)>1e-8
                or np.any(np.diff(x)<=0)):
            raise ValueError('invalid time-warp domain')
        self.source=source;self.weight=CubicSpline(x,w,bc_type='natural')
        points=np.r_[x,self.weight.derivative().roots(extrapolate=False)]
        points=points[np.isfinite(points)&(points>=0)&(points<=source.duration)]
        if np.min(self.weight(points))<.2 or np.max(self.weight(points))>3.:
            raise ValueError('time-warp clock must stay positive and bounded')
        self.clock=self.weight.antiderivative()
        self.origin=float(self.clock(0.))
        self.duration=float(self.clock(source.duration)-self.origin)
        self.start=source.start.copy();self.end=source.end.copy()
        self.velocity_limit=source.velocity_limit;self.acceleration_limit=source.acceleration_limit
        self.lower=source.lower;self.upper=source.upper
        self.lookup_s=np.linspace(0,source.duration,4097)
        self.lookup_t=self.time_at(self.lookup_s)
        self.breakpoints=np.unique(np.r_[self.time_at(x),
            self.time_at(np.unique(source.spline.t)/source.scale)])
        stamps=np.unique(np.r_[np.linspace(0,self.duration,int(np.ceil(self.duration/.002))+1),self.breakpoints])
        _,v,a=self.sample(stamps)
        self.peak_velocity=float(np.max(abs(v)));self.peak_acceleration=float(np.max(abs(a)))
        if self.peak_velocity>self.velocity_limit+1e-8 or self.peak_acceleration>self.acceleration_limit+1e-8:
            raise ValueError('time-warp exceeds dynamic budget')

    def time_at(self, source_s):
        return self.clock(np.clip(source_s,0.,self.source.duration))-self.origin

    def source_at(self, stamp):
        t=np.clip(stamp,0.,self.duration)
        s=np.interp(t,self.lookup_t,self.lookup_s)
        for _ in range(3):
            s=np.clip(s-(self.time_at(s)-t)/self.weight(s),0.,self.source.duration)
        return s

    def sample(self, stamp):
        s=self.source_at(stamp);w=self.weight(s);dw=self.weight(s,1)
        q,v=self.source.at(s)
        a=self.source.spline(s*self.source.scale,nu=2)*self.source.scale**2
        # dq/dt = dq/ds / w; d²q/dt² includes the changing clock speed.
        return (q,v/np.expand_dims(w,-1),a/np.expand_dims(w**2,-1)
                -v*np.expand_dims(dw/w**3,-1))

    def at(self, stamp):
        q,v,_=self.sample(stamp)
        return q,v

    def acceleration_at(self, stamp):
        return self.sample(stamp)[2]

    def polynomial_pieces(self):
        """C5 quintic Hermite approximation with shared q/v/a at every boundary.

        The C5 format stores degree-five polynomials, so a composite spline clock
        needs adaptive intervals. Check interior position, velocity and acceleration
        errors; dense independent C/PC comparison is also part of host regression.
        """
        out=[]
        def split(begin,end):
            dt=end-begin;q0,v0,a0=self.sample(begin);q1,v1,a1=self.sample(end)
            poly=PPoly.from_bernstein_basis(BPoly.from_derivatives([0.,1.],
                [[q0,v0*dt,a0*dt*dt],[q1,v1*dt,a1*dt*dt]]))
            u=np.linspace(0,1,17);q,v,a=self.sample(begin+u*dt)
            errors=(np.max(abs(poly(u)-q)),np.max(abs(poly(u,nu=1)/dt-v)),
                    np.max(abs(poly(u,nu=2)/dt**2-a)))
            if dt<=.12 and all(e<=limit for e,limit in zip(errors,(2e-7,2e-5,.002))):
                out.append((float(begin),float(end),poly.c[::-1,0,:].T.copy()))
            else:
                if dt<.0001 or len(out)>4096:raise ValueError('cannot export time-warp accurately')
                middle=(begin+end)/2;split(begin,middle);split(middle,end)
        # Preserve original path knot boundaries; clock knots are C2 and can
        # share intervals if the independent interpolation-error checks pass.
        points=np.unique(self.time_at(np.unique(self.source.spline.t)/self.source.scale))
        for begin,end in zip(points[:-1],points[1:]):
            if end-begin>1e-10:split(begin,end)
        return out
