"""Two nuzzles: descend, sway J5 at the low pose, then rise. No I/O."""
import math

import numpy as np
from scipy.interpolate import BPoly, PPoly
from scipy.signal import find_peaks
from scipy.optimize import brentq

from rs_motion import Curve
from rs_record_playback import LOWER, UPPER

SWAY_AMPLITUDE_DEG = 20.
SWAY_DURATION_S = 2.1
SWAY_SCURVE = dict(ramp_fraction=.04,outward_cruise_fraction=.04,center_cruise_fraction=.1)
DESCENT_DURATION_S = 1.1
SECOND_DESCENT_DURATION_S = .9
RISE_DURATIONS_S = (.85, 1.35)
TURNAROUND_FACTOR = 6.


def scurve_sway(duration, amplitude, config):
    """Integrate continuous, piecewise-linear acceleration into a C2 sway.

    Mirror the first half with opposite acceleration. Solve the initial
    acceleration plateau so the midpoint crosses center; then normalize the
    position extrema to +/- amplitude. Endpoint velocity/acceleration are zero.
    """
    ramp=float(config['ramp_fraction'])
    outward=float(config['outward_cruise_fraction'])
    center=float(config['center_cruise_fraction'])
    if (not all(math.isfinite(x) and x>0 for x in (ramp,outward,center))
            or 4*ramp+outward+center>=.5-1e-6):
        raise ValueError('invalid nuzzle S-curve timing')

    def first_half(plateau):
        times=np.array([0.,ramp,ramp+plateau,2*ramp+plateau,
                        2*ramp+plateau+outward,3*ramp+plateau+outward,
                        .5-center-ramp,.5-center,.5])
        acceleration=np.array([0.,1.,1.,0.,0.,-1.,-1.,0.,0.])
        return PPoly(np.array([np.diff(acceleration)/np.diff(times),acceleration[:-1]]),times).antiderivative(2)

    plateau=brentq(lambda h:first_half(h)(.5),1e-8,.5-4*ramp-outward-center-1e-8)
    half=first_half(plateau);times=half.x;acceleration=half(times,nu=2)
    times=np.r_[times,1-times[-2::-1]]
    acceleration=np.r_[acceleration,-acceleration[-2::-1]]
    curve=PPoly(np.array([np.diff(acceleration)/np.diff(times),acceleration[:-1]]),times).antiderivative(2)
    extrema=curve.derivative().roots(extrapolate=False)
    peak=max(curve(extrema[np.isfinite(extrema)]))
    return PPoly(curve.c*(amplitude/peak)/(duration**np.arange(3,-1,-1))[:,None],curve.x*duration)


class Nuzzle:
    """C2 quintics; each J5 sway starts after descent, with other axes held."""
    def __init__(self, poses_rad, durations_s, amplitude_deg=SWAY_AMPLITUDE_DEG,
                 velocity_limit=1.2, acceleration_limit=5.5, turnaround_factor=0.,
                 sway_peak_fraction=.25, sway_acceleration_factor=4*math.pi**2,
                 sway_scurve=None):
        poses=np.asarray(poses_rad,float);durations=np.asarray(durations_s,float)
        if (poses.shape!=(5,7) or durations.shape!=(6,) or not np.all(np.isfinite(poses))
                or not np.all(np.isfinite(durations)) or np.any(durations<=0)
                or not math.isfinite(amplitude_deg) or not 0<amplitude_deg<=SWAY_AMPLITUDE_DEG):
            raise ValueError('invalid two-descent nuzzle')
        if not np.all(poses[1:4,4]==poses[0,4]) or not np.all(poses[:,6]==poses[0,6]):
            raise ValueError('nuzzle requires centered J5 and fixed gripper anchors')
        if (velocity_limit,acceleration_limit)!=(1.2,5.5):
            raise ValueError('unrecognized nuzzle speed profile')
        if not math.isfinite(turnaround_factor) or not 0<=turnaround_factor<=6.:
            raise ValueError('invalid nuzzle turnaround factor')
        if (not math.isfinite(sway_peak_fraction) or not 0<sway_peak_fraction<.5
                or not math.isfinite(sway_acceleration_factor) or sway_acceleration_factor<0):
            raise ValueError('invalid nuzzle sway profile')
        self.velocity_limit=velocity_limit;self.acceleration_limit=acceleration_limit
        self.phase_times=np.r_[0.,np.cumsum(durations)]
        self.duration=float(self.phase_times[-1]);self.scale=1.
        self.start=poses[0].copy();self.end=poses[-1].copy()
        zero=np.zeros(7)
        # Duplicate each low pose for the separate sway phase. Holding the
        # other six joints here makes the ordering explicit on PC and C5.
        timeline_poses=poses[[0,1,1,2,3,3,4]]
        derivatives=[[q,zero,zero] for q in timeline_poses]
        # At the upper reversal, a shared nonzero acceleration removes the
        # flat stop caused by zero acceleration on both sides. Reversing axes
        # still have zero instantaneous velocity; J5 remains centered.
        incoming=poses[2]-poses[1];outgoing=poses[3]-poses[2]
        reversing=incoming*outgoing<0
        turn_acceleration=np.where(reversing,
            -np.sign(incoming)*np.minimum(abs(incoming),abs(outgoing))*
            turnaround_factor/((durations[2]+durations[3])/2)**2,0.)
        derivatives[3]=[poses[2],zero,turn_acceleration]
        body=PPoly.from_bernstein_basis(BPoly.from_derivatives(
            self.phase_times,derivatives))
        self.descent_windows=[(self.phase_times[i],self.phase_times[i+1]) for i in (0,3)]
        self.sway_windows=[(self.phase_times[i],self.phase_times[i+1]) for i in (1,4)]
        sways=[];breaks=list(self.phase_times)
        for begin,end in self.sway_windows:
            if sway_scurve is not None:
                local=scurve_sway(end-begin,math.radians(amplitude_deg),sway_scurve)
                sway=PPoly(local.c,begin+local.x);times=sway.x
            else:
                times=begin+(end-begin)*np.array([0.,sway_peak_fraction,1-sway_peak_fraction,1.])
                levels=np.radians([0.,amplitude_deg,-amplitude_deg,0.])
                acceleration=-levels*sway_acceleration_factor/(end-begin)**2
                sway=PPoly.from_bernstein_basis(BPoly.from_derivatives(
                    times,[[q,0.,a] for q,a in zip(levels,acceleration)]))
            sways.append((begin,end,sway));breaks.extend(times)
        self.breakpoints=np.unique(breaks)
        coefficients=[]
        for begin in self.breakpoints[:-1]:
            row=[]
            for k in range(6):
                value=body(begin,nu=k).copy()
                for a,b,sway in sways:
                    if a<=begin<b:value[4]+=sway(begin,nu=k)
                row.append(value/math.factorial(k))
            coefficients.append(row[::-1])
        self.spline=PPoly(np.asarray(coefficients).transpose(1,0,2),self.breakpoints)
        self.lower=[];self.upper=[];peaks=[0.,0.]
        for j in range(7):
            p=PPoly(self.spline.c[:,:,j],self.breakpoints)
            for k in range(3):
                roots=p.derivative(k+1).roots(extrapolate=False)
                points=np.r_[self.breakpoints,roots[np.isfinite(roots)]]
                values=p(points,nu=k)
                if k==0:
                    lo=float(min(values));hi=float(max(values))
                    if lo<LOWER[j]-1e-8 or hi>UPPER[j]+1e-8:
                        raise ValueError(f'nuzzle exceeds motor {j+1} position bounds')
                    self.lower.append(lo);self.upper.append(hi)
                else:peaks[k-1]=max(peaks[k-1],float(max(abs(values))))
        self.peak_velocity,self.peak_acceleration=peaks
        if self.peak_velocity>velocity_limit or self.peak_acceleration>acceleration_limit:
            raise ValueError('nuzzle exceeds dynamic budget')

    def at(self, stamp):
        stamp=np.clip(stamp,0.,self.duration)
        return self.spline(stamp),self.spline(stamp,nu=1)


def author_nuzzle(fit, gravity):
    """Reuse the first two recorded low poses and intervening raised pose."""
    q=np.array(fit['q_rad'],float)
    if 'j5_sway' in fit:q[:,4]=fit['j5_sway']['source_rad']
    source=Curve(fit['t_s'],q,velocity_limit=1.2,acceleration_limit=5.5)
    stamps=np.linspace(0.,source.duration,2001);samples=source.at(stamps)[0]
    frame=gravity.model.getFrameId('gripper_end');data=gravity.model.createData()
    if frame>=gravity.model.nframes:raise ValueError('gripper_end frame missing')
    heights=[]
    for pose in samples:
        gravity.pin.framesForwardKinematics(gravity.model,data,pose[:6])
        heights.append(data.oMf[frame].translation[2])
    heights=np.asarray(heights)
    bottoms,_=find_peaks(-heights,prominence=.035)
    if len(bottoms)<2:raise ValueError('recording needs two distinct downward nuzzles')
    first,second=bottoms[:2]
    middle=first+int(np.argmax(heights[first:second]))
    poses=samples[[0,first,middle,second,-1]].copy()
    poses[1:4,4]=poses[0,4];poses[:,6]=poses[0,6]
    # A shared acceleration at the upper reversal joins the middle pair.
    # Each low-pose sway keeps its separate clock and torque headroom.
    return dict(kind='nuzzle',label='recorded_gesture',poses_rad=poses.tolist(),
                durations_s=[DESCENT_DURATION_S,SWAY_DURATION_S,RISE_DURATIONS_S[0],
                             SECOND_DESCENT_DURATION_S,SWAY_DURATION_S,RISE_DURATIONS_S[1]],
                amplitude_deg=SWAY_AMPLITUDE_DEG,turnaround_factor=TURNAROUND_FACTOR,
                sway_scurve=dict(SWAY_SCURVE),
                limits=dict(velocity_limit=1.2,acceleration_limit=5.5),
                design=dict(descents=2,j5_cycles_per_descent=1,sway_timing='after_descent_before_rise',
                            phase_order=['descend','sway','rise','descend','sway','rise'],
                            source_pose_times_s=stamps[[0,first,middle,second,-1]].tolist(),
                            path_policy='recorded key poses; reauthored C2 paths'))
