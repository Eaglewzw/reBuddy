"""Compact twelve-action dance library, generated and checked entirely offline.

Joint space choreography is authored around the calibrated Daily idle pose.
Quintic transitions and enveloped phase waves have zero endpoint velocity and
acceleration. This is not a collision planner or hardware acceptance record.
"""
from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np
from scipy.interpolate import PPoly

from rs_gravity_record import ROOT, CAPS, digest, preflight, vector
from rs_motion import Curve
from rs_record_playback import LOWER, UPPER

DAILY_DIR = ROOT / 'record/daily_ready'
GIF_MANIFEST = ROOT / 'assets/gifs/dance/manifest.json'
DEFAULT_OUTPUT = ROOT / 'record/dance_designed/v6'
RECIPE_VERSIONS = ('legacy_v5', 'fast_v6')
BPM = 60.
ARM_VMAX, ARM_AMAX = 1., 2.
GRIPPER_VMAX, GRIPPER_AMAX = 1.6, 5.1
GRIPPER_CLOSED, GRIPPER_OPEN = 0., math.radians(250.)
GRIPPER_BEAT_OPEN = math.radians(45.)
# Same independent gripper tracking window as the existing Daily Clap: the
# reference rs_mit_split.yaml allows 12 motor degrees, used here at 90%.
GRIPPER_FOLLOWING_LIMIT = math.radians(10.8)
CATALOG = (
    ('start', '开场', 43), ('nod', '点头打拍', 33),
    ('lift', '抬臂舒展', 34), ('dip', '深蹲回升', 35),
    ('reach', '前探伸展', 36), ('base_turn', '底盘回转', 37),
    ('sway', '大幅摇摆', 38), ('elbow', '折肘展开', 39),
    ('wrist', '抬头俯看', 40), ('grip', '夹爪开合', 41),
    ('wave', '大幅波浪', 42), ('end', '收尾', 44),
)


def _number(value):
    number = int(value)
    if not 0 <= number < len(CATALOG) or str(value) not in (str(number), f'{number:02d}'):
        raise ValueError('动作编号为 00～11')
    return number


def _finite_duration(value):
    value = float(value)
    if not math.isfinite(value) or not 0 < value <= 30:
        raise ValueError('invalid stage duration')
    return value


class PoseSegment:
    """Seven synchronized monotonic quintics; constants represent a hold."""
    def __init__(self, spec):
        self.start, self.end = vector(spec['start_rad']), vector(spec['end_rad'])
        self.duration = _finite_duration(spec['duration_s'])
        self.delta = self.end-self.start
        self.lower, self.upper = np.minimum(self.start,self.end), np.maximum(self.start,self.end)
        self.peak_velocity = 1.875*abs(self.delta)/self.duration
        self.peak_acceleration = (10/math.sqrt(3))*abs(self.delta)/self.duration**2

    def at(self, stamp):
        u = np.clip(float(stamp)/self.duration, 0., 1.)
        s = u**3*(10-15*u+6*u*u)
        v = 30*u*u*(1-u)**2/self.duration
        return self.start+self.delta*s, self.delta*v

    def acceleration_at(self, stamp):
        u = np.clip(float(stamp)/self.duration, 0., 1.)
        return self.delta*(60*u*(1-u)*(1-2*u)/self.duration**2)


class WaveSegment:
    """sin(pi*u)^3 envelope; joint phase offsets propagate an arm wave."""
    def __init__(self, spec):
        self.start = vector(spec['base_rad'])
        self.end = self.start.copy()
        self.amplitude = vector(spec['amplitude_rad'])
        self.phase = vector(spec['phase_rad'])
        self.duration = _finite_duration(spec['duration_s'])
        self.cycles = float(spec['cycles'])
        if not math.isfinite(self.cycles) or not 0 < self.cycles <= 4:
            raise ValueError('invalid wave cycles')
        self.omega = 2*math.pi*self.cycles/self.duration
        self.lower, self.upper = self.start-abs(self.amplitude), self.start+abs(self.amplitude)
        envelope_v = 2*math.pi/(math.sqrt(3)*self.duration)
        envelope_a = 3*math.pi**2/self.duration**2
        self.peak_velocity = abs(self.amplitude)*(envelope_v+self.omega)
        self.peak_acceleration = abs(self.amplitude)*(envelope_a+2*envelope_v*self.omega+self.omega**2)

    def _terms(self, stamp):
        u = np.clip(float(stamp)/self.duration, 0., 1.)
        s, c = math.sin(math.pi*u), math.cos(math.pi*u)
        envelope = s**3
        speed = 3*s*s*c*math.pi/self.duration
        accel = 3*(2*s*c*c-s**3)*(math.pi/self.duration)**2
        phase = self.omega*u*self.duration+self.phase
        return envelope, speed, accel, np.sin(phase), np.cos(phase)

    def at(self, stamp):
        w, dw, _, s, c = self._terms(stamp)
        return self.start+self.amplitude*w*s, self.amplitude*(dw*s+w*c*self.omega)

    def acceleration_at(self, stamp):
        w, dw, ddw, s, c = self._terms(stamp)
        return self.amplitude*(ddw*s+2*dw*c*self.omega-w*s*self.omega**2)


class SplineSegment:
    """One C2 phrase: nonzero interior velocity, bounded analytic extrema."""
    def __init__(self, spec):
        self.t_s = np.asarray(spec['t_s'], dtype=float)
        self.q_rad = np.asarray(spec['q_rad'], dtype=float)
        if not 4 <= len(self.t_s) <= 32 or self.q_rad.shape != (len(self.t_s), 7):
            raise ValueError('invalid continuous dance knots')
        self.curve = Curve(self.t_s, self.q_rad, velocity_limit=ARM_VMAX,
                           acceleration_limit=ARM_AMAX)
        if self.curve.scale < 1.-1e-9:
            raise ValueError('continuous dance timing does not meet stored limits')
        self.duration = _finite_duration(self.t_s[-1])
        if abs(float(spec['duration_s'])-self.duration)>1e-9:
            raise ValueError('continuous dance duration mismatch')
        self.start, self.end = self.q_rad[0].copy(), self.q_rad[-1].copy()
        self.lower, self.upper = np.asarray(self.curve.lower), np.asarray(self.curve.upper)
        self.peak_velocity, self.peak_acceleration = np.zeros(7), np.zeros(7)
        # Curve already checks positional extrema. Keep per-axis derivative
        # bounds for reporting and the shared segment validator.
        for j in range(7):
            p = PPoly.from_spline((self.curve.spline.t, self.curve.spline.c[:,j], self.curve.spline.k))
            for order, result in ((1,self.peak_velocity),(2,self.peak_acceleration)):
                candidates=np.r_[self.t_s,p.derivative(order+1).roots(extrapolate=False)]
                candidates=candidates[np.isfinite(candidates)&(candidates>=0)&(candidates<=self.duration)]
                result[j]=float(np.max(abs(p(candidates,nu=order))))

    def at(self, stamp):
        stamp=float(np.clip(stamp,0,self.duration))
        return self.curve.spline(stamp), self.curve.spline(stamp,nu=1)

    def acceleration_at(self, stamp):
        return self.curve.spline(float(np.clip(stamp,0,self.duration)),nu=2)


class GripperCruiseSegment:
    """J7-only cruise with quintic velocity ramps; q/v/a/jerk are continuous."""
    def __init__(self, spec):
        self.start, self.end=vector(spec['start_rad']),vector(spec['end_rad'])
        if not np.array_equal(self.start[:6],self.end[:6]):
            raise ValueError('gripper cruise cannot move arm joints')
        self.speed_rad_s=float(spec['speed_rad_s'])
        self.ramp_s=_finite_duration(spec['ramp_s'])
        if not math.isfinite(self.speed_rad_s) or not 0 < self.speed_rad_s <= GRIPPER_VMAX:
            raise ValueError('invalid gripper cruise speed')
        self.distance=abs(float(self.end[6]-self.start[6]))
        self.direction=math.copysign(1.,float(self.end[6]-self.start[6]))
        self.cruise_s=self.distance/self.speed_rad_s-self.ramp_s
        if self.cruise_s < 0:raise ValueError('gripper move too short for chosen velocity ramps')
        self.duration=_finite_duration(2*self.ramp_s+self.cruise_s)
        if abs(float(spec['duration_s'])-self.duration)>1e-9:
            raise ValueError('gripper cruise duration mismatch')
        self.lower,self.upper=np.minimum(self.start,self.end),np.maximum(self.start,self.end)
        self.peak_velocity=np.zeros(7);self.peak_velocity[6]=self.speed_rad_s
        self.peak_acceleration=np.zeros(7);self.peak_acceleration[6]=1.875*self.speed_rad_s/self.ramp_s

    def _sample(self, stamp):
        t=float(np.clip(stamp,0,self.duration));V=self.speed_rad_s;T=self.ramp_s
        if t<=0:return np.zeros(4)
        if t>=self.duration:return np.array([self.distance,0.,0.,0.])*self.direction
        if t < T:
            u=t/T;h=u**3*(10-15*u+6*u*u);H=2.5*u**4-3*u**5+u**6
            q=V*T*H;v=V*h;a=V/T*30*u*u*(1-u)**2;j=V/T**2*60*u*(1-u)*(1-2*u)
        elif t < T+self.cruise_s:
            q=.5*V*T+V*(t-T);v=V;a=j=0.
        else:
            u=(t-T-self.cruise_s)/T;h=u**3*(10-15*u+6*u*u);H=2.5*u**4-3*u**5+u**6
            q=self.distance-.5*V*T+V*T*(u-H);v=V*(1-h)
            a=-V/T*30*u*u*(1-u)**2;j=-V/T**2*60*u*(1-u)*(1-2*u)
        return np.array([q,v,a,j])*self.direction

    def at(self, stamp):
        displacement,velocity,_,_=self._sample(stamp)
        q=self.start.copy();q[6]+=displacement
        v=np.zeros(7);v[6]=velocity
        return q,v

    def acceleration_at(self, stamp):
        a=np.zeros(7);a[6]=self._sample(stamp)[2]
        return a

    def jerk_at(self, stamp):
        j=np.zeros(7);j[6]=self._sample(stamp)[3]
        return j


class GripperPulseSegment:
    """One degree-six pulse: no dwell and no internal reversal seam."""
    def __init__(self,spec):
        self.start=vector(spec['base_rad']);self.end=self.start.copy()
        self.duration=_finite_duration(spec['duration_s'])
        self.amplitude=float(spec['amplitude_rad'])
        if abs(self.start[6])>1e-12 or abs(self.duration-2.)>1e-12 or abs(self.amplitude-GRIPPER_BEAT_OPEN)>1e-12:
            raise ValueError('fast gripper pulse must be absolute 0..45 degrees in two seconds')
        self.polynomial=np.polynomial.Polynomial([0,0,0,64,-192,192,-64])*self.amplitude
        self.lower=self.start.copy();self.upper=self.start.copy();self.upper[6]=self.amplitude
        self.peak_velocity=np.zeros(7);self.peak_acceleration=np.zeros(7)
        for order,out in ((1,self.peak_velocity),(2,self.peak_acceleration)):
            roots=self.polynomial.deriv(order+1).roots()
            candidates=[0.,1.]+[float(r.real) for r in roots if abs(r.imag)<1e-10 and 0<=r.real<=1]
            out[6]=max(abs(self.polynomial.deriv(order)(candidates)))/self.duration**order

    def at(self,stamp):
        u=float(np.clip(stamp/self.duration,0,1));q=self.start.copy();v=np.zeros(7)
        if 0<u<1:
            q[6]=self.polynomial(u);v[6]=self.polynomial.deriv()(u)/self.duration
        return q,v

    def acceleration_at(self,stamp):
        u=float(np.clip(stamp/self.duration,0,1));a=np.zeros(7)
        if 0<u<1:a[6]=self.polynomial.deriv(2)(u)/self.duration**2
        return a

    def jerk_at(self,stamp):
        u=float(np.clip(stamp/self.duration,0,1));j=np.zeros(7)
        j[6]=self.polynomial.deriv(3)(u)/self.duration**3
        return j


class DancePlan:
    def __init__(self, specs, *, bookend=False, gripper_motion=False):
        if not isinstance(specs,list) or not 1 <= len(specs) <= 32:
            raise ValueError('invalid dance stage count')
        self.velocity_limit = .5 if bookend else ARM_VMAX
        self.acceleration_limit = 1. if bookend else ARM_AMAX
        # J7 has its own profile; arm limits and all other actions are unchanged.
        self.gripper_velocity_limit = GRIPPER_VMAX if gripper_motion else 1.
        self.gripper_acceleration_limit = GRIPPER_AMAX if gripper_motion else 2.
        self.gripper_motion = bool(gripper_motion)
        if self.gripper_motion:
            self.gripper_following_limit = GRIPPER_FOLLOWING_LIMIT
        self.stages=[]
        for stage in specs:
            kind=stage.get('kind')
            if kind=='quintic': curve=PoseSegment(stage)
            elif kind=='wave': curve=WaveSegment(stage)
            elif kind=='spline': curve=SplineSegment(stage)
            elif kind=='gripper_cruise': curve=GripperCruiseSegment(stage)
            elif kind=='gripper_pulse': curve=GripperPulseSegment(stage)
            else: raise ValueError('unknown dance stage kind')
            if np.any(curve.lower < LOWER) or np.any(curve.upper > UPPER):
                raise ValueError('dance target exceeds joint limits')
            if curve.lower[6] < GRIPPER_CLOSED-1e-10 or curve.upper[6] > GRIPPER_OPEN+1e-10:
                raise ValueError('J7 command must stay within absolute 0..250 motor degrees')
            if (np.any(curve.peak_velocity[:6] > self.velocity_limit+1e-9)
                    or np.any(curve.peak_acceleration[:6] > self.acceleration_limit+1e-9)
                    or curve.peak_velocity[6] > self.gripper_velocity_limit+1e-9
                    or curve.peak_acceleration[6] > self.gripper_acceleration_limit+1e-9):
                raise ValueError('dance segment exceeds dynamic budget')
            if not gripper_motion and abs(curve.upper[6]-curve.lower[6]) > 1e-10:
                raise ValueError('only 09 may move the gripper')
            if self.stages and np.max(abs(self.stages[-1][1].end-curve.start)) > 1e-10:
                raise ValueError('dance stages are discontinuous')
            # Session.execute_plan executes each curve separately, so carry the
            # already-validated budgets onto the executable segment as well.
            curve.velocity_limit = self.velocity_limit
            curve.acceleration_limit = self.acceleration_limit
            curve.gripper_velocity_limit = self.gripper_velocity_limit
            curve.gripper_acceleration_limit = self.gripper_acceleration_limit
            if self.gripper_motion:
                curve.gripper_following_limit = self.gripper_following_limit
            self.stages.append((str(stage['label']),curve))
        self.ends=np.cumsum([c.duration for _,c in self.stages])
        self.duration=float(self.ends[-1])
        self.start=self.stages[0][1].start.copy()
        self.end=self.stages[-1][1].end.copy()
        self.peak_velocity=np.max([c.peak_velocity for _,c in self.stages],axis=0)
        self.peak_acceleration=np.max([c.peak_acceleration for _,c in self.stages],axis=0)

    def _stage(self, stamp):
        stamp=float(np.clip(stamp,0,self.duration))
        i=min(len(self.stages)-1,int(np.searchsorted(self.ends,stamp,side='right')))
        return self.stages[i][1], stamp-(self.ends[i-1] if i else 0.)

    def at(self, stamp):
        curve, stamp=self._stage(stamp)
        return curve.at(stamp)

    def acceleration_at(self, stamp):
        curve, stamp=self._stage(stamp)
        return curve.acceleration_at(stamp)


def _anchors(identity):
    path=DAILY_DIR/'manifest.json'
    data=json.loads(path.read_text())
    if data.get('schema')!='rebuddy.daily.prepared.v1':
        raise ValueError('invalid Daily manifest')
    for key in ('calibration_sha256','urdf_sha256'):
        if data.get(key)!=identity[key]:raise ValueError('Daily calibration/model identity changed')
    idle,zero=vector(data['idle_rad']),vector(data['zero_rad'])
    sources={str(path.resolve()):digest(path)}
    bookends={}
    for action in ('WAKE','REST'):
        item=data['actions'][action]
        file=DAILY_DIR/item['plan_file']
        if file.parent.resolve()!=DAILY_DIR.resolve() or digest(file)!=item['plan_sha256']:
            raise ValueError('Daily bookend source changed')
        sources[str(file.resolve())]=digest(file)
        spec=json.loads(file.read_text())
        if len(spec['stages'])!=1:raise ValueError('Daily bookend must be one positioning curve')
        stage=spec['stages'][0]
        if stage['kind']!='curve' or stage.get('limits')!={'velocity_limit':.5,'acceleration_limit':1.}:
            raise ValueError('Daily positioning limits changed')
        curve=Curve(stage['t_s'],stage['q_rad'],**stage['limits'])
        factor=float(stage.get('speed',1.))
        if not 0 < factor <= 1:raise ValueError('invalid Daily speed')
        curve.duration/=factor
        expected=(zero,idle) if action=='WAKE' else (idle,zero)
        if np.max(abs(curve.start-expected[0]))>1e-10 or np.max(abs(curve.end-expected[1]))>1e-10:
            raise ValueError('Daily bookend endpoints changed')
        if len(stage['t_s'])!=2:raise ValueError('Daily bookend must be a two-pose quintic')
        bookends[action]=dict(kind='quintic',label=action.lower(),duration_s=curve.duration,
                              start_rad=curve.start.tolist(),end_rad=curve.end.tolist())
    # Reuse the six-axis Daily poses. Encoder noise in its saved gripper pose
    # must not become an offset on the user-specified absolute 0..250 range.
    idle[6]=zero[6]=GRIPPER_CLOSED
    for stage in bookends.values():
        stage['start_rad'][6]=stage['end_rad'][6]=GRIPPER_CLOSED
    return idle,zero,sources,bookends


def _pose(idle, **degrees):
    result=idle.copy()
    for joint,value in degrees.items(): result[int(joint[1:])-1]+=math.radians(value)
    return result


def _sequence(idle, points):
    stages=[]
    last_beat,last_pose,last_label=points[0]
    if last_beat!=0:raise ValueError('sequence must start at beat zero')
    for beat,pose,label in points[1:]:
        stages.append(dict(kind='quintic',label=label,duration_s=(beat-last_beat)*60/BPM,
                           start_rad=last_pose.tolist(),end_rad=pose.tolist()))
        last_beat,last_pose,last_label=beat,pose,label
    return stages


def _continuous(points, label):
    # Eight beats initially occupy ten seconds. If necessary, uniformly slow
    # the phrase; never shrink positions or increase the controller limits.
    times=np.asarray([beat*1.25 for beat,_ in points])
    poses=np.asarray([pose for _,pose in points])
    curve=Curve(times,poses,velocity_limit=ARM_VMAX,acceleration_limit=ARM_AMAX)
    times=times/curve.scale
    if times[-1]>12.:raise ValueError(f'{label}: requires more than 12 seconds; revise choreography')
    return [dict(kind='spline',label=label,duration_s=float(times[-1]),
                 t_s=times.tolist(),q_rad=poses.tolist())]


def _nod_phrase(idle):
    # Keep 01 at eight seconds, rather than using the ten-second body phrase.
    # Only phrase endpoints stop; internal poses are passed through smoothly.
    times=np.array([0.,1.15,2.3,3.,4.2,5.5,6.3,8.])
    poses=np.tile(idle,(len(times),1))
    poses[:,1]+=np.radians([0.,1.5,0.,-.3,2.,0.,-.3,0.])
    poses[:,2]+=np.radians([0.,-1.,0.,.25,-1.5,0.,.25,0.])
    poses[:,3]+=np.radians([0.,-10.,0.,3.,-13.,0.,3.,0.])
    probe=Curve(times,poses,velocity_limit=ARM_VMAX,acceleration_limit=ARM_AMAX)
    # Normalize analytic wrist extrema into the original -14..+4 degree
    # envelope without clamping samples or flattening individual turning points.
    depth=idle[3]-probe.lower[3];height=probe.upper[3]-idle[3]
    scale=min(math.radians(14)/depth,math.radians(4)/height)
    poses[:,3]=idle[3]+(poses[:,3]-idle[3])*scale
    return [dict(kind='spline',label='continuous_nod',duration_s=8.,
                 t_s=times.tolist(),q_rad=poses.tolist())]


def _gripper_cycle(idle):
    opened=idle.copy();opened[6]=GRIPPER_BEAT_OPEN
    hold=lambda pose,seconds,label:dict(kind='quintic',label=label,duration_s=seconds,
        start_rad=pose.tolist(),end_rad=pose.tolist())
    move=lambda start,end,label:dict(kind='quintic',label=label,duration_s=.95,
        start_rad=start.tolist(),end_rad=end.tolist())
    return [move(idle,opened,'open'),hold(opened,.1,'open_hold'),move(opened,idle,'close')]


def _recipes(idle, bookends):
    p=lambda **kw:_pose(idle,**kw)
    wave=lambda amp,phase,cycles=2: [dict(kind='wave',label='phase_wave',duration_s=10.,
        base_rad=idle.tolist(),amplitude_rad=np.radians(amp).tolist(),phase_rad=np.radians(phase).tolist(),cycles=cycles)]
    return [
        ('从休息位舒展到统一互动姿态，复用 Daily 唤醒。',[bookends['WAKE']]),
        ('连续两次点头，轻点接深点，抬头连续回中；保留8秒和原腕部幅度，不插入定格。',_nod_phrase(idle)),
        ('肩肘大幅舒展，身体上升时腕部延迟打开，沿弧线连续收回。',_continuous([
            (0,idle),(1.5,p(j2=-8,j3=12,j4=-8,j1=-5,j5=4)),
            (3,p(j2=-16,j3=24,j4=-10,j1=-8,j5=8)),
            (4.5,p(j2=-11,j3=21,j4=14,j1=7,j5=-7)),
            (6,p(j2=-5,j3=11,j4=8,j1=9,j5=-5)),(8,idle)],'continuous_lift')),
        ('连续两次下沉与弹起，肩肘反向配合，腕部随身体舒展。',_continuous([
            (0,idle),(1.5,p(j2=14,j3=-15,j4=10,j1=-6)),
            (3,p(j2=-9,j3=14,j4=-7,j1=6,j5=5)),
            (4.5,p(j2=12,j3=-12,j4=8,j1=-6,j5=-5)),
            (6,p(j2=-7,j3=11,j4=-6,j1=5)),(8,idle)],'continuous_dip')),
        ('肩肘向前铺开，头部左右掠过后连续收臂，形成伸展弧线。',_continuous([
            (0,idle),(1.5,p(j2=10,j3=-8,j4=12,j1=-7,j5=6)),
            (3,p(j2=25,j3=-16,j4=23,j1=-10,j5=12)),
            (4.5,p(j2=19,j3=-12,j4=18,j1=10,j5=-12)),
            (6,p(j2=6,j3=-3,j4=7,j1=8,j5=-6)),(8,idle)],'continuous_reach')),
        ('底盘以约70°总摆幅连续扫过左右，肩肘和头部交错跟随。',_continuous([
            (0,idle),(1.5,p(j1=22,j2=-4,j3=6,j5=10,j6=7)),
            (2.75,p(j1=35,j2=-7,j3=10,j5=16,j6=10)),
            (4,p(j1=0,j2=4,j3=-4,j5=0,j6=-4)),
            (5.25,p(j1=-35,j2=-5,j3=8,j5=-16,j6=-10)),
            (6.5,p(j1=-22,j2=-2,j3=4,j5=-10,j6=-7)),(8,idle)],'continuous_turn')),
        ('大幅侧摆带动肩肘起伏，头部反向摆动，两轮连续摆动渐入渐出。',
         wave([35,12,16,16,16,14,0],[0,90,90,-30,180,150,0])),
        ('大幅折肘与展开之间连续穿行，肩腕补偿，轮廓有收放变化。',_continuous([
            (0,idle),(1.5,p(j2=-6,j3=16,j4=-14,j1=-7)),
            (2.75,p(j2=-10,j3=24,j4=-20,j1=-10,j5=8)),
            (4,p(j2=2,j3=0,j4=5,j1=0)),
            (5.25,p(j2=10,j3=-22,j4=21,j1=10,j5=-8)),
            (6.5,p(j2=5,j3=-12,j4=12,j1=7)),(8,idle)],'continuous_elbow')),
        ('抬头、侧看、俯看沿连续弧线衔接，肩肘与头部互相呼应。',_continuous([
            (0,idle),(1.5,p(j4=17,j2=-3,j3=4,j5=-10,j6=-8)),
            (2.75,p(j4=25,j2=-5,j3=6,j5=-14,j6=-12)),
            (4,p(j4=0,j2=1,j3=-1,j5=0,j6=3)),
            (5.25,p(j4=-25,j2=6,j3=-5,j5=14,j6=12)),
            (6.5,p(j4=-16,j2=3,j3=-3,j5=9,j6=8)),(8,idle)],'continuous_wrist')),
        ('仅7号电机短行程快拍：0°→45°→0°，打开0.95秒、停留0.1秒、闭合0.95秒；不是250°全开，六轴保持待机。',
         _gripper_cycle(idle)),
        ('肩、肘、腕以更大幅度依次传递两轮波浪，底盘和侧倾错拍呼应。',
         wave([16,16,23,25,12,15,0],[30,0,-55,-110,90,-90,0])),
        ('收拢回休息姿态，复用 Daily 慢慢入睡。',[bookends['REST']]),
    ]


def _retime_fast_spline(stage):
    q=np.asarray(stage['q_rad']);t=np.asarray(stage['t_s'],float)
    t=t/t[-1]
    curve=Curve(t,q,velocity_limit=ARM_VMAX,acceleration_limit=ARM_AMAX)
    # A half-percent timing margin covers numerical extrema evaluation only.
    duration=curve.duration*1.005
    return dict(kind='spline',label='fast_'+stage['label'],duration_s=duration,
                t_s=(t*duration).tolist(),q_rad=q.tolist())


def _fast_recipes(idle,bookends):
    recipes=_recipes(idle,bookends)
    for number in (1,2,3,4,5,7,8):
        description,stages=recipes[number]
        if number==1:
            stages=[dict(stages[0],q_rad=[list(q[:1])+idle[1:3].tolist()+list(q[3:]) for q in stages[0]['q_rad']])]
            description+=' 肩肘保持待机，不叠加微小往复。'
        elif number==5:
            q=np.asarray(stages[0]['q_rad']).copy();t=np.asarray(stages[0]['t_s']);u=t/t[-1]
            support=64*u**3*(1-u)**3
            q[:,1]=idle[1]-math.radians(6)*support
            q[:,2]=idle[2]+math.radians(9)*support
            stages=[dict(stages[0],q_rad=q.tolist())]
            description='底盘约70度连续左右回转，肩肘仅做一次协调支撑，去掉多次微小反转。'
        description=description.replace('保留8秒和原腕部幅度','保留原腕部幅度').replace('不插入定格','解析缩时，不插入定格')
        recipes[number]=(description+' 按当前速度与加速度预算压缩整段时长。',
                         [_retime_fast_spline(stages[0])])
    for number in (6,10):
        description,stages=recipes[number]
        wave_spec=dict(stages[0],cycles=1.,duration_s=1.)
        wave=WaveSegment(wave_spec)
        t=np.linspace(0.,1.,17)
        q=np.array([wave.at(float(x))[0] for x in t])
        q[0]=q[-1]=idle
        old_wave=WaveSegment(stages[0])
        previous=np.array([old_wave.at(float(x))[0] for x in np.linspace(0,old_wave.duration,1001)])
        probe=Curve(t,q,velocity_limit=ARM_VMAX,acceleration_limit=ARM_AMAX)
        current_range=np.asarray(probe.upper)-np.asarray(probe.lower)
        for joint in range(6):
            if current_range[joint]>1e-10:
                q[:,joint]=idle[joint]+(q[:,joint]-idle[joint])*np.ptp(previous[:,joint])/current_range[joint]
        # Keep the familiar shoulder/elbow/wrist phase offsets, with one clear
        # sweep and no second, fading repetition at the end of the phrase.
        spec=dict(kind='spline',label='single_phase_sweep',duration_s=1.,t_s=t.tolist(),q_rad=q.tolist())
        recipes[number]=(description.replace('两轮','一轮')+' 去掉重复慢尾，整段连续快速收回。',
                         [_retime_fast_spline(spec)])
    recipes[9]=('仅7号电机连续短拍：0°→45°→0°，整轮2秒；开口顶点平滑反向，无中间停留，六轴保持待机。',
                [dict(kind='gripper_pulse',label='continuous_short_pulse',duration_s=2.,
                      base_rad=idle.tolist(),amplitude_rad=GRIPPER_BEAT_OPEN)])
    return recipes


def _validate_plan(plan, gravity):
    samples=np.linspace(0,plan.duration,math.ceil(plan.duration/.02)+1)
    max_gravity=np.zeros(6)
    max_inverse=np.zeros(6)
    inverse=all(hasattr(gravity,k) for k in ('pin','model'))
    data=gravity.model.createData() if inverse else None
    positions=[]
    speeds=[]
    frame=gravity.model.getFrameId('gripper_end') if inverse else None
    kinematics=inverse and frame < gravity.model.nframes
    for stamp in samples:
        q,v=plan.at(stamp);a=plan.acceleration_at(stamp)
        if .1*plan.duration < stamp < .9*plan.duration:
            speeds.append(float(np.max(abs(v[:6]))))
        if np.any(q[:6]<gravity.lower) or np.any(q[:6]>gravity.upper):raise ValueError('model joint limit exceeded')
        tau=vector(gravity(q[:6]),6)
        max_gravity=np.maximum(max_gravity,abs(tau))
        if np.any(abs(tau)>CAPS[:6]):raise ValueError('gravity budget exceeded')
        if inverse:
            required=gravity.pin.rnea(gravity.model,data,q[:6],v[:6],a[:6])
            if not np.isfinite(required).all() or np.any(abs(required)>CAPS[:6]):
                raise ValueError('model inverse dynamics budget exceeded')
            max_inverse=np.maximum(max_inverse,abs(required))
            if kinematics:
                gravity.pin.forwardKinematics(gravity.model,data,q[:6])
                gravity.pin.updateFramePlacements(gravity.model,data)
                positions.append(data.oMf[frame].translation.copy())
    return dict(gravity_peak_nm=max_gravity.tolist(),inverse_dynamics_checked=inverse,
                inverse_dynamics_peak_nm=max_inverse.tolist() if inverse else None,
                check_period_s=.02,collision_checked=False,
                tool_xyz_min_m=np.min(positions,axis=0).tolist() if positions else None,
                tool_xyz_max_m=np.max(positions,axis=0).tolist() if positions else None,
                inner_quiet_fraction=float(np.mean(np.asarray(speeds)<math.radians(.4))),
                peak_velocity_rad_s=plan.peak_velocity.tolist(),peak_acceleration_rad_s2=plan.peak_acceleration.tolist())


def _spec(number, idle, zero, identity, sources, bookends, recipe_version="legacy_v5", recipes=None):
    key,name,action_id=CATALOG[number]
    if recipe_version not in RECIPE_VERSIONS:raise ValueError('unknown dance recipe version')
    if recipes is None:recipes=(_fast_recipes if recipe_version=='fast_v6' else _recipes)(idle,bookends)
    description,stages=recipes[number]
    plan=DancePlan(stages,bookend=number in (0,11),gripper_motion=number==9)
    spec=dict(schema='rebuddy.dance_action.v1',number=number,key=key,name=name,action_id=action_id,
        description=description,beats=None if number in (0,11) else 2 if number==9 else 8,
        bpm=None if number in (0,11) else (2 if number==9 else 8)*60/plan.duration,
        duration_s=plan.duration,start_rad=plan.start.tolist(),end_rad=plan.end.tolist(),
        start_pose='ZERO' if number==0 else 'ATTENTIVE',end_pose='ZERO' if number==11 else 'ATTENTIVE',
        gripper_motion=number==9,hardware_accepted=False,
        source='daily_wake' if number==0 else 'daily_rest' if number==11 else 'authored_joint_choreography',
        calibration_sha256=identity['calibration_sha256'],urdf_sha256=identity['urdf_sha256'],
        dependencies=sources,stages=stages)
    if number==9:
        spec['gripper_control']=dict(following_limit_deg=10.8,reference_window_deg=12.,
            reference='lerobot-teleoperator-rebot-vr/config/rs_mit_split.yaml: gripper_relative_target_deg; same 90% window as Daily Clap',
            motor_range_deg=[0.,250.],velocity_limit_rad_s=GRIPPER_VMAX,
            profile='short_stroke',command_range_deg=[0.,45.],full_open=False,
            acceleration_limit_rad_s2=GRIPPER_AMAX,torque_budget_nm=float(CAPS[6]),kp=5.,kd=.3,
            reference_max_speed_deg_s=300.,reference_max_acceleration_deg_s2=1200.,
            motion_profile='0..45 motor degrees: 0.95s quintic open, 0.1s hold, 0.95s quintic close; physical maximum remains 250deg')
    if recipe_version=='fast_v6':
        spec['recipe_version']=recipe_version
        if number==9:
            spec['gripper_control']['motion_profile']='0..45 motor degrees: one 2-second degree-six polynomial pulse; no open dwell; physical maximum remains 250deg'
    return plan,spec


def build_library(calibration, urdf, output=DEFAULT_OUTPUT, recipe_version='fast_v6'):
    if recipe_version not in RECIPE_VERSIONS:raise ValueError('unknown dance recipe version')
    output=Path(output)
    gravity,identity=preflight(Path(calibration),Path(urdf))
    idle,zero,sources,bookends=_anchors(identity)
    recipes=(_fast_recipes if recipe_version=='fast_v6' else _recipes)(idle,bookends)
    actions=[];payloads={}
    for number in range(12):
        plan,spec=_spec(number,idle,zero,identity,sources,bookends,recipe_version,recipes)
        spec['validation']=_validate_plan(plan,gravity)
        file=f'{number:02d}_{spec["key"]}.json'
        payloads[file]=(json.dumps(spec,ensure_ascii=False,indent=2)+'\n').encode()
        import hashlib
        actions.append(dict(number=number,key=spec['key'],name=spec['name'],action_id=spec['action_id'],
            file=file,sha256=hashlib.sha256(payloads[file]).hexdigest(),duration_s=plan.duration,
            beats=spec['beats'],bpm=spec['bpm'],description=spec['description'],gripper_motion=spec['gripper_motion']))
    manifest=dict(schema='rebuddy.dance_library.v1',recipe_version=recipe_version,hardware_accepted=False,actions=actions,
        calibration_sha256=identity['calibration_sha256'],urdf_sha256=identity['urdf_sha256'],
        idle_rad=idle.tolist(),zero_rad=zero.tolist(),dependencies=sources,
        json_action_bytes=sum(len(v) for v in payloads.values()),
        compact_motion_json_bytes=sum(len(json.dumps(json.loads(v)['stages'],separators=(',',':')).encode()) for v in payloads.values()),
        firmware_integrated=False,screen_sync=False)
    output.mkdir(parents=True,exist_ok=True)
    # No silent overwrite of an edited or previously accepted library.
    payloads['manifest.json']=(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n').encode()
    for name,content in payloads.items():
        path=output/name
        if path.exists() and path.read_bytes()!=content:raise ValueError(f'目标已有不同数据：{path}')
    for name,content in payloads.items():
        path=output/name
        if not path.exists():path.write_bytes(content)
    return manifest


def _load(calibration, urdf, folder, numbers):
    folder=Path(folder)
    gravity,identity=preflight(Path(calibration),Path(urdf))
    idle,zero,sources,bookends=_anchors(identity)
    manifest_file=folder/'manifest.json'
    manifest=json.loads(manifest_file.read_text())
    if manifest.get('schema')!='rebuddy.dance_library.v1':raise ValueError('invalid dance library')
    recipe_version=manifest.get('recipe_version','legacy_v5')
    if recipe_version not in RECIPE_VERSIONS:raise ValueError('unknown dance recipe version')
    for key in ('calibration_sha256','urdf_sha256'):
        if manifest.get(key)!=identity[key]:raise ValueError('dance library identity changed')
    if manifest.get('dependencies')!=sources:raise ValueError('dance source dependencies changed')
    if not np.array_equal(manifest.get('idle_rad'),idle) or not np.array_equal(manifest.get('zero_rad'),zero):
        raise ValueError('dance anchors changed')
    entries=manifest.get('actions',[])
    if [a.get('number') for a in entries]!=list(range(12)):raise ValueError('invalid twelve-action catalog')
    plans,reports={},{}
    recipes=(_fast_recipes if recipe_version=='fast_v6' else _recipes)(idle,bookends)
    for number in numbers:
        item=entries[number]
        path=folder/item['file']
        if path.parent.resolve()!=folder.resolve() or digest(path)!=item['sha256']:
            raise ValueError('dance action file checksum changed')
        actual=json.loads(path.read_text())
        plan,expected=_spec(number,idle,zero,identity,sources,bookends,recipe_version,recipes)
        checks=_validate_plan(plan,gravity)
        expected['validation']=checks
        if actual!=expected:raise ValueError('dance design changed; regenerate a separate version')
        code=f'{number:02d}'
        plans[code]=plan
        reports[code]=dict(actual,source=str(path.resolve()),source_sha256=digest(path),
                          library_manifest=str(manifest_file.resolve()),library_manifest_sha256=digest(manifest_file),
                          playback_s=plan.duration)
    return manifest,plans,gravity,identity,reports


def load_action(number, calibration, urdf, folder=DEFAULT_OUTPUT):
    number=_number(number)
    _,plans,gravity,_,reports=_load(calibration,urdf,folder,[number])
    return plans[f'{number:02d}'],gravity,reports[f'{number:02d}']


def load_library(calibration, urdf, folder=DEFAULT_OUTPUT):
    return _load(calibration,urdf,folder,range(12))
