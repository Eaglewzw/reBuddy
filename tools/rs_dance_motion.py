"""Shared trajectory primitives, Daily anchors and model validation for the current dance library."""
import json
import math
import numpy as np
from scipy.interpolate import PPoly
from rs_gravity_record import ROOT, CAPS, digest, preflight, vector
from rs_motion import Curve
from rs_record_playback import LOWER, UPPER
DAILY_DIR = ROOT / 'record/daily_ready'
ARM_VMAX, ARM_AMAX = 1., 2.
GRIPPER_VMAX, GRIPPER_AMAX = 1.6, 5.1
FAST_WIDE_GRIPPER_VMAX, FAST_WIDE_GRIPPER_AMAX = 2.8, 10.
GRIPPER_CLOSED, GRIPPER_OPEN = 0., math.radians(250.)
GRIPPER_BEAT_OPEN = math.radians(45.)
GRIPPER_FOLLOWING_LIMIT = math.radians(10.8)

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


class GripperPulseSegment:
    """One degree-six pulse: no dwell and no internal reversal seam."""
    command_amplitude = GRIPPER_BEAT_OPEN
    def __init__(self,spec):
        self.start=vector(spec['base_rad']);self.end=self.start.copy()
        self.duration=_finite_duration(spec['duration_s'])
        self.amplitude=float(spec['amplitude_rad'])
        if abs(self.start[6])>1e-12 or abs(self.duration-2.)>1e-12 or abs(self.amplitude-self.command_amplitude)>1e-12:
            raise ValueError(f'fast gripper pulse must be absolute 0..{math.degrees(self.command_amplitude):g} degrees in two seconds')
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


class FastWideGripperPulseSegment(GripperPulseSegment):
    command_amplitude = math.radians(90.)


class DancePlan:
    def __init__(self, specs, *, bookend=False, gripper_motion=False):
        if not isinstance(specs,list) or not 1 <= len(specs) <= 32:
            raise ValueError('invalid dance stage count')
        fast_wide=any(s.get('kind')=='gripper_wide_pulse' for s in specs)
        if fast_wide and (len(specs)!=1 or not gripper_motion or bookend):
            raise ValueError('wide fast pulse requires a standalone gripper motion profile')
        self.velocity_limit = .5 if bookend else ARM_VMAX
        self.acceleration_limit = 1. if bookend else ARM_AMAX
        # J7 has its own profile; arm limits and all other actions are unchanged.
        self.gripper_velocity_limit = GRIPPER_VMAX if gripper_motion else 1.
        self.gripper_acceleration_limit = GRIPPER_AMAX if gripper_motion else 2.
        if fast_wide:
            self.gripper_velocity_limit=FAST_WIDE_GRIPPER_VMAX
            self.gripper_acceleration_limit=FAST_WIDE_GRIPPER_AMAX
        self.gripper_motion = bool(gripper_motion)
        if self.gripper_motion:
            self.gripper_following_limit = GRIPPER_FOLLOWING_LIMIT
        self.stages=[]
        for stage in specs:
            kind=stage.get('kind')
            if kind=='quintic': curve=PoseSegment(stage)
            elif kind=='spline': curve=SplineSegment(stage)
            elif kind=='gripper_wide_pulse': curve=FastWideGripperPulseSegment(stage)
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
                raise ValueError('gripper motion must be explicitly enabled')
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


def _retime_fast_spline(stage):
    q=np.asarray(stage['q_rad']);t=np.asarray(stage['t_s'],float)
    t=t/t[-1]
    curve=Curve(t,q,velocity_limit=ARM_VMAX,acceleration_limit=ARM_AMAX)
    # A half-percent timing margin covers numerical extrema evaluation only.
    duration=curve.duration*1.005
    return dict(kind='spline',label='fast_'+stage['label'],duration_s=duration,
                t_s=(t*duration).tolist(),q_rad=q.tolist())


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

