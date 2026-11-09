"""Short left/right, then up/down motion about one virtual point; offline only."""
from __future__ import annotations

import math
import numpy as np
from scipy.spatial.transform import Rotation

import rs_dance_motion as base
import rs_dance_pivot_kinematics as kin
from rs_dance_pivot_trajectory import _DenseCurve
from rs_dance_pivot_ik import VirtualIK, config_dict
from rs_dance_speed import checked_trial_speed

CONFIG = dict(offset_m=.20, amplitude_deg=2., amplitude_scales=[1.,.75,.5],
              phase_min_s=1.2, max_duration_s=4.8, sample_period_s=.02)
LOWERED_CONFIG = dict(CONFIG, amplitude_deg=10., pivot_drop_m=.03, horizontal_deg=6., vertical_deg=10.,
                     phase_min_s=2., transition_min_s=.6,
                     max_duration_s=11., amplitude_scales=[1.])
AMPLIFIED_CONFIG = dict(LOWERED_CONFIG, amplitude_deg=20., pivot_drop_m=.12,
                       horizontal_deg=20., vertical_deg=18., reference_pitch_deg=10.,
                       swing_profile='sweep_legs', default_trial_speed=1.,
                       phase_min_s=3., max_duration_s=20.)


def checked_config(config=None):
    cfg=dict(CONFIG)
    if config:
        if set(config)-set(AMPLIFIED_CONFIG):raise ValueError('unknown short pivot setting')
        cfg.update(config)
    for key in ('amplitude_deg','phase_min_s','max_duration_s'):
        if not math.isfinite(cfg[key]) or cfg[key]<=0:raise ValueError('invalid short pivot setting')
    if not math.isfinite(cfg['offset_m']) or cfg['offset_m']<0:raise ValueError('invalid pivot offset')
    if cfg['sample_period_s']!=.02:raise ValueError('short pivot uses 50 Hz')
    if not cfg['amplitude_scales'] or any(not math.isfinite(s) or not 0<s<=1 for s in cfg['amplitude_scales']):
        raise ValueError('invalid amplitude scale')
    for key in ('horizontal_deg','vertical_deg','transition_min_s'):
        if key in cfg and (not math.isfinite(cfg[key]) or cfg[key]<=0):raise ValueError('invalid lowered pivot setting')
    if not math.isfinite(cfg.get('pivot_drop_m',0.)) or cfg.get('pivot_drop_m',0.)<0:raise ValueError('invalid pivot drop')
    if not math.isfinite(cfg.get('reference_pitch_deg',0.)):raise ValueError('invalid reference pitch')
    if cfg.get('swing_profile','pulse') not in ('pulse','sweep_legs'):raise ValueError('invalid swing profile')
    checked_trial_speed(cfg.get('default_trial_speed',.3))
    return cfg


def ik_config(cfg):
    return config_dict(dict(cone_axis='idle_tool',tcp_offset_m=[cfg['offset_m'],0.,0.]))


def pulse(u):
    """One positive/negative swing, with zero velocity/acceleration at ends."""
    u=float(np.clip(u,0.,1.))
    return math.sin(2*math.pi*u)*math.sin(math.pi*u)**2/(3*math.sqrt(3)/8)


def orientation(reference,phase,u,degrees):
    axis=np.array([0.,0.,1.]) if phase==0 else reference[:,1]
    return Rotation.from_rotvec(axis*math.radians(degrees)*pulse(u)).as_matrix()@reference


def rotated(reference,phase,degrees):
    axis=np.array([0.,0.,1.]) if phase==0 else reference[:,1]
    return Rotation.from_rotvec(axis*math.radians(degrees)).as_matrix()@reference


def prepared_reference(reference,cfg):
    return Rotation.from_rotvec(reference[:,1]*math.radians(cfg.get('reference_pitch_deg',0.))).as_matrix()@reference


class SweepCurve:
    """Independently timed outbound, cross-over, and return; no added holds."""
    def __init__(self,spec,amplitude):
        self.angles=[(0.,amplitude),(amplitude,-amplitude),(-amplitude,0.)]
        if len(spec['legs'])!=3 or [leg['angle_deg'] for leg in spec['legs']]!=[list(a) for a in self.angles]:
            raise ValueError('sweep angular schedule changed')
        self.curves=[_DenseCurve(leg) for leg in spec['legs']]
        self.ends=np.cumsum([c.duration for c in self.curves]);self.duration=float(self.ends[-1])
        self.start=self.curves[0].start.copy();self.end=self.curves[-1].end.copy()
        for attr,op in (('lower',np.min),('upper',np.max),('peak_velocity',np.max),('peak_acceleration',np.max)):
            setattr(self,attr,op([getattr(c,attr) for c in self.curves],axis=0))
        for left,right in zip(self.curves,self.curves[1:]):
            if max(np.max(abs(left.end-right.start)),np.max(abs(left.at(left.duration)[1]-right.at(0)[1])),
                   np.max(abs(left.acceleration_at(left.duration)-right.acceleration_at(0))))>1e-7:
                raise ValueError('sweep reversal is not C2')
    def locate(self,t):
        t=float(np.clip(t,0,self.duration));i=min(2,int(np.searchsorted(self.ends,t,side='right')))
        return i,self.curves[i],t-(self.ends[i-1] if i else 0.)
    def at(self,t):_,c,t=self.locate(t);return c.at(t)
    def acceleration_at(self,t):_,c,t=self.locate(t);return c.acceleration_at(t)
    def jerk_at(self,t):_,c,t=self.locate(t);return c.jerk_at(t)
    def angle_at(self,t):
        i,c,t=self.locate(t);u=t/c.duration;a,b=self.angles[i]
        return a+(b-a)*u**3*(10-15*u+6*u*u)


class SwaySegment:
    def __init__(self,spec):
        self.config=checked_config(spec['config']);self.amplitude_scale=float(spec['amplitude_scale'])
        if self.amplitude_scale not in self.config['amplitude_scales']:raise ValueError('invalid selected amplitude')
        chunks=spec['chunks'];lowered=self.config.get('pivot_drop_m',0.)>0 or self.config.get('reference_pitch_deg',0.)!=0
        names=('entry','left_right','up_down','exit') if lowered else ('left_right','up_down')
        if len(chunks)!=len(names):raise ValueError('invalid pivot swing/transition layout')
        self.stages=[]
        for name,chunk in zip(names,chunks):
            if name in ('entry','exit'):curve=base.PoseSegment(chunk)
            elif self.config.get('swing_profile')=='sweep_legs':
                key='horizontal_deg' if name=='left_right' else 'vertical_deg'
                curve=SweepCurve(chunk,self.config.get(key,self.config['amplitude_deg'])*self.amplitude_scale)
            else:curve=_DenseCurve(chunk)
            self.stages.append((name,curve))
        self.ends=np.cumsum([c.duration for _,c in self.stages]);self.duration=float(self.ends[-1])
        if abs(self.duration-spec['duration_s'])>1e-8 or self.duration>self.config['max_duration_s']:
            raise ValueError('short pivot duration mismatch or too long')
        self.start=self.stages[0][1].start.copy();self.end=self.stages[-1][1].end.copy()
        for attr,op in (('lower',np.min),('upper',np.max),('peak_velocity',np.max),('peak_acceleration',np.max)):
            setattr(self,attr,op([getattr(c,attr) for _,c in self.stages],axis=0))
        self.velocity_limit=1.;self.acceleration_limit=2.;self.gripper_motion=False
        self.default_trial_speed=checked_trial_speed(self.config.get('default_trial_speed',.3))
        self.gripper_velocity_limit=1.;self.gripper_acceleration_limit=2.
        if max(abs(self.lower[6]),abs(self.upper[6]))>1e-10:raise ValueError('keep gripper closed')
        if max(self.peak_velocity)>1+1e-8 or max(self.peak_acceleration)>2+1e-8:raise ValueError('short pivot exceeds dynamics')
        self.motion_start_index=1 if lowered else 0
        self.prepared=self.stages[self.motion_start_index][1].start.copy()
        self.dance_interval_s=(float(self.ends[0]) if lowered else 0.,float(self.ends[-2]) if lowered else self.duration)
        for index,(name,curve) in enumerate(self.stages):
            if name in ('left_right','up_down') and max(np.max(abs(curve.start-self.prepared)),np.max(abs(curve.end-self.prepared)))>1e-8:
                raise ValueError('each swing must return to the same posture')
            if index and np.max(abs(self.stages[index-1][1].end-curve.start))>1e-8:raise ValueError('pivot transition gap')
            for t in (0,curve.duration):
                if max(np.max(abs(curve.at(t)[1])),np.max(abs(curve.acceleration_at(t))))>1e-7:
                    raise ValueError('short pivot seam is not stationary C2')
    def chunk_at(self,t):
        t=float(np.clip(t,0,self.duration));i=min(len(self.stages)-1,int(np.searchsorted(self.ends,t,side='right')))
        return i,self.stages[i][1],t-(self.ends[i-1] if i else 0.)
    def at(self,t):_,c,t=self.chunk_at(t);return c.at(t)
    def acceleration_at(self,t):_,c,t=self.chunk_at(t);return c.acceleration_at(t)
    def jerk_at(self,t):
        _,c,t=self.chunk_at(t)
        if hasattr(c,'jerk_at'):return c.jerk_at(t)
        u=t/c.duration;return 60*(1-6*u+6*u*u)*c.delta/c.duration**3


def transition(start,end,minimum):
    delta=abs(np.asarray(end)-start)
    duration=max(minimum,float(np.max(1.875*delta)),math.sqrt(float(np.max(10/math.sqrt(3)*delta/2))))*1.005
    return dict(start_rad=np.asarray(start).tolist(),end_rad=np.asarray(end).tolist(),duration_s=duration)


def build_recipe(gravity,idle,config=None):
    cfg=checked_config(config);idle=np.asarray(idle,float);solver=VirtualIK(gravity,idle,ik_config(cfg));failures=[]
    point=solver.idle_P-np.array([0.,0.,cfg.get('pivot_drop_m',0.)]);prepared=idle.copy()
    reference=prepared_reference(solver.reference,cfg)
    needs_entry=bool(cfg.get('pivot_drop_m',0.) or cfg.get('reference_pitch_deg',0.))
    if needs_entry:
        preparation=solver.solve(point,reference,solver.seeds())
        if not preparation['accepted']:raise ValueError('lowered pivot preparation failed: '+preparation['reason'])
        prepared=np.asarray(preparation['q_rad'])
    amplitudes=[cfg.get('horizontal_deg',cfg['amplitude_deg']),cfg.get('vertical_deg',cfg['amplitude_deg'])]
    for scale in cfg['amplitude_scales']:
        try:
            chunks=[];previous=prepared.copy()
            for phase in range(2):
                if cfg.get('swing_profile')=='sweep_legs':
                    amplitude=amplitudes[phase]*scale;legs=[]
                    for a,b in ((0.,amplitude),(amplitude,-amplitude),(-amplitude,0.)):
                        duration=cfg['phase_min_s']/3
                        times=np.linspace(0,duration,max(4,math.ceil(duration/.02)+1));poses=[];start=previous.copy()
                        for stamp in times:
                            u=stamp/duration;angle=a+(b-a)*u**3*(10-15*u+6*u*u)
                            result=solver.solve(point,rotated(reference,phase,angle),[previous[:6]])
                            if not result['accepted']:raise ValueError(result['reason'])
                            q=np.asarray(result['q_rad'])
                            if np.max(abs(q-previous))>.2:raise ValueError('IK branch jump')
                            poses.append(q);previous=q
                        if np.max(abs(poses[0]-start))>1e-6:raise ValueError('sweep initial branch changed')
                        poses[0]=start
                        if b==0.:
                            if np.max(abs(previous-prepared))>1e-6:raise ValueError('IK did not close starting branch')
                            poses[-1]=prepared.copy();previous=prepared.copy()
                        curve=base.Curve(times,poses,velocity_limit=1.,acceleration_limit=2.)
                        legs.append(dict(angle_deg=[a,b],t_s=(times/curve.scale*1.005).tolist(),q_rad=np.asarray(poses).tolist()))
                    chunks.append(dict(legs=legs))
                    continue
                times=np.linspace(0,cfg['phase_min_s'],math.ceil(cfg['phase_min_s']/.02)+1);poses=[]
                for stamp in times:
                    desired=orientation(reference,phase,stamp/times[-1],amplitudes[phase]*scale)
                    result=solver.solve(point,desired,[previous[:6]])
                    if not result['accepted']:raise ValueError(result['reason'])
                    q=np.asarray(result['q_rad'])
                    if np.max(abs(q-previous))>.2:raise ValueError('IK branch jump')
                    poses.append(q);previous=q
                if np.max(abs(previous-prepared))>1e-6:raise ValueError('IK did not close starting branch')
                poses[0]=prepared.copy();poses[-1]=prepared.copy();previous=prepared.copy()
                curve=base.Curve(times,poses,velocity_limit=1.,acceleration_limit=2.)
                chunks.append(dict(t_s=(times/curve.scale*1.005).tolist(),q_rad=np.asarray(poses).tolist()))
            if needs_entry:
                chunks=[transition(idle,prepared,cfg['transition_min_s']),*chunks,transition(prepared,idle,cfg['transition_min_s'])]
            stage=dict(kind='virtual_pivot_sway',label='left_right_up_down',config=cfg,amplitude_scale=scale,
                       duration_s=sum(sum(leg['t_s'][-1] for leg in c['legs']) if 'legs' in c else c['t_s'][-1] if 't_s' in c else c['duration_s'] for c in chunks),chunks=chunks)
            plan=SwaySegment(stage)
            extra=dict(pivot_sway_design=dict(config=cfg,amplitude_scale=scale,
                actual_amplitude_deg=cfg['amplitude_deg']*scale,model_tcp_offset_m=[0.,0.,0.],
                virtual_offset_m=[cfg['offset_m'],0.,0.],P_world_m=point.tolist(),
                dance_interval_s=list(plan.dance_interval_s),actual_duration_s=plan.duration,
                collision_identity=solver.collision.identity,physical_tool_calibrated=False,
                default_trial_speed=plan.default_trial_speed,rejected_scales=failures))
            if needs_entry:
                extra['pivot_sway_design'].update(pivot_drop_m=cfg.get('pivot_drop_m',0.),
                    actual_horizontal_deg=amplitudes[0]*scale,actual_vertical_deg=amplitudes[1]*scale,
                    initial_P_world_m=solver.idle_P.tolist(),prepared_rad=prepared.tolist())
            if 'reference_pitch_deg' in cfg:
                extra['pivot_sway_design']['reference_pitch_deg']=cfg['reference_pitch_deg']
            validate_sway(plan,gravity,idle,extra)
            return [stage],extra,['固定同一空间点，左右摆一次','上下摆一次，回到互动姿态']
        except ValueError as error:failures.append(dict(scale=scale,reason=str(error)))
    raise ValueError('short pivot could not meet duration and geometry constraints: '+str(failures))


def validate_sway(plan,gravity,idle,spec):
    design=spec['pivot_sway_design'];cfg=checked_config(design['config'])
    segment=plan if isinstance(plan,SwaySegment) else plan.stages[0][1]
    if cfg!=segment.config or design['amplitude_scale']!=segment.amplitude_scale:raise ValueError('short pivot metadata mismatch')
    amplitude=cfg['amplitude_deg']*segment.amplitude_scale
    if abs(design['actual_amplitude_deg']-amplitude)>1e-9 or abs(design['actual_duration_s']-plan.duration)>1e-8:
        raise ValueError('short pivot amplitude/duration mismatch')
    solver=VirtualIK(gravity,idle,ik_config(cfg));solver.collision.validate_identity(design['collision_identity'])
    reference=prepared_reference(solver.reference,cfg)
    point=solver.idle_P-np.array([0.,0.,cfg.get('pivot_drop_m',0.)])
    if not np.allclose(design['P_world_m'],point,rtol=0,atol=1e-9):raise ValueError('short pivot point changed')
    if not np.allclose(design['dance_interval_s'],segment.dance_interval_s,rtol=0,atol=1e-9):raise ValueError('pivot motion interval changed')
    amplitudes=[cfg.get('horizontal_deg',cfg['amplitude_deg'])*segment.amplitude_scale,
                cfg.get('vertical_deg',cfg['amplitude_deg'])*segment.amplitude_scale]
    if cfg.get('pivot_drop_m',0.) or cfg.get('reference_pitch_deg',0.):
        if design['pivot_drop_m']!=cfg.get('pivot_drop_m',0.) or [design['actual_horizontal_deg'],design['actual_vertical_deg']]!=amplitudes:
            raise ValueError('lowered pivot amplitude metadata changed')
        if not np.allclose(design['prepared_rad'],segment.prepared,rtol=0,atol=1e-9):raise ValueError('pivot preparation changed')
    if 'reference_pitch_deg' in cfg and design.get('reference_pitch_deg')!=cfg['reference_pitch_deg']:raise ValueError('pivot reference pitch changed')
    if design['virtual_offset_m']!=solver.cfg['tcp_offset_m'] or design['model_tcp_offset_m']!=[0.,0.,0.] or design['default_trial_speed']!=segment.default_trial_speed:
        raise ValueError('short pivot coordinates or playback speed changed')
    if not np.allclose(plan.start,idle,rtol=0,atol=1e-9) or not np.allclose(plan.end,idle,rtol=0,atol=1e-9):raise ValueError('short pivot idle endpoints changed')
    errors=[];angles=[];positions=[];joints=[]
    for stamp in np.unique(np.r_[np.linspace(0,plan.duration,math.ceil(plan.duration/.01)+1),segment.ends]):
        q,_=plan.at(stamp);index,curve,local=segment.chunk_at(stamp)
        p,r=kin._pose(solver.parts,q,solver.cfg)
        if segment.stages[index][0] in ('left_right','up_down'):
            errors.append(float(np.linalg.norm(p-point)))
            phase=index-segment.motion_start_index
            target=rotated(reference,phase,curve.angle_at(local)) if isinstance(curve,SweepCurve) else orientation(reference,phase,local/curve.duration,amplitudes[phase])
            angles.append(math.degrees(float(np.linalg.norm(solver.pin.log3(target.T@r)))))
        kin._check_workspace(solver.parts,q,solver.cfg,solver.workspace,solver.lower,solver.upper)
        if solver.collision.check(q)['colliding']:raise ValueError('short pivot self collision')
        joints.append(q);positions.append(kin._pose(solver.parts,q,dict(solver.cfg,tcp_offset_m=[0.,0.,0.]))[0])
    if max(errors)>.002 or max(angles)>.2:raise ValueError('short pivot fixed point/orientation interpolation error')
    base._validate_plan(plan,gravity)
    return dict(pivot_sway=dict(position_error_max_m=max(errors),orientation_error_max_deg=max(angles),
        joint_ranges_deg=np.degrees(np.ptp(joints,axis=0)).tolist(),model_tcp_span_m=np.ptp(positions,axis=0).tolist(),
        collision_checked=True,collision_samples=len(joints),environment_collision_checked=False))
