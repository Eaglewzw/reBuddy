"""Blend only the horizontal-to-vertical handoff, keeping v9's clock and peaks."""
import copy
import math
import numpy as np
from scipy.interpolate import make_interp_spline, PPoly
from scipy.spatial.transform import Rotation

import rs_dance_design as base
import rs_dance_pivot_sway as sway
import rs_dance_pivot_kinematics as kin
from rs_dance_virtual_pivot_scan import VirtualIK
from rs_record_playback import LOWER,UPPER


def angle_state(curve,t):
    i,leg,t=curve.locate(t);a,b=curve.angles[i];u=t/leg.duration
    return np.array([a+(b-a)*u**3*(10-15*u+6*u*u),
        (b-a)*30*u*u*(1-u)**2/leg.duration,
        (b-a)*60*u*(1-u)*(1-2*u)/leg.duration**2])


def hermite(left,right,duration):
    a=np.array([left[0],left[1]*duration,left[2]*duration**2/2,0.,0.,0.])
    a[3:]=np.linalg.solve([[1,1,1],[3,4,5],[6,12,20]],
        [right[0]-sum(a),right[1]*duration-a[1]-2*a[2],right[2]*duration**2-2*a[2]])
    return a


def angle_coefficients(source,start,end):
    horizontal=source.stages[1][1];vertical=source.stages[2][1]
    return np.array([hermite(angle_state(horizontal,start-source.ends[0]),[0,0,0],end-start),
                     hermite([0,0,0],angle_state(vertical,end-source.ends[1]),end-start)])


def target_rotation(reference,coefficients,u):
    yaw,pitch=np.polynomial.polynomial.polyval(u,coefficients.T)
    return Rotation.from_rotvec([0,0,math.radians(yaw)]).as_matrix()@sway.rotated(reference,1,pitch)


class BridgeCurve:
    """Quintic spline with nonzero endpoint derivatives from the original path."""
    def __init__(self,spec,source):
        self.start_s=float(spec['start_s']);self.end_s=float(spec['end_s'])
        self.duration=self.end_s-self.start_s
        # Touch only the horizontal return and vertical departure. All extrema
        # and both entry/exit transitions must remain outside the blend.
        h=source.stages[1][1];v=source.stages[2][1];seam=source.ends[1]
        if not seam-h.curves[-1].duration<self.start_s<seam<self.end_s<seam+v.curves[0].duration:
            raise ValueError('bridge would replace a swing extremum or leave its handoff')
        t=np.asarray(spec['t_s'],float);q=np.asarray(spec['q_rad'],float)
        if (len(t)<4 or q.shape!=(len(t),7) or not np.isfinite(t).all() or not np.isfinite(q).all()
            or t[0]!=0 or abs(t[-1]-self.duration)>1e-9 or np.any(np.diff(t)<=0)):
            raise ValueError('invalid handoff knots')
        left,lv=source.at(self.start_s);right,rv=source.at(self.end_s)
        if max(np.max(abs(q[0]-left)),np.max(abs(q[-1]-right)))>1e-9:raise ValueError('handoff endpoint changed')
        self.spline=make_interp_spline(t,q,k=5,bc_type=([(1,lv),(2,source.acceleration_at(self.start_s))],
                                                      [(1,rv),(2,source.acceleration_at(self.end_s))]))
        self.start=left.copy();self.end=right.copy()
        self.lower=np.zeros(7);self.upper=np.zeros(7);self.peak_velocity=np.zeros(7);self.peak_acceleration=np.zeros(7)
        for joint in range(7):
            p=PPoly.from_spline((self.spline.t,self.spline.c[:,joint],5))
            for order in range(3):
                points=np.r_[t,p.derivative(order+1).roots(extrapolate=False)]
                points=points[np.isfinite(points)&(points>=0)&(points<=self.duration)]
                values=p(points,nu=order)
                if order==0:self.lower[joint]=min(values);self.upper[joint]=max(values)
                elif order==1:self.peak_velocity[joint]=max(abs(values))
                else:self.peak_acceleration[joint]=max(abs(values))
        if np.any(self.lower<LOWER-1e-8) or np.any(self.upper>UPPER+1e-8):raise ValueError('handoff joint bound exceeded')
        if max(self.peak_velocity)>1+1e-8 or max(self.peak_acceleration)>2+1e-8:raise ValueError('handoff exceeds original speed/acceleration budget')
    def at(self,t):return self.spline(t),self.spline(t,nu=1)
    def acceleration_at(self,t):return self.spline(t,nu=2)
    def jerk_at(self,t):return self.spline(t,nu=3)


class JoinedSwaySegment:
    def __init__(self,spec):
        self.source=sway.SwaySegment(spec['source']);self.bridge=BridgeCurve(spec['bridge'],self.source)
        if spec['config']!=self.source.config or spec['duration_s']!=self.source.duration:raise ValueError('joined pivot must preserve source settings and duration')
        self.coefficients=angle_coefficients(self.source,self.bridge.start_s,self.bridge.end_s)
        for attr in ('config','duration','start','end','prepared','dance_interval_s','default_trial_speed',
                     'velocity_limit','acceleration_limit','gripper_velocity_limit','gripper_acceleration_limit','gripper_motion'):
            setattr(self,attr,getattr(self.source,attr))
        for attr,op in (('lower',np.minimum),('upper',np.maximum),('peak_velocity',np.maximum),('peak_acceleration',np.maximum)):
            setattr(self,attr,op(getattr(self.source,attr),getattr(self.bridge,attr)))
        if max(abs(self.lower[6]),abs(self.upper[6]))>1e-10:raise ValueError('handoff must keep gripper closed')
    def locate(self,t):
        t=float(np.clip(t,0,self.duration))
        return (self.bridge,t-self.bridge.start_s) if self.bridge.start_s<=t<=self.bridge.end_s else (self.source,t)
    def at(self,t):curve,t=self.locate(t);return curve.at(t)
    def acceleration_at(self,t):curve,t=self.locate(t);return curve.acceleration_at(t)
    def jerk_at(self,t):curve,t=self.locate(t);return curve.jerk_at(t)


def build_recipe(gravity,idle,config=None):
    settings=dict(half_window_s=1.7)
    if config:
        if set(config)-set(settings):raise ValueError('join config accepts half_window_s only; v9 amplitude/speed are retained')
        settings.update(config)
    half=float(settings['half_window_s'])
    if not math.isfinite(half) or half<=0:raise ValueError('invalid handoff window')
    stages,extra,_=sway.build_recipe(gravity,idle,sway.AMPLIFIED_CONFIG)
    source=sway.SwaySegment(stages[0]);seam=float(source.ends[1]);start,end=seam-half,seam+half
    if half>=min(source.stages[1][1].curves[-1].duration,source.stages[2][1].curves[0].duration):
        raise ValueError('handoff window crosses a swing extremum')
    coeff=angle_coefficients(source,start,end)
    solver=VirtualIK(gravity,idle,sway.ik_config(source.config));reference=sway.prepared_reference(solver.reference,source.config)
    point=np.asarray(extra['pivot_sway_design']['P_world_m']);previous=source.at(start)[0];poses=[]
    times=np.linspace(0,end-start,math.ceil((end-start)/.01)+1)
    for stamp in times:
        result=solver.solve(point,target_rotation(reference,coeff,stamp/(end-start)),[previous[:6]])
        if not result['accepted']:raise ValueError('handoff IK failed: '+result['reason'])
        q=np.asarray(result['q_rad'])
        if np.max(abs(q-previous))>.2:raise ValueError('handoff IK branch jump')
        poses.append(q);previous=q
    if max(np.max(abs(poses[0]-source.at(start)[0])),np.max(abs(poses[-1]-source.at(end)[0])))>1e-6:
        raise ValueError('handoff endpoints are on a different IK branch')
    poses[0]=source.at(start)[0];poses[-1]=source.at(end)[0]
    stage=dict(kind='virtual_pivot_joined',label='joined_left_right_up_down',config=source.config,
               source=stages[0],duration_s=source.duration,
               bridge=dict(start_s=start,end_s=end,t_s=times.tolist(),q_rad=np.asarray(poses).tolist()))
    segment=JoinedSwaySegment(stage)
    extra=copy.deepcopy(extra);extra['pivot_join_design']=dict(half_window_s=half,
        interval_s=[start,end],original_seam_s=seam,duration_preserved=True,angle_coefficients_deg=coeff.tolist())
    validate_join(segment,gravity,idle,extra)
    return [stage],extra,['左右摆动回收途中接入上下摆动','保持原摆幅、播放倍率和总时长，完成后回待机']


def validate_join(plan,gravity,idle,spec):
    segment=plan if isinstance(plan,JoinedSwaySegment) else plan.stages[0][1]
    source=segment.source;bridge=segment.bridge;design=spec['pivot_join_design']
    old=sway.validate_sway(source,gravity,idle,spec)
    if (not np.allclose(design['interval_s'],[bridge.start_s,bridge.end_s],rtol=0,atol=1e-9)
        or design['original_seam_s']!=source.ends[1]
        or not np.allclose(design['angle_coefficients_deg'],segment.coefficients,rtol=0,atol=1e-9)):
        raise ValueError('handoff metadata changed')
    solver=VirtualIK(gravity,idle,sway.ik_config(source.config));reference=sway.prepared_reference(solver.reference,source.config)
    point=np.asarray(spec['pivot_sway_design']['P_world_m']);errors=[];angles=[];speeds=[]
    times=np.linspace(0,bridge.duration,math.ceil(bridge.duration/.005)+1)
    for stamp in times:
        q,v=bridge.at(stamp);p,r=kin._pose(solver.parts,q,solver.cfg)
        errors.append(float(np.linalg.norm(p-point)))
        target=target_rotation(reference,segment.coefficients,stamp/bridge.duration)
        angles.append(math.degrees(float(np.linalg.norm(solver.pin.log3(target.T@r)))))
        kin._check_workspace(solver.parts,q,solver.cfg,solver.workspace,solver.lower,solver.upper)
        if solver.collision.check(q)['colliding']:raise ValueError('handoff self collision')
        speeds.append(float(max(abs(v[:6]))))
    if max(errors)>.002 or max(angles)>.2:raise ValueError('handoff fixed point/orientation error')
    if min(speeds)<.05:raise ValueError('handoff still contains a perceptible stop')
    # Preserve the original angular extrema: the bridge remains strictly
    # between the completed left excursion and upcoming upper excursion.
    for coefficients,lower,upper in ((segment.coefficients[0],-source.config['horizontal_deg'],0.),
                                     (segment.coefficients[1],0.,source.config['vertical_deg'])):
        roots=np.polynomial.polynomial.polyroots(np.polynomial.polynomial.polyder(coefficients))
        nodes=[0.,1.]+[r.real for r in roots if abs(r.imag)<1e-9 and 0<=r.real<=1]
        values=np.polynomial.polynomial.polyval(nodes,coefficients)
        if min(values)<lower-1e-8 or max(values)>upper+1e-8:raise ValueError('handoff changed angular amplitude')
    base._validate_plan(plan,gravity)
    return dict(source_pivot_sway=old['pivot_sway'],pivot_join=dict(interval_s=[bridge.start_s,bridge.end_s],
        position_error_max_m=max(errors),orientation_error_max_deg=max(angles),
        minimum_fastest_joint_velocity_rad_s=min(speeds),collision_samples=len(times),
        collision_checked=True,environment_collision_checked=False,duration_preserved=True,outside_bridge_unchanged=True))
