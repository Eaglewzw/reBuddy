"""Fixed model TCP, conspicuous body motion, and unconstrained orientation.

J1 is prescribed explicitly. The other five joints solve position-only IK with
a soft posture preference. This never opens CAN and does not alter legacy 21.
"""
from __future__ import annotations

import argparse
import copy
import json
import math
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares

import rs_dance_design as base
import rs_dance_pivot_kinematics as kin
from rs_dance_pivot_trajectory import _DenseCurve
from rs_record_playback import LOWER, UPPER

BODY_CONFIG=dict(beats=8,requested_duration_s=6.,sample_period_s=.02,base_amplitude_deg=20.,
    amplitude_scales=[1.,.85,.7],phase_ramp_fraction=.15,
    shoulder_preference_deg=-10.,elbow_preference_deg=14.,wrist_flex_preference_deg=-8.,
    wrist_yaw_base_ratio=-1.4,wrist_roll_preference_deg=15.,
    position_weight=1000.,posture_weight=.05,tcp_offset_m=[0.,0.,0.],fixed_point_xyz_m=None,
    workspace_reach_fraction=.7,workspace_inner_fraction=.15,workspace_min_z_m=.15,
    joint_margin_deg=10.,jac_characteristic_length_m=.3,jac_sigma_min=.08,jac_condition_max=40.,
    position_tolerance_m=.0005,max_duration_s=10.)


def _config(config=None):
    cfg=copy.deepcopy(BODY_CONFIG)
    if config:
        if set(config)-set(cfg):raise ValueError('unknown body-pivot configuration field')
        cfg.update(copy.deepcopy(config))
    positive=('beats','requested_duration_s','base_amplitude_deg','position_weight','posture_weight',
        'joint_margin_deg','jac_characteristic_length_m','jac_sigma_min','jac_condition_max',
        'position_tolerance_m','max_duration_s')
    if any(not math.isfinite(float(cfg[k])) or cfg[k]<=0 for k in positive):raise ValueError('invalid body-pivot configuration')
    if cfg['requested_duration_s']>cfg['max_duration_s']:raise ValueError('requested body-pivot duration exceeds maximum')
    if cfg['sample_period_s']!=.02:raise ValueError('body pivot uses the existing 50 Hz controller')
    if not math.isfinite(cfg['phase_ramp_fraction']) or not 0<cfg['phase_ramp_fraction']<=.5:raise ValueError('invalid phase ramp')
    if not math.isfinite(cfg['workspace_min_z_m']) or cfg['workspace_min_z_m']<0:raise ValueError('invalid workspace height')
    if not 0<cfg['workspace_inner_fraction']<cfg['workspace_reach_fraction']<=.7:raise ValueError('invalid operational envelope')
    for key in ('shoulder_preference_deg','elbow_preference_deg','wrist_flex_preference_deg','wrist_yaw_base_ratio','wrist_roll_preference_deg'):
        if not math.isfinite(cfg[key]):raise ValueError('invalid posture preference')
    for key in ('tcp_offset_m','fixed_point_xyz_m'):
        if cfg[key] is None and key=='fixed_point_xyz_m':continue
        if np.asarray(cfg[key]).shape!=(3,) or not np.isfinite(cfg[key]).all():raise ValueError('invalid TCP coordinates')
    if not cfg['amplitude_scales'] or any(not 0<float(scale)<=1 for scale in cfg['amplitude_scales']):raise ValueError('invalid body amplitude scale')
    return cfg


def phase_at(fraction,config=None):
    cfg=_config(config);ramp=cfg['phase_ramp_fraction'];u=float(np.clip(fraction,0,1))
    def enter(t):
        x=t/ramp;return ramp/(1-ramp)*(2.5*x**4-3*x**5+x**6)
    if u<ramp:progress=enter(u)
    elif u>1-ramp:progress=1-enter(1-u)
    else:progress=(u-.5*ramp)/(1-ramp)
    return 2*math.pi*progress


def preferred_pose(fraction,idle,config=None,scale=1.):
    cfg=_config(config);theta=phase_at(fraction,cfg);sine=math.sin(theta);lift=math.sin(theta/2)**2
    q=np.asarray(idle,float).copy()
    q[0]+=math.radians(cfg['base_amplitude_deg']*scale)*sine
    q[1]+=math.radians(cfg['shoulder_preference_deg']*scale)*lift
    q[2]+=math.radians(cfg['elbow_preference_deg']*scale)*lift
    q[3]+=math.radians(cfg['wrist_flex_preference_deg']*scale)*sine
    q[4]+=math.radians(cfg['base_amplitude_deg']*scale*cfg['wrist_yaw_base_ratio'])*sine
    q[5]+=math.radians(cfg['wrist_roll_preference_deg']*scale)*sine
    q[6]=0.
    return q


class BodyPivotSegment(_DenseCurve):
    """One globally smooth dense spline, without holds or orientation locks."""
    def __init__(self,spec):
        self.config=_config(spec['config']);self.amplitude_scale=float(spec['amplitude_scale'])
        if self.amplitude_scale not in self.config['amplitude_scales']:raise ValueError('unlisted body amplitude scale')
        super().__init__(spec)
        if abs(self.duration-float(spec['duration_s']))>1e-8:raise ValueError('body-pivot duration mismatch')
        self.velocity_limit=1.;self.acceleration_limit=2.;self.gripper_velocity_limit=1.;self.gripper_acceleration_limit=2.
        self.gripper_motion=False
        if max(abs(self.lower[6]),abs(self.upper[6]))>1e-10:raise ValueError('body pivot must keep gripper closed')
        for stamp in (0,self.duration):
            if max(np.max(abs(self.at(stamp)[1])),np.max(abs(self.acceleration_at(stamp))))>1e-7:
                raise ValueError('body-pivot endpoints must have zero velocity and acceleration')


def _solve_samples(gravity,idle,cfg,scale):
    parts=kin._model(gravity);anchor,reference=kin._pose(parts,idle,cfg);workspace=kin._workspace(parts,idle,cfg)
    if cfg['fixed_point_xyz_m'] is not None and np.linalg.norm(anchor-np.asarray(cfg['fixed_point_xyz_m']))>1e-6:
        raise ValueError('fixed_point_xyz_m must match idle TCP; change the common prepared idle pose before selecting a different point')
    lower=np.maximum(LOWER[:6],gravity.lower);upper=np.minimum(UPPER[:6],gravity.upper)
    margin=math.radians(cfg['joint_margin_deg']);bounds=(lower[1:]+margin,upper[1:]-margin)
    kin._check_workspace(parts,idle,cfg,workspace,lower,upper)
    count=math.ceil(cfg['requested_duration_s']/cfg['sample_period_s']);ramp=cfg['phase_ramp_fraction']
    u=np.unique(np.round(np.r_[np.linspace(0,1,count+1),ramp,1-ramp,.5*ramp+.25*(1-ramp),.5,.5*ramp+.75*(1-ramp)],12))
    previous=np.asarray(idle[1:6],float).copy();samples=[];errors=[];steps=[]
    for fraction in u:
        preference=preferred_pose(float(fraction),idle,cfg,scale);q=np.asarray(idle,float).copy();q[0]=preference[0]
        def residual(remaining):
            candidate=q.copy();candidate[1:6]=remaining
            return np.r_[cfg['position_weight']*(kin._pose(parts,candidate,cfg)[0]-anchor),
                         cfg['posture_weight']*(remaining-preference[1:6])]
        solved=least_squares(residual,previous,bounds=bounds,ftol=1e-11,xtol=1e-11,gtol=1e-11,max_nfev=100)
        if not solved.success:raise ValueError('position-only body IK did not converge')
        q[1:6]=solved.x;q[6]=0.;error=float(np.linalg.norm(kin._pose(parts,q,cfg)[0]-anchor))
        if error>cfg['position_tolerance_m']:raise ValueError('body IK TCP position residual exceeds tolerance')
        step=float(np.max(abs(solved.x-previous)))
        if step>.2:raise ValueError('body IK changed solution branch between adjacent samples')
        kin._check_workspace(parts,q,cfg,workspace,lower,upper)
        samples.append(q);errors.append(error);steps.append(step);previous=solved.x.copy()
    poses=np.asarray(samples);raw_endpoint_error=float(np.max(abs(poses[-1]-idle)))
    if raw_endpoint_error>1e-6:raise ValueError('body IK did not return to the idle joint branch')
    # Only remove floating-point residue after checking the unmodified IK result.
    poses[0]=poses[-1]=idle
    return u,poses,dict(tcp_anchor_m=anchor.tolist(),reference_rotation=reference.tolist(),workspace=workspace,
        ik_samples=len(u),ik_seed='previous solved J2..J6; J1 prescribed analytically',
        ik_position_error_max_m=max(errors),ik_adjacent_joint_step_max_rad=max(steps),
        raw_end_joint_error_rad=raw_endpoint_error)


def build_recipe(gravity,idle,config=None):
    cfg=_config(config);idle=np.asarray(idle,float);failures=[]
    for scale in cfg['amplitude_scales']:
        try:
            u,q,evidence=_solve_samples(gravity,idle,cfg,scale)
            probe=base.Curve(u,q,velocity_limit=1.,acceleration_limit=2.)
            duration=max(cfg['requested_duration_s'],probe.duration*1.005)
            if duration>cfg['max_duration_s']:raise ValueError('body-pivot path requires a longer duration than configured maximum')
            stage=dict(kind='fixed_tcp_body',label='fixed_tcp_body_orbit',duration_s=duration,
                config=cfg,amplitude_scale=scale,t_s=(u*duration).tolist(),q_rad=q.tolist())
            plan=BodyPivotSegment(stage)
            extra=dict(body_design=dict(config=cfg,amplitude_scale=scale,position_fixed=True,orientation_fixed=False,
                model_frame='gripper_end',tcp_offset_m=cfg['tcp_offset_m'],physical_tip_calibrated=False,
                requested_base_amplitude_deg=cfg['base_amplitude_deg'],actual_base_amplitude_deg=cfg['base_amplitude_deg']*scale,
                requested_duration_s=cfg['requested_duration_s'],actual_duration_s=duration,
                actual_bpm=cfg['beats']*60/duration,recommended_playback_speed=1.,
                posture_constraint='soft J2..J6 preference only; full orientation is deliberately free',
                rejected_scales=failures,**evidence))
            validate_pivot_body(plan,gravity,idle,extra)
            return [stage],extra,['固定模型TCP，底盘明显摆向左侧','肩肘腕持续补偿，身体穿过中央摆向右侧','保持TCP位置，连贯回到互动姿态']
        except ValueError as error:failures.append(dict(amplitude_scale=scale,reason=str(error)))
    raise ValueError('no feasible fixed-TCP body motion: '+json.dumps(failures,ensure_ascii=False))


def validate_pivot_body(plan,gravity,idle,spec):
    design=spec['body_design'];cfg=_config(design['config']);scale=float(design['amplitude_scale'])
    segment=plan if isinstance(plan,BodyPivotSegment) else plan.stages[0][1]
    if not isinstance(segment,BodyPivotSegment) or segment.config!=cfg or segment.amplitude_scale!=scale:
        raise ValueError('body-pivot metadata and executable curve differ')
    if design.get('orientation_fixed') is not False or design.get('position_fixed') is not True:
        raise ValueError('body-pivot semantics must fix position and release orientation')
    if not np.allclose(plan.start,idle,rtol=0,atol=1e-8) or not np.allclose(plan.end,idle,rtol=0,atol=1e-8):
        raise ValueError('body pivot must start and end at current idle')
    parts=kin._model(gravity);pin=parts[0];anchor,reference=kin._pose(parts,idle,cfg)
    if cfg['fixed_point_xyz_m'] is not None and np.linalg.norm(anchor-np.asarray(cfg['fixed_point_xyz_m']))>1e-6:
        raise ValueError('configured fixed point does not equal current idle TCP')
    lower=np.maximum(LOWER[:6],gravity.lower);upper=np.minimum(UPPER[:6],gravity.upper);workspace=kin._workspace(parts,idle,cfg)
    if np.any(segment.lower[:6]<lower+math.radians(cfg['joint_margin_deg'])-1e-8) or np.any(segment.upper[:6]>upper-math.radians(cfg['joint_margin_deg'])+1e-8):
        raise ValueError('analytic body-pivot extrema exceed joint margin')
    times=np.linspace(0,plan.duration,max(1001,math.ceil(plan.duration/.01)+1))
    errors=[];rotations=[];speeds=[];joints=[];workspace_checks=[];base_errors=[]
    for stamp in times:
        q,v=plan.at(float(stamp));position,rotation=kin._pose(parts,q,cfg)
        expected_base=preferred_pose(stamp/plan.duration,idle,cfg,scale)[0]
        base_errors.append(abs(float(q[0]-expected_base)));errors.append(float(np.linalg.norm(position-anchor)))
        rotations.append(float(np.linalg.norm(pin.log3(reference.T@rotation))))
        speeds.append(float(np.linalg.norm(kin._tcp_jacobian(parts,q,cfg)[:3]@v[:6])))
        workspace_checks.append(kin._check_workspace(parts,q,cfg,workspace,lower,upper));joints.append(q)
    if max(errors)>cfg['position_tolerance_m']:raise ValueError('interpolated body motion no longer fixes the TCP')
    if max(base_errors)>1e-5:raise ValueError('body motion no longer follows the explicit J1 sweep')
    ranges=np.ptp(np.asarray(joints),axis=0)
    if ranges[0]<math.radians(1.99*cfg['base_amplitude_deg']*scale):raise ValueError('body base excursion is smaller than declared')
    if max(abs(segment.lower[6]),abs(segment.upper[6]))>1e-10:raise ValueError('body pivot must leave gripper closed')
    if abs(design['actual_duration_s']-plan.duration)>1e-8 or abs(design['actual_bpm']-cfg['beats']*60/plan.duration)>1e-8:
        raise ValueError('body-pivot timing metadata mismatch')
    dynamics=base._validate_plan(plan,gravity);checks=np.asarray(workspace_checks)
    return dict(pivot_body=dict(position_fixed=True,orientation_fixed=False,model_frame='gripper_end',
        tcp_offset_m=cfg['tcp_offset_m'],tcp_anchor_m=anchor.tolist(),physical_tip_calibrated=False,
        duration_s=plan.duration,bpm=cfg['beats']*60/plan.duration,joint_ranges_deg=np.degrees(ranges).tolist(),
        tcp_position_error_max_m=max(errors),tcp_position_error_rms_m=float(np.sqrt(np.mean(np.asarray(errors)**2))),
        tcp_peak_speed_m_s=max(speeds),orientation_excursion_max_deg=math.degrees(max(rotations)),
        explicit_base_error_max_deg=math.degrees(max(base_errors)),
        minimum_joint_margin_deg=math.degrees(float(min(checks[:,1]))),
        scaled_6d_jacobian_min_singular=float(min(checks[:,2])),scaled_6d_jacobian_max_condition=float(max(checks[:,3])),
        peak_velocity_rad_s=plan.peak_velocity.tolist(),peak_acceleration_rad_s2=plan.peak_acceleration.tolist(),
        inverse_dynamics_peak_nm=dynamics['inverse_dynamics_peak_nm'],workspace=workspace,
        feedback_period_s=.02,collision_checked=False))


def main(argv=None):
    from rs_gravity_record import DEFAULT_CALIBRATION,DEFAULT_URDF,preflight
    parser=argparse.ArgumentParser(description='Generate body-around-fixed-TCP motion offline; never opens CAN')
    parser.add_argument('--calibration',type=Path,default=DEFAULT_CALIBRATION);parser.add_argument('--urdf',type=Path,default=DEFAULT_URDF)
    parser.add_argument('--config',type=Path);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args(argv);gravity,identity=preflight(args.calibration,args.urdf);idle,*_=base._anchors(identity)
    stages,extra,intentions=build_recipe(gravity,idle,json.loads(args.config.read_text()) if args.config else None)
    validation=validate_pivot_body(BodyPivotSegment(stages[0]),gravity,idle,extra)
    payload=dict(stages=stages,**extra,intentions=intentions,validation=validation,identity=identity)
    args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(payload,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(validation,ensure_ascii=False,indent=2));return 0


if __name__=='__main__':raise SystemExit(main())
