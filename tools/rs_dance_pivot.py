"""Generate and validate fixed-TCP choreography offline; never opens CAN.

Public imports retain the original API. See the primitives, kinematics and
trajectory modules for orientation scheduling, IK and interpolation.
"""
from __future__ import annotations
import argparse
import json
import math
from pathlib import Path
import numpy as np
from scipy.interpolate import PPoly
import rs_dance_design as base
from rs_record_playback import LOWER, UPPER
from rs_dance_pivot_primitives import (
    CONFIG,
    _config,
    _ease,
    _back_coefficient,
    _progress,
    _slerp,
    _swing,
    orientation_at
)

from rs_dance_pivot_kinematics import (
    _model,
    _pose,
    tcp_pose,
    _tcp_jacobian,
    tcp_jacobian,
    _workspace,
    _check_workspace,
    _solve_samples
)

from rs_dance_pivot_trajectory import (
    _DenseCurve,
    PivotSegment
)

def _phase_budgets(times,q,cfg):
    sections=np.cumsum(cfg['beat_sections'])*60/cfg['bpm']
    selected=times<=sections[2]+1e-9
    spline=base.Curve(times[selected],q[selected],velocity_limit=1.,acceleration_limit=2.).spline
    result=[]
    for name,start,end in zip(('nod','cone','orientation_eight'),np.r_[0,sections[:2]],sections[:3]):
        row=dict(phase=name,start_requested_s=float(start),end_requested_s=float(end))
        for order,label in ((1,'velocity'),(2,'acceleration')):
            best=(0.,0,0.)
            for joint in range(6):
                polynomial=PPoly.from_spline((spline.t,spline.c[:,joint],5))
                roots=polynomial.derivative(order+1).roots(extrapolate=False)
                candidates=np.r_[start,end,roots[(roots>=start)&(roots<=end)]]
                values=abs(polynomial(candidates,nu=order));index=int(np.argmax(values))
                if values[index]>best[0]:best=(float(values[index]),joint+1,float(candidates[index]))
            row[label]=dict(peak=best[0],joint=best[1],at_requested_s=best[2])
        result.append(row)
    return result


def build_recipe(gravity,idle,config=None):
    cfg=_config(config);idle=np.asarray(idle,float);failures=[]
    for scale in cfg['amplitude_scales']:
        try:
            times,q,evidence=_solve_samples(gravity,idle,cfg,scale)
            sections=np.cumsum(cfg['beat_sections']);requested_duration=cfg['beats']*60/cfg['bpm']
            hold_start=sections[2]*60/cfg['bpm'];hold_end=sections[3]*60/cfg['bpm']
            first=np.flatnonzero(times<=hold_start+1e-10);last=np.flatnonzero(times>=hold_end-1e-10)
            held=q[first[-1]].copy();q[last[0]]=held
            motion=[(times[first],q[first]),(times[last]-hold_end,q[last])]
            factor=max(1.,*(1/base.Curve(t,pose,velocity_limit=1.,acceleration_limit=2.).scale for t,pose in motion))
            if factor>1:factor*=1.005
            duration=requested_duration*factor
            if duration>cfg['max_duration_s']:raise ValueError('pivot timing exceeds configured maximum duration')
            chunks=[dict(kind='dense_curve',t_s=(motion[0][0]*factor).tolist(),q_rad=motion[0][1].tolist()),
                dict(kind='hold',duration_s=(hold_end-hold_start)*factor,q_rad=held.tolist()),
                dict(kind='dense_curve',t_s=(motion[1][0]*factor).tolist(),q_rad=motion[1][1].tolist())]
            stage=dict(kind='fixed_tcp_pivot',label='fixed_tcp_16_beat',duration_s=duration,
                       config=cfg,amplitude_scale=scale,chunks=chunks)
            segment=PivotSegment(stage)
            extra=dict(pivot_design=dict(config=cfg,amplitude_scale=scale,requested_bpm=cfg['bpm'],actual_bpm=cfg['beats']*60/duration,
                requested_duration_s=requested_duration,actual_duration_s=duration,uniform_time_factor=factor,
                requested_phase_dynamic_peaks=_phase_budgets(times,q,cfg),
                model_frame='gripper_end',tcp_offset_m=cfg['tcp_offset_m'],pointing_axis_local=cfg['pointing_axis_local'],
                physical_tip_calibrated=False,physical_pointing_axis_calibrated=False,
                requested_nod_deg=cfg['nod_deg'],actual_nod_deg=cfg['nod_deg']*scale,
                requested_cone_half_angle_deg=cfg['cone_deg'],actual_cone_half_angle_deg=cfg['cone_deg']*scale,
                phase_beats={name:[float(start),float(end)] for name,start,end in zip(('nod','cone','orientation_eight','hold','return'),np.r_[0,sections[:-1]],sections)},
                overshoot=dict(kind='smoothstep5(u)+K*u^3*(1-u)^3; K solves actual peak excess',ratio=cfg['overshoot_ratio'],scope='first two nod transitions; true peak then settle to nominal target'),
                rejected_scales=failures,**evidence))
            # Use the same real-FK and dynamics gate during generation and load.
            validate_pivot(segment,gravity,idle,extra)
            return [stage],extra,['固定模型TCP点头，平滑进入锥形起点','模型TCP +Z姿态轴绕竖直轴画锥形，再画姿态8字','保持两拍，然后平滑回到互动姿态']
        except ValueError as error:failures.append(dict(amplitude_scale=scale,reason=str(error)))
    raise ValueError('no feasible fixed-TCP amplitude: '+json.dumps(failures,ensure_ascii=False))


def validate_pivot(plan,gravity,idle,spec):
    design=spec['pivot_design'];cfg=_config(design['config']);scale=float(design['amplitude_scale'])
    segment=plan if isinstance(plan,PivotSegment) else plan.stages[0][1]
    if not isinstance(segment,PivotSegment) or segment.config!=cfg or segment.amplitude_scale!=scale:
        raise ValueError('pivot metadata does not match executable segment')
    if not np.allclose(plan.start,idle,rtol=0,atol=1e-8) or not np.allclose(plan.end,idle,rtol=0,atol=1e-8):raise ValueError('pivot must return to current idle')
    parts=_model(gravity);pin=parts[0];anchor,reference=_pose(parts,idle,cfg);workspace=_workspace(parts,idle,cfg)
    if cfg['fixed_point_xyz_m'] is not None and np.linalg.norm(anchor-np.asarray(cfg['fixed_point_xyz_m']))>1e-6:
        raise ValueError('fixed_point_xyz_m must match idle TCP; change the common prepared idle pose and rebuild before selecting a different point')
    lower=np.maximum(LOWER[:6],gravity.lower);upper=np.minimum(UPPER[:6],gravity.upper)
    if np.any(segment.lower[:6]<lower+math.radians(cfg['joint_margin_deg'])-1e-8) or np.any(segment.upper[:6]>upper-math.radians(cfg['joint_margin_deg'])+1e-8):
        raise ValueError('analytic pivot extrema violate joint margin')
    times=np.unique(np.r_[np.linspace(0,plan.duration,max(1001,math.ceil(plan.duration/.01)+1)),segment.ends])
    position_errors=[];orientation_errors=[];workspace_samples=[];cone_angles=[]
    for stamp in times:
        q,v=plan.at(float(stamp));position,rotation=_pose(parts,q,cfg)
        desired=orientation_at(stamp/plan.duration*cfg['beats'],reference,cfg,scale)
        position_errors.append(float(np.linalg.norm(position-anchor)))
        orientation_errors.append(float(np.linalg.norm(pin.log3(desired.T@rotation))))
        workspace_samples.append(_check_workspace(parts,q,cfg,workspace,lower,upper))
        if cfg['beat_sections'][0]<=stamp/plan.duration*cfg['beats']<=sum(cfg['beat_sections'][:2]):
            axis=rotation@np.asarray(cfg['pointing_axis_local'])
            cone_angles.append(math.degrees(math.acos(float(np.clip(axis[2],-1,1)))))
    if max(position_errors)>cfg['position_tolerance_m'] or math.degrees(max(orientation_errors))>cfg['orientation_tolerance_deg']:
        raise ValueError('interpolated pivot no longer preserves its fixed TCP/desired orientation')
    hold_start,hold_end=segment.hold_interval_s
    hold_q=plan.at((hold_start+hold_end)/2)[0]
    for stamp in np.linspace(hold_start,hold_end,101):
        q,v=plan.at(float(stamp))
        if max(np.max(abs(q-hold_q)),np.max(abs(v)),np.max(abs(plan.acceleration_at(float(stamp)))))>1e-7:raise ValueError('pivot hold is not exactly stationary')
    dynamics=base._validate_plan(plan,gravity);checks=np.asarray(workspace_samples)
    actual_bpm=cfg['beats']*60/plan.duration
    if abs(actual_bpm-design['actual_bpm'])>1e-8 or abs(plan.duration-design['actual_duration_s'])>1e-8:
        raise ValueError('pivot timing metadata mismatch')
    return dict(pivot=dict(tcp_frame='gripper_end',tcp_offset_m=cfg['tcp_offset_m'],tcp_anchor_m=anchor.tolist(),
        physical_tip_calibrated=False,pointing_axis_local=cfg['pointing_axis_local'],model_pointing_axis_only=True,
        requested_bpm=cfg['bpm'],actual_bpm=actual_bpm,amplitude_scale=scale,
        position_error_max_m=max(position_errors),position_error_rms_m=float(np.sqrt(np.mean(np.asarray(position_errors)**2))),
        orientation_error_max_deg=math.degrees(max(orientation_errors)),
        cone_half_angle_min_deg=min(cone_angles),cone_half_angle_max_deg=max(cone_angles),
        hold_interval_s=[hold_start,hold_end],hold_duration_s=hold_end-hold_start,
        workspace=workspace,minimum_joint_margin_deg=math.degrees(float(min(checks[:,1]))),
        scaled_6d_jacobian_min_singular=float(min(checks[:,2])),scaled_6d_jacobian_max_condition=float(max(checks[:,3])),
        feedback_period_s=.02,validation_period_s=float(np.max(np.diff(times))),collision_checked=False,
        inverse_dynamics_peak_nm=dynamics['inverse_dynamics_peak_nm']))


def main(argv=None):
    from rs_gravity_record import DEFAULT_CALIBRATION,DEFAULT_URDF,preflight
    parser=argparse.ArgumentParser(description='Offline fixed-TCP choreography generation; never opens CAN')
    parser.add_argument('--calibration',type=Path,default=DEFAULT_CALIBRATION);parser.add_argument('--urdf',type=Path,default=DEFAULT_URDF)
    parser.add_argument('--config',type=Path);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args(argv);gravity,identity=preflight(args.calibration,args.urdf);idle,*_=base._anchors(identity)
    stages,extra,intentions=build_recipe(gravity,idle,json.loads(args.config.read_text()) if args.config else None)
    payload=dict(stages=stages,**extra,intentions=intentions,identity=identity)
    args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(payload,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(extra['pivot_design'],ensure_ascii=False,indent=2))
    return 0


if __name__=='__main__':raise SystemExit(main())


