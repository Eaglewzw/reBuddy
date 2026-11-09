"""Fixed-TCP forward/inverse kinematics and conservative operational envelope."""
from __future__ import annotations
import math
import numpy as np
from scipy.optimize import least_squares
from rs_record_playback import LOWER, UPPER
from rs_dance_pivot_primitives import (
    _config,
    orientation_at
)

def _model(gravity):
    if not all(hasattr(gravity,k) for k in ('pin','model')):raise ValueError('fixed TCP validation requires the real URDF model')
    pin,model=gravity.pin,gravity.model;frame=model.getFrameId('gripper_end')
    if frame>=model.nframes:raise ValueError('gripper_end frame missing')
    return pin,model,model.createData(),frame


def _pose(model_parts,q,cfg):
    pin,model,data,frame=model_parts
    pin.forwardKinematics(model,data,np.asarray(q)[:6]);pin.updateFramePlacements(model,data)
    transform=data.oMf[frame];rotation=transform.rotation.copy()
    return transform.translation+rotation@np.asarray(cfg['tcp_offset_m']),rotation


def tcp_pose(gravity,q,config=None):
    return _pose(_model(gravity),q,_config(config))


def _tcp_jacobian(parts,q,cfg):
    pin,model,data,frame=parts
    _,rotation=_pose(parts,q,cfg)
    jac=pin.computeFrameJacobian(model,data,np.asarray(q)[:6],frame,pin.ReferenceFrame.LOCAL_WORLD_ALIGNED).copy()
    jac[:3]-=pin.skew(rotation@np.asarray(cfg['tcp_offset_m']))@jac[3:]
    return jac


def tcp_jacobian(gravity,q,config=None):
    return _tcp_jacobian(_model(gravity),q,_config(config))


def _workspace(parts,idle,cfg):
    pin,model,data,frame=parts;_pose(parts,idle,cfg)
    joint=int(model.frames[frame].parentJoint);chain=[]
    while joint:
        chain.append(joint);joint=int(model.parents[joint])
    root=chain[-1];origin=data.oMi[root].translation.copy()
    upper=sum(np.linalg.norm(model.jointPlacements[j].translation) for j in chain if j!=root)
    upper+=np.linalg.norm(model.frames[frame].placement.translation)+np.linalg.norm(cfg['tcp_offset_m'])
    return dict(method='joint-root reach sphere intersected with joint-margin and scaled full-6D-Jacobian tests',
        sphere_origin_m=origin.tolist(),chain_length_upper_bound_m=float(upper),
        sphere_outer_radius_m=float(upper*cfg['workspace_reach_fraction']),
        sphere_inner_radius_m=float(upper*cfg['workspace_inner_fraction']),
        reach_fraction=cfg['workspace_reach_fraction'],reachable_volume_fraction_claimed=False,
        collision_checked=False,joint_margin_deg=cfg['joint_margin_deg'],
        jac_characteristic_length_m=cfg['jac_characteristic_length_m'],jac_sigma_min_required=cfg['jac_sigma_min'],
        jac_condition_max_required=cfg['jac_condition_max'])


def _check_workspace(parts,q,cfg,workspace,lower,upper):
    position,_=_pose(parts,q,cfg);radius=float(np.linalg.norm(position-workspace['sphere_origin_m']))
    if not workspace['sphere_inner_radius_m']<=radius<=workspace['sphere_outer_radius_m'] or position[2]<cfg['workspace_min_z_m']:
        raise ValueError('TCP lies outside the configured conservative operational envelope')
    margin=float(np.min(np.minimum(q[:6]-lower,upper-q[:6])))
    if margin<math.radians(cfg['joint_margin_deg'])-1e-8:raise ValueError('pivot joint margin too small')
    jac=_tcp_jacobian(parts,q,cfg);jac[:3]/=cfg['jac_characteristic_length_m']
    singular=np.linalg.svd(jac,compute_uv=False);condition=float(singular[0]/max(singular[-1],1e-15))
    if singular[-1]<cfg['jac_sigma_min'] or condition>cfg['jac_condition_max']:raise ValueError('pivot approaches a full-6D kinematic singularity')
    return radius,margin,float(singular[-1]),condition


def _solve_samples(gravity,idle,cfg,scale):
    parts=_model(gravity);pin=parts[0];anchor,reference=_pose(parts,idle,cfg)
    workspace=_workspace(parts,idle,cfg)
    if cfg['fixed_point_xyz_m'] is not None and np.linalg.norm(anchor-np.asarray(cfg['fixed_point_xyz_m']))>1e-6:
        raise ValueError('fixed_point_xyz_m must match idle TCP; change the common prepared idle pose and rebuild before selecting a different point')
    lower=np.maximum(LOWER[:6],gravity.lower);upper=np.minimum(UPPER[:6],gravity.upper)
    margin=math.radians(cfg['joint_margin_deg']);bounds=(lower+margin,upper-margin)
    _check_workspace(parts,idle,cfg,workspace,lower,upper)
    duration=cfg['beats']*60/cfg['bpm'];count=math.ceil(duration/cfg['sample_period_s'])
    sections=np.cumsum(cfg['beat_sections'])
    knot_beats=np.r_[sections[0]*np.cumsum(cfg['nod_transition_fractions'])[:2],sections]
    times=np.unique(np.round(np.r_[np.linspace(0,duration,count+1),knot_beats*60/cfg['bpm']],12))
    previous=idle[:6].copy();samples=[];peak_position=peak_orientation=0.
    for stamp in times:
        desired=orientation_at(stamp*cfg['bpm']/60,reference,cfg,scale)
        def residual(q):
            position,rotation=_pose(parts,q,cfg)
            return np.r_[(position-anchor)/cfg['jac_characteristic_length_m'],pin.log3(desired.T@rotation)]
        solved=least_squares(residual,previous,bounds=bounds,ftol=1e-11,xtol=1e-11,gtol=1e-11,max_nfev=100)
        if not solved.success:raise ValueError('previous-seed pivot IK did not converge')
        q=np.r_[solved.x,0.];position,rotation=_pose(parts,q,cfg)
        position_error=float(np.linalg.norm(position-anchor));orientation_error=float(np.linalg.norm(pin.log3(desired.T@rotation)))
        if position_error>cfg['position_tolerance_m'] or math.degrees(orientation_error)>cfg['orientation_tolerance_deg']:
            raise ValueError('pivot IK residual exceeds position/orientation tolerance')
        _check_workspace(parts,q,cfg,workspace,lower,upper)
        if np.max(abs(solved.x-previous))>.2:raise ValueError('pivot IK jumped to another solution branch')
        samples.append(q);previous=solved.x.copy();peak_position=max(peak_position,position_error);peak_orientation=max(peak_orientation,orientation_error)
    q=np.asarray(samples)
    if np.max(abs(q[-1]-idle))>1e-5:raise ValueError('pivot IK did not return to the idle branch')
    q[0]=q[-1]=idle
    return times,q,dict(workspace=workspace,tcp_anchor_m=anchor.tolist(),reference_rotation=reference.tolist(),
        ik_samples=len(times),ik_seed='previous solved six-joint state; first sample is calibrated idle',
        ik_peak_position_error_m=peak_position,ik_peak_orientation_error_deg=math.degrees(peak_orientation))


