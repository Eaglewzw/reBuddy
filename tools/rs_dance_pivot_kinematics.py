"""Fixed-point kinematics and operating-envelope checks."""
import math
import numpy as np

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


def _tcp_jacobian(parts,q,cfg):
    pin,model,data,frame=parts
    _,rotation=_pose(parts,q,cfg)
    jac=pin.computeFrameJacobian(model,data,np.asarray(q)[:6],frame,pin.ReferenceFrame.LOCAL_WORLD_ALIGNED).copy()
    jac[:3]-=pin.skew(rotation@np.asarray(cfg['tcp_offset_m']))@jac[3:]
    return jac


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

