"""Three additional offline stage phrases; no CAN or hardware commands.

Called by rs_dance_showcase.build_library. IK is solved only during generation;
validate_extra recomputes kinematic evidence for the serialized playback curve.
"""
from __future__ import annotations

import math
import numpy as np
from scipy.optimize import least_squares

import rs_dance_design as base
from rs_record_playback import LOWER, UPPER

NODE_COUNT=31
SNAKE_AMPLITUDE_DEG=np.array([18.,20.,26.,30.,18.,12.,0.])
SNAKE_PHASE_DEG=np.array([60.,0.,-60.,-120.,-180.,-240.,0.])
SWAY_AMPLITUDE_DEG=35.
EIGHT_A_M=.10
EIGHT_B_M=.055
LARGE_EIGHT_SIZE=(.20,.12)
FAST_EIGHT_TIMING=dict(profile='smooth_cruise',ramp_fraction=.15)


def _ease(u):return np.asarray(u)**3*(10-15*np.asarray(u)+6*np.asarray(u)**2)


def _eight_progress(u,timing=None):
    if timing is None or timing=='theta=2*pi*smoothstep5(t/duration)':
        return float(_ease(u))
    if not isinstance(timing,dict) or timing.get('profile')!='smooth_cruise':
        raise ValueError('unknown figure-eight timing profile')
    ramp=float(timing['ramp_fraction'])
    if not math.isfinite(ramp) or not 0<ramp<=.5:
        raise ValueError('invalid figure-eight acceleration ramp')
    def enter(t):
        z=t/ramp
        return ramp/(1-ramp)*(2.5*z**4-3*z**5+z**6)
    u=float(np.clip(u,0,1))
    if u<ramp:return enter(u)
    if u>1-ramp:return 1-enter(1-u)
    return (u-.5*ramp)/(1-ramp)


def _kinematics(gravity):
    if not all(hasattr(gravity,k) for k in ('pin','model','data')):
        raise ValueError('generating spatial choreography requires the real URDF model')
    pin,model=gravity.pin,gravity.model;data=model.createData()
    frame=model.getFrameId('gripper_end')
    if frame>=model.nframes:raise ValueError('gripper_end frame missing')
    def pose(q):
        pin.forwardKinematics(model,data,np.asarray(q)[:6]);pin.updateFramePlacements(model,data)
        transform=data.oMf[frame]
        return transform.translation.copy(),transform.rotation.copy()
    return pin,model,data,frame,pose


def _bounds(gravity):
    return (np.maximum(np.asarray(gravity.lower)[:6],LOWER[:6])+1e-6,
            np.minimum(np.asarray(gravity.upper)[:6],UPPER[:6])-1e-6)


def _spline(points,label):
    q=np.asarray(points,float);t=np.linspace(0,1,len(q))
    curve=base.Curve(t,q,velocity_limit=1.,acceleration_limit=2.)
    duration=curve.duration*1.005
    return dict(kind='spline',label=label,duration_s=duration,
                t_s=(t*duration).tolist(),q_rad=q.tolist())


def _snake_points(idle):
    u=np.linspace(0,1,NODE_COUNT)
    carrier=4*math.pi*u[:,None]+np.radians(SNAKE_PHASE_DEG)
    q=idle+np.sin(math.pi*u[:,None])**3*np.radians(SNAKE_AMPLITUDE_DEG)*np.sin(carrier)
    q[0]=q[-1]=idle
    return q


def _stabilized_points(gravity,idle):
    pin,_,_,_,pose=_kinematics(gravity);_,reference=pose(idle)
    lo,hi=_bounds(gravity);previous=idle[3:6].copy();points=[];errors=[]
    for u in np.linspace(0,1,NODE_COUNT):
        q=idle.copy();q[0]+=math.radians(SWAY_AMPLITUDE_DEG)*math.sin(2*math.pi*float(_ease(u)))
        def residual(wrist):
            candidate=q.copy();candidate[3:6]=wrist
            return pin.log3(reference.T@pose(candidate)[1])
        solved=least_squares(residual,previous,bounds=(lo[3:6],hi[3:6]),
                             ftol=1e-12,xtol=1e-12,gtol=1e-12,max_nfev=150)
        if not solved.success:raise ValueError('world-orientation wrist IK did not converge')
        q[3:6]=solved.x;previous=solved.x
        errors.append(float(np.linalg.norm(residual(solved.x))))
        points.append(q)
    points[0]=idle.copy();points[-1]=idle.copy()
    if max(errors)>math.radians(.1):raise ValueError('world-orientation IK residual exceeds 0.1 degrees')
    return np.asarray(points),dict(kind='world_orientation_sway',frame='gripper_end',
        world_rotation_target=reference.tolist(),base_amplitude_deg=SWAY_AMPLITUDE_DEG,
        position_fixed=False,shoulder_elbow_fixed=True,carrier_cycles=1.,
        timing='theta=2*pi*smoothstep5(t/duration)',knot_orientation_error_max_deg=math.degrees(max(errors)))


def _plane():
    u=np.array([.2,.98,0.]);u/=np.linalg.norm(u)
    v=np.array([-.25,.05,.967]);v-=u*np.dot(u,v);v/=np.linalg.norm(v)
    return u,v


def _eight_target(anchor,u,size=None,timing=None):
    axis_u,axis_v=_plane();theta=2*math.pi*_eight_progress(u,timing)
    a,b=(EIGHT_A_M,EIGHT_B_M) if size is None else size
    return anchor+a*math.sin(theta)*axis_u+b*math.sin(2*theta)*axis_v


def _figure_eight_points(gravity,idle,size=None,timing=None):
    size=(EIGHT_A_M,EIGHT_B_M) if size is None else tuple(map(float,size))
    if len(size)!=2 or not all(math.isfinite(x) and x>0 for x in size):
        raise ValueError('figure-eight dimensions must be finite positive half-width/height')
    _,_,_,_,pose=_kinematics(gravity);anchor=pose(idle)[0]
    lo,hi=_bounds(gravity);previous=idle[:6].copy();points=[];errors=[]
    for u in np.linspace(0,1,NODE_COUNT):
        theta=2*math.pi*_eight_progress(u,timing);target=_eight_target(anchor,u,size,timing)
        preferred=idle[:6].copy();preferred[5]+=math.radians(15)*math.sin(theta)
        def residual(q):return np.r_[100*(pose(q)[0]-target),.10*(q-preferred)]
        solved=least_squares(residual,previous,bounds=(lo,hi),ftol=1e-11,xtol=1e-11,gtol=1e-11,max_nfev=150)
        if not solved.success:raise ValueError('figure-eight IK did not converge')
        previous=solved.x;points.append(np.r_[previous,0.])
        errors.append(float(np.linalg.norm(pose(previous)[0]-target)))
    points[0]=idle.copy();points[-1]=idle.copy()
    if max(errors)>.002:raise ValueError('figure-eight IK residual exceeds 2 mm')
    axis_u,axis_v=_plane()
    return np.asarray(points),dict(kind='cartesian_gerono_figure_eight',frame='gripper_end',
        center_xyz_m=anchor.tolist(),plane_u=axis_u.tolist(),plane_v=axis_v.tolist(),
        half_width_m=size[0],half_height_m=size[1],
        formula='center + a*sin(theta)*plane_u + b*sin(2*theta)*plane_v',
        timing=timing or 'theta=2*pi*smoothstep5(t/duration)',internal_center_crossing_fraction=.5,
        knot_position_error_max_m=max(errors),orientation_fixed=False)


def extra_recipes(gravity,idle,*,eight_size=None,eight_timing=None):
    stabilized,stabilized_design=_stabilized_points(gravity,idle)
    if eight_timing is not None:eight,eight_design=_figure_eight_points(gravity,idle,eight_size,eight_timing)
    elif eight_size is not None:eight,eight_design=_figure_eight_points(gravity,idle,eight_size)
    else:eight,eight_design=_figure_eight_points(gravity,idle)
    return {
        18:([_spline(_snake_points(idle),'traveling_snake_wave')],
            {'motion_design':dict(kind='same_frequency_phase_wave',carrier_cycles=2.,
                amplitude_deg=SNAKE_AMPLITUDE_DEG.tolist(),phase_deg=SNAKE_PHASE_DEG.tolist(),
                envelope='sin(pi*t/duration)^3',main_phase_lag_deg=60.,
                phase_order=['shoulder_lift','elbow_flex','wrist_flex','wrist_yaw','wrist_roll'])},
            ['从肩部送出蛇形波','同频波峰依次传到肘腕','连贯收波回到互动姿态']),
        19:([_spline(stabilized,'world_orientation_sway')],{'motion_design':stabilized_design},
            ['底盘向一侧大幅转动','腕部反向补偿，让头部世界朝向保持稳定','扫过另一侧回正，末端位置允许移动']),
        20:([_spline(eight,'cartesian_figure_eight')],{'motion_design':eight_design},
            ['末端画出第一侧环路','不中停地穿过中央交叉点','画出另一侧环路并闭合回原位']),
    }


def validate_extra(plan,gravity,number,idle,design=None):
    """Recompute model evidence; do not treat stored design measurements as truth."""
    if number not in (18,19,20):return {}
    if not np.allclose(plan.start,idle,rtol=0,atol=1e-9) or not np.allclose(plan.end,idle,rtol=0,atol=1e-9):
        raise ValueError('extra choreography must start and end at idle')
    times=np.linspace(0,plan.duration,1001)
    samples=[plan.at(float(t)) for t in times]
    q=np.asarray([v[0] for v in samples]);velocity=np.asarray([v[1] for v in samples])
    acceleration=np.asarray([plan.acceleration_at(float(t)) for t in times])
    if np.max(abs(q[:,6]))>1e-10 or np.max(abs(velocity[:,:6]))>1.+1e-8 or np.max(abs(acceleration[:,:6]))>2.+1e-8:
        raise ValueError('extra choreography exceeds joint dynamics or moves gripper')
    result=dict(extra_semantics_checked=False,semantic_frame='gripper_end')
    if number==18:
        stage=plan.stages[0][1]
        if (len(plan.stages)!=1 or not hasattr(stage,'q_rad')
            or not np.allclose(stage.q_rad,_snake_points(idle),rtol=0,atol=1e-9)
            or not np.allclose(stage.t_s,np.linspace(0,plan.duration,NODE_COUNT),rtol=0,atol=1e-9)):
            raise ValueError('snake wave no longer follows its common-frequency phase design')
        result.update(extra_semantics_checked=True,snake_wave=dict(carrier_cycles=2.,
            carrier_hz=2/plan.duration,phase_lag_deg=60.,phase_delay_s=plan.duration/12,
            joint_ranges_deg=np.degrees(np.ptp(q,axis=0)).tolist()))
    if not all(hasattr(gravity,k) for k in ('pin','model','data')):return result
    pin,model,data,frame,pose=_kinematics(gravity)
    actual=[];rotations=[];position_singular=[];rotation_singular=[];tool_speeds=[]
    for sample,joint_velocity in zip(q,velocity):
        xyz,rotation=pose(sample);actual.append(xyz);rotations.append(rotation)
        jac=pin.computeFrameJacobian(model,data,sample[:6],frame,pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        position_singular.append(float(np.linalg.svd(jac[:3,:],compute_uv=False)[-1]))
        rotation_singular.append(float(np.linalg.svd(jac[3:,3:6],compute_uv=False)[-1]))
        tool_speeds.append(float(np.linalg.norm(jac[:3,:]@joint_velocity[:6])))
    actual=np.asarray(actual);reference_xyz,reference_rotation=pose(idle)
    result.update(extra_semantics_checked=True,
        position_jacobian_min_singular_m_per_rad=min(position_singular),
        wrist_orientation_jacobian_min_singular=min(rotation_singular))
    if number==19:
        if np.max(abs(q[:,1:3]-idle[1:3]))>1e-8 or np.ptp(q[:,0])<math.radians(65):
            raise ValueError('stabilized sway must retain large base sweep and fixed shoulder/elbow')
        errors=np.asarray([np.linalg.norm(pin.log3(reference_rotation.T@r)) for r in rotations])
        baseline=[]
        for sample in q:
            uncompensated=sample.copy();uncompensated[3:6]=idle[3:6]
            baseline.append(float(np.linalg.norm(pin.log3(reference_rotation.T@pose(uncompensated)[1]))))
        if max(errors)>math.radians(.3):raise ValueError('stabilized world orientation exceeds 0.3 degree model error')
        result['world_orientation']=dict(max_error_deg=float(np.degrees(max(errors))),
            rms_error_deg=float(np.degrees(np.sqrt(np.mean(errors**2)))),
            uncompensated_max_error_deg=math.degrees(max(baseline)),
            uncompensated_rms_error_deg=math.degrees(math.sqrt(float(np.mean(np.asarray(baseline)**2)))),
            tool_xyz_span_m=np.ptp(actual,axis=0).tolist(),position_fixed=False,
            comparison='same base trajectory and fixed shoulder/elbow, with all wrist joints held at idle')
    elif number==20:
        dimensions=(design['motion_design'] if design is not None else
                    dict(half_width_m=EIGHT_A_M,half_height_m=EIGHT_B_M))
        size=(float(dimensions['half_width_m']),float(dimensions['half_height_m']))
        if not all(math.isfinite(x) and x>0 for x in size):
            raise ValueError('invalid figure-eight dimensions')
        timing=dimensions.get('timing')
        expected=np.asarray([_eight_target(reference_xyz,t/plan.duration,size,timing) for t in times])
        errors=np.linalg.norm(actual-expected,axis=1);axis_u,axis_v=_plane();normal=np.cross(axis_u,axis_v)
        relative=actual-reference_xyz;center_index=len(times)//2
        jac=pin.computeFrameJacobian(model,data,q[center_index,:6],frame,pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        center_speed=float(np.linalg.norm(jac[:3,:]@velocity[center_index,:6]))
        if max(errors)>.003:raise ValueError('figure-eight model path error exceeds 3 mm')
        if center_speed<.02:raise ValueError('figure-eight center crossing must not stop')
        projected=np.c_[relative@axis_u,relative@axis_v]
        areas=[]
        for lobe in (projected[:center_index+1],projected[center_index:]):
            following=np.roll(lobe,-1,axis=0)
            areas.append(float(.5*np.sum(lobe[:,0]*following[:,1]-following[:,0]*lobe[:,1])))
        if areas[0]*areas[1]>=0 or min(abs(np.asarray(areas)))<.003:
            raise ValueError('figure-eight must have two distinct, oppositely oriented lobes')
        result['figure_eight']=dict(trajectory_error_max_m=float(max(errors)),
            trajectory_error_rms_m=float(np.sqrt(np.mean(errors**2))),
            closure_error_m=float(np.linalg.norm(actual[-1]-actual[0])),
            center_crossing_error_m=float(np.linalg.norm(relative[center_index])),
            center_crossing_time_s=plan.duration/2,center_crossing_speed_m_s=center_speed,
            peak_tool_speed_m_s=max(tool_speeds),lobe_signed_area_m2=areas,
            plane_error_max_m=float(np.max(abs(relative@normal))),
            actual_width_m=float(np.ptp(relative@axis_u)),actual_height_m=float(np.ptp(relative@axis_v)),
            tool_xyz_span_m=np.ptp(actual,axis=0).tolist())
    return result
