"""Two continuous full-amplitude sways with world-orientation compensation."""
import math
import numpy as np
from scipy.optimize import least_squares

import rs_dance_motion as basic
import rs_dance_spatial as extra
from rs_dance_virtual_collision import CollisionAudit


def recipe(gravity,idle):
    pin,_,_,_,pose=extra._kinematics(gravity)
    reference=pose(idle)[1];lower,upper=extra._bounds(gravity)
    previous=idle[3:6].copy();points=[]
    # Short smooth ramps, constant phase speed through both cycles. There is
    # no stop or return-to-idle segment between the two repetitions.
    timing=dict(profile='smooth_cruise',ramp_fraction=.1)
    for u in np.linspace(0,1,31):
        q=idle.copy()
        q[0]+=math.radians(35)*math.sin(4*math.pi*extra._eight_progress(u,timing))
        def residual(wrist):
            candidate=q.copy();candidate[3:6]=wrist
            return pin.log3(reference.T@pose(candidate)[1])
        solved=least_squares(residual,previous,bounds=(lower[3:6],upper[3:6]),
            ftol=1e-12,xtol=1e-12,gtol=1e-12,max_nfev=150)
        if not solved.success or np.linalg.norm(residual(solved.x))>math.radians(.1):
            raise ValueError('double sway wrist orientation IK failed')
        q[3:6]=solved.x
        if np.max(abs(q[3:6]-previous))>.5:raise ValueError('double sway wrist branch jump')
        previous=solved.x;points.append(q)
    points[0]=idle.copy();points[-1]=idle.copy()
    return [extra._spline(points,'continuous_double_steady_sway')]


def make_spec(gravity,idle,identity):
    stages=recipe(gravity,idle);plan=basic.DancePlan(stages)
    checks=basic._validate_plan(plan,gravity)
    checks.update(extra.validate_extra(plan,gravity,19,idle))
    collision=CollisionAudit(gravity)
    times=np.linspace(0,plan.duration,int(np.ceil(plan.duration/.005))+1)
    for t in times:
        if collision.check(plan.at(t)[0])['colliding']:
            raise ValueError('double steady sway self collision')
    checks.update(collision_checked=True,collision_samples=len(times),environment_collision_checked=False)
    return dict(schema='rebuddy.selected_steady.v1',number=8,name='稳定摇摆',
        action_id=None,gripper_motion=False,hardware_accepted=False,firmware_integrated=False,
        description='底盘左右各35度连续摆动两个来回，腕部补偿世界朝向；中间不停顿，最后回待机。',
        calibration_sha256=identity['calibration_sha256'],urdf_sha256=identity['urdf_sha256'],
        duration_s=plan.duration,start_rad=plan.start.tolist(),end_rad=plan.end.tolist(),
        motion_design=dict(kind='world_orientation_sway',carrier_cycles=2.,base_amplitude_deg=35.,
            timing=dict(profile='smooth_cruise',ramp_fraction=.1),position_fixed=False,shoulder_elbow_fixed=True),
        collision_identity=collision.identity,stages=stages,validation=checks)
