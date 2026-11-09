"""Emphatic nods and rebounds for the selected dance library; offline only."""
import numpy as np

import rs_dance_motion as basic
from rs_dance_virtual_collision import CollisionAudit


def recipe(idle, count=2):
    # A single continuous spline replaces two small nods and their slow tails.
    # Shoulder/elbow stay still; the gripper remains closed throughout.
    if count!=2:raise ValueError('current nod phrase contains two nods')
    angles=[0.,-28.,12.,-28.,12.,0.]
    times=[0.,.22,.44,.66,.88,1.]
    q=np.tile(idle,(len(times),1))
    q[:,3]+=np.radians(angles)
    return [basic._retime_fast_spline(dict(label='deep_nod_rebound',
        t_s=times,q_rad=q.tolist()))]


def make_spec(gravity,idle,identity,count=2):
    stages=recipe(idle,count);plan=basic.DancePlan(stages)
    checks=basic._validate_plan(plan,gravity)
    collision=CollisionAudit(gravity)
    times=np.linspace(0,plan.duration,int(np.ceil(plan.duration/.005))+1)
    for t in times:
        if collision.check(plan.at(t)[0])['colliding']:
            raise ValueError('deep nod self collision')
    checks.update(collision_checked=True,collision_samples=len(times),
                  environment_collision_checked=False)
    curve=plan.stages[0][1]
    return dict(schema='rebuddy.selected_nod.v1',number=1,name='点头打拍',
        action_id=None,gripper_motion=False,hardware_accepted=False,firmware_integrated=False,
        description='连续两次深点头与上扬回弹，第二次结束后回中；肩肘保持待机，两拍之间不回位停顿。',
        calibration_sha256=identity['calibration_sha256'],urdf_sha256=identity['urdf_sha256'],
        duration_s=plan.duration,start_rad=plan.start.tolist(),end_rad=plan.end.tolist(),
        wrist_excursion_deg=np.degrees([curve.lower[3]-idle[3],curve.upper[3]-idle[3]]).tolist(),
        collision_identity=collision.identity,stages=stages,validation=checks)
