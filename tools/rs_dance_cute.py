"""Three expressive interaction phrases, with optional existing J7 beat pulses."""
import numpy as np

import rs_dance_motion as basic
from rs_dance_plan import DancePlan,fast_wide_gripper_cycle
from rs_dance_virtual_collision import CollisionAudit


DESIGNS={
    '02':dict(name='歪头招手',description='歪头左右招手，两次夹爪短拍像打招呼，连续回到待机。',
        grip=True,offsets_deg=[
            [0,0,0,0,0,0,0],[-10,-4,6,6,-18,-14,0],[-6,-6,8,10,20,-8,0],
            [10,-4,6,4,-20,14,0],[6,-5,8,10,18,8,0],[0,0,0,0,0,0,0]]),
    '05':dict(name='探头撒娇',description='向前探头，左右轻歪头，轻点后收回；夹爪保持闭合。',
        grip=False,offsets_deg=[
            [0,0,0,0,0,0,0],[0,12,-8,8,0,0,0],[-8,20,-12,12,-16,-12,0],
            [8,20,-12,6,16,12,0],[0,10,-5,-8,0,0,0],[0,0,0,0,0,0,0]]),
    '06':dict(name='开心欢呼',description='身体连续两次扬起欢呼，配合左右轻倾与两次夹爪开合；不定格。',
        grip=True,offsets_deg=[
            [0,0,0,0,0,0,0],[-6,-12,18,12,-6,-10,0],[0,6,-7,-10,0,0,0],
            [6,-14,20,15,6,10,0],[0,5,-6,-8,0,0,0],[0,0,0,0,0,0,0]]),
}


def recipe(number,idle):
    design=DESIGNS[number]
    stage=basic._retime_fast_spline(dict(label='cute_'+number,t_s=[0,.2,.4,.6,.8,1.],
        q_rad=(idle+np.radians(design['offsets_deg'])).tolist()))
    if not design['grip']:return [stage]
    # Keep the accepted arm clock; two 2-second J7 pulses share this timeline.
    duration=max(stage['duration_s'],6.526990816987241)
    stage['t_s']=(np.asarray(stage['t_s'])*duration/stage['duration_s']).tolist()
    stage['duration_s']=duration
    return [dict(kind='body_gripper_fast_wide_overlay',label='cute_'+number+'_with_gripper',
        duration_s=duration,body_stages=[stage],pulse_stages=fast_wide_gripper_cycle(idle),
        pulse_starts_s=[.25*duration-1.,.75*duration-1.])]


def make_spec(gravity,idle,identity,number):
    design=DESIGNS[number];stages=recipe(number,idle)
    open_deg=90. if design['grip'] else 45.
    plan=DancePlan(stages,gripper_motion=design['grip'])
    checks=basic._validate_plan(plan,gravity)
    collision=CollisionAudit(gravity)
    times=np.linspace(0,plan.duration,int(np.ceil(plan.duration/.005))+1)
    for t in times:
        q=plan.at(t)[0].copy();q[6]=0.
        if collision.check(q)['colliding']:
            raise ValueError('cute phrase closed-finger model self collision')
    # The CAD model only has calibrated closed fingers. This proxy cannot
    # establish collision freedom for an opening motor-driven gripper.
    checks.update(collision_checked=not design['grip'],
        closed_gripper_proxy_checked=True,collision_samples=len(times),
        moving_fingers_collision_checked=False,environment_collision_checked=False)
    return dict(schema='rebuddy.selected_cute.v1',number=int(number),name=design['name'],
        action_id=None,gripper_motion=design['grip'],hardware_accepted=False,firmware_integrated=False,
        description=design['description'],
        calibration_sha256=identity['calibration_sha256'],urdf_sha256=identity['urdf_sha256'],
        duration_s=plan.duration,start_rad=plan.start.tolist(),end_rad=plan.end.tolist(),
        gripper_control=dict(motor_id=7,closed_deg=0.,open_deg=open_deg,physical_max_deg=250.,
            pulses=2 if design['grip'] else 0,
            pulse_duration_s=sum(s['duration_s'] for s in stages[0]['pulse_stages']) if design['grip'] else 2.,
            pulse_starts_s=stages[0].get('pulse_starts_s',[]),
            velocity_limit_rad_s=plan.gripper_velocity_limit if design['grip'] else basic.GRIPPER_VMAX,
            acceleration_limit_rad_s2=plan.gripper_acceleration_limit if design['grip'] else basic.GRIPPER_AMAX),
        collision_identity=collision.identity,stages=stages,validation=checks)
