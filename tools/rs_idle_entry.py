"""Checked idle approach and continuous handoff to gravity teaching.

Startup preparation runs before enable. Clip-end returns validate their gravity
path incrementally while holding the measured stop pose, one step per tick.
Reuses the CSV player's curve/controller and the recorder's teaching law.
"""
import math

import numpy as np

from rs_gravity_play import Controller, prepare_entry, ARRIVAL_TOLERANCE
from rs_gravity_record import CAPS, Control, vector


class IdleEntry:
    def __init__(self, gravity, initial, pose, *, from_teaching=None, via_zero=False, zero_tolerance_deg=2., settle_s=.5):
        if not math.isfinite(settle_s) or not .5 <= settle_s <= 2.:
            raise ValueError('invalid idle settle duration')
        self.settle_s = settle_s
        self.initial = vector(initial).copy()
        self.target = vector(pose['q_rad']).copy()
        if via_zero and from_teaching is not None:
            raise ValueError('clip return must not detour through zero')
        self.zero_target = self.initial.copy()
        self.zero_target[:6] = 0.
        self.zero_completed = False
        # Same arrival tolerance as the prepared VR exit-zero configuration.
        # This is distinct from the 3-degree measurement margin at hard limits.
        if not math.isfinite(zero_tolerance_deg) or not 0 < zero_tolerance_deg <= 3.:
            raise ValueError('invalid startup zero tolerance')
        self.zero_tolerance = math.radians(zero_tolerance_deg)
        self.idle_curve = (prepare_entry(self.zero_target,self.target,gravity,.5,hold_gripper=False)
                           if via_zero else None)
        self.curve = prepare_entry(self.initial, self.zero_target if via_zero else self.target, gravity, .5, hold_gripper=False,
                                   validate_gravity=from_teaching is None)
        self.planned_approach_s = self.curve.duration + (self.idle_curve.duration if self.idle_curve is not None else 0.)
        self.gravity = gravity
        self.from_teaching = from_teaching
        # During live recording, spread path checks across control ticks while
        # holding the measured stop pose; never block compensation to plan.
        self.validation = (iter(np.linspace(0.,self.curve.duration,math.ceil(self.curve.duration/.02)+1))
                           if from_teaching is not None else None)
        self.position = Controller(gravity,initial=self.initial)
        self.position.command(self.initial, 0, self.initial, np.zeros(7))
        self.position = Controller(gravity,initial=self.initial)
        self.teaching = Control(gravity, self.target)
        self.phase = 'hold'
        self.phase_start = None
        self.stable_since = None
        self.done = False
        self.gravity_gain = 0.

    def step(self, q, stamp):
        if self.done:
            raise RuntimeError('idle entry already completed')
        if self.phase_start is None:
            self.phase_start = stamp
        elapsed = (stamp - self.phase_start) / 1e9
        motion_target = self.zero_target if self.idle_curve is not None else self.target
        target, velocity = motion_target, np.zeros(7)
        self.gravity_gain = 1.
        if self.phase == 'hold':
            target = self.initial
            self.gravity_gain = min(1., elapsed / 1.5)
        elif self.phase == 'approach':
            target, velocity = self.curve.at(elapsed)
        command = self.position.command(q, stamp, target, velocity, self.gravity_gain)
        phase = self.phase
        if phase == 'hold' and self.from_teaching is not None:
            teach = self.from_teaching.step(q,stamp,1.)
            weight = min(1.,elapsed/1.5)
            # KP/KD from the position controller are already ramped; retain
            # full gravity and blend out teaching damping without a torque dip.
            for key in ('kp','kd','feedforward_nm'):
                command[key] = (np.array(command[key]) + (1.-weight)*np.array(teach[key])).tolist()
            self.gravity_gain = 1.
            estimated = (np.array(command['feedforward_nm']) + np.array(command['kp'])*(target-q)
                         - np.array(command['kd'])*self.position.velocity)
            if np.any(abs(estimated)>CAPS):
                raise RuntimeError('teaching-to-idle handoff exceeds torque budget')
        if self.validation is not None:
            for _ in range(16):
                stamp_to_check=next(self.validation,None)
                if stamp_to_check is None:
                    self.validation=None;break
                planned,_=self.curve.at(stamp_to_check)
                if np.any(abs(vector(self.gravity(planned[:6]),6))>CAPS[:6]):
                    raise RuntimeError('自动补回待机路径的重力补偿超出力矩预算')
        if phase == 'hold' and elapsed >= 1.5 and self.validation is None:
            self.phase = 'approach'; self.phase_start = stamp
        elif phase == 'approach' and elapsed >= self.curve.duration:
            self.phase = 'settle'; self.phase_start = stamp
        elif phase == 'settle':
            tolerance = self.zero_tolerance if self.idle_curve is not None else ARRIVAL_TOLERANCE
            if (np.max(abs(motion_target - q)) <= tolerance
                    and np.max(abs(self.position.velocity)) <= math.radians(1.)):
                if self.stable_since is None:
                    self.stable_since = stamp
                if stamp - self.stable_since >= self.settle_s * 1e9:
                    if self.idle_curve is not None:
                        self.zero_completed=True
                        self.curve=self.idle_curve;self.idle_curve=None
                        self.phase='approach';self.stable_since=None
                    else:
                        self.phase = 'release'
                    self.phase_start = stamp
            else:
                self.stable_since = None
            if self.phase == 'settle' and elapsed >= 5.:
                stage = '启动回零' if self.idle_curve is not None else '自动待机'
                errors = ', '.join(f'J{i+1}={value:+.3f}°' for i,value in enumerate(np.degrees(q-motion_target)))
                speed = math.degrees(float(np.max(abs(self.position.velocity))))
                raise RuntimeError(f'{stage}未稳定到位；误差 [{errors}]；允许 ±{math.degrees(tolerance):g}°，'
                                   f'最大估算速度 {speed:.3f}°/s（需 ≤1°/s 持续 0.5 秒）；未开始拖动录制')
        elif phase == 'release':
            teach = self.teaching.step(q, stamp, 1.)
            weight = min(1., elapsed)
            for key in ('kp', 'kd', 'feedforward_nm'):
                command[key] = ((1. - weight) * np.array(command[key]) + weight * np.array(teach[key])).tolist()
            estimated = (np.array(command['feedforward_nm']) + np.array(command['kp']) * (target - q)
                         - np.array(command['kd']) * self.position.velocity)
            if np.any(abs(estimated) > CAPS):
                raise RuntimeError('idle-to-teaching handoff exceeds torque budget')
            if weight == 1.:
                command = teach
                self.done = True
        return command, phase
