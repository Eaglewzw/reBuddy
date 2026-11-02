"""Adapt the existing VR zero-return routine to the recorder's checked backend.

Prepare before enabling: imports, configuration and model loading must not pause
an active compensation loop. No hardware is opened by this module.
"""
from pathlib import Path
from types import SimpleNamespace
import hashlib
import math
import time

import numpy as np
import yaml

from lerobot_teleoperator_rebot_vr.constants import JOINT_NAMES
from lerobot_teleoperator_rebot_vr.control.mit import MITCommandDispatcher
from lerobot_teleoperator_rebot_vr.runtime.safety import move_to_zero_pose
from rs_record_playback import MEASUREMENT_TOLERANCE_RAD, REFERENCE

DEFAULT_CONFIG = REFERENCE / 'config/rs_mit_split.yaml'
NAMES = list(JOINT_NAMES)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


class _Motor:
    """Collect the reference dispatcher's seven commands into one checked batch."""
    def __init__(self, io, index):
        self.io, self.index = io, index

    def send_mit(self, position, velocity, kp, kd, torque):
        if self.index != len(self.io.pending):
            raise RuntimeError('zero-return command order mismatch')
        self.io.pending.append([position, velocity, kp, kd, torque])
        if len(self.io.pending) == 7:
            self.io.commit()


class _IO:
    def __init__(self, backend, gravity, config, journal, verify, should_stop):
        self.backend, self.gravity, self.config = backend, gravity, config
        self.journal, self.verify, self.should_stop = journal, verify, should_stop
        self.pending = []
        self.motors = {name: _Motor(self, i) for i, name in enumerate(NAMES)}
        self.q = None
        self.read_begin = None
        self.last_read = None

    def get_observation(self):
        if self.should_stop():
            raise RuntimeError('zero return interrupted')
        self.verify()
        self.backend.heartbeat()
        q, begin, end = self.backend.read()
        q = np.asarray(q, dtype=float)
        if q.shape != (7,) or not np.all(np.isfinite(q)):
            raise RuntimeError('zero return needs seven finite measured positions')
        if (end < begin or end - begin > 100_000_000
                or time.monotonic_ns() - begin > 100_000_000
                or (self.last_read is not None and not 0 < end - self.last_read <= 100_000_000)):
            raise RuntimeError('zero-return feedback exceeded 100 ms')
        if (np.any(q[:6] < self.gravity.lower - MEASUREMENT_TOLERANCE_RAD)
                or np.any(q[:6] > self.gravity.upper + MEASUREMENT_TOLERANCE_RAD)):
            raise RuntimeError('zero-return feedback outside model bounds')
        self.q, self.read_begin, self.last_read = q, begin, end
        self.pending = []
        return {f'{name}.pos': float(np.degrees(q[i])) for i, name in enumerate(NAMES)}

    def commit(self):
        values = np.asarray(self.pending, dtype=float)
        self.pending = []
        if values.shape != (7, 5) or not np.all(np.isfinite(values)):
            raise RuntimeError('invalid zero-return command')
        if self.should_stop():
            raise RuntimeError('zero return interrupted before command')
        if self.read_begin is None or time.monotonic_ns() - self.read_begin > 100_000_000:
            raise RuntimeError('zero-return sample expired before command')
        command = dict(zip(('target_rad', 'velocity_rad_s', 'kp', 'kd', 'feedforward_nm'), values.T.tolist()))
        self.journal.put(dict(event='zero_return_command', monotonic_ns=time.monotonic_ns(),
                              actual_rad=self.q.tolist(), **command))
        self.backend.send(command)

    def motor_feedback_telemetry(self):
        # Do not trigger another SDK read for the reference routine's optional log.
        return dict(feedback_freshness='checked_mechpos_request_response')


class ZeroReturn:
    def __init__(self, backend, gravity, metadata, args, journal, *, config_path=DEFAULT_CONFIG,
                 exit_speed_rad_s=None,exit_acceleration_rad_s2=None,exit_tolerance_deg=None):
        self.backend, self.gravity, self.journal = backend, gravity, journal
        self.config_path = Path(config_path)
        data = yaml.safe_load(self.config_path.read_text())
        mit, startup, motion, runtime, gripper = (
            data[k] for k in ('mit', 'startup', 'motion_limits', 'runtime', 'gripper'))
        if data['robot']['robot_model'] != 'b601_rs':
            raise ValueError('zero return requires the RS reference configuration')
        self.paths = [(Path(args.calibration), metadata['calibration_sha256']),
                      (Path(args.urdf), metadata['urdf_sha256']),
                      (self.config_path, sha(self.config_path))]
        self.verify(force=True)
        keys = ('exit_zero_tolerance_deg', 'exit_zero_speed_rad_s', 'exit_zero_acceleration_rad_s2',
                'initial_move_tolerance_deg', 'initial_move_timeout', 'initial_stall_timeout')
        options = {k: startup[k] for k in keys}
        if exit_tolerance_deg is not None:
            if not math.isfinite(exit_tolerance_deg) or not 0<exit_tolerance_deg<=3:
                raise ValueError('invalid zero-return tolerance override')
            options['exit_zero_tolerance_deg']=exit_tolerance_deg
        options.update({k: motion[k] for k in ('max_joint_speed_rad_s', 'max_joint_acceleration_rad_s2',
                                               'wrist_speed_rad_s', 'wrist_acceleration_rad_s2',
                                               'max_relative_target_deg', 'wrist_relative_target_deg')})
        options.update(robot_model='b601_rs', fps=runtime['fps'], status_rate=runtime['status_rate'],
                       exit_zero_stall_timeout=startup.get('exit_zero_stall_timeout'))
        for value,key,maximum in (
                (exit_speed_rad_s,'exit_zero_speed_rad_s',min(motion['max_joint_speed_rad_s'],motion['wrist_speed_rad_s'])),
                (exit_acceleration_rad_s2,'exit_zero_acceleration_rad_s2',min(motion['max_joint_acceleration_rad_s2'],motion['wrist_acceleration_rad_s2']))):
            if value is not None:
                if not math.isfinite(value) or not 0<value<=maximum:
                    raise ValueError(f'invalid zero-return override: {key}, reference maximum {maximum}')
                options[key]=value
        for key, value in options.items():
            if key != 'robot_model' and value is not None and (not math.isfinite(value) or value <= 0):
                raise ValueError(f'invalid zero-return setting: {key}')
        self.args = SimpleNamespace(**options)
        config = SimpleNamespace(joint_limits={name: (float(np.degrees(lo)), float(np.degrees(hi)))
                                              for name, lo, hi in zip(NAMES[:6], gravity.lower, gravity.upper)},
                                 max_relative_target=motion['max_relative_target_deg'],
                                 gripper_control_mode='mit', gripper_mit_kp=gripper['gripper_mit_kp'],
                                 gripper_mit_kd=gripper['gripper_mit_kd'])
        self.cancel = lambda: False
        self.io = _IO(backend, gravity, config, journal, self.verify, lambda: self.cancel())
        self.robot = MITCommandDispatcher(
            self.io, kp=np.asarray(mit['mit_kp']), kd=np.asarray(mit['mit_kd']),
            torque_limit_nm=np.asarray(mit['mit_torque_limit_nm']),
            arm_velocity_limit_rad_s=np.full(6, options['exit_zero_speed_rad_s']),
            gravity_scale=mit['mit_gravity_scale'], gravity_ramp_s=0.,
            dynamics_urdf=args.urdf, robot_model='b601_rs')
        self.kp, self.kd = self.robot.kp.copy(), self.robot.kd.copy()
        journal.put(dict(event='zero_return_policy', reference='runtime.safety.move_to_zero_pose',
                         config=str(self.config_path), config_sha256=self.paths[-1][1],
                         motion_options=options, kp=self.kp.tolist(), kd=self.kd.tolist(),
                         gripper='hold entry opening', gain_transition_s=.5))

    def verify(self, force=False):
        now = time.monotonic()
        if force or now >= getattr(self, 'next_verify', 0.):
            for path, expected in self.paths:
                if sha(path) != expected:
                    raise RuntimeError(f'zero-return calibration/model/config changed: {path}')
            self.next_verify = now + 1.

    def run(self, should_stop, *, initial_gravity_gain=1., initial_kp=None, initial_kd=None):
        if not math.isfinite(initial_gravity_gain) or not 0 <= initial_gravity_gain <= 1:
            raise ValueError('invalid initial gravity ramp')
        initial_kp = np.zeros_like(self.kp) if initial_kp is None else np.asarray(initial_kp, dtype=float)
        initial_kd = np.zeros_like(self.kd) if initial_kd is None else np.asarray(initial_kd, dtype=float)
        for gains, expected in ((initial_kp, self.kp), (initial_kd, self.kd)):
            if gains.shape != expected.shape or not np.all(np.isfinite(gains)) or np.any(gains < 0):
                raise ValueError('invalid initial position/damping gains')
        self.cancel = should_stop
        self.verify(force=True)
        self.io.last_read = None
        # Gravity is already active. Keep it continuous while gradually engaging
        # position/damping gains at the measured pose, before the reference move.
        hold = self.robot.get_observation()
        started = time.monotonic()
        final_gravity_scale = self.robot.gravity_scale
        while True:
            if should_stop():
                return False
            self.robot.get_observation()
            gain = min(1., (time.monotonic() - started) / .5)
            self.robot.kp = initial_kp * (1. - gain) + self.kp * gain
            self.robot.kd = initial_kd * (1. - gain) + self.kd * gain
            self.robot.gravity_scale = initial_gravity_gain + (final_gravity_scale - initial_gravity_gain) * gain
            self.robot.send_action(hold)
            if gain == 1.:
                break
            time.sleep(1. / self.args.fps)
        return move_to_zero_pose(
            self.robot, lower_limit_rad=self.gravity.lower, upper_limit_rad=self.gravity.upper,
            args=self.args, should_stop=should_stop,
            write_row=lambda row: self.journal.put(dict(event='zero_return_motion', **row)))


def prepare_zero_return(backend, gravity, metadata, args, journal, *,
                        exit_speed_rad_s=None,exit_acceleration_rad_s2=None,exit_tolerance_deg=None):
    return ZeroReturn(backend, gravity, metadata, args, journal,
                      exit_speed_rad_s=exit_speed_rad_s,exit_acceleration_rad_s2=exit_acceleration_rad_s2,
                      exit_tolerance_deg=exit_tolerance_deg)
