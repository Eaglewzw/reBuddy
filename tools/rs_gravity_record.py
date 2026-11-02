#!/usr/bin/env python3
"""RS gravity-assisted teaching. Default: offline check; never replay recordings.

Control law follows the supplied MuJoCo example's Pinocchio g(q), with
finite-difference velocity damping applied ONCE in feedforward (MIT kd=0).
During teaching the gripper holds the idle opening (or initial opening for manual
startup) and arm kp is zero.
Normal quit/first Ctrl+C uses the VR position-controlled zero return before disable.
"""
from __future__ import annotations

import argparse
from collections import deque
from concurrent.futures import Future
import csv
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import queue
import re
import signal
import sys
import threading
import time

import numpy as np
from rs_record_playback import MEASUREMENT_TOLERANCE_RAD

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CALIBRATION = Path.home() / '.cache/huggingface/lerobot/calibration/robots/rebot_b601_rs_follower/rebot_b601_rs_vr.json'
DEFAULT_URDF = Path('/home/verser/Python/lerobot-teleoperator-rebot-vr/src/lerobot_teleoperator_rebot_vr/urdf/rebot_b601_rs.urdf')
NAMES = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_yaw', 'wrist_roll', 'gripper']
# Shared software estimate budgets for recording, entry, CSV and daily playback.
# These are not measured torque or motor firmware limits. RS06/RS00 peak specs
# (36/14 Nm) are ceilings, not continuous operating targets. Keep J7 lower for
# the gripper mechanism. Raised 50% at the operator's request on 2026-11-02.
CAPS = np.array([18., 18., 18., 4.5, 4.5, 4.5, 1.5])
DAMPING = np.array([.2, .2, .2, .2, .2, .2, .05])
MAX_GAP = .1
PERIOD = .02
POSE_MATCH_RAD = math.radians(2.)
TEMPERATURE_RETURN_C = 100.
IDLE_ROUND_TRIPS = {'idle', 'probe', 'nuzzle', 'tilt', 'happy'}


class TemperatureReturnRequested(RuntimeError):
    """Soft thermal exit, raised only after all feedback integrity checks pass."""
    def __init__(self, motors):
        self.motors = motors
        super().__init__('temperature return requested: ' + json.dumps(motors))


class JournalError(RuntimeError):
    """Storage failure, distinct from feedback/control failures."""


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def vector(value, size=7):
    result = np.asarray(value, dtype=float)
    if result.shape != (size,) or not np.all(np.isfinite(result)):
        raise ValueError(f'expected {size} finite values')
    return result


def load_idle_pose(path, metadata):
    pose = json.loads(path.read_text())
    if pose.get('schema') != 'rebuddy.idle_pose.v1' or pose.get('name') != 'ATTENTIVE':
        raise ValueError('待机预设格式不匹配')
    for key in ('calibration_sha256', 'urdf_sha256', 'mock'):
        if key not in pose or pose[key] != metadata.get(key):
            raise ValueError(f'待机预设与本次会话不匹配：{key}')
    vector(pose['q_rad'])
    return pose


def capture_idle_pose(history, metadata, folder):
    if not history or history[-1][0] - history[0][0] < 950_000_000:
        raise ValueError('请保持待机姿态至少 1 秒，再输入 pose idle')
    times = np.asarray([stamp for stamp, q in history], dtype=np.int64)
    positions = np.asarray([vector(q) for stamp, q in history])
    if np.any(np.diff(times) <= 0) or np.any(np.diff(times) > int(MAX_GAP * 1e9)):
        raise ValueError('待机姿态采样不连续，请保持后重试')
    spread = np.ptp(positions, axis=0)
    if np.any(spread > math.radians(.5)):
        raise ValueError('最近 1 秒姿态仍在移动（波动超过 0.5°），请保持后重试')
    return dict(schema='rebuddy.idle_pose.v1', name='ATTENTIVE',
                created_at=datetime.now(timezone.utc).isoformat(), source_session=str(folder),
                calibration_sha256=metadata['calibration_sha256'], urdf_sha256=metadata['urdf_sha256'],
                mock=metadata['mock'], joint_names=NAMES, q_rad=np.mean(positions, axis=0).tolist(),
                samples=len(history), spread_deg=np.degrees(spread).tolist(),
                source_start_ns=int(times[0]), source_end_ns=int(times[-1]),
                hardware_accepted=False, purpose='shared idle anchor; not an executable move or zero calibration')


def require_idle_match(q, pose):
    error = np.degrees(vector(q) - vector(pose['q_rad']))
    outside = np.flatnonzero(np.abs(error) > math.degrees(POSE_MATCH_RAD))
    if len(outside):
        detail = ', '.join(f'J{i+1} 偏差 {error[i]:+.2f}°' for i in outside)
        raise ValueError(f'尚未回到待机预设（允许 ±2°）：{detail}；请手动回位，补偿继续')


def clip_anchor(name, pose, q):
    # Canonical names allow takes such as probe_2 and happy-3.
    action = re.split(r'[_-]', name.lower())[0]
    if action not in IDLE_ROUND_TRIPS | {'rest', 'wake'}:
        return None
    if pose is None:
        raise ValueError('请先设置待机预设：摆好姿态并保持 1 秒，输入 pose idle')
    if action != 'wake':
        require_idle_match(q, pose)
    return dict(action=action, idle_pose=pose, entry_idle=action != 'wake', exit_idle=action != 'rest')


class Gravity:
    """Compatibility wrapper over the user's VR dynamics implementation."""

    def __init__(self, urdf):
        from lerobot_teleoperator_rebot_vr.control.dynamics import B601GravityCompensator
        self.engine = B601GravityCompensator(urdf)
        self.pin = self.engine._pin
        self.model = self.engine.model
        self.data = self.engine.data
        self.lower = self.model.lowerPositionLimit.copy()
        self.upper = self.model.upperPositionLimit.copy()

    def __call__(self, q):
        return self.engine.gravity_torque(vector(q, 6))


def preflight(calibration, urdf):
    cal = json.loads(calibration.read_text())
    if (cal.get('status') != 'complete' or cal.get('robot_type') != 'rebot_b601_rs_follower'
            or cal.get('zeroed_joints') != NAMES):
        raise ValueError('complete seven-axis RS calibration required')
    for i, name in enumerate(NAMES, 1):
        if cal.get('motors', {}).get(name) != dict(id=i, host_id=253, model='rs-06' if i <= 3 else 'rs-00'):
            raise ValueError(f'calibration motor identity mismatch: {name}')
    if cal.get('model_sha256') != digest(urdf):
        raise ValueError('URDF does not match calibration model identity')
    from rs_record_playback import validate_driver_version
    version = validate_driver_version()  # No controller/socket is created.
    gravity = Gravity(urdf)
    return gravity, dict(
        calibration_sha256=digest(calibration), calibration=cal,
        urdf_sha256=digest(urdf), urdf=str(urdf.resolve()), motorbridge=version,
        python=sys.executable, coordinate='raw motor radians; same identity mapping as calibrated RS model',
        gravity_source='lerobot_teleoperator_rebot_vr.control.dynamics.B601GravityCompensator',
        feedback_source='lerobot_teleoperator_rebot_vr.hardware.rs.RSConnection.read_positions -> motorbridge',
        status_policy='periodic reports for startup only; MIT command replies during control',
        control=dict(rate_hz=50, arm_kp=0, mit_kd=0, damping_nm_s_rad=DAMPING.tolist(),
                     torque_budget_nm=CAPS.tolist(), max_sample_gap_s=MAX_GAP,
                     max_measured_speed_rad_s=None,
                     speed_policy='record measured speed; no speed threshold or warning',
                     measurement_tolerance_deg=math.degrees(MEASUREMENT_TOLERANCE_RAD),
                     gripper='hold saved idle opening after automatic entry; otherwise initial measured opening'),
    )


class Control:
    def __init__(self, gravity, initial):
        self.gravity = gravity
        self.initial = vector(initial).copy()
        self.previous = None
        self.previous_ns = None
        self.velocity = np.zeros(7)

    def step(self, q, now_ns, ramp):
        q = vector(q)
        if np.any(q[:6] < self.gravity.lower - MEASUREMENT_TOLERANCE_RAD) or np.any(q[:6] > self.gravity.upper + MEASUREMENT_TOLERANCE_RAD):
            raise ValueError('measured arm position outside model bounds')
        if abs(q[6] - self.initial[6]) > math.radians(2):
            raise ValueError('gripper moved more than 2 degrees; this version teaches arm motion only')
        raw_velocity = np.zeros(7)
        if self.previous is not None:
            dt = (now_ns - self.previous_ns) / 1e9
            if not 0 < dt <= MAX_GAP:
                raise ValueError('feedback interval outside 0..100 ms')
            raw_velocity = (q - self.previous) / dt
            alpha = dt / (.04 + dt)
            self.velocity += alpha * (raw_velocity - self.velocity)
        self.previous = q.copy()
        self.previous_ns = now_ns
        g = vector(self.gravity(q[:6]), 6)
        # Never silently clip insufficient gravity compensation.
        if np.any(np.abs(g) > CAPS[:6]):
            raise ValueError('gravity exceeds torque budget')
        ff = np.r_[g * ramp, 0.] - DAMPING * self.velocity
        kp = np.zeros(7)
        kp[6] = 5. * ramp
        target = q.copy()
        target[6] = self.initial[6]
        estimated_total = ff + kp * (target - q)
        if np.any(np.abs(estimated_total) > CAPS):
            raise ValueError('gravity plus damping/hold exceeds torque budget')
        return dict(q_rad=q.tolist(), dq_raw_rad_s=raw_velocity.tolist(),
                    dq_est_rad_s=self.velocity.tolist(),
                    gravity_nm=g.tolist(), feedforward_nm=ff.tolist(),
                    target_rad=target.tolist(), kp=kp.tolist(), kd=[0.] * 7)


class Journal:
    """Bounded asynchronous disk writer; control thread never waits for storage."""

    def __init__(self, path):
        self.path = path
        self.file = path.open('x', encoding='utf-8')
        self.queue = queue.Queue(maxsize=4096)
        self.error = None
        self.failure = None
        self.allow_failure = False
        self.thread = threading.Thread(target=self._write, daemon=True)
        self.thread.start()

    def _write(self):
        try:
            with self.file:
                while True:
                    row = self.queue.get()
                    if row is None:
                        break
                    if isinstance(row, tuple):
                        path, value, future = row
                        temporary = path.with_name(path.name + f'.{time.time_ns()}.tmp')
                        try:
                            path.parent.mkdir(parents=True, exist_ok=True)
                            with temporary.open('x', encoding='utf-8') as output:
                                output.write(json.dumps(value, ensure_ascii=False, allow_nan=False, indent=2) + '\n')
                                output.flush()
                                os.fsync(output.fileno())
                            os.replace(temporary, path)
                            future.set_result(value)
                        except BaseException as exc:
                            future.set_exception(JournalError(f'pose save failed: {exc}'))
                            raise
                        finally:
                            temporary.unlink(missing_ok=True)
                        continue
                    self.file.write(json.dumps(row, ensure_ascii=False, allow_nan=False) + '\n')
                    self.file.flush()
        except BaseException as exc:
            self.error = exc

    def fail(self, error):
        # Once storage fails, stop enqueueing. Keep only the first error in
        # memory; neither feedback decoding nor bounded homing depends on disk.
        if self.failure is None:
            self.failure = f'{self.path.name}: {error}'
        if not self.allow_failure:
            raise JournalError(self.failure)

    def check(self):
        if self.error:
            self.fail(self.error)
        elif self.failure:
            self.fail(self.failure)

    def put(self, row):
        self.check()
        if self.failure:
            return
        try:
            self.queue.put_nowait(row)
        except queue.Full:
            self.fail('journal queue full')

    def save_pose(self, path, value):
        future = Future()
        self.put((path, value, future))
        return future

    def close(self):
        # Called only after hardware stop; bounded even on a blocked disk.
        deadline = time.monotonic() + 3
        while self.thread.is_alive() and not self.error:
            try:
                self.queue.put(None, timeout=.05)
                break
            except queue.Full:
                if time.monotonic() > deadline:
                    raise RuntimeError('journal close timed out')
        self.thread.join(timeout=3)
        if self.thread.is_alive() or self.error:
            raise RuntimeError(f'journal incomplete: {self.error or "writer timeout"}')


class Clips:
    def __init__(self, journal):
        self.journal = journal
        self.active = None
        self.anchor = None
        self.used = set()

    def start(self, name, now_ns, anchor=None):
        if self.active:
            raise ValueError('当前片段尚未 stop')
        if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]{0,63}', name) or name in self.used:
            raise ValueError('名称须为未使用的英文/数字/下划线/短横线，最多64字符')
        self.journal.put(dict(event='clip_start', clip=name, monotonic_ns=now_ns, anchor=anchor))
        self.active = name
        self.anchor = anchor
        self.used.add(name)

    def stop(self, now_ns, reason='operator_stop', q=None):
        if self.active:
            if reason == 'operator_stop' and self.anchor and self.anchor['exit_idle']:
                require_idle_match(q, self.anchor['idle_pose'])
            self.journal.put(dict(event='clip_end', clip=self.active, monotonic_ns=now_ns, reason=reason))
            self.active = None
            self.anchor = None


class LiveBackend:
    """Reuse existing supported startup/watchdog, never the raw recording player."""

    def __init__(self, port, raw_journal, *, supported=False):
        if not supported:
            raise ValueError('实机录制需要确认防坠支撑可独立承重（--supported）')
        self.supported = supported
        self.returning_to_zero = False
        self.raw_journal = raw_journal
        from rs_record_playback import Hardware
        from rs_feedback_audit import Monitor
        # Monitor expects a text writer; move its disk I/O to the bounded journal.
        class Sink:
            def write(self, line):
                raw_journal.put(json.loads(line))
        self.hw = None
        self.monitor = Monitor(port, Sink())
        try:
            self.hw = Hardware(port)
            self.hw.reports()
        except BaseException:
            self.close()
            raise
        self.mode = 0

    def read(self):
        self.monitor.drain()
        begin_wall = time.time_ns()
        begin = time.monotonic_ns()
        try:
            q = self.hw.positions()  # One seven-axis mechPos request batch. No zero MIT reads.
        finally:
            # Previously a rejected SDK value skipped this drain, losing the
            # very raw reply needed to distinguish bus data from SDK corruption.
            self.monitor.drain()
            writer = getattr(self, 'raw_journal', None)
            batch = getattr(self.hw, 'last_read', None)
            if writer is not None and batch is not None:
                writer.put(dict(event='sdk_position_batch', begin_unix_ns=begin_wall,
                                finished_unix_ns=time.time_ns(), **batch))
        self.hw.check_status(self.mode)
        self.monitor.drain()
        end_wall = time.time_ns()
        end = time.monotonic_ns()
        if end - begin > int(MAX_GAP * 1e9):
            raise RuntimeError('feedback batch exceeded 100 ms')
        hot_motors = []
        for mid, value in enumerate(q, 1):
            raw = self.monitor.positions.get(mid)
            state = self.monitor.status.get(mid)
            if raw is None or not begin_wall <= raw['received_unix_ns'] <= end_wall:
                raise RuntimeError(f'motor {mid}: missing fresh raw position reply')
            if abs(value - raw['position_rad']) > 1e-7:
                raise RuntimeError(f'motor {mid}: SDK/raw position mismatch: ' + json.dumps(dict(
                    sdk_rad=float(value), raw_rad=raw['position_rad'],
                    raw_received_unix_ns=raw['received_unix_ns'],
                    raw_faults=self.monitor.faults[-3:]), ensure_ascii=False))
            if state is None or not 0 <= end_wall - state['received_unix_ns'] <= 100_000_000:
                age_ms = None if state is None else (end_wall-state['received_unix_ns'])/1e6
                raise RuntimeError(f'motor {mid}: stale independent status: age_ms={age_ms}, state={state}')
            if state['mode'] != self.mode or state['fault_bits']:
                raise RuntimeError(f'motor {mid}: unexpected mode or fault')
            if not math.isfinite(state['temperature_c']):
                raise RuntimeError(f'motor {mid}: invalid temperature')
            if state['temperature_c'] >= TEMPERATURE_RETURN_C:
                hot_motors.append(dict(motor_id=mid, temperature_c=state['temperature_c']))
            # Status and mechPos replies are asynchronous. Their position delta
            # cannot be validated using an assumed hand-guiding speed ceiling.
            # SDK position is still checked against its fresh raw parameter reply
            # above; status freshness, mode and faults stay checked during homing.
        if self.monitor.faults:
            raise RuntimeError('raw CAN fault/anomaly recorded: ' +
                               json.dumps(self.monitor.faults[-3:], ensure_ascii=False))
        # Never mask a bad reply/fault on a later axis with a thermal exit on
        # an earlier one. The thermal threshold requests one bounded zero return;
        # motor-reported faults still abort that motion on every read.
        if hot_motors and not getattr(self, 'returning_to_zero', False):
            raise TemperatureReturnRequested(hot_motors)
        return q, begin, end

    def enable(self, q):
        if not self.supported:
            raise ValueError('实机录制需要确认防坠支撑可独立承重（--supported）')
        self.monitor.accepted_modes = {0, 2}
        self.hw.enable_supported_trial(q, fall_arrest_confirmed=self.supported,
                                       progress=self._startup_feedback)
        self.hw.stop_reports()
        if self.raw_journal is not None:
            self.raw_journal.put(dict(event='periodic_reports_disable_sent',
                                     unix_ns=time.time_ns()))
        self.monitor.drain()
        self.mode = 2
        self.monitor.accepted_modes = {2}

    def _startup_feedback(self):
        # Startup used to leave this socket unread during watchdog spawn and
        # enable. Its queue could retain mode-0 reports and lose enable replies.
        # Drain throughout the transition; never discard a latched real fault.
        self.monitor.drain()
        if self.monitor.faults:
            raise RuntimeError('raw CAN fault/anomaly during enable: ' +
                               json.dumps(self.monitor.faults[-3:], ensure_ascii=False))

    def send(self, command):
        for i, motor in enumerate(self.hw.motors):
            motor.send_mit(command['target_rad'][i], command.get('velocity_rad_s', [0.] * 7)[i],
                           command['kp'][i], command['kd'][i], command['feedforward_nm'][i])

    def heartbeat(self):
        self.hw.heartbeat()

    def disable(self):
        self.monitor.accepted_modes = {0, 2}
        try:
            return self.hw.disable()
        finally:
            self.monitor.drain()

    def close(self):
        try:
            if self.hw is not None:
                self.hw.close()
        finally:
            self.monitor.close()


class MockBackend:
    """Synthetic teaching input; NOT a physics or hardware validation."""
    def __init__(self):
        self.started = time.monotonic()
        self.enabled = False

    def read(self):
        begin = time.monotonic_ns()
        t = time.monotonic() - self.started
        q = np.array([0., .7, 1.1, .03 * math.sin(t), 0., 0., 0.])
        return q, begin, time.monotonic_ns()

    def enable(self, q):
        self.enabled = True

    def send(self, command):
        if not self.enabled:
            raise RuntimeError('mock not enabled')

    def heartbeat(self):
        pass

    def disable(self):
        self.enabled = False
        return True

    def close(self):
        pass


def export_clips(folder):
    """Export measured rows, retaining actual irregular timestamps and abnormal ends."""
    clips = {}
    for line in (folder / 'session.jsonl').read_text().splitlines():
        row = json.loads(line)
        if row['event'] == 'clip_start':
            clips[row['clip']] = dict(start_ns=row['monotonic_ns'], end_reason='unclosed',
                                     anchor=row.get('anchor'), samples=[])
        elif row['event'] == 'sample' and row.get('clip') in clips:
            clips[row['clip']]['samples'].append(row)
        elif row['event'] == 'clip_end':
            clips[row['clip']]['end_reason'] = row['reason']
        elif row['event'] == 'clip_manual_end' and row.get('clip') in clips:
            clips[row['clip']].update(manual_end_ns=row['monotonic_ns'],
                                     manual_end_rad=row['q_rad'],auto_return=True)
    result = []
    for name, clip in clips.items():
        samples = clip.pop('samples')
        path = folder / f'{name}.csv'
        with path.open('x', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['t_s', 'monotonic_ns', 'read_start_ns', 'read_end_ns'] +
                            [f'q{i}_rad' for i in range(1, 8)] + [f'dq{i}_estimated_rad_s' for i in range(1, 8)] +
                            [f'ff{i}_nm' for i in range(1, 8)])
            for row in samples:
                writer.writerow([(row['monotonic_ns'] - clip['start_ns']) / 1e9,
                                 row['monotonic_ns'], row['read_start_ns'], row['read_end_ns'],
                                 *row['q_rad'], *row['dq_est_rad_s'], *row['feedforward_nm']])
        result.append(dict(name=name, file=path.name, samples=len(samples), sha256=digest(path),
                           complete=clip['end_reason'] == 'operator_stop' and len(samples) >= 2,
                           hardware_accepted=False, **clip))
    return result


def read_commands(commands):
    while True:
        line = sys.stdin.readline()
        if not line:
            commands.put('eof')
            return
        commands.put(line.strip())


def run(args, gravity, metadata, *, backend_factory=None, commands=None, zero_return_factory=None, idle_entry_factory=None):
    live = args.mode == 'record'
    if live and not args.supported:
        raise ValueError('record 需要 --supported：已安装可靠防坠支撑，退出/故障会失能')
    if not math.isfinite(args.seconds) or (args.seconds < 0 if live else not 0 < args.seconds <= 600):
        raise ValueError('--seconds: record requires >= 0 (0 = no time limit); mock requires (0, 600]')
    folder = args.output / (datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ') + f'-{time.time_ns()}')
    folder.mkdir(parents=True, exist_ok=False)
    metadata = dict(metadata, schema='rebuddy.gravity_record.v1', mock=not live,
                    hardware_accepted=False, supported_disable_policy=live,
                    sample_semantics='seven sequential measured positions, not an atomic snapshot',
                    session_limit_s=args.seconds or None,
                    temperature_exit_c=TEMPERATURE_RETURN_C,
                    startup_policy='saved idle: verified arm zero, then seven-axis idle, then gravity teaching; --manual-start skips automatic startup',
                    exit_policy='quit, EOF, first SIGINT/SIGTERM, storage failure, explicit session timeout or temperature-only exit: return arm J1-J6 to calibrated zero, keep gripper opening, verify then disable; '
                                'repeated termination signal/control fault or failed zero return: disable onto installed support')
    (folder / 'metadata.json').write_text(json.dumps(metadata, ensure_ascii=False, indent=2) + '\n')
    pose_path = args.output / 'presets' / ('idle.json' if live else 'idle.mock.json')
    idle_pose = None
    if pose_path.exists():
        try:
            idle_pose = load_idle_pose(pose_path, metadata)
            print(f'已载入待机预设：{pose_path}', flush=True)
        except (ValueError, KeyError, TypeError) as error:
            print(f'{error}。可用 pose idle replace 重新设置。', flush=True)
    pending_pose = None
    history = deque()
    journal = Journal(folder / 'session.jsonl')
    try:
        raw = Journal(folder / 'can.jsonl')
    except BaseException:
        journal.close()
        raise
    clips = Clips(journal)
    backend = None
    zero_return = None
    idle_entry = None
    returning_clip = False
    last_gravity_gain = 0.
    last_kp, last_kd = np.zeros(6), np.zeros(6)
    attempted_enable = False
    stop = [None]
    zero_return_active = [False]
    graceful_exits = ('quit', 'eof', f'signal_{signal.SIGINT}', f'signal_{signal.SIGTERM}',
                      'session_timeout', 'temperature', 'logging_error')
    handlers = {}
    result = dict(success=False, enable_requested=False, disable_requested=False,
                  disable_confirmed=False, zero_return_requested=False, zero_return_completed=False,
                  idle_entry_requested=False, idle_entry_completed=False,
                  startup_zero_requested=False, startup_zero_completed=False,
                  folder=str(folder), mock=not live, errors=[])
    if commands is None:
        commands = queue.Queue()
        if live:
            threading.Thread(target=read_commands, args=(commands,), daemon=True).start()
    try:
        if threading.current_thread() is threading.main_thread():
            def request_stop(number, frame):
                if zero_return_active[0] or stop[0] in graceful_exits or stop[0] == 'zero_return_abort':
                    stop[0] = 'zero_return_abort'
                else:
                    stop[0] = f'signal_{number}'
            for sig in (signal.SIGINT, signal.SIGTERM):
                handlers[sig] = signal.signal(sig, request_stop)
        backend = backend_factory() if backend_factory else (LiveBackend(args.port, raw, supported=args.supported) if live else MockBackend())
        if live:
            if zero_return_factory is None:
                from rs_zero_return import prepare_zero_return
                zero_return_factory = prepare_zero_return
            # Load reference code/config/model BEFORE enabling, never in finally.
            zero_return = zero_return_factory(backend, gravity, metadata, args, journal)
            if idle_entry_factory is None:
                from rs_idle_entry import IdleEntry
                idle_entry_factory = IdleEntry
        initial, _, _ = backend.read()
        control = Control(gravity, initial)
        # Validate complete gravity budget even before enable and before soft ramp.
        control.step(initial, time.monotonic_ns(), 1.)
        for _ in range(3):
            time.sleep(PERIOD)
            q, _, _ = backend.read()
            if np.max(abs(q - initial)) > math.radians(.2):
                raise ValueError('initial pose moving; support it before enabling')
        if live and idle_pose is not None and not getattr(args, 'manual_start', False):
            if idle_entry_factory is None:
                from rs_idle_entry import IdleEntry
                idle_entry_factory = IdleEntry
            zero_tolerance_deg = getattr(getattr(zero_return, 'args', None), 'exit_zero_tolerance_deg', 2.)
            idle_entry = idle_entry_factory(gravity, initial, idle_pose,via_zero=True,
                                            zero_tolerance_deg=zero_tolerance_deg)
            result['idle_entry_requested'] = True
            result['startup_zero_requested'] = True
            journal.put(dict(event='idle_entry_plan', initial_rad=initial.tolist(),
                             target_rad=idle_pose['q_rad'], via_zero=True,
                             zero_tolerance_deg=zero_tolerance_deg,
                             approach_s=idle_entry.planned_approach_s))
            q, _, _ = backend.read()
            if np.max(abs(q - initial)) > math.radians(.2):
                raise ValueError('规划待机路径期间姿态发生变化；未使能')
        if stop[0]:
            raise RuntimeError('cancelled before enable')
        if live and digest(args.calibration) != metadata['calibration_sha256']:
            raise ValueError('calibration changed during setup')
        attempted_enable = True  # Even partial enable must enter disable cleanup.
        result['enable_requested'] = True
        journal.put(dict(event='enable_requested', monotonic_ns=time.monotonic_ns(), unix_ns=time.time_ns()))
        backend.enable(initial)
        # Only a completed enable may recover through a controlled return.
        # Before/within partial enable, storage errors still prevent movement.
        if live:
            journal.allow_failure = raw.allow_failure = True
        journal.put(dict(event='enable_completed', monotonic_ns=time.monotonic_ns(), unix_ns=time.time_ns()))
        control = Control(gravity, initial)
        start = time.monotonic()
        elapsed = 0.  # A termination signal may arrive just as enable completes.
        deadline = start
        previous = start
        next_identity_check = start
        ready = False
        ready_since = None
        if live:
            print('会话不设自动结束时限；使用 quit 或 Ctrl+C 回零退出。' if args.seconds == 0 else
                  f'会话时限：就绪后 {args.seconds:g} 秒，到时结束采集并回零退出。', flush=True)
            if idle_entry is not None:
                print(f'即将先回机械零位（J1～J6），再到已保存的七轴待机姿态（运动约 {idle_entry.planned_approach_s:.1f} 秒）；请等提示“可拖动录制”后再拖动。', flush=True)
            else:
                print('重力补偿启动中（1.5秒渐入），保持支撑。就绪后 pose idle / start 名称 / stop / status；quit 或 Ctrl+C 回零后退出。', flush=True)
        while not stop[0]:
            tick = time.monotonic()
            if tick - previous > MAX_GAP:
                raise RuntimeError('control deadline exceeded 100 ms')
            previous = tick
            elapsed = tick - start
            if ready_since is not None and args.seconds > 0 and tick - ready_since >= args.seconds:
                if not live:
                    clips.stop(time.monotonic_ns())
                stop[0] = 'session_timeout'
                break
            backend.heartbeat()
            try:
                q, read_start, read_end = backend.read()
            except TemperatureReturnRequested as thermal:
                if not live:
                    raise
                result['temperature_exit'] = thermal.motors
                journal.put(dict(event='temperature_exit', motors=thermal.motors,
                                 monotonic_ns=time.monotonic_ns()))
                print(f'温度达到 {TEMPERATURE_RETURN_C:g}°C，结束录制并回到机械校零位后失能。', flush=True)
                # A concurrent abort/termination still takes precedence.
                stop[0] = stop[0] or 'temperature'
                break
            # Keep rejected measurements in the journal before deriving commands.
            journal.put(dict(event='measurement', monotonic_ns=read_end, read_start_ns=read_start,
                             read_end_ns=read_end, q_rad=np.asarray(q).tolist()))
            automatic = idle_entry is not None and not idle_entry.done
            if automatic:
                command, entry_phase = idle_entry.step(q, read_end)
                command_gravity_gain = idle_entry.gravity_gain
            else:
                command_gravity_gain = 1. if idle_entry is not None else min(1., elapsed / 1.5)
                command = control.step(q, read_end, command_gravity_gain)
            if time.monotonic_ns() - read_start > int(MAX_GAP * 1e9):
                raise RuntimeError('sample expired before command')
            if live and tick >= next_identity_check:
                if digest(args.calibration) != metadata['calibration_sha256']:
                    raise ValueError('calibration changed during session')
                next_identity_check = tick + 1.
            for writer in (journal, raw):
                writer.check()
            if live and (journal.failure or raw.failure):
                stop[0] = stop[0] or 'logging_error'
                print('日志写入异常，结束采集并回零；录制数据将标记为不完整。', flush=True)
                break
            backend.send(command)
            last_gravity_gain = command_gravity_gain
            last_kp, last_kd = np.array(command['kp'][:6]), np.array(command['kd'][:6])
            if automatic:
                journal.put(dict(event='clip_return_command' if returning_clip else 'idle_entry_command', phase=entry_phase,
                                 monotonic_ns=read_end, **command))
                if not returning_clip and idle_entry.zero_completed and not result['startup_zero_completed']:
                    result['startup_zero_completed']=True
                    journal.put(dict(event='startup_zero_completed',monotonic_ns=read_end,q_rad=q.tolist()))
                    print('机械零位已稳定到达，继续移动到保存的待机姿态。',flush=True)
                if idle_entry.done:
                    control = idle_entry.teaching
                    if not returning_clip:
                        result['idle_entry_completed'] = True
                        journal.put(dict(event='idle_entry_completed', monotonic_ns=read_end))
            if ready:
                history.append((read_end, q.copy()))
                while history and read_end - history[0][0] > 1_000_000_000:
                    history.popleft()
            if pending_pose is not None and pending_pose.done():
                try:
                    idle_pose = pending_pose.result()
                except JournalError as error:
                    journal.fail(error)
                    stop[0] = stop[0] or 'logging_error'
                    break
                pending_pose = None
                journal.put(dict(event='idle_pose_saved', path=str(pose_path), pose=idle_pose))
                print(f'待机预设已保存：{pose_path}；补偿继续，保存不会自动锁定或移动机械臂。', flush=True)
            if not ready and (idle_entry.done if idle_entry is not None else elapsed >= 1.5):
                ready = True
                if ready_since is None:
                    ready_since = tick
                if idle_entry is not None:
                    print('待机已到位，已切换为重力补偿，可拖动录制。输入 start 名称；quit 或 Ctrl+C 回零退出。', flush=True)
                else:
                    print(('就绪。已载入待机预设；摆回待机后输入 start idle / start probe。'
                       '需要更新预设时保持 1 秒后输入 pose idle replace。') if idle_pose is not None else
                      '就绪。先摆好待机姿态，保持 1 秒后输入 pose idle；然后 start probe 等录制动作。', flush=True)
                if not live:
                    clips.start(args.name, read_end)
            if automatic and returning_clip:
                # Store measured return motion with continuous timestamps, not
                # synthetic waypoints. Playback recomputes gravity as usual.
                journal.put(dict(event='sample', phase='auto_return', monotonic_ns=read_end,
                                 read_start_ns=read_start,read_end_ns=read_end,clip=clips.active,
                                 q_rad=q.tolist(),dq_est_rad_s=idle_entry.position.velocity.tolist(),
                                 feedforward_nm=command['feedforward_nm']))
                if idle_entry.done:
                    clips.stop(read_end,q=q)
                    returning_clip=False
                    history.clear()
                    print('已自动补回待机，片段完整结束；重力补偿继续，可录制下一动作。',flush=True)
            elif not automatic:
                journal.put(dict(event='sample', monotonic_ns=read_end, read_start_ns=read_start,
                                 read_end_ns=read_end, clip=clips.active, **command))
            # At most one user command per cycle; input never blocks compensation.
            try:
                line = commands.get_nowait()
            except queue.Empty:
                line = ''
            if line:
                parts = line.split()
                try:
                    if parts[0] == 'start' and len(parts) == 2:
                        if not ready:
                            raise ValueError('请等待自动待机与重力补偿切换完成，提示就绪后再输入 start')
                        if pending_pose is not None:
                            raise ValueError('正在保存待机预设，请稍后重试')
                        anchor = clip_anchor(parts[1], idle_pose, q)
                        clips.start(parts[1], read_end, anchor)
                        print(f'开始片段：{parts[1]}', flush=True)
                    elif line in ('pose idle', 'pose idle replace'):
                        if not ready or clips.active or pending_pose is not None:
                            raise ValueError('请等待就绪、结束当前片段并等待预设保存完成后再设置')
                        if pose_path.exists() and line != 'pose idle replace':
                            raise ValueError('已有待机预设；明确替换请用 pose idle replace，旧片段保留原预设')
                        pose = capture_idle_pose(history, metadata, folder)
                        pending_pose = journal.save_pose(pose_path, pose)
                        print('正在保存待机预设，补偿继续。', flush=True)
                    elif line == 'stop':
                        if clips.active is None:
                            print('当前没有录制中的片段；先输入 start 名称。重力补偿继续。', flush=True)
                        elif returning_clip:
                            print('正在自动补回待机，请等待完成；quit 或 Ctrl+C 可回零退出。',flush=True)
                        elif live and clips.anchor and clips.anchor['exit_idle']:
                            # Imports were prepared before enable. Curve setup
                            # is bounded; gravity path checks run on later ticks.
                            idle_entry = idle_entry_factory(gravity,q,clips.anchor['idle_pose'],from_teaching=control)
                            journal.put(dict(event='clip_manual_end',clip=clips.active,monotonic_ns=read_end,
                                             q_rad=q.tolist(),return_duration_s=idle_entry.curve.duration))
                            returning_clip=True
                            ready=False
                            history.clear()
                            print('手动段已结束，正在接入位置保持并自动补回待机；请停止拖动并让出回位路径。回位将录入当前片段。',flush=True)
                        else:
                            clips.stop(read_end, q=q)
                            print('片段已结束，重力补偿继续。', flush=True)
                    elif line in ('quit', 'eof'):
                        stop[0] = line
                    elif line == 'status':
                        print(f'片段={clips.active or "未录制"} 角度={np.degrees(q).round(2).tolist()}', flush=True)
                        if idle_pose is not None:
                            print(f'相对待机偏差（度）={np.degrees(q-vector(idle_pose["q_rad"])).round(2).tolist()}', flush=True)
                        else:
                            print('尚无当前标定下的待机预设；使用 pose idle 设置。', flush=True)
                    else:
                        print('命令：pose idle / pose idle replace / start 名称 / stop / status / quit（六轴回零后退出）', flush=True)
                except ValueError as error:
                    print(str(error), flush=True)
            deadline = max(deadline + PERIOD, time.monotonic())
            time.sleep(max(0., deadline - time.monotonic()))
        if live and stop[0] in graceful_exits:
            # Close capture before any exit motion; quit is not an operator_stop
            # for an unfinished clip and must not make that clip replayable.
            trigger = stop[0]
            zero_return_active[0] = True
            clips.stop(time.monotonic_ns(), reason=trigger)
            result['zero_return_requested'] = True
            journal.put(dict(event='zero_return_requested', trigger=trigger, monotonic_ns=time.monotonic_ns()))
            print('正在回到机械校零位（J1～J6），夹爪保持当前开度；再按 Ctrl+C 可中断，保持支撑。', flush=True)
            backend.returning_to_zero = True
            try:
                result['zero_return_completed'] = bool(zero_return.run(
                    lambda: stop[0] != trigger, initial_gravity_gain=last_gravity_gain,
                    initial_kp=last_kp, initial_kd=last_kd))
            finally:
                backend.returning_to_zero = False
            journal.put(dict(event='zero_return_completed', confirmed=result['zero_return_completed'],
                             monotonic_ns=time.monotonic_ns()))
            if not result['zero_return_completed']:
                raise RuntimeError('zero return interrupted/not verified; disabling onto installed support')
            print('机械校零位已稳定到达，正在失能并退出。', flush=True)
        result['success'] = stop[0] in ('quit', 'session_timeout') or result['zero_return_completed']
        result['reason'] = stop[0]
    except BaseException as error:
        result['errors'].append(f'{type(error).__name__}: {error}')
        result['reason'] = 'zero_return_failed' if result['zero_return_requested'] else 'fault'
        if result['zero_return_requested']:
            try:
                journal.put(dict(event='zero_return_failed', detail=str(error), monotonic_ns=time.monotonic_ns()))
            except Exception:
                pass  # Cleanup must still run if logging itself has failed.
    finally:
        # Hardware cleanup precedes any export, join or potentially blocking file I/O.
        if backend is not None:
            try:
                if attempted_enable:
                    result['disable_requested'] = True
                    try:
                        journal.put(dict(event='disable_requested', monotonic_ns=time.monotonic_ns(), unix_ns=time.time_ns()))
                    except Exception as error:
                        result['errors'].append(f'disable event logging: {error}')
                    result['disable_confirmed'] = backend.disable()
                    try:
                        journal.put(dict(event='disable_completed', confirmed=result['disable_confirmed'],
                                         monotonic_ns=time.monotonic_ns(), unix_ns=time.time_ns()))
                    except Exception as error:
                        result['errors'].append(f'disable event logging: {error}')
                    if not result['disable_confirmed']:
                        result['errors'].append('disable not confirmed; use installed independent stop')
            except BaseException as error:
                result['errors'].append(f'disable: {error}')
            try:
                backend.close()
            except BaseException as error:
                result['errors'].append(f'close: {error}')
        for sig, handler in handlers.items():
            signal.signal(sig, handler)
        try:
            clips.stop(time.monotonic_ns(), result.get('reason', 'interrupted'))
            journal.put(dict(event='session_end', **result))
        except Exception as error:
            result['errors'].append(str(error))
        for writer in (journal, raw):
            try:
                writer.close()
            except Exception as error:
                result['errors'].append(str(error))
        result['logging_errors'] = [str(writer.failure or writer.error) for writer in (journal, raw)
                                    if writer.failure or writer.error]
        if result['logging_errors']:
            result['errors'].extend(result['logging_errors'])
        try:
            result['clips'] = export_clips(folder)
            if result['logging_errors']:
                for clip in result['clips']:
                    clip['complete'] = False
                    clip['end_reason'] = 'logging_error'
        except Exception as error:
            result['errors'].append(f'export: {error}; original journal retained')
        result['success'] = result['success'] and not result['errors']
        try:
            (folder / 'summary.json').write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n')
        except OSError as error:
            result['success'] = False
            result['errors'].append(f'summary save failed: {error}; hardware cleanup already attempted')
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode', nargs='?', choices=['check', 'mock', 'record', 'diagnose'], default='check')
    parser.add_argument('--port', default='can0')
    parser.add_argument('--calibration', type=Path, default=DEFAULT_CALIBRATION)
    parser.add_argument('--urdf', type=Path, default=DEFAULT_URDF)
    parser.add_argument('--output', type=Path, default=ROOT / 'record/gravity')
    parser.add_argument('--seconds', type=float, default=None, help='就绪后会话时长上限；实机默认0（不限时），正数到时回零退出；mock默认3秒')
    parser.add_argument('--name', default='demo', help='mock 片段名称')
    parser.add_argument('--read-mode',choices=['parallel','serial','raw'],default='parallel',help='diagnose 的位置读取方式；raw 严格解析原始参数回包')
    parser.add_argument('--supported', action='store_true', help='实机已安装防坠支撑，可承接退出/故障/看门狗失能')
    parser.add_argument('--manual-start', action='store_true', help='跳过自动到待机，直接进入重力补偿以手动设置/更换预设')
    args = parser.parse_args(argv)
    args.seconds = args.seconds if args.seconds is not None else (0. if args.mode == 'record' else 30. if args.mode == 'diagnose' else 3.)
    try:
        if args.mode == 'diagnose':
            from rs_feedback_audit import audit
            result=audit(args.seconds,args.port,args.read_mode)
            print(json.dumps(result,ensure_ascii=False,indent=2))
            return 0 if result['success'] else 1
        if args.mode == 'record' and not args.supported:
            raise ValueError('实机录制需要 --supported；请先落实防坠支撑')
        if args.mode == 'record' and not sys.stdin.isatty():
            raise ValueError('实机录制需要交互终端，不能从管道或已关闭的 stdin 启动')
        gravity, meta = preflight(args.calibration, args.urdf)
        if args.mode == 'check':
            print(json.dumps(dict(success=True, hardware_opened=False, **meta), ensure_ascii=False, indent=2))
            return 0
        result = run(args, gravity, meta)
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return 0 if result['success'] else 1
    except Exception as error:
        print(f'录制工具：{error}', file=sys.stderr)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
