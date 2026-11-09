#!/usr/bin/env python3
"""Inspect or explicitly play a complete, calibration-bound gravity-recorded CSV."""
from __future__ import annotations

import argparse
import csv
import json
import math
import queue
from pathlib import Path
import signal
import sys
import threading
import time

import numpy as np

from rs_gravity_record import (
    ROOT, DEFAULT_CALIBRATION, DEFAULT_URDF, CAPS, MAX_GAP, PERIOD,
    Journal, LiveBackend, digest, preflight, vector, read_commands,
)
from rs_record_playback import LOWER, UPPER, MEASUREMENT_TOLERANCE_RAD
from rs_motion import Curve, VMAX, AMAX
from rs_recovery_policy import RecoverableControlError, FollowingError, damping_gain, filtered_velocity

KP = np.array([50., 50., 50., 30., 50., 50., 5.])
KD = np.array([3., 5., 5., 3., 4., 4., .3])
ARRIVAL_TOLERANCE = math.radians(1.)
FOLLOW_TOLERANCE = math.radians(5.)
# Startup hold may target the measured pose just outside the nominal boundary.
# This command allowance is independent of the user-adjustable feedback tolerance.
INITIAL_HOLD_TARGET_TOLERANCE_RAD = math.radians(.2)


class PlaybackInterrupted(RuntimeError):
    """Operator interruption, distinct from control/feedback faults."""


class TorqueBudgetExceeded(RecoverableControlError):
    def __init__(self, details):
        super().__init__('estimated gravity + PD torque exceeds budget: '+
                         json.dumps(details,ensure_ascii=False),details)


def load_clip(path, identity, *, recover_interrupted=False, recover_endpoint_failure=False):
    path = Path(path).resolve()
    if path.suffix.lower() != '.csv':
        raise ValueError('需要录制生成的 CSV；不接受 TCH 或 pose idle 的 JSON 预设')
    if path.stat().st_size > 50_000_000:
        raise ValueError('CSV too large')
    metadata = json.loads((path.parent / 'metadata.json').read_text())
    summary = json.loads((path.parent / 'summary.json').read_text())
    if metadata.get('schema') != 'rebuddy.gravity_record.v1':
        raise ValueError('不是重力录制会话')
    if metadata.get('mock') is not False or summary.get('mock') is not False:
        raise ValueError('mock CSV 不能用于实机播放')
    for key in ('calibration_sha256', 'urdf_sha256'):
        if metadata.get(key) != identity[key]:
            raise ValueError(f'录制与当前标定/模型不匹配：{key}')
    clips = [c for c in summary.get('clips', []) if c.get('file') == path.name]
    complete = len(clips) == 1 and clips[0].get('complete') is True and clips[0].get('end_reason') == 'operator_stop'
    # Offline daily preparation may recover checked samples from an operator
    # interruption, then author a NEW return path. The CSV player stays strict.
    recovered = (recover_interrupted and len(clips) == 1 and clips[0].get('end_reason') == 'signal_2'
                 and summary.get('reason') in ('zero_return_failed', 'signal_2')
                 and not summary.get('logging_errors'))
    endpoint_recovery = (recover_endpoint_failure and len(clips) == 1
                         and clips[0].get('end_reason') == 'protective_stop'
                         and clips[0].get('auto_return') is True
                         and summary.get('reason') == 'fault'
                         and not summary.get('logging_errors')
                         and len(summary.get('errors', [])) == 1
                         and summary['errors'][0].startswith('RuntimeError: 自动回位未通过首尾检查：'))
    if not complete and not recovered and not endpoint_recovery:
        raise ValueError('需要通过 stop 完整结束的录制片段；不能播放中断片段')
    clip = clips[0]
    if digest(path) != clip.get('sha256'):
        raise ValueError('CSV 校验值与录制摘要不一致')
    with path.open(newline='') as source:
        reader = csv.DictReader(source)
        required = ['t_s', 'monotonic_ns', 'read_start_ns', 'read_end_ns'] + [f'q{i}_rad' for i in range(1, 8)]
        if reader.fieldnames is None or any(k not in reader.fieldnames for k in required):
            raise ValueError('CSV 缺少录制时间或关节位置列')
        rows = list(reader)
    if not 2 <= len(rows) <= 30001 or len(rows) != clip.get('samples'):
        raise ValueError('CSV sample count mismatch')
    t = np.array([float(r['t_s']) for r in rows])
    q = np.array([[float(r[f'q{i}_rad']) for i in range(1, 8)] for r in rows])
    stamps = np.array([int(r['monotonic_ns']) for r in rows], dtype=np.int64)
    begins = np.array([int(r['read_start_ns']) for r in rows], dtype=np.int64)
    ends = np.array([int(r['read_end_ns']) for r in rows], dtype=np.int64)
    if (not np.all(np.isfinite(t)) or not np.all(np.isfinite(q)) or t[0] < 0
            or np.any(np.diff(stamps) <= 0) or np.any(np.diff(stamps) > int(MAX_GAP * 1e9))
            or np.any(begins > ends) or np.any(ends != stamps)
            or np.any(ends - begins > int(MAX_GAP * 1e9))
            or np.any(np.abs((t - t[0]) - (stamps - stamps[0]) / 1e9) > 1e-6)):
        raise ValueError('CSV contains invalid/stale/nonfinite samples or timestamps')
    t -= t[0]
    if not 0 < t[-1] <= 600 or np.any(q < LOWER) or np.any(q > UPPER):
        raise ValueError('录制时长或位置超出播放范围；测量容差不扩展运动目标限位')
    spread = float(np.ptp(q[:, 6]))
    if spread > math.radians(2.):
        raise ValueError('本播放器只播放六轴动作，不支持夹爪开合轨迹')
    # Gravity recording holds the initial gripper opening. Do not replay encoder noise.
    q[:, 6] = q[0, 6]
    return t, q, dict(source=str(path), source_sha256=digest(path), samples=len(rows),
                     source_complete=complete, recovered_interrupted=bool(recovered),
                     recovered_endpoint_failure=bool(endpoint_recovery),
                     recorded_duration_s=float(t[-1]), gripper_spread_deg=math.degrees(spread),
                     calibration_sha256=identity['calibration_sha256'], urdf_sha256=identity['urdf_sha256'])


def prepare(path, calibration, urdf, speed):
    if not math.isfinite(speed) or not 0 < speed <= 1:
        raise ValueError('--speed must be in (0, 1]')
    gravity, identity = preflight(calibration, urdf)  # Does not open hardware.
    t, q, report = load_clip(path, identity)
    curve = Curve(t, q)
    # Curve already enforces velocity/acceleration limits; user speed can only slow it.
    curve.scale *= speed
    curve.duration /= speed
    curve.peak_velocity *= speed
    curve.peak_acceleration *= speed ** 2
    if curve.duration > 600:
        raise ValueError('经速度/加速度约束后超过 600 秒，请录制更短、更平稳的片段')
    # Model gravity is checked at recorded positions and interval midpoints.
    for stamp in np.r_[t, (t[:-1] + t[1:]) / 2]:
        if np.any(abs(vector(gravity(curve.spline(stamp)[:6]), 6)) > CAPS[:6]):
            raise ValueError('轨迹重力补偿超出力矩预算')
    report.update(schema='rebuddy.gravity_play.v1', hardware_accepted=False,
                  playback_s=curve.duration, effective_speed=curve.scale,
                  start_deg=np.degrees(curve.start).tolist(), end_deg=np.degrees(curve.end).tolist(),
                  peak_velocity_rad_s=curve.peak_velocity, peak_acceleration_rad_s2=curve.peak_acceleration,
                  speed_limit_rad_s=VMAX, acceleration_limit_rad_s2=AMAX,
                  kp=KP.tolist(), kd=KD.tolist(), torque_budget_nm=CAPS.tolist(),
                  gripper='hold measured opening at playback startup; no gripper trajectory',
                  entry_policy='plan from measured current pose to clip start, settle, then play',
                  exit_policy='CLI: completion/stop/fault after full gravity tries latched hold; explicit home/quit returns to zero; release disables; unavailable hold falls back to support',
                  path_validation='joint and dynamic bounds only; no collision verification')
    return curve, gravity, report


def prepare_entry(initial, recorded_start, gravity, speed, *, hold_gripper=True, validate_gravity=True,
                  velocity_limit=VMAX,acceleration_limit=AMAX):
    """Reuse the bounded C2 curve for a synchronized six-axis approach."""
    initial, target = vector(initial).copy(), vector(recorded_start).copy()
    if not math.isfinite(speed) or not 0 < speed <= 1:
        raise ValueError('--speed must be in (0, 1]')
    if hold_gripper:
        target[6] = initial[6]  # Playback preserves the current gripper opening.
    entry = Curve([0., 1.], np.array([initial, target]),
                  minimum_duration=2., initial_position_margin=MEASUREMENT_TOLERANCE_RAD,
                  velocity_limit=velocity_limit,acceleration_limit=acceleration_limit)
    entry.scale *= speed
    entry.duration /= speed
    entry.peak_velocity *= speed
    entry.peak_acceleration *= speed ** 2
    if entry.duration > 600:
        raise ValueError('移动到起点需要超过 600 秒，请检查播放速度设置')
    # Check the entire approach at <=20 ms planned intervals before enabling.
    # This samples gravity only; it is not a collision or inverse-dynamics proof.
    for stamp in (np.linspace(0., entry.duration, math.ceil(entry.duration / PERIOD) + 1)
                  if validate_gravity else []):
        q, _ = entry.at(stamp)
        if np.any(abs(vector(gravity(q[:6]), 6)) > CAPS[:6]):
            raise ValueError('移动到起点的路径重力补偿超出力矩预算；未使能')
    return entry


class PlaybackBackend(LiveBackend):
    def send(self, command):
        for i, motor in enumerate(self.hw.motors):
            motor.send_mit(command['target_rad'][i], command['velocity_rad_s'][i],
                           command['kp'][i], command['kd'][i], command['feedforward_nm'][i])


class Controller:
    def __init__(self, gravity, *, initial=None, gripper_velocity_limit=VMAX):
        self.gravity = gravity
        self.target_lower = LOWER - INITIAL_HOLD_TARGET_TOLERANCE_RAD
        self.target_upper = UPPER + INITIAL_HOLD_TARGET_TOLERANCE_RAD
        if initial is not None:
            initial = vector(initial)
            if np.any(initial<LOWER-MEASUREMENT_TOLERANCE_RAD) or np.any(initial>UPPER+MEASUREMENT_TOLERANCE_RAD):
                raise ValueError('initial feedback outside joint bounds')
            self.target_lower=np.minimum(LOWER,initial)-1e-8
            self.target_upper=np.maximum(UPPER,initial)+1e-8
        self.previous = None
        self.previous_ns = None
        self.velocity = np.zeros(7)
        if not math.isfinite(gripper_velocity_limit) or gripper_velocity_limit <= 0:
            raise ValueError('invalid gripper velocity limit')
        self.velocity_limit = np.r_[np.full(6,VMAX),gripper_velocity_limit]
        self.following_limit = np.full(7,FOLLOW_TOLERANCE)

    def observe(self,q,stamp):
        q=vector(q)
        if np.any(q<LOWER-MEASUREMENT_TOLERANCE_RAD) or np.any(q>UPPER+MEASUREMENT_TOLERANCE_RAD):
            raise RuntimeError('feedback/target outside joint bounds')
        if self.previous is not None:
            dt=(stamp-self.previous_ns)/1e9
            if not 0<dt<=MAX_GAP:raise RuntimeError('feedback interval exceeded 100 ms')
            if dt<.01:return q
            self.velocity=filtered_velocity(self.velocity,q-self.previous,dt)
        self.previous,self.previous_ns=q.copy(),stamp
        return q

    def command(self, q, stamp, target, velocity, gain=1.):
        q, target, velocity = vector(q), vector(target), vector(velocity)
        if (np.any(q < LOWER - MEASUREMENT_TOLERANCE_RAD) or np.any(q > UPPER + MEASUREMENT_TOLERANCE_RAD)
                or np.any(target < self.target_lower)
                or np.any(target > self.target_upper)):
            raise RuntimeError('feedback/target outside joint bounds')
        self.observe(q,stamp)
        if np.any(abs(target - q) > self.following_limit):
            raise FollowingError('following error exceeds limits: error_deg=' +
                               str(np.degrees(target-q).round(3).tolist()) +
                               ', limits_deg=' + str(np.degrees(self.following_limit).round(3).tolist()))
        if np.any(abs(velocity) > self.velocity_limit + 1e-8):
            raise RuntimeError('command velocity exceeds budget')
        g = np.r_[vector(self.gravity(q[:6]), 6), 0.]
        if np.any(abs(g) > CAPS):
            raise RuntimeError('gravity exceeds torque budget')
        kp, kd, ff = KP * gain, KD * gain, g * gain
        position_term=kp*(target-q)
        velocity_term=kd*(velocity-self.velocity)
        estimated = ff + position_term + velocity_term
        if np.any(abs(estimated) > CAPS):
            raise TorqueBudgetExceeded([dict(motor_id=int(i+1),estimated_nm=float(estimated[i]),
                budget_nm=float(CAPS[i]),gravity_nm=float(ff[i]),position_pd_nm=float(position_term[i]),
                velocity_pd_nm=float(velocity_term[i]),error_deg=math.degrees(float(target[i]-q[i])),
                target_velocity_deg_s=math.degrees(float(velocity[i])),
                estimated_velocity_deg_s=math.degrees(float(self.velocity[i])))
                for i in np.flatnonzero(abs(estimated)>CAPS)])
        return dict(target_rad=target.tolist(), velocity_rad_s=velocity.tolist(),
                    kp=kp.tolist(), kd=kd.tolist(), feedforward_nm=ff.tolist(),
                    estimated_torque_nm=estimated.tolist(), actual_rad=q.tolist())

    def recovery_command(self,q,stamp,anchor=None,gain=0.,brake_kd=None):
        q=self.observe(q,stamp)
        g=np.r_[vector(self.gravity(q[:6]),6),0.]
        if np.any(abs(g)>CAPS):raise RuntimeError('recovery gravity exceeds budget')
        kp=KP*gain
        kd=damping_gain(g,self.velocity,KD,CAPS) if anchor is None else brake_kd+gain*(KD-brake_kd)
        target=q.copy() if anchor is None else vector(anchor)
        estimated=g+kp*(target-q)-kd*self.velocity
        if (not 0<=gain<=1 or not np.all(np.isfinite(estimated)) or np.any(abs(estimated)>CAPS) or
                (anchor is not None and np.any(abs(target-q)>FOLLOW_TOLERANCE))):
            raise RuntimeError('recovery hold cannot meet budget or tracking')
        return dict(target_rad=target.tolist(),velocity_rad_s=[0.]*7,kp=kp.tolist(),kd=kd.tolist(),
                    feedforward_nm=g.tolist(),estimated_torque_nm=estimated.tolist(),actual_rad=q.tolist())


def play(curve, gravity, report, args, *, backend_factory=None, zero_return_factory=None, commands=None):
    if args.supported is not True:
        raise ValueError('播放需要 --supported：失能后由独立支撑承重')
    folder = args.log_dir / f'gravity-play-{time.time_ns()}'
    folder.mkdir(parents=True, exist_ok=False)
    (folder / 'plan.json').write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
    journal = Journal(folder / 'control.jsonl')
    try:
        raw = Journal(folder / 'can.jsonl')
    except BaseException:
        journal.close()
        raise
    result = dict(success=False, hardware_accepted=False, folder=str(folder), enable_requested=False,
                  disable_requested=False, disable_confirmed=False, entry_completed=False,
                  zero_return_requested=False, zero_return_completed=False, errors=[])
    backend = None
    zero_return = None
    enable_completed = False
    returning = False
    last_gain = 0.
    last_kp, last_kd = np.zeros(6), np.zeros(6)
    handlers = {}
    cancel = [None]
    protective = getattr(args, 'protective_hold', False)
    # Import before enable; the recovery module also reuses this controller.
    from rs_protective_stop import maintain_hold
    if protective and commands is None:
        commands = queue.Queue()
        threading.Thread(target=read_commands, args=(commands,), daemon=True).start()

    def request_stop(number, _frame):
        if number == signal.SIGINT and cancel[0] is None and not returning:
            cancel[0] = 'sigint'
        else:
            cancel[0] = 'abort'

    def check_cancel():
        if protective:
            try:
                word = commands.get_nowait().strip().lower()
            except queue.Empty:
                word = ''
            if word in ('stop', 'estop', 'pause', 'quit', 'home', 'eof', 'release'):
                cancel[0] = 'hold'
        if cancel[0] is not None:
            raise PlaybackInterrupted('operator stop')

    def verify_identity():
        for path, key in ((args.calibration, 'calibration_sha256'), (args.urdf, 'urdf_sha256'),
                          (Path(report['source']), 'source_sha256')):
            if digest(path) != report[key]:
                raise RuntimeError(f'file identity changed: {path}')

    try:
        if threading.current_thread() is threading.main_thread():
            for sig in (signal.SIGINT, signal.SIGTERM):
                handlers[sig] = signal.signal(sig, request_stop)
        verify_identity()
        backend = backend_factory() if backend_factory else PlaybackBackend(args.port, raw, supported=True)
        if zero_return_factory is None:
            from rs_zero_return import prepare_zero_return
            zero_return_factory = prepare_zero_return
        # Import and prepare reference homing before enabling, not on Ctrl+C.
        zero_return = zero_return_factory(backend, gravity, report, args, journal)
        initial, _, _ = backend.read()
        for _ in range(5):
            time.sleep(PERIOD)
            measured, _, _ = backend.read()
            if np.max(abs(measured - initial)) > math.radians(.2):
                raise ValueError('起始姿态仍在移动；未使能')
        entry = prepare_entry(initial, curve.start, gravity, getattr(args, 'speed', .5))
        entry_report = dict(initial_deg=np.degrees(initial).tolist(),
                            target_deg=np.degrees(entry.end).tolist(), duration_s=entry.duration,
                            peak_velocity_rad_s=entry.peak_velocity,
                            peak_acceleration_rad_s2=entry.peak_acceleration,
                            gripper='hold current opening',
                            path_validation='joint/velocity/acceleration bounds and sampled gravity; no collision verification')
        (folder / 'entry-plan.json').write_text(json.dumps(entry_report, ensure_ascii=False, indent=2) + '\n')
        result['entry_duration_s'] = entry.duration
        # Planning/storage may take time; reject movement since the measured start.
        measured, _, _ = backend.read()
        if np.max(abs(measured - initial)) > math.radians(.2):
            raise ValueError('规划期间当前姿态发生变化；未使能，请保持后重试')
        controller = Controller(gravity,initial=initial)
        controller.command(initial, time.monotonic_ns(), initial, np.zeros(7))
        check_cancel()
        verify_identity()
        result['enable_requested'] = True  # Also clean up a partially failed enable.
        journal.put(dict(event='enable_requested', monotonic_ns=time.monotonic_ns()))
        backend.enable(initial)
        enable_completed = True
        journal.put(dict(event='enabled', monotonic_ns=time.monotonic_ns()))
        controller = Controller(gravity,initial=initial)
        next_identity = time.monotonic()

        def segment(name, at, duration, ramp=False, require_settled=False):
            nonlocal next_identity, last_gain, last_kp, last_kd
            started = previous = deadline = time.monotonic()
            stable_since = None
            while True:
                tick = time.monotonic()
                check_cancel()
                if tick - previous > MAX_GAP:
                    raise RuntimeError('control deadline exceeded 100 ms')
                previous = tick
                backend.heartbeat()
                q, begin, end = backend.read()
                elapsed = min(time.monotonic() - started, duration)
                target, velocity = at(elapsed)
                gain = min(1., elapsed / 1.5) if ramp else 1.
                command = controller.command(q, end, target, velocity, gain)
                if tick >= next_identity:
                    verify_identity()
                    next_identity = tick + 1.
                if time.monotonic_ns() - begin > int(MAX_GAP * 1e9):
                    raise RuntimeError('feedback expired before command')
                journal.put(dict(event='command', phase=name, monotonic_ns=end, **command))
                check_cancel()
                backend.send(command)
                last_gain = gain
                last_kp, last_kd = np.asarray(command['kp'][:6]), np.asarray(command['kd'][:6])
                check_cancel()
                if require_settled:
                    if (np.max(abs(target - q)) <= ARRIVAL_TOLERANCE
                            and np.max(abs(controller.velocity)) <= math.radians(1.)):
                        if stable_since is None:
                            stable_since = tick
                        if tick - stable_since >= .5:
                            return
                    else:
                        stable_since = None
                if elapsed >= duration:
                    if require_settled:
                        raise RuntimeError('未稳定到达录制起点（误差需≤1°、速度≤1°/秒持续0.5秒）；未开始播放')
                    return
                deadline = max(deadline + PERIOD, time.monotonic())
                time.sleep(max(0., deadline - time.monotonic()))

        print('正在建立位置保持与重力补偿；stop / Ctrl+C 制动保持。' if protective else
              '正在建立位置保持与重力补偿；首次 Ctrl+C 回零后退出，回零中再次 Ctrl+C 中止并失能。', flush=True)
        segment('startup', lambda _: (initial, np.zeros(7)), 1.5, ramp=True)
        print(f'正在从当前姿态平滑移动到录制起点，预计 {entry.duration:.1f} 秒；夹爪保持当前开度。', flush=True)
        segment('approach', entry.at, entry.duration)
        segment('approach_settle', lambda _: (entry.end, np.zeros(7)), 5., require_settled=True)
        result['entry_completed'] = True
        def playback_at(stamp):
            q, velocity = curve.at(stamp)
            q, velocity = q.copy(), velocity.copy()
            q[6], velocity[6] = initial[6], 0.
            return q, velocity
        endpoint, _ = playback_at(curve.duration)
        print(f'播放一次，预计 {curve.duration:.1f} 秒；' +
              ('结束后保持力矩，等待 home/quit 回零退出。' if protective else '结束后失能，由支撑承重。'), flush=True)
        controller.velocity_limit[:6] = getattr(curve, 'velocity_limit', VMAX)
        segment('play', playback_at, curve.duration)
        controller.velocity_limit[:6] = VMAX
        segment('settle', lambda _: (endpoint, np.zeros(7)), 1.)
        actual, _, _ = backend.read()
        if np.max(abs(actual - endpoint)) > ARRIVAL_TOLERANCE:
            raise RuntimeError('endpoint did not settle within 1 degree')
        check_cancel()
        result['success'] = True
        result['reason'] = 'completed'
    except PlaybackInterrupted as error:
        if enable_completed and cancel[0] == 'sigint' and not protective:
            returning = True
            result['zero_return_requested'] = True
            result['reason'] = f'signal_{signal.SIGINT}'
            backend.returning_to_zero = True
            try:
                journal.put(dict(event='zero_return_requested', trigger='ctrl_c', monotonic_ns=time.monotonic_ns()))
                print('已停止播放，正在回到机械校零位（J1～J6），夹爪保持开度；再次 Ctrl+C 中止并失能。', flush=True)
                result['zero_return_completed'] = bool(zero_return.run(
                    lambda: cancel[0] != 'sigint', initial_gravity_gain=last_gain,
                    initial_kp=last_kp, initial_kd=last_kd))
                journal.put(dict(event='zero_return_completed', confirmed=result['zero_return_completed'],
                                 monotonic_ns=time.monotonic_ns()))
                if not result['zero_return_completed']:
                    raise RuntimeError('zero return interrupted/not verified')
                result['success'] = True
                print('机械校零位已稳定到达，正在失能并退出。', flush=True)
            except BaseException as home_error:
                result['errors'].append(f'{type(home_error).__name__}: {home_error}')
                result['reason'] = 'zero_return_failed'
            finally:
                backend.returning_to_zero = False
        else:
            result['errors'].append(str(error))
            result['reason'] = 'operator_abort' if enable_completed else 'cancelled_before_enable'
    except BaseException as error:
        result['errors'].append(f'{type(error).__name__}: {error}')
        result['reason'] = 'fault'
    finally:
        if protective and enable_completed and last_gain >= 1. and not result['zero_return_completed']:
            result['protective_hold_requested'] = True
            try:
                choice = maintain_hold(backend, gravity, commands, cancel, journal, verify_identity)
                result['protective_hold_exit'] = choice
                if choice == 'home':
                    cancel[0] = None
                    returning = True
                    backend.returning_to_zero = True
                    result['zero_return_requested'] = True
                    print('收到回零命令，正在回零；故障时尝试重新保持。', flush=True)
                    result['zero_return_completed'] = bool(zero_return.run(
                        lambda: cancel[0] is not None, initial_gravity_gain=1.,
                        initial_kp=KP[:6], initial_kd=KD[:6]))
                    if not result['zero_return_completed']:
                        raise RuntimeError('explicit home interrupted/not verified')
            except BaseException as hold_error:
                result['errors'].append(f'protective hold/home: {hold_error}')
                # A failed home must not silently release a loaded arm. If
                # feedback permits, remain holding until explicit release/home.
                if returning:
                    backend.returning_to_zero = False
                    try:
                        # Do not resume a failed home automatically.
                        while maintain_hold(backend, gravity, commands, cancel, journal, verify_identity) != 'release':
                            print('本次回零失败；请承托机械臂后输入 release 明确失能。', flush=True)
                    except BaseException as recovery_error:
                        result['errors'].append(f'hold unavailable: {recovery_error}')
                print(f'播放保护流程结束：{hold_error}；若无法保持，由支撑承重。', flush=True)
            finally:
                backend.returning_to_zero = False
        if backend is not None:
            if result['enable_requested']:
                result['disable_requested'] = True
                try:
                    result['disable_confirmed'] = backend.disable()
                    if not result['disable_confirmed']:
                        result['errors'].append('disable not confirmed; use independent stop')
                except BaseException as error:
                    result['errors'].append(f'disable: {error}')
            try:
                backend.close()
            except BaseException as error:
                result['errors'].append(f'close: {error}')
        for sig, handler in handlers.items():
            signal.signal(sig, handler)
        for writer in (journal, raw):
            try:
                writer.close()
            except BaseException as error:
                result['errors'].append(f'log: {error}')
        result['success'] = result['success'] and not result['errors']
        (folder / 'summary.json').write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n')
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode', choices=['inspect', 'play'])
    parser.add_argument('csv', type=Path)
    parser.add_argument('--supported', action='store_true')
    parser.add_argument('--speed', type=float, default=.5, help='额外减速比例 (0,1]；仍执行播放速度/加速度约束')
    parser.add_argument('--port', default='can0')
    parser.add_argument('--calibration', type=Path, default=DEFAULT_CALIBRATION)
    parser.add_argument('--urdf', type=Path, default=DEFAULT_URDF)
    parser.add_argument('--log-dir', type=Path, default=ROOT / 'build/record-review')
    parser.set_defaults(protective_hold=True)
    args = parser.parse_args(argv)
    try:
        if args.mode == 'play' and (not args.supported or not sys.stdin.isatty()):
            raise ValueError('实机播放需要交互终端和 --supported')
        curve, gravity, report = prepare(args.csv, args.calibration, args.urdf, args.speed)
        print(json.dumps(dict(hardware_opened=False, **report), ensure_ascii=False, indent=2), flush=True)
        if args.mode == 'inspect':
            return 0
        result = play(curve, gravity, report, args)
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return 0 if result['success'] else 1
    except Exception as error:
        print(f'CSV 播放：{error}', file=sys.stderr)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
