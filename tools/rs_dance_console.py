#!/usr/bin/env python3
"""Numbered, persistent trial console for the designed dance library."""
import argparse
import json
import math
from pathlib import Path
import queue
import signal
import sys
import threading
import time

import numpy as np

from rs_daily_session import Session, StopRequested, prepare_daily_zero_return
from rs_gravity_record import ROOT, DEFAULT_CALIBRATION, DEFAULT_URDF, Journal, PERIOD, read_commands
from rs_gravity_play import KP, KD
from rs_protective_stop import maintain_hold
from rs_recovery_policy import RecoverableControlError
from rs_dance_speed import apply_trial_speed, checked_trial_speed, trial_speed_for, pivot_code_for


class DanceSession(Session):
    # User confirmed physical closure at +4.496 motor degrees on 2026-11-07.
    # This is closed-stop repeatability, not a new encoder zero or arm margin.
    closed_gripper_tolerance = math.radians(6.)

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.pause_requested = False
        self.completed = []

    def tick(self, target, velocity=None, *args, **kwargs):
        original = self.controller.following_limit[6]
        def closed_target(q, v):
            if q is not None and abs(q[6]) < 1e-9 and (v is None or abs(v[6]) < 1e-9):
                # The post-feedback sampler may have switched from a gripper
                # pulse to an ordinary sequence phase. Use its current profile.
                self.controller.following_limit[6] = max(self.controller.following_limit[6], self.closed_gripper_tolerance)
        sampler = kwargs.get('target_sampler')
        if sampler is not None:
            def sample():
                q, v, gain = sampler()
                closed_target(q, v)
                return q, v, gain
            kwargs['target_sampler'] = sample
        else:
            closed_target(target, velocity)
        try:
            return super().tick(target, velocity, *args, **kwargs)
        finally:
            self.controller.following_limit[6] = original

    def settle(self, target, state):
        arm = math.radians(self.home.args.exit_zero_tolerance_deg) if state == 'ZERO' else math.radians(1.)
        gripper = self.closed_gripper_tolerance if abs(target[6]) < 1e-9 else math.radians(3.)
        self.segment('settle_'+state, target=target, duration=8.,
                     tolerance=np.r_[np.full(6, arm), gripper])
        self.state = state
        self.event('state', state=state)

    def request_stop(self, *_):
        if self.homing:
            self.stop = 'abort'
        elif self.enabled:
            self.pause_requested = True
        else:
            self.stop = 'quit'

    def check_stop(self, busy=True):
        if busy:
            # One command per tick; pasted input must not block control.
            try:
                word = self.commands.get_nowait().strip().lower()
            except queue.Empty:
                word = ''
            if word in ('home', 'quit', 'exit', '退出'):
                self.stop = 'quit'
            elif word in ('stop', 'estop', 'pause', 'eof', 'release'):
                self.pause_requested = True
            elif word == 'status':
                print(f'当前状态：{self.state}', flush=True)
            elif word:
                print('动作进行中，请完成后再输入编号。', flush=True)
        if self.stop:
            raise StopRequested(self.stop)
        if self.pause_requested:
            self.pause_requested = False
            raise RecoverableControlError('operator requested protective hold')

    def _execute_action(self, number):
        if number not in self.plans:
            raise ValueError('请输入菜单中的动作编号')
        self.event('action_requested', action=number)
        if number == '00':
            if not self.enabled:
                self.connect(self.zero, 'ZERO')
            if self.state == 'ATTENTIVE':
                print('已在统一待机姿态，跳过起身。', flush=True)
            elif self.state == 'ZERO':
                self.execute_plan('00', 'ATTENTIVE')
            else:
                raise RuntimeError('当前状态不能执行开场')
        else:
            if not self.enabled:
                self.connect(self.idle, 'ATTENTIVE')
            elif self.state == 'ZERO':
                self.execute_plan('00', 'ATTENTIVE')
            if self.state != 'ATTENTIVE':
                raise RuntimeError('当前状态不能开始舞蹈')
            self.execute_plan(number, 'ZERO' if number == '11' else 'ATTENTIVE')
        self.completed.append(number)
        self.event('action_completed', action=number, state=self.state)
        print(f'{number} 完成，保持位置。输入下个编号；quit 回零退出。', flush=True)

    def home_and_exit(self):
        if not self.enabled:
            return
        self.homing = True
        self.backend.returning_to_zero = True
        print('正在回零；中断或失败会尝试保护保持。', flush=True)
        command = self.last_command or dict(kp=[0.]*7, kd=[0.]*7)
        def cancel_home():
            try:
                word = self.commands.get_nowait().strip().lower()
            except queue.Empty:
                word = ''
            if word in ('stop', 'estop', 'pause', 'eof', 'release'):
                self.stop = 'abort'
            elif word:
                print('正在回零；stop 或 Ctrl+C 可中断并尝试保持。', flush=True)
            return self.stop == 'abort'
        try:
            completed = self.home.run(cancel_home, initial_gravity_gain=self.last_gain,
                                      initial_kp=command['kp'][:6], initial_kd=command['kd'][:6])
            self.event('zero_return_completed', confirmed=bool(completed))
            if not completed:
                raise RuntimeError('回零中断或未到位')
        finally:
            self.homing = False
            self.backend.returning_to_zero = False


def number_for(word, plans):
    try:
        number = f'{int(word):02d}'
    except ValueError:
        raise ValueError('请输入菜单中的动作编号') from None
    if number not in plans:
        raise ValueError('请输入菜单中的动作编号')
    return number


def run(args, manifest, plans, gravity, identity, reports, *, commands=None,
        backend_factory=None, zero_factory=prepare_daily_zero_return):
    if not args.supported:
        raise ValueError('安装防坠支撑后使用 --supported')
    plans=apply_trial_speed(plans,getattr(args,'pivot_speed',None))
    folder = ROOT/'build/dance-trials'/f'session-{time.time_ns()}'
    folder.mkdir(parents=True)
    journal = Journal(folder/'control.jsonl')
    try:
        raw = Journal(folder/'can.jsonl')
    except BaseException:
        journal.close()
        raise
    if commands is None:
        commands = queue.Queue()
        threading.Thread(target=read_commands, args=(commands,), daemon=True).start()
    session = DanceSession(args, manifest, plans, gravity, identity, journal, raw, commands,
                           backend_factory, zero_factory)
    # The library has already been checked and loaded into memory. During
    # motion use shared feedback/model checks, not repeated action-file reads.
    library = dict(schema=manifest.get('schema'), recipe_version=manifest.get('recipe_version'),
                   library_version=manifest.get('library_version'),
                   bundle=str(args.bundle) if getattr(args, 'bundle', None) is not None else None,
                   actions={code: dict(source=report.get('source'), sha256=report.get('source_sha256'),
                                       duration_s=plans[code].duration,
                                       nominal_duration_s=report.get('duration_s'),
                                       source_library=report.get('source_library'),
                                       source_number=report.get('source_number',code),
                                       execution_speed=getattr(plans[code],'speed',1.))
                            for code, report in reports.items()})
    result = dict(success=False, hardware_accepted=False, folder=str(folder), errors=[], library=library)
    handlers = {}
    try:
        session.event('library_loaded', **library)
        if threading.current_thread() is threading.main_thread():
            for sig in (signal.SIGINT, signal.SIGTERM):
                handlers[sig] = signal.signal(sig, session.request_stop)
        if args.action is not None:
            session.execute(number_for(args.action, plans))
        while True:
            session.check_stop(busy=False)
            try:
                word = commands.get_nowait().strip().lower()
            except queue.Empty:
                word = ''
            if word in ('home', 'quit', 'exit', '退出') or (word == 'eof' and not session.enabled):
                session.stop = 'quit'
                raise StopRequested('quit')
            if word in ('stop', 'estop', 'pause', 'eof'):
                if session.enabled and session.state != 'PROTECTIVE_HOLD':
                    session.enter_protective_hold(RecoverableControlError('operator stop'), 'operator')
            elif word == 'release':
                if session.state == 'PROTECTIVE_HOLD':
                    result['reason'] = 'explicit_release'
                    result['success'] = True
                    break
                print('需要停止时先输入 stop；正常退出用 quit。', flush=True)
            elif word == 'status':
                print(f'当前状态：{session.state}', flush=True)
            elif word:
                try:
                    number = number_for(word, plans)
                except ValueError as error:
                    print(error, flush=True)
                else:
                    session.execute(number)
            if session.enabled:
                session.hold_tick()
            time.sleep(PERIOD)
    except StopRequested:
        try:
            session.home_and_exit()
            result.update(success=True, reason='quit')
        except BaseException as error:
            result['errors'].append(str(error))
    except BaseException as error:
        result['errors'].append(f'{type(error).__name__}: {error}')
    finally:
        # Unhandled failures (including a failed home) must not silently drop
        # torque. Recovery only uses fresh feedback, with no automatic resume.
        if result['errors'] and session.enabled and getattr(session, 'last_gain', 0.) >= 1.:
            result['protective_hold_requested'] = True
            try:
                stop = [None]
                def hold_signal(*_):
                    stop[0] = 'hold'
                if handlers:
                    for sig in handlers:
                        signal.signal(sig, hold_signal)
                while True:
                    choice = maintain_hold(session.backend, gravity, commands, stop, journal, session.verify)
                    if choice == 'release':
                        break
                    session.stop = None
                    session.last_gain = 1.
                    session.last_command = dict(kp=KP.tolist(), kd=KD.tolist())
                    if handlers:
                        for sig in handlers:
                            signal.signal(sig, session.request_stop)
                    try:
                        session.home_and_exit()
                        break
                    except BaseException as error:
                        result['errors'].append(f'home: {error}')
                        if handlers:
                            for sig in handlers:
                                signal.signal(sig, hold_signal)
            except BaseException as error:
                result['errors'].append(f'无法可靠保持：{error}')
                print(f'无法可靠保持：{error}；将尝试失能，由支撑承接。', flush=True)
        try:
            session.disconnect()
        except BaseException as error:
            result['errors'].append(str(error))
        for sig, handler in handlers.items():
            signal.signal(sig, handler)
        for writer in (journal, raw):
            try:
                writer.close()
            except BaseException as error:
                result['errors'].append(str(error))
        result['success'] = result['success'] and not result['errors']
        result['completed'] = session.completed
        result['protective_stops'] = session.protective_stops
        (folder/'summary.json').write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
    return result


def main(argv=None):
    from rs_dance_library import DEFAULT_OUTPUT, load_library
    parser = argparse.ArgumentParser(description='机械臂舞蹈：输入菜单编号逐个试播，完成后保持位置')
    parser.add_argument('action', nargs='?')
    parser.add_argument('--supported', action='store_true')
    parser.add_argument('--list', action='store_true')
    parser.add_argument('--check', action='store_true')
    parser.add_argument('--selected', action='store_true', help=argparse.SUPPRESS)
    parser.add_argument('--pivot-speed', type=float, help='仅10定点动作的播放倍率，默认1.0；范围0.1～1.0')
    parser.add_argument('--bundle', type=Path, help='指定所选类型的动作库目录')
    parser.add_argument('--port', default='can0')
    parser.add_argument('--calibration', type=Path, default=DEFAULT_CALIBRATION)
    parser.add_argument('--urdf', type=Path, default=DEFAULT_URDF)
    args = parser.parse_args(argv)
    try:
        if args.pivot_speed is not None:args.pivot_speed=checked_trial_speed(args.pivot_speed)
        if args.bundle is None:
            args.bundle = DEFAULT_OUTPUT
        manifest, plans, gravity, identity, reports = load_library(args.calibration, args.urdf, args.bundle)
        playback_speed=trial_speed_for(plans,args.pivot_speed)
        pivot_code=pivot_code_for(plans)
        print('基础舞蹈 · 10组＋开场/收尾')
        for number in sorted(reports, key=lambda code: 999 if code == '11' else int(code)):
            report = reports[number]
            suffix=(f'；播放 {plans[number].duration/playback_speed:.1f}s / {playback_speed:.0%}速度'
                    if number==pivot_code else '')
            print(f'{number} {report["name"]}  {plans[number].duration:.1f}s{suffix}')
        virtual=reports.get(pivot_code,{}).get('virtual_pivot_design')
        if virtual:
            axis='世界竖直轴' if virtual['config']['cone_axis']=='world_z' else '入口工具轴（备选）'
            print(f'{pivot_code}：支点前移 {virtual["virtual_offset_m"][0]*100:.0f}cm；'
                  f'{axis}；锥半角 {virtual["actual_cone_deg"]:.2f}°。')
        sway=reports.get(pivot_code,{}).get('pivot_sway_design')
        if sway:
            if sway.get('pivot_drop_m'):
                print(f'{pivot_code}：P下移 {sway["pivot_drop_m"]*100:.0f}cm；左右 ±{sway["actual_horizontal_deg"]:g}°、上下 ±{sway["actual_vertical_deg"]:g}°，各一次。')
            else:
                print(f'{pivot_code}：固定指尖前方 {sway["virtual_offset_m"][0]*100:.0f}cm 的点；左右一次、上下一次。')
        if args.list or args.check or not args.supported:
            command = './dance --supported'
            print('离线检查通过，未连接 CAN。' if args.check else f'试播：{command}；输入菜单编号。')
            return 0
        if not sys.stdin.isatty():
            raise ValueError('请在交互终端运行')
        if args.action is not None:
            number_for(args.action, plans)
        print('输入编号播放；完成后保持。stop/Ctrl+C 制动保持；quit 回零退出。')
        result = run(args, manifest, plans, gravity, identity, reports)
        print(f'日志：{result["folder"]}')
        for error in result['errors']:
            print(error, file=sys.stderr)
        return 0 if result['success'] else 1
    except Exception as error:
        print(f'舞蹈终端：{error}', file=sys.stderr)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
