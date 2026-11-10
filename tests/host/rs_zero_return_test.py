"""Reference zero-return and recorder exit tests, using simulated I/O only."""
import io
import importlib.util
import json
from pathlib import Path
import queue
import signal
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tools'))
import rs_gravity_record as record
REFERENCE_AVAILABLE = (importlib.util.find_spec('lerobot_teleoperator_rebot_vr') is not None
                       and importlib.util.find_spec('pinocchio') is not None)
if REFERENCE_AVAILABLE:
    import rs_zero_return as zero


class Clock:
    now = 10.
    def monotonic(self): return self.now
    def monotonic_ns(self): return int(self.now * 1e9)
    def sleep(self, seconds): self.now += seconds


@unittest.skipUnless(REFERENCE_AVAILABLE, 'VR reference package and Pinocchio required')
class Tests(unittest.TestCase):
    def test_daily_motion_overrides_reach_dispatcher_and_leave_default_policy_unchanged(self):
        with tempfile.TemporaryDirectory() as tmp:
            args,metadata,config=self.fixture(Path(tmp));rows=[]
            gravity=record.Gravity(args.urdf)
            journal=types.SimpleNamespace(put=rows.append)
            default=zero.ZeroReturn(None,gravity,metadata,args,journal,config_path=config)
            fast=zero.ZeroReturn(None,gravity,metadata,args,journal,config_path=config,
                                 exit_speed_rad_s=1.,exit_acceleration_rad_s2=2.)
            self.assertEqual(default.args.exit_zero_speed_rad_s,.5)
            self.assertEqual(default.args.exit_zero_acceleration_rad_s2,1.)
            self.assertEqual(fast.args.exit_zero_speed_rad_s,1.)
            self.assertEqual(fast.args.exit_zero_acceleration_rad_s2,2.)
            self.assertEqual(fast.args.exit_zero_tolerance_deg,default.args.exit_zero_tolerance_deg)
            daily=zero.ZeroReturn(None,gravity,metadata,args,journal,config_path=config,
                                 exit_tolerance_deg=3.,exit_speed_rad_s=1.,exit_acceleration_rad_s2=2.)
            self.assertEqual(daily.args.exit_zero_tolerance_deg,3.)
            self.assertEqual(default.args.exit_zero_tolerance_deg,2.)
            np.testing.assert_allclose(fast.robot.velocity_limit_rad_s,np.ones(6))
            self.assertEqual(rows[-1]['motion_options']['exit_zero_speed_rad_s'],1.)
            for kwargs in (dict(exit_speed_rad_s=1.01),dict(exit_acceleration_rad_s2=2.01),
                           dict(exit_speed_rad_s=float('nan')),dict(exit_acceleration_rad_s2=0.),
                           dict(exit_tolerance_deg=0.),dict(exit_tolerance_deg=3.1)):
                with self.assertRaisesRegex(ValueError,'override'):
                    zero.ZeroReturn(None,gravity,metadata,args,journal,config_path=config,**kwargs)

    def fixture(self, folder):
        cal = folder / 'cal.json'; cal.write_text('{}')
        config = folder / 'reference.yaml'; config.write_text(zero.DEFAULT_CONFIG.read_text())
        args = types.SimpleNamespace(calibration=cal, urdf=record.DEFAULT_URDF)
        metadata = dict(calibration_sha256=record.digest(cal), urdf_sha256=record.digest(args.urdf))
        return args, metadata, config

    def simulated_zero(self, case):
        with tempfile.TemporaryDirectory() as tmp:
            args, metadata, config = self.fixture(Path(tmp))
            if case == 'timeout':
                settings = zero.yaml.safe_load(config.read_text())
                settings['startup']['initial_move_timeout'] = .25
                config.write_text(zero.yaml.safe_dump(settings))
            gravity = record.Gravity(args.urdf)
            clock = Clock(); rows = []; sent = []; cancelled = [False]
            class Backend:
                q = np.array([.05, .12, .08, -.06, .02, 0., .4])
                def heartbeat(self): pass
                def read(self):
                    if case == 'bad_feedback': raise RuntimeError('SDK/raw mismatch')
                    if case == 'cancel_read' and len(sent) == 3: cancelled[0] = True
                    begin = clock.monotonic_ns()
                    clock.now += .2 if case == 'stale' else .001
                    return self.q.copy(), begin, clock.monotonic_ns()
                def send(self, command):
                    sent.append(command)
                    if case not in ('stalled', 'timeout'):
                        self.q = np.array(command['target_rad'])
                    if case == 'cancel' and len(sent) == 10: cancelled[0] = True
            backend = Backend()
            motion=dict(exit_speed_rad_s=1.,exit_acceleration_rad_s2=2.) if case=='daily_fast' else {}
            home = zero.ZeroReturn(backend, gravity, metadata, args,
                                   types.SimpleNamespace(put=rows.append), config_path=config,**motion)
            if case == 'changed_calibration': args.calibration.write_text('changed')
            with patch.object(zero.time, 'monotonic', clock.monotonic), \
                    patch.object(zero.time, 'monotonic_ns', clock.monotonic_ns), \
                    patch.object(zero.time, 'sleep', clock.sleep), patch('sys.stdout', io.StringIO()):
                if case in ('bad_feedback', 'stale', 'changed_calibration'):
                    with self.assertRaises(RuntimeError): home.run(lambda: cancelled[0])
                    self.assertEqual(sent, [])
                    return
                if case in ('stalled', 'timeout', 'cancel_read'):
                    with self.assertRaises(RuntimeError): home.run(lambda: cancelled[0])
                    if case == 'cancel_read': self.assertEqual(len(sent), 3)
                    else:
                        self.assertGreater(len(sent), 25)
                        self.assertTrue(any(r.get('motion_result') == 'failed' for r in rows))
                    return
                kwargs=dict(initial_kp=home.kp*.5,initial_kd=home.kd*.5) if case=='playback_gains' else {}
                reached = home.run(lambda: cancelled[0], initial_gravity_gain=.25,**kwargs)
            if case == 'cancel':
                self.assertFalse(reached)
                self.assertEqual(len(sent), 10)
            else:
                self.assertTrue(reached)
                self.assertLess(np.max(abs(backend.q[:6])), np.radians(.05))
                self.assertAlmostEqual(backend.q[6], .4)
                self.assertGreater(len(sent), 25)
                self.assertTrue(any(r.get('motion_result') == 'completed' for r in rows))
                # Exit trajectory is separate from teaching samples.
                self.assertFalse(any(r.get('event') == 'sample' for r in rows))
                first = np.array(sent[0]['feedforward_nm'][:6])
                np.testing.assert_allclose(first, gravity(np.array([.05,.12,.08,-.06,.02,0.])) * .2515, rtol=.02)
                if case=='playback_gains':
                    np.testing.assert_allclose(sent[0]['kp'][:6],home.kp*.501,rtol=.001)
                    np.testing.assert_allclose(sent[0]['kd'][:6],home.kd*.501,rtol=.001)

    def test_reference_routine_reaches_zero_and_keeps_gripper(self):
        self.simulated_zero('success')

    def test_faster_daily_reference_return_reaches_zero_and_keeps_gripper(self):
        self.simulated_zero('daily_fast')

    def test_playback_gain_handoff_does_not_drop_position_hold(self):
        self.simulated_zero('playback_gains')

    def test_live_backend_forwards_return_velocity_and_damping(self):
        calls=[]
        backend=object.__new__(record.LiveBackend)
        backend.hw=types.SimpleNamespace(motors=[types.SimpleNamespace(
            send_mit=lambda *args,i=i:calls.append((i,args))) for i in range(7)])
        command=dict(target_rad=[.1]*7,velocity_rad_s=[.02]*7,kp=[30.]*7,kd=[3.]*7,feedforward_nm=[.5]*7)
        backend.send(command)
        self.assertEqual(calls,[(i,(.1,.02,30.,3.,.5)) for i in range(7)])

    def test_cancel_stops_return(self):
        self.simulated_zero('cancel')

    def test_cancel_during_feedback_prevents_next_send(self):
        self.simulated_zero('cancel_read')

    def test_reference_stall_and_timeout_do_not_report_arrival(self):
        for case in ('stalled', 'timeout'):
            with self.subTest(case=case): self.simulated_zero(case)

    def test_invalid_feedback_and_changed_identity_never_send(self):
        for case in ('bad_feedback', 'stale', 'changed_calibration'):
            with self.subTest(case=case): self.simulated_zero(case)

    def exit_case(self, case):
        commands = queue.Queue(); events = []; handlers = {}; saved_rows = []; backends = []
        class Gravity:
            lower = np.full(6, -3.); upper = np.full(6, 3.)
            def __call__(self, q): return np.zeros(6)
        def set_handler(sig, fn):
            old = handlers.get(sig, signal.SIG_DFL); handlers[sig] = fn
            return old
        class Backend:
            triggered = False
            enabled = False
            def __init__(self): backends.append(self)
            def read(self):
                if case.startswith('temperature') and (self.enabled or case=='temperature_startup'):
                    if not getattr(self,'returning_to_zero',False):
                        raise record.TemperatureReturnRequested([dict(motor_id=5,temperature_c=100.)])
                    if case=='temperature_feedback_fault':raise RuntimeError('bad feedback during return')
                stamp = record.time.monotonic_ns()
                return np.zeros(7), stamp, stamp
            def enable(self, q):
                events.append('enable')
                self.enabled = True
                if case == 'enable_failure': raise RuntimeError('partial enable')
                if case == 'early_sigterm': handlers[signal.SIGTERM](signal.SIGTERM, None)
            def heartbeat(self):
                if case == 'feedback_fault': raise RuntimeError('feedback fault')
            def send(self, command):
                if case.startswith('timeout'): return
                if not self.triggered:
                    self.triggered = True
                    if case in ('ctrl_c', 'ctrl_c_twice'):
                        handlers[signal.SIGINT](signal.SIGINT, None)
                    elif case in ('sigterm','sigterm_twice'): handlers[signal.SIGTERM](signal.SIGTERM, None)
                    else: commands.put('eof' if case == 'eof' else 'quit')
            def disable(self): events.append('disable'); return True
            def close(self): events.append('close')
        class Home:
            def run(self, should_stop, **kwargs):
                events.append('home')
                if case in ('ctrl_c_twice', 'home_failure', 'timeout_failure', 'timeout_interrupt',
                            'temperature_feedback_fault', 'temperature_interrupt', 'sigterm_twice'):
                    # A failed return now latches torque hold when feedback is
                    # usable. Simulate an explicit operator release so this exit
                    # test does not wait forever for input. Hold behavior itself
                    # is exercised by rs_protective_stop_test.py.
                    commands.put('release')
                if case.startswith('temperature'):
                    backends[0].read()  # Threshold must not immediately abort the return.
                if case in ('home_failure', 'timeout_failure'): raise RuntimeError('zero-pose stalled')
                if case in ('ctrl_c_twice', 'timeout_interrupt', 'temperature_interrupt'): handlers[signal.SIGINT](signal.SIGINT, None)
                if case == 'sigterm_twice': handlers[signal.SIGTERM](signal.SIGTERM, None)
                return not should_stop()
        with tempfile.TemporaryDirectory() as tmp, patch.object(record.signal, 'signal', side_effect=set_handler), \
                patch('sys.stdout', io.StringIO()):
            folder = Path(tmp); cal = folder / 'cal.json'; cal.write_text('{}')
            args = types.SimpleNamespace(mode='record', supported=True, seconds=1.,
                                         output=folder, name='test', calibration=cal)
            metadata = dict(calibration_sha256=record.digest(cal))
            def prepare(*_):
                events.append('prepare')
                return Home()
            result = record.run(args, Gravity(), metadata, backend_factory=Backend,
                                commands=commands, zero_return_factory=prepare)
            saved_rows = [json.loads(line) for line in (Path(result['folder']) / 'session.jsonl').read_text().splitlines()]
            self.assertFalse(getattr(backends[0],'returning_to_zero',False))
        return result, events, saved_rows

    def test_quit_and_first_ctrl_c_return_before_disable(self):
        for case in ('quit', 'ctrl_c', 'timeout', 'temperature', 'eof', 'sigterm', 'early_sigterm'):
            result, events, rows = self.exit_case(case)
            self.assertTrue(result['success'], result)
            self.assertTrue(result['zero_return_completed'])
            self.assertEqual(events, ['prepare', 'enable', 'home', 'disable', 'close'])
            phases = [r['event'] for r in rows]
            self.assertLess(phases.index('zero_return_completed'), phases.index('disable_requested'))
            if case == 'timeout': self.assertEqual(result['reason'], 'session_timeout')
            if case == 'temperature':
                self.assertEqual(result['reason'],'temperature')
                self.assertEqual(result['temperature_exit'],[dict(motor_id=5,temperature_c=100.)])
                self.assertLess(phases.index('temperature_exit'),phases.index('zero_return_requested'))

    def test_second_ctrl_c_and_failure_are_not_successful_returns(self):
        for case in ('ctrl_c_twice', 'home_failure', 'timeout_failure', 'timeout_interrupt',
                     'temperature_feedback_fault', 'temperature_interrupt', 'sigterm_twice'):
            result, events, rows = self.exit_case(case)
            self.assertFalse(result['success'])
            self.assertTrue(result['zero_return_requested'])
            self.assertFalse(result['zero_return_completed'])
            self.assertEqual(result['reason'], 'zero_return_failed')
            self.assertEqual(events[-2:], ['disable', 'close'])
            if result.get('protective_hold_exit'):
                self.assertEqual(result['protective_hold_exit'], 'release')

    def test_hot_startup_does_not_enable_or_attempt_return(self):
        result,events,_=self.exit_case('temperature_startup')
        self.assertFalse(result['success'])
        self.assertFalse(result['enable_requested'])
        self.assertFalse(result['zero_return_requested'])
        self.assertEqual(events,['prepare','close'])

    def test_fault_and_partial_enable_do_not_home(self):
        for case in ('feedback_fault', 'enable_failure'):
            result, events, rows = self.exit_case(case)
            self.assertNotIn('home', events)
            self.assertFalse(result['zero_return_requested'])
            self.assertEqual(events[-2:], ['disable', 'close'])


if __name__ == '__main__': unittest.main()
