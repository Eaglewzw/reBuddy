"""CSV validation and playback lifecycle tests; never opens real CAN."""
import csv
import io
import json
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tools'))
import rs_gravity_play as m


class Gravity:
    def __call__(self, q):
        return np.array([0., 2., 3., .5, 0., 0.])


class Clock:
    def __init__(self):
        self.now = 10.
    def monotonic(self):
        return self.now
    def monotonic_ns(self):
        return int(self.now * 1e9)
    def sleep(self, duration):
        self.now += duration


class Tests(unittest.TestCase):
    def test_all_seven_new_budget_boundaries_and_shared_consumers(self):
        import rs_gravity_record as recorder
        import rs_idle_entry as entry
        import rs_daily_prepare as daily
        expected=np.array([18.,18.,18.,4.5,4.5,4.5,1.5])
        np.testing.assert_array_equal(m.CAPS,expected)
        for consumer in (recorder,entry,daily):self.assertIs(consumer.CAPS,m.CAPS)
        q=np.array([0.,.7,1.1,-.3,0.,0.,0.])
        for i,cap in enumerate(expected):
            for sign in (-1,1):
                for excess in (-.001,.001):
                    with self.subTest(axis=i+1,sign=sign,excess=excess):
                        ctrl=m.Controller(lambda q:np.zeros(6))
                        ctrl.velocity[i]=-sign*(cap+excess)/m.KD[i]
                        if excess>0:
                            with self.assertRaises(m.TorqueBudgetExceeded) as error:
                                ctrl.command(q,1,q,np.zeros(7))
                            self.assertEqual(error.exception.details[0]['motor_id'],i+1)
                            self.assertEqual(error.exception.details[0]['budget_nm'],cap)
                        else:
                            c=ctrl.command(q,1,q,np.zeros(7))
                            self.assertAlmostEqual(c['estimated_torque_nm'][i],sign*(cap+excess))

    def test_logged_j4_return_sample_passes_new_budget(self):
        # Exact terms from session-1793584676254855839, not a changed trajectory.
        q=np.array([-.0563738085,1.2819392681,2.0660591125,-1.0023031235,-.0541495346,-.0110830069,.0359718315])
        target=q.copy();target[3]+=.45102095932707753/m.KP[3]
        velocity=np.zeros(7);velocity[3]=np.radians(16.36265149364215)
        gravity=np.zeros(6);gravity[3]=1.9993931838984396
        ctrl=m.Controller(lambda q:gravity)
        ctrl.velocity_limit[:6]=1.
        ctrl.velocity[3]=np.radians(5.589937660333519)
        c=ctrl.command(q,1,target,velocity)
        self.assertAlmostEqual(c['estimated_torque_nm'][3],3.0144721205246428)
        self.assertLess(c['estimated_torque_nm'][3],m.CAPS[3])

    def fixture(self, folder, *, t=None, q=None, mock=False, complete=True):
        t = np.array([0., .02, .04, .06]) if t is None else np.array(t)
        q = np.tile([0., .7, 1.1, -.3, 0., 0., 0.], (len(t), 1)) if q is None else np.array(q)
        cal, urdf = folder / 'cal.json', folder / 'model.urdf'
        cal.write_text('{}'); urdf.write_text('test model')
        identity = dict(calibration_sha256=m.digest(cal), urdf_sha256=m.digest(urdf))
        path = folder / 'idle.csv'
        with path.open('w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['t_s', 'monotonic_ns', 'read_start_ns', 'read_end_ns'] +
                            [f'q{i}_rad' for i in range(1, 8)] + [f'ff{i}_nm' for i in range(1, 8)])
            for stamp, row in zip(t, q):
                ns = 1000000000 + int(round(stamp * 1e9))
                # Huge feedforward columns must NEVER become replay commands.
                writer.writerow([stamp + .02, ns, ns - 1000000, ns, *row, *([999.] * 7)])
        (folder / 'metadata.json').write_text(json.dumps(dict(schema='rebuddy.gravity_record.v1', mock=mock, **identity)))
        (folder / 'summary.json').write_text(json.dumps(dict(mock=mock, clips=[dict(
            name='idle', file='idle.csv', complete=complete, end_reason='operator_stop',
            samples=len(t), sha256=m.digest(path))])))
        return path, identity, types.SimpleNamespace(calibration=cal, urdf=urdf, port='can0', supported=True, log_dir=folder / 'logs')

    def test_load_measured_positions_and_real_timing_only(self):
        with tempfile.TemporaryDirectory() as tmp:
            path, identity, _ = self.fixture(Path(tmp), t=[0., .023, .047, .07])
            t, q, report = m.load_clip(path, identity)
            np.testing.assert_allclose(t, [0., .023, .047, .07])
            self.assertEqual(q.shape, (4, 7))
            self.assertEqual(report['source_sha256'], m.digest(path))
            command = m.Controller(Gravity()).command(q[0], 1000, q[0], np.zeros(7))
            np.testing.assert_allclose(command['feedforward_nm'], [0., 2., 3., .5, 0., 0., 0.])

    def test_reject_mock_incomplete_wrong_identity_and_changed_csv(self):
        for case in ('mock', 'incomplete', 'identity', 'changed'):
            with self.subTest(case=case), tempfile.TemporaryDirectory() as tmp:
                path, identity, _ = self.fixture(Path(tmp), mock=case == 'mock', complete=case != 'incomplete')
                if case == 'identity': identity['calibration_sha256'] = 'wrong'
                if case == 'changed': path.write_text(path.read_text() + '\n')
                with self.assertRaises(ValueError): m.load_clip(path, identity)

    def test_reject_nonfinite_out_of_bounds_and_bad_timing(self):
        for case in ('nonfinite', 'huge', 'gap', 'duplicate', 'backwards', 'gripper'):
            with self.subTest(case=case), tempfile.TemporaryDirectory() as tmp:
                t = [0., .02, .04, .06]
                q = np.tile([0., .7, 1.1, -.3, 0., 0., 0.], (4, 1))
                if case == 'nonfinite': q[1, 3] = np.nan
                if case == 'huge': q[1, 3] = 1.78e26
                if case == 'gap': t = [0., .02, .2, .22]
                if case == 'duplicate': t = [0., .02, .02, .06]
                if case == 'backwards': t = [0., .04, .02, .06]
                if case == 'gripper': q[2, 6] = .1
                path, identity, _ = self.fixture(Path(tmp), t=t, q=q)
                with self.assertRaises(ValueError): m.load_clip(path, identity)

    def test_plan_retimes_and_has_zero_endpoint_velocity(self):
        with tempfile.TemporaryDirectory() as tmp:
            q = np.tile([0., .7, 1.1, -.3, 0., 0., 0.], (4, 1))
            q[:, 3] += [0., .01, .02, .03]
            path, identity, args = self.fixture(Path(tmp), q=q)
            with patch.object(m, 'preflight', return_value=(Gravity(), identity)):
                curve, gravity, report = m.prepare(path, args.calibration, args.urdf, .5)
            self.assertGreater(curve.duration, .06)
            self.assertLessEqual(report['peak_velocity_rad_s'], m.VMAX)
            self.assertLessEqual(report['peak_acceleration_rad_s2'], m.AMAX)
            np.testing.assert_allclose(curve.at(0)[1], 0., atol=1e-10)
            np.testing.assert_allclose(curve.at(curve.duration)[1], 0., atol=1e-10)

    def test_gravity_budget_checked_before_backend(self):
        with tempfile.TemporaryDirectory() as tmp:
            path, identity, args = self.fixture(Path(tmp))
            with patch.object(m, 'preflight', return_value=(lambda q: np.ones(6) * 99, identity)), \
                    patch.object(m, 'PlaybackBackend') as backend:
                with self.assertRaisesRegex(ValueError, '力矩预算'):
                    m.prepare(path, args.calibration, args.urdf, .5)
                backend.assert_not_called()

    def simulate(self, case):
        with tempfile.TemporaryDirectory() as tmp:
            path, identity, args = self.fixture(Path(tmp))
            t, q, report = m.load_clip(path, identity)
            curve = m.Curve(t, q)
            clock = Clock(); events = []; sent = []; handlers={}; phase=[None]
            def set_handler(sig, fn):
                old=handlers.get(sig,m.signal.SIG_DFL);handlers[sig]=fn;return old
            class Backend:
                signalled=False
                def __init__(self):
                    self.q = curve.start.copy()
                    self.enabled = False
                    if case in ('far','approach_fault'): self.q[0] += .2
                    if case in ('zero','entry_gravity'): self.q[:6] = 0.
                    if case == 'gripper': self.q[6] = .5
                    if case == 'outside': self.q[1] = -.1
                def read(self):
                    clock.now += .001
                    if case=='ctrl_before_enable' and not self.signalled:
                        self.signalled=True;handlers[m.signal.SIGINT](m.signal.SIGINT,None)
                    if self.enabled and case=='during_read_ctrl' and not self.signalled:
                        self.signalled=True;handlers[m.signal.SIGINT](m.signal.SIGINT,None)
                    if self.enabled and sent and case=='fault_and_ctrl':
                        handlers[m.signal.SIGINT](m.signal.SIGINT,None)
                        raise RuntimeError('feedback fault with pending Ctrl+C')
                    if self.enabled and sent and case == 'read_fault':
                        raise RuntimeError('invalid J4 feedback')
                    if self.enabled and sent and case == 'stale': clock.now += .2
                    if self.enabled and case=='approach_fault' and self.q[0]<.199:
                        raise RuntimeError('feedback lost during approach')
                    n = clock.monotonic_ns()
                    return self.q.copy(), n, n
                def enable(self, q):
                    events.append('enable'); self.enabled = True
                    if case in ('ctrl_after_enable','partial_enable_ctrl'):
                        handlers[m.signal.SIGINT](m.signal.SIGINT,None)
                    if case=='partial_enable_ctrl':raise RuntimeError('partial enable with Ctrl+C')
                    if case == 'partial_enable': raise RuntimeError('partial enable')
                def send(self, command):
                    sent.append(command)
                    self.q = np.array(command['target_rad'])
                    if case == 'not_arrived': self.q[0] -= np.radians(2.)
                    stages={'ctrl_startup':'startup','ctrl_approach':'approach','ctrl_play':'play',
                            'ctrl_settle':'settle','ctrl_twice':'play','home_interrupt':'play',
                            'home_fault':'play','sigterm':'play'}
                    if not self.signalled and phase[0]==stages.get(case) and case in stages:
                        self.signalled=True
                        sig=m.signal.SIGTERM if case=='sigterm' else m.signal.SIGINT
                        handlers[sig](sig,None)
                        if case=='ctrl_twice':handlers[sig](sig,None)
                def heartbeat(self): pass
                def disable(self):
                    events.append('disable'); self.enabled = False
                    return case != 'disable_failure'
                def close(self): events.append('close')
            if case == 'identity': args.calibration.write_text('changed')
            def gravity(q):
                if case=='entry_gravity' and .3 < q[1] < .4: return np.ones(6)*99
                return Gravity()(q)
            def prepare_home(backend,gravity,metadata,args,journal):
                original_put=journal.put
                def put(row):
                    if row.get('event')=='command':phase[0]=row['phase']
                    original_put(row)
                journal.put=put
                def home(cancel,**kwargs):
                    events.append('home')
                    expected=sent[-1] if sent else dict(kp=[0.]*7,kd=[0.]*7)
                    np.testing.assert_allclose(kwargs['initial_kp'],expected['kp'][:6])
                    np.testing.assert_allclose(kwargs['initial_kd'],expected['kd'][:6])
                    self.assertAlmostEqual(kwargs['initial_gravity_gain'],expected['kp'][0]/m.KP[0])
                    if case=='home_fault':raise RuntimeError('invalid feedback during home')
                    if case=='home_interrupt':handlers[m.signal.SIGINT](m.signal.SIGINT,None)
                    return not cancel()
                return types.SimpleNamespace(run=home)
            with patch.object(m.time, 'monotonic', clock.monotonic), \
                    patch.object(m.time, 'monotonic_ns', clock.monotonic_ns), \
                    patch.object(m.signal,'signal',side_effect=set_handler), \
                    patch.object(m.time, 'sleep', clock.sleep), patch('sys.stdout', io.StringIO()):
                result = m.play(curve, gravity, report, args, backend_factory=Backend,
                                zero_return_factory=prepare_home)
            summary = json.loads((Path(result['folder']) / 'summary.json').read_text())
            self.assertEqual(summary, result)
            if case=='not_arrived':
                phases=[json.loads(line).get('phase') for line in (Path(result['folder'])/'control.jsonl').read_text().splitlines()]
                self.assertIn('approach_settle',phases)
                self.assertNotIn('play',phases)
            return result, events, sent

    def test_full_playback_and_supported_cleanup(self):
        result, events, sent = self.simulate('success')
        self.assertTrue(result['success'], result)
        self.assertTrue(result['disable_confirmed'])
        self.assertEqual(events, ['enable', 'disable', 'close'])
        self.assertGreater(len(sent), 100)
        self.assertTrue(all(max(abs(np.array(c['feedforward_nm']))) <= 3. for c in sent))

    def test_first_ctrl_c_returns_before_disable_in_every_motion_phase(self):
        for case in ('ctrl_startup','ctrl_approach','ctrl_play','ctrl_settle','ctrl_after_enable','during_read_ctrl'):
            with self.subTest(case=case):
                result,events,sent=self.simulate(case)
                self.assertTrue(result['success'],result)
                self.assertTrue(result['zero_return_requested'])
                self.assertTrue(result['zero_return_completed'])
                self.assertEqual(events,['enable','home','disable','close'])
                self.assertEqual(result['reason'],f'signal_{m.signal.SIGINT}')

    def test_return_fault_and_second_ctrl_c_do_not_claim_home(self):
        for case in ('home_fault','home_interrupt'):
            result,events,_=self.simulate(case)
            self.assertFalse(result['success'])
            self.assertTrue(result['zero_return_requested'])
            self.assertFalse(result['zero_return_completed'])
            self.assertEqual(result['reason'],'zero_return_failed')
            self.assertEqual(events,['enable','home','disable','close'])

    def test_fault_abort_and_incomplete_enable_never_home(self):
        for case in ('ctrl_before_enable','partial_enable_ctrl','fault_and_ctrl','ctrl_twice','sigterm'):
            with self.subTest(case=case):
                result,events,_=self.simulate(case)
                self.assertFalse(result['success'])
                self.assertFalse(result['zero_return_requested'])
                self.assertNotIn('home',events)

    def test_invalid_entry_or_changed_files_do_not_enable(self):
        for case in ('outside', 'identity', 'entry_gravity'):
            result, events, sent = self.simulate(case)
            self.assertFalse(result['success'])
            self.assertFalse(result['enable_requested'])
            self.assertNotIn('enable', events)
            self.assertEqual(sent, [])

    def test_far_and_zero_starts_approach_smoothly_before_play(self):
        for case in ('far','zero','gripper'):
            result,events,sent=self.simulate(case)
            self.assertTrue(result['success'],result)
            self.assertTrue(result['entry_completed'])
            initial=np.array([0.,.7,1.1,-.3,0.,0.,0.])
            if case=='far':initial[0]+=.2
            if case=='zero':initial[:6]=0.
            if case=='gripper':initial[6]=.5
            np.testing.assert_allclose(sent[0]['target_rad'],initial,atol=1e-12)
            targets=np.array([c['target_rad'] for c in sent])
            self.assertLess(np.max(abs(np.diff(targets,axis=0))),m.VMAX*.025)
            np.testing.assert_allclose(targets[:,6],initial[6],atol=1e-12)
            self.assertTrue(all(np.max(np.abs(c['velocity_rad_s']))<=m.VMAX+1e-8 for c in sent))

    def test_entry_must_settle_before_playback(self):
        result,events,sent=self.simulate('not_arrived')
        self.assertFalse(result['success'])
        self.assertFalse(result['entry_completed'])
        self.assertTrue(result['disable_confirmed'])
        self.assertIn('未稳定到达',result['errors'][0])

    def test_feedback_fault_during_approach_never_starts_playback(self):
        result,events,sent=self.simulate('approach_fault')
        self.assertFalse(result['success'])
        self.assertFalse(result['entry_completed'])
        self.assertTrue(result['disable_confirmed'])
        self.assertIn('feedback lost during approach',result['errors'][0])
        self.assertGreater(len(sent),75)

    def test_entry_bounds_speed_acceleration_and_endpoint_continuity(self):
        initial=np.zeros(7);initial[6]=.5
        target=np.array([-.03,.78,1.16,-.49,-.05,-.01,0.])
        entry=m.prepare_entry(initial,target,Gravity(),.5)
        self.assertLessEqual(entry.peak_velocity,m.VMAX*.5+1e-10)
        self.assertLessEqual(entry.peak_acceleration,m.AMAX*.25+1e-10)
        np.testing.assert_allclose(entry.at(0)[0],initial,atol=1e-12)
        target[6]=initial[6]
        np.testing.assert_allclose(entry.at(entry.duration)[0],target,atol=1e-12)
        for t in (0,entry.duration):
            np.testing.assert_allclose(entry.at(t)[1],0,atol=1e-12)
            np.testing.assert_allclose(entry.spline(t*entry.scale,nu=2)*entry.scale**2,0,atol=1e-12)

    def test_measured_start_allowance_only_allows_inward_entry(self):
        for degrees in (-.112,-.200195886,-2.999):
            initial=np.zeros(7);initial[1]=np.radians(degrees)
            target=np.zeros(7);target[1]=.7
            curve=m.prepare_entry(initial,target,Gravity(),.5)
            control=m.Controller(Gravity(),initial=initial)
            control.command(initial,1_000_000_000,initial,np.zeros(7))
            positions=np.array([curve.at(t)[0] for t in np.linspace(0,curve.duration,101)])
            self.assertGreaterEqual(positions[:,1].min(),initial[1]-1e-10)
            self.assertTrue(np.all(np.diff(positions[:,1])>=-1e-10))
            outward=initial.copy();outward[1]-=.001
            with self.assertRaisesRegex(RuntimeError,'outside joint bounds'):
                m.Controller(Gravity(),initial=initial).command(initial,1_000_000_000,outward,np.zeros(7))
        initial=np.zeros(7);initial[1]=np.radians(-3.001)
        with self.assertRaises(ValueError):m.prepare_entry(initial,np.zeros(7),Gravity(),.5)
        initial[1]=np.radians(-.112)
        bad_target=initial.copy()
        with self.assertRaises(ValueError):m.prepare_entry(initial,bad_target,Gravity(),.5)

    def test_feedback_failure_partial_enable_and_disable_failure(self):
        for case in ('read_fault', 'partial_enable', 'stale', 'disable_failure'):
            with self.subTest(case=case):
                result, events, sent = self.simulate(case)
                self.assertFalse(result['success'])
                self.assertTrue(result['disable_requested'])
                self.assertEqual(events, ['enable', 'disable', 'close'])
                if case == 'partial_enable': self.assertEqual(sent, [])
                if case == 'read_fault': self.assertEqual(len(sent), 1)

    def test_controller_rejects_tracking_and_combined_torque_excess(self):
        q = np.array([0., .7, 1.1, -.3, 0., 0., 0.])
        for index, change in ((0, .2), (4, .085)):
            target = q.copy(); target[index] += change
            velocity=np.zeros(7);velocity[index]=m.VMAX
            with self.assertRaises(RuntimeError):
                m.Controller(Gravity()).command(q, 1000, target, velocity)

    def test_larger_measurement_tolerance_does_not_expand_targets(self):
        q = np.array([0., -.01, 1.1, -.3, 0., 0., 0.])
        target = q.copy(); target[1] = 0.
        m.Controller(Gravity()).command(q, 1000, target, np.zeros(7))
        with self.assertRaisesRegex(RuntimeError, 'outside joint bounds'):
            m.Controller(Gravity()).command(q, 1000, q, np.zeros(7))

    def test_cli_inspect_never_plays_and_play_requires_explicit_support(self):
        with patch.object(m, 'prepare', return_value=(None, None, {})) as prepare, \
                patch.object(m, 'play') as play, patch('sys.stdout', io.StringIO()), \
                patch('sys.stderr', io.StringIO()), patch.object(m.sys.stdin, 'isatty', return_value=False):
            self.assertEqual(m.main(['play', 'idle.csv']), 2)
            self.assertEqual(m.main(['play', 'idle.csv', '--supported']), 2)
            prepare.assert_not_called()
            self.assertEqual(m.main(['inspect', 'idle.csv']), 0)
            play.assert_not_called()


if __name__ == '__main__':
    unittest.main()
