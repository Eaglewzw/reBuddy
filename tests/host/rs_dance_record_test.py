"""Dance recording checks with synthetic feedback only; never opens CAN."""
import io
import json
from pathlib import Path
import queue
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tools'))
import rs_dance_record as dance
import rs_gravity_record as rec


class Gravity:
    lower = np.full(6, -3.)
    upper = np.full(6, 3.)

    def __call__(self, q):
        return np.zeros(6)


class Tests(unittest.TestCase):
    def setUp(self):
        self.profile = dance.DanceProfile()
        self.meta = dict(calibration_sha256='cal', urdf_sha256='urdf', mock=False)
        self.q = np.zeros(7)
        self.history = [(i * 20_000_000, self.q.copy()) for i in range(51)]
        self.pose = rec.capture_idle_pose(self.history, self.meta, Path('/tmp'))

    def test_catalog_and_unique_takes(self):
        self.assertEqual([a[2] for a in dance.ACTIONS], list(range(33, 43)))
        metadata = self.profile.metadata()
        self.assertEqual(sum(a['recording_required'] for a in metadata['actions']), 9)
        self.assertEqual(metadata['actions'][8]['source'], 'procedural')
        self.assertFalse(metadata['bookend_recording_required'])
        self.assertEqual([(a['action_id'], a['daily_action']) for a in metadata['bookends']],
                         [(43, 'WAKE'), (44, 'REST')])
        self.assertTrue(all(not a['recording_required'] for a in metadata['bookends']))
        self.assertEqual(metadata['motion_binding_status'], 'planned_not_connected')
        for i, (key, name, _) in enumerate(dance.ACTIONS, 1):
            for alias in (str(i), f'{i:02d}', name, key):
                if key == 'grip':
                    with self.assertRaisesRegex(ValueError, '无需录制'):
                        self.profile.command(alias, set())
                else:
                    self.assertEqual(self.profile.command(alias, {f'dance_{key}_01'}),
                                     f'start dance_{key}_02')
        self.assertEqual(dance.DEFAULT_OUTPUT, ROOT / 'record/dance')

    def test_endpoints_require_same_stable_pose(self):
        args = (self.meta, Path('/tmp'))
        anchor = self.profile.anchor('dance_nod_01', self.pose, self.q, self.history, *args)
        self.assertEqual(anchor['action_id'], 33)
        self.profile.validate_end(anchor, self.q, self.history, *args)
        wrong = self.q.copy(); wrong[0] = np.deg2rad(3.)
        moving = [(t, q.copy()) for t, q in self.history]
        moving[20][1][0] = np.deg2rad(.6)
        gap = self.history[:10] + self.history[20:]
        for q, history in ((wrong, self.history), (self.q, moving),
                           (self.q, gap), (self.q, self.history[:20])):
            with self.subTest(q=q, size=len(history)):
                with self.assertRaises(ValueError):
                    self.profile.anchor('dance_nod_01', self.pose, q, history, *args)
                with self.assertRaises(ValueError):
                    self.profile.validate_end(anchor, q, history, *args)
        with self.assertRaises(ValueError):
            self.profile.anchor('dance_grip_01', self.pose, self.q, self.history, *args)
        with self.assertRaises(ValueError):
            self.profile.anchor('probe', self.pose, self.q, self.history, *args)
        with self.assertRaises(ValueError):
            self.profile.anchor('dance_nod_01', None, self.q, self.history, *args)

    def test_cli_does_not_open_hardware_for_list_check_or_rejected_start(self):
        with patch.object(rec, 'run') as run, patch.object(rec, 'preflight', return_value=(None, {})) as preflight, \
                patch.object(dance, 'daily_idle_pose', return_value=self.pose), \
                patch('sys.stdout', io.StringIO()), patch('sys.stderr', io.StringIO()):
            self.assertEqual(dance.main(['--list']), 0)
            preflight.assert_not_called()
            self.assertEqual(dance.main([]), 2)
            preflight.assert_not_called()
            self.assertEqual(dance.main(['--check']), 0)
            with patch('sys.stdin.isatty', return_value=False):
                self.assertEqual(dance.main(['--supported']), 2)
            run.assert_not_called()

    def test_daily_anchor_identity_and_default_startup(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'manifest.json'
            data=dict(schema='rebuddy.daily.prepared.v1',idle_rad=[0.]*7,
                      calibration_sha256='cal',urdf_sha256='urdf')
            path.write_text(json.dumps(data))
            pose=dance.daily_idle_pose(self.meta,Gravity(),path)
            self.assertEqual(pose['q_rad'],[0.]*7)
            self.assertEqual(pose['source_sha256'],rec.digest(path))
            for key in ('calibration_sha256','urdf_sha256'):
                wrong=dict(self.meta);wrong[key]='different'
                with self.assertRaises(ValueError):dance.daily_idle_pose(wrong,Gravity(),path)
            data['idle_rad']=[0.]*6+[float('nan')];path.write_text(json.dumps(data))
            with self.assertRaises(ValueError):dance.daily_idle_pose(self.meta,Gravity(),path)
        with patch.object(rec,'preflight',return_value=(Gravity(),self.meta)), \
                patch.object(dance,'daily_idle_pose',return_value=pose), \
                patch.object(rec,'run',return_value=dict(folder='synthetic',success=True,errors=[])) as run, \
                patch('sys.stdin.isatty',return_value=True), patch('sys.stdout',io.StringIO()):
            self.assertEqual(dance.main(['--supported']),0)
            args=run.call_args.args[0]
            self.assertFalse(args.manual_start)
            self.assertEqual(args.startup_pose,pose)
            self.assertTrue(args.recording_profile.shared_daily_pose)
            with self.assertRaises(ValueError):args.recording_profile.command('pose replace',set())

    def simulate_return(self, case):
        from rs_idle_entry import IdleEntry
        commands = queue.Queue(); events = []; entries = []; clock = [10.]
        target = np.array([0., .4, .6, -.2, 0., 0., 0.])
        offset = np.radians([0., 2.70, 0., -2.67, 0., 0., 0.])

        class Backend(rec.MockBackend):
            ticks = 0
            after_return = 0
            q = target.copy()
            def read(self):
                stamp = rec.time.monotonic_ns()
                return self.q.copy(), stamp, stamp
            def send(self, command):
                super().send(command)
                self.ticks += 1
                if entries:
                    if case != 'stuck':
                        self.q = np.array(command['target_rad'])
                    if case == 'release_drift' and entries[-1].phase == 'release':
                        self.q[5] += np.radians(1.2)
                    if entries[-1].done:
                        self.after_return += 1
                elif 170 <= self.ticks <= 190:
                    self.q = target + offset * ((self.ticks - 170) / 20.)
                script = {150: '01', 205: 'stop', 210: 'stop', 211: '02'}
                if case in ('quit', 'estop'):
                    script[210] = case
                if self.ticks in script:
                    commands.put(script[self.ticks])
                if self.after_return == 60:
                    commands.put('02')
                if self.after_return == 70 or self.ticks == 2000:
                    commands.put('quit')
            def disable(self):
                events.append('disable')
                return super().disable()

        def factory(gravity, initial, pose, **kw):
            if case == 'plan_failure':
                raise ValueError('synthetic invalid return plan')
            entry = IdleEntry(gravity, initial, pose, **kw)
            entries.append(entry)
            return entry

        def zero_factory(*_):
            return types.SimpleNamespace(run=lambda cancel, **kw: events.append('zero') or True)

        def hold(*_):
            events.append('hold')
            return 'release'  # Simulated explicit operator release after hold.

        with tempfile.TemporaryDirectory() as tmp, patch('sys.stdout', io.StringIO()) as output, \
                patch.object(rec.time, 'monotonic', side_effect=lambda: clock[0]), \
                patch.object(rec.time, 'monotonic_ns', side_effect=lambda: int(clock[0]*1e9)), \
                patch.object(rec.time, 'sleep', side_effect=lambda dt: clock.__setitem__(0, clock[0]+dt)), \
                patch('rs_protective_stop.maintain_hold', side_effect=hold):
            folder = Path(tmp); cal = folder / 'cal.json'; cal.write_text('{}')
            meta = dict(self.meta, calibration_sha256=rec.digest(cal))
            pose = dict(self.pose, calibration_sha256=meta['calibration_sha256'], q_rad=target.tolist())
            args = types.SimpleNamespace(mode='record', supported=True, seconds=0., output=folder,
                                         calibration=cal, manual_start=True, startup_pose=pose,
                                         recording_profile=self.profile)
            if case == 'endpoint_failure':
                self.profile.validate_end = lambda *a: (_ for _ in ()).throw(ValueError('unstable endpoint'))
            result = rec.run(args, Gravity(), meta, backend_factory=Backend, commands=commands,
                             zero_return_factory=zero_factory, idle_entry_factory=factory)
            rows = [json.loads(line) for line in (Path(result['folder']) / 'session.jsonl').read_text().splitlines()]
            if case in ('success', 'release_drift'):
                from rs_gravity_play import load_clip
                t, q, report = load_clip(Path(result['folder']) / 'dance_nod_01.csv', meta)
                np.testing.assert_allclose(q[-1], target, atol=1e-9)
                self.assertLessEqual(np.max(np.diff(t)), .1)
                self.assertTrue(report['source_complete'])
            return result, rows, events, output.getvalue(), entries

    def test_teaching_release_drift_is_outside_completed_capture(self):
        result, rows, events, output, entries = self.simulate_return('release_drift')
        self.assertTrue(result['success'], result)
        self.assertTrue(result['clips'][0]['complete'])
        end = next(r['monotonic_ns'] for r in rows if r['event']=='clip_end')
        release = next(r['monotonic_ns'] for r in rows
                       if r['event']=='idle_entry_command' and r['phase']=='release')
        self.assertLess(end, release)

    def test_stop_records_measured_return_then_verifies_and_completes(self):
        result, rows, events, output, entries = self.simulate_return('success')
        self.assertTrue(result['success'], result)
        self.assertEqual(events, ['zero', 'disable'])
        self.assertEqual(len(entries), 1)  # Repeated stop did not restart motion.
        self.assertFalse(entries[0].zero_completed)  # Direct return, no zero detour.
        self.assertIn('正在自动补回待机', output)
        self.assertEqual(len(result['clips']), 2)
        first, second = result['clips']
        self.assertTrue(first['complete'])
        self.assertTrue(first['auto_return'])
        self.assertIn('exit_verified', first)
        self.assertGreaterEqual(first['exit_verified']['samples'], 48)
        initial = np.array(first['anchor']['idle_pose']['q_rad'])
        delta = np.degrees(np.array(first['manual_end_rad']) - initial)
        np.testing.assert_allclose(delta[[1, 3]], [2.70, -2.67], atol=1e-8)
        tail = [r for r in rows if r.get('phase') == 'auto_return']
        self.assertGreater(len(tail), 100)
        self.assertGreater(tail[0]['monotonic_ns'], first['manual_end_ns'])
        self.assertFalse(second['complete'])
        self.assertEqual(second['name'], 'dance_lift_01')
        self.assertEqual(second['end_reason'], 'quit')

    def test_failed_or_interrupted_return_never_marks_complete(self):
        for case in ('plan_failure', 'stuck', 'endpoint_failure', 'estop', 'quit'):
            with self.subTest(case=case):
                self.profile = dance.DanceProfile()
                result, rows, events, output, entries = self.simulate_return(case)
                self.assertEqual(len(result['clips']), 1, result)
                clip = result['clips'][0]
                self.assertFalse(clip['complete'])
                self.assertNotIn('exit_verified', clip)
                if case == 'quit':
                    self.assertEqual(clip['end_reason'], 'quit')
                    self.assertEqual(events, ['zero', 'disable'])
                else:
                    self.assertTrue(result['protective_hold_requested'])
                    self.assertEqual(clip['end_reason'], 'protective_stop')
                    self.assertEqual(events, ['hold', 'disable'])


if __name__ == '__main__':
    unittest.main()
