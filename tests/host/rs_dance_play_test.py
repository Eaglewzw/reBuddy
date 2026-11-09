"""Prepared dance and protected playback tests; synthetic feedback, no CAN."""
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
sys.path.insert(0, str(ROOT/'tools'))
import rs_dance_play as dance
import rs_gravity_play as player
import rs_gravity_play_test as fixtures
Gravity, Clock = fixtures.Gravity, fixtures.Clock


class Tests(unittest.TestCase):
    def source(self, folder):
        t = np.arange(151)*.02
        q = np.tile([0., .7, 1.1, -.3, 0., 0., 0.], (len(t), 1))
        q[:101, 3] += .4*np.sin(2*np.pi*t[:101]/.7)
        path, identity, args = fixtures.Tests().fixture(folder, t=t, q=q, complete=False)
        summary = json.loads((folder/'summary.json').read_text())
        summary.update(reason='fault', errors=['RuntimeError: 自动回位未通过首尾检查：unstable'], logging_errors=[])
        summary['clips'][0].update(end_reason='protective_stop', auto_return=True,
            manual_end_ns=3_000_000_000, manual_end_rad=q[100].tolist(),
            anchor=dict(action='DANCE_NOD', entry_verified={'samples':51}, exit_idle=True,
                        idle_pose=dict(q_rad=q[0].tolist(), **identity)))
        (folder/'summary.json').write_text(json.dumps(summary))
        return path, identity, args

    def test_preserves_time_reduces_amplitude_and_keeps_sources_immutable(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder=Path(tmp); path, identity, args=self.source(folder)
            before={p.name:p.read_bytes() for p in folder.iterdir()}
            with patch.object(dance, 'preflight', return_value=(Gravity(), identity)):
                plan, _, spec=dance.derive(path,args.calibration,args.urdf)
                self.assertAlmostEqual(plan.duration,3.)
                self.assertAlmostEqual(spec['manual_duration_s'],2.)
                self.assertLess(spec['amplitude_scale'],.2)
                for _, curve in plan.stages:
                    self.assertEqual(curve.scale,1.)
                    self.assertLessEqual(curve.peak_velocity,1.)
                    self.assertLessEqual(curve.peak_acceleration,2.)
                    for stamp in (0,curve.duration):
                        np.testing.assert_allclose(curve.at(stamp)[1],0.,atol=1e-9)
                saved=folder/'prepared.json';saved.write_text(json.dumps(spec))
                dance.load_prepared(saved,args.calibration,args.urdf)
                spec['amplitude_scale']=1.;saved.write_text(json.dumps(spec))
                with self.assertRaisesRegex(ValueError,'变化'):
                    dance.load_prepared(saved,args.calibration,args.urdf)
            for name, contents in before.items():self.assertEqual((folder/name).read_bytes(),contents)

    def test_recovery_is_narrow_and_generic_player_stays_strict(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder=Path(tmp);path,identity,args=self.source(folder)
            with self.assertRaises(ValueError):player.load_clip(path,identity)
            for field,value in [('errors',['RuntimeError: invalid motor feedback']),('logging_errors',['disk full'])]:
                summary_path=folder/'summary.json';old=summary_path.read_text();summary=json.loads(old)
                summary[field]=value;summary_path.write_text(json.dumps(summary))
                with self.assertRaises(ValueError):
                    player.load_clip(path,identity,recover_endpoint_failure=True)
                summary_path.write_text(old)

    def test_play_completion_fault_and_estop_hold_until_explicit_command(self):
        for case in ('complete','fault','estop','home_failure'):
            with self.subTest(case=case), tempfile.TemporaryDirectory() as tmp:
                folder=Path(tmp);path,identity,args=fixtures.Tests().fixture(folder)
                t,q,report=player.load_clip(path,identity);curve=player.Curve(t,q)
                args.protective_hold=True;commands=queue.Queue();clock=Clock()
                state=dict(phase='',held=0,hold_ticks=0,injected=False);events=[]
                class Backend:
                    actual=curve.start.copy()
                    def read(self):
                        if case=='fault' and state['phase']=='play' and not state['injected']:
                            state['injected']=True;raise RuntimeError('synthetic feedback interruption')
                        clock.now += .001
                        n=clock.monotonic_ns();return self.actual.copy(),n,n
                    def enable(self,q):events.append('enable')
                    def heartbeat(self):pass
                    def send(self,c):
                        self.actual=np.array(c['target_rad'])
                        if case=='estop' and state['phase']=='play' and not state['injected']:
                            state['injected']=True;commands.put('estop')
                        if state['held']:
                            state['hold_ticks']+=1
                            if state['hold_ticks']==10:
                                events.append('operator_command')
                                commands.put('release' if state['held']==2 else 'home')
                    def disable(self):events.append('disable');return True
                    def close(self):events.append('close')
                def zero_factory(backend,gravity,report,args,journal):
                    put=journal.put
                    def observe(row):
                        if row['event']=='command':state['phase']=row['phase']
                        if row['event']=='protective_hold_entered':
                            state['held']+=1;state['hold_ticks']=0;events.append('held')
                        put(row)
                    journal.put=observe
                    def home(*a,**kw):
                        events.append('home');state['held']=1
                        return case!='home_failure'
                    return types.SimpleNamespace(run=home)
                with patch.object(player.time,'monotonic',clock.monotonic), \
                        patch.object(player.time,'monotonic_ns',clock.monotonic_ns), \
                        patch.object(player.time,'sleep',clock.sleep),patch('sys.stdout',io.StringIO()):
                    result=player.play(curve,Gravity(),report,args,backend_factory=Backend,
                                       zero_return_factory=zero_factory,commands=commands)
                self.assertTrue(result['protective_hold_requested'],result)
                self.assertLess(events.index('held'),events.index('operator_command'))
                self.assertLess(events.index('operator_command'),events.index('home'))
                self.assertLess(events.index('home'),events.index('disable'))
                if case=='home_failure':self.assertEqual(events.count('held'),2)
                self.assertEqual(result['success'],case=='complete',result)


if __name__=='__main__':unittest.main()
