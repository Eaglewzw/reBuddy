"""Trajectory compilation and mocked continuous execution; never opens CAN."""
import json
import io
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'tools'))
from rs_dance_sequence import QuinticBridge, SequencePlan, check_bridge, state
from rs_dance_library import load_library
from rs_dance_speed import ScaledPlan
from rs_dance_virtual_collision import CollisionAudit
from rs_gravity_record import DEFAULT_CALIBRATION, DEFAULT_URDF
from rs_dance_sequence_cli import main, select_numbers
from dance_mock import MockPlayback


class PolynomialTests(unittest.TestCase):
    def test_random_selection_is_unique_and_reproducible(self):
        for seed in range(30):
            for count in (1,5,10):
                numbers,selection=select_numbers([],count,seed)
                self.assertEqual(len(numbers),count)
                self.assertEqual(len(set(numbers)),count)
                self.assertTrue(set(numbers)<=set(f'{i:02}' for i in range(1,11)))
                self.assertEqual(select_numbers([],count,selection['seed']),(numbers,selection))
        with patch('rs_dance_sequence_cli.random.SystemRandom') as rng:
            rng.return_value.getrandbits.return_value=42
            self.assertEqual(select_numbers([],5),select_numbers([],5,42))
        self.assertEqual(select_numbers(['01','01'])[0],['01','01'])
        self.assertEqual(len(select_numbers([])[0]),10)

    def test_closed_gripper_uses_new_sample_profile(self):
        from rs_dance_console import DanceSession
        session=DanceSession.__new__(DanceSession)
        original=np.radians(10.8)
        session.controller=SimpleNamespace(following_limit=np.full(7,original))
        def sampler():
            session.controller.following_limit[6]=np.radians(5.)
            return np.zeros(7),np.zeros(7),1.
        def tick(*args,**kwargs):
            kwargs['target_sampler']()
            self.assertAlmostEqual(session.controller.following_limit[6],np.radians(6.))
        with patch('rs_daily_session.Session.tick',side_effect=tick):
            session.tick(None,target_sampler=sampler)
        self.assertEqual(session.controller.following_limit[6],original)

    def test_nonzero_boundaries_and_extrema(self):
        rng=np.random.default_rng(20261110)
        for _ in range(20):
            left,right=rng.normal(size=(2,3,7))
            T=float(rng.uniform(.2,2))
            c=QuinticBridge(left,right,T)
            np.testing.assert_allclose(state(c,0),left,atol=1e-10)
            np.testing.assert_allclose(state(c,T),right,atol=1e-10)
            samples=np.array([state(c,t) for t in np.linspace(0,T,101)])
            self.assertTrue(np.all(samples[:,0]>=c.lower-1e-10))
            self.assertTrue(np.all(samples[:,0]<=c.upper+1e-10))
            self.assertTrue(np.all(abs(samples[:,1])<=c.peak_velocity+1e-10))
            self.assertTrue(np.all(abs(samples[:,2])<=c.peak_acceleration+1e-10))
            h=T*1e-5;t=.43*T
            np.testing.assert_allclose((c.at(t+h)[0]-c.at(t-h)[0])/(2*h),c.at(t)[1],atol=1e-7)
            np.testing.assert_allclose((c.at(t+h)[1]-c.at(t-h)[1])/(2*h),c.acceleration_at(t),atol=1e-6)

    def test_interior_overshoot_rejected(self):
        left=np.zeros((3,7));right=left.copy()
        left[1,0]=40.;right[1,0]=-40.
        c=QuinticBridge(left,right,1.)
        self.assertGreater(c.upper[0],2.8)
        with self.assertRaisesRegex(ValueError,'joint bound'):
            check_bridge(c,None,None)
        for T in (0,-1,float('nan')):
            with self.assertRaises(ValueError):QuinticBridge(left,right,T)

    def test_invalid_cli_never_loads_or_connects(self):
        with patch('rs_dance_sequence_cli.load_library',side_effect=AssertionError('should not load')):
            self.assertEqual(main(['check','00']),2)
            self.assertEqual(main(['check','12']),2)
            self.assertEqual(main(['check','01','--speed','nan']),2)
            self.assertEqual(main(['play','01']),2)
            for args in (['check','01','--random'],['check','--random','0'],
                         ['check','--random','11'],['check','--seed','42'],
                         ['check','--random','--seed','-1']):
                self.assertEqual(main(args),2)


class IntegrationTests(unittest.TestCase):
    simulate_console=MockPlayback.simulate_console

    @classmethod
    def setUpClass(cls):
        cls.manifest,cls.sources,cls.gravity,cls.identity,cls.source_reports=load_library(DEFAULT_CALIBRATION,DEFAULT_URDF)
        cls.collision=CollisionAudit(cls.gravity)
        cls.sequence=SequencePlan([f'{i:02}' for i in range(1,11)],cls.sources,cls.gravity,cls.collision)
        cls.idle=np.asarray(cls.manifest['idle_rad'])

    def setUp(self):
        tmp=tempfile.TemporaryDirectory();self.addCleanup(tmp.cleanup);self.root=Path(tmp.name)
        self.plans={n:self.sources[n] for n in ('00','11')}
        self.plans['12']=ScaledPlan(self.sequence,.5)
        self.reports={n:self.source_reports[n] for n in ('00','11')}
        self.reports['12']={'name':'test sequence','duration_s':self.sequence.duration}

    def test_whole_sequence_and_preserved_bodies(self):
        s=self.sequence
        report=s.audit(self.gravity,self.collision)
        self.assertLess(max(report['seam_max_errors_q_v_a']),1e-8)
        self.assertEqual(len(s.connections),9)
        self.assertEqual(len(s.stages),1)
        for entry,curve in zip(s.layout,s.parts):
            if entry['kind']!='action':continue
            original=self.sources[entry['number']]
            for t in np.linspace(0,curve.duration,31):
                np.testing.assert_allclose(state(curve,t),state(original,t+entry['source_start_s']),atol=1e-10)
            core=original.stages[0][1]
            if hasattr(core,'pulse_starts_s'):
                self.assertLess(entry['source_start_s'],core.pulse_starts_s[0])
                self.assertGreater(entry['source_end_s'],core.pulse_starts_s[-1]+core.pulse.duration)
            if hasattr(core,'dance_interval_s'):
                self.assertLess(entry['source_start_s'],core.dance_interval_s[0])
                self.assertGreater(entry['source_end_s'],core.dance_interval_s[1])
        scaled=self.plans['12']
        for t in np.linspace(0,s.duration,51):
            q,v,a=state(s,t);actual=state(scaled,2*t)
            np.testing.assert_allclose(actual,np.array([q,v*.5,a*.25]),atol=1e-10)
        from rs_gravity_play import FOLLOW_TOLERANCE
        for piece in s.layout:
            t=piece['start_s']+.5*piece['duration_s']
            speed,following=scaled.gripper_profile_at(2*t)
            if piece['kind']=='bridge' or piece['number'] not in ('02','06'):
                self.assertAlmostEqual(following,FOLLOW_TOLERANCE)
            else:
                self.assertAlmostEqual(following,self.sources[piece['number']].gripper_following_limit)

    def test_single_and_repeated_actions(self):
        single=SequencePlan(['02'],self.sources,self.gravity,self.collision)
        self.assertEqual(single.duration,self.sources['02'].duration)
        repeated=SequencePlan(['08','08','01'],self.sources,self.gravity,self.collision)
        self.assertEqual(len(repeated.connections),2)
        self.assertLess(max(repeated.seam_errors),1e-8)
        with self.assertRaises(ValueError):SequencePlan([],self.sources,self.gravity,self.collision)
        with self.assertRaises(ValueError):SequencePlan(['11'],self.sources,self.gravity,self.collision)

    def test_random_selection_compiles_and_plays_with_same_bridges(self):
        numbers,selection=select_numbers([],5,42)
        sequence=SequencePlan(numbers,self.sources,self.gravity,self.collision)
        checks=sequence.audit(self.gravity,self.collision)
        self.assertLess(max(checks['seam_max_errors_q_v_a']),1e-8)
        self.assertEqual(len(sequence.connections),4)
        self.plans['12']=ScaledPlan(sequence,.5)
        result,events,rows=self.simulate_console(action='12')
        self.assertTrue(result['success'],result['errors'])
        self.assertEqual(result['completed'],['12'])
        self.assertEqual(sum(r['event']=='segment_start' and r['phase'].startswith('12:') for r in rows),1)
        self.assertEqual(sequence.numbers,numbers)
        self.assertEqual(select_numbers([],5,selection['seed'])[0],numbers)

    def test_random_play_cli_default_count_and_report(self):
        loaded=(self.manifest,self.sources,self.gravity,self.identity,self.source_reports)
        result=dict(success=True,folder=str(self.root/'mock-session'),errors=[])
        with patch('rs_dance_sequence_cli.load_library',return_value=loaded), \
                patch('sys.stdin.isatty',return_value=True), \
                patch('sys.stdout',io.StringIO()), \
                patch('rs_dance_console.run',return_value=result) as playback:
            status=main(['play','--random','--seed','42','--supported','--output',str(self.root)])
        self.assertEqual(status,0)
        playback.assert_called_once()
        numbers,selection=select_numbers([],5,42)
        compiled=playback.call_args.args[2]['12']
        self.assertEqual(compiled.source.numbers,numbers)
        saved=json.loads((self.root/'random-42/report.json').read_text())
        self.assertEqual(saved['numbers'],numbers)
        self.assertEqual(saved['selection'],selection)
        self.assertEqual(len(saved['connections']),4)

    def test_one_control_segment_no_intermediate_settle(self):
        result,events,rows=self.simulate_console(action='12')
        self.assertTrue(result['success'],result['errors'])
        self.assertEqual(result['completed'],['12'])
        starts=[r for r in rows if r['event']=='segment_start' and r['phase'].startswith('12:')]
        self.assertEqual(len(starts),1)
        commands=[r for r in rows if r['event']=='command' and r['phase']=='12:continuous_sequence']
        self.assertGreater(len(commands),100)
        self.assertTrue(all(b['target_sample_ns']>a['target_sample_ns'] for a,b in zip(commands,commands[1:])))
        for r in commands:self.assertGreaterEqual(r['target_sample_ns'],r['feedback_end_ns'])
        self.assertEqual(events.count('enable'),1)
        self.assertEqual(events.count('home'),1)

    def test_fault_latches_hold_without_next_action(self):
        result,events,rows=self.simulate_console(action='12',fail=True)
        self.assertEqual(result['completed'],[])
        self.assertIn('held',events)
        self.assertLess(events.index('held'),events.index('operator_exit'))
        self.assertLess(events.index('operator_exit'),events.index('home'))
        self.assertLess(events.index('home'),events.index('disable'))
        self.assertEqual(sum(r['event']=='action_requested' for r in rows),1)


if __name__=='__main__':unittest.main()
