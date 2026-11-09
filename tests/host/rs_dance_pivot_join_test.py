"""Offline regression for the continuous v10 horizontal/vertical handoff."""
import copy
import json
from pathlib import Path
import sys
import tempfile
import unittest
import numpy as np

ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT/'tools'))
import rs_dance_showcase as showcase
import rs_dance_pivot_join as join
import rs_dance_speed as speed
import rs_dance_pivot_test as legacy
from rs_gravity_record import DEFAULT_CALIBRATION,DEFAULT_URDF,CAPS
from rs_gravity_play import Controller


class Tests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.manifest,cls.plans,cls.gravity,cls.identity,cls.reports=showcase.load_library(
            DEFAULT_CALIBRATION,DEFAULT_URDF,ROOT/'record/dance_showcase/v10')
        cls.plan=cls.plans['21'];cls.segment=cls.plan.stages[0][1]
        cls.idle=np.asarray(cls.manifest['idle_rad'])

    def setUp(self):
        tmp=tempfile.TemporaryDirectory();self.addCleanup(tmp.cleanup);self.root=Path(tmp.name)

    def test_original_amplitudes_clock_and_unedited_motion_preserved(self):
        old=ROOT/'record/dance_showcase/v9';new=old.with_name('v10')
        previous=json.loads((old/'21_fixed_tcp_pivot.json').read_text())
        spec=self.reports['21'];segment=self.segment;source=segment.source;bridge=segment.bridge
        self.assertEqual(spec['stages'][0]['source'],previous['stages'][0])
        self.assertEqual(spec['pivot_sway_design'],previous['pivot_sway_design'])
        self.assertEqual(self.plan.duration,previous['duration_s'])
        self.assertEqual(speed.trial_speed_for(self.plans),1.)
        for f in old.glob('*.json'):
            if f.name not in ('manifest.json','21_fixed_tcp_pivot.json'):
                self.assertEqual(f.read_bytes(),(new/f.name).read_bytes())
        for t in np.r_[np.linspace(0,bridge.start_s-1e-6,250),np.linspace(bridge.end_s+1e-6,source.duration,250)]:
            np.testing.assert_array_equal(segment.at(t),source.at(t))
            np.testing.assert_array_equal(segment.acceleration_at(t),source.acceleration_at(t))
        for phase in (1,2):
            curve=source.stages[phase][1]
            for local in curve.ends[:2]:
                t=source.ends[phase-1]+local
                np.testing.assert_array_equal(segment.at(t),source.at(t))

    def test_join_is_c2_and_has_no_stationary_handoff(self):
        s=self.segment;b=s.bridge
        for t,local in ((b.start_s,0.),(b.end_s,b.duration)):
            np.testing.assert_allclose(b.at(local),s.source.at(t),atol=1e-10)
            np.testing.assert_allclose(b.acceleration_at(local),s.source.acceleration_at(t),atol=1e-9)
        seam=s.source.ends[1]
        self.assertLess(max(abs(s.source.at(seam)[1])),1e-8)
        for t in np.linspace(b.start_s,b.end_s,1001):
            self.assertGreater(max(abs(s.at(t)[1][:6])),.05)
        self.assertLessEqual(max(s.peak_velocity),1.)
        self.assertLessEqual(max(s.peak_acceleration),2.)
        changed=copy.deepcopy(self.reports['21']['stages'][0])
        changed['bridge']['start_s']=s.source.ends[0]
        with self.assertRaisesRegex(ValueError,'extremum'):
            join.JoinedSwaySegment(changed)

    def test_independent_fk_keeps_original_virtual_point(self):
        pin=self.gravity.pin;model=self.gravity.model;data=model.createData();frame=model.getFrameId('gripper_end')
        idle=np.asarray(self.manifest['idle_rad'])
        def point(q):
            pin.forwardKinematics(model,data,q[:6]);pin.updateFramePlacements(model,data)
            pose=data.oMf[frame];return pose.translation+.2*pose.rotation[:,0]
        anchor=point(idle)-[0,0,.12]
        for t in np.linspace(*self.segment.dance_interval_s,1601):
            self.assertLess(np.linalg.norm(point(self.plan.at(t)[0])-anchor),.002)

    def test_actual_controller_delay_at_original_speed(self):
        for lag in (.04,.06):
            controller=Controller(self.gravity,gripper_velocity_limit=self.plan.gripper_velocity_limit)
            controller.velocity_limit[:6]=self.plan.velocity_limit
            for i,t in enumerate(np.arange(0,self.plan.duration+.02,.02)):
                q,v=self.plan.at(t);measured,_=self.plan.at(max(0,t-lag))
                command=controller.command(measured,1_000_000_000+i*20_000_000,q,v)
                self.assertTrue(np.all(np.abs(command['estimated_torque_nm'])<=CAPS))

    simulate_console=legacy.RealModelTests.simulate_console

    def test_runtime_uses_join_at_original_speed(self):
        result,events,rows=self.simulate_console(trial_speed=None,bundle=ROOT/'record/dance_showcase/v10')
        self.assertTrue(result['success'],result);self.assertEqual(result['completed'],['21'])
        self.assertEqual(result['library']['actions']['21']['execution_speed'],1.)
        start=next(r['monotonic_ns'] for r in rows if r['event']=='segment_start' and r.get('phase','').startswith('21:'))
        for row in rows:
            if row['event']=='command' and row.get('phase','').startswith('21:'):
                q,v=self.plan.at((row['target_sample_ns']-start)/1e9)
                np.testing.assert_allclose(row['target_rad'],q,atol=1e-8)
                np.testing.assert_allclose(row['velocity_rad_s'],v,atol=1e-8)

    def test_fault_holds_until_explicit_operator_exit(self):
        result,events,_=self.simulate_console(fail=True,trial_speed=None,bundle=ROOT/'record/dance_showcase/v10')
        self.assertEqual(result['completed'],[])
        self.assertLess(events.index('held'),events.index('operator_exit'))
        self.assertLess(events.index('operator_exit'),events.index('home'))
        self.assertLess(events.index('home'),events.index('disable'))


if __name__=='__main__':unittest.main()
