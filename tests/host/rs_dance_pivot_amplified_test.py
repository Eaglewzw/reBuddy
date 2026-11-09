"""Real-model checks and mocked playback for the larger, faster v9 pivot."""
import copy
from pathlib import Path
import sys
import tempfile
import unittest
import numpy as np

ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT/'tools'))
import rs_dance_showcase as showcase
import rs_dance_pivot_sway as sway
import rs_dance_speed as speed
import rs_dance_pivot_test as legacy
from rs_gravity_record import DEFAULT_CALIBRATION,DEFAULT_URDF,CAPS
from rs_gravity_play import Controller


class Tests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.manifest,cls.plans,cls.gravity,cls.identity,cls.reports=showcase.load_library(
            DEFAULT_CALIBRATION,DEFAULT_URDF,ROOT/'record/dance_showcase/v9')
        cls.plan=cls.plans['21'];cls.segment=cls.plan.stages[0][1];cls.idle=np.asarray(cls.manifest['idle_rad'])

    def setUp(self):
        tmp=tempfile.TemporaryDirectory();self.addCleanup(tmp.cleanup);self.root=Path(tmp.name)

    def test_independent_fk_point_and_visible_excursions(self):
        pin=self.gravity.pin;model=self.gravity.model;data=model.createData();frame=model.getFrameId('gripper_end')
        def fk(q):
            pin.forwardKinematics(model,data,q[:6]);pin.updateFramePlacements(model,data)
            pose=data.oMf[frame];return pose.translation+.2*pose.rotation[:,0],pose.rotation.copy()
        initial,original=fk(self.idle);anchor=initial-[0,0,.12]
        prepared,reference=fk(self.segment.prepared)
        np.testing.assert_allclose(prepared,anchor,atol=1e-8)
        np.testing.assert_allclose(reference,pin.exp3(original[:,1]*np.radians(10.))@original,atol=1e-8)
        np.testing.assert_allclose(self.reports['21']['pivot_sway_design']['P_world_m'],anchor,atol=1e-9)
        joints=[]
        for index,(_,curve) in enumerate(self.segment.stages[1:3]):
            angles=[]
            for t in np.linspace(0,curve.duration,701):
                q,_=curve.at(t);point,rotation=fk(q);joints.append(q)
                self.assertLess(np.linalg.norm(point-anchor),.002)
                direction=rotation[:,0]
                if index==0:
                    angles.append(np.degrees(np.arctan2(direction[1],direction[0])-np.arctan2(reference[1,0],reference[0,0])))
                    self.assertAlmostEqual(direction[2],reference[2,0],delta=1e-5)
                else:
                    angles.append(np.degrees(np.arctan2(-direction@reference[:,2],direction@reference[:,0])))
                    self.assertLess(abs(direction@reference[:,1]),1e-5)
            self.assertAlmostEqual(max(angles),(20.,18.)[index],delta=.01)
            self.assertAlmostEqual(min(angles),-(20.,18.)[index],delta=.01)
        ranges=np.degrees(np.ptp(joints,axis=0))
        self.assertGreater(ranges[0],75.);self.assertGreater(ranges[2],95.)

    def test_separate_sweep_timing_and_stationary_c2_reversals(self):
        self.assertLess(self.plan.duration,17.)
        self.assertEqual(speed.trial_speed_for(self.plans),1.)
        self.assertEqual(speed.trial_speed_for(self.plans,.3),.3)
        self.assertLessEqual(max(self.plan.peak_velocity),1.)
        self.assertLessEqual(max(self.plan.peak_acceleration),2.)
        curves=[]
        for _,curve in self.segment.stages:
            curves.extend(curve.curves if isinstance(curve,sway.SweepCurve) else [curve])
        self.assertEqual(len(curves),8)  # enter + 3 horizontal + 3 vertical + return
        for left,right in zip(curves,curves[1:]):
            np.testing.assert_allclose(left.at(left.duration)[0],right.at(0)[0],atol=1e-8)
            np.testing.assert_allclose(left.at(left.duration)[1],right.at(0)[1],atol=1e-8)
            np.testing.assert_allclose(left.acceleration_at(left.duration),right.acceleration_at(0),atol=1e-7)
        np.testing.assert_allclose(self.plan.start,self.idle,atol=1e-9)
        np.testing.assert_allclose(self.plan.end,self.idle,atol=1e-9)
        changed=copy.deepcopy(self.reports['21']['stages'][0]);changed['chunks'][1]['legs'][0]['angle_deg'][1]+=1
        with self.assertRaisesRegex(ValueError,'angular schedule'):
            sway.SwaySegment(changed)

    def test_real_controller_following_delay_at_new_default(self):
        for lag in (.04,.06):
            controller=Controller(self.gravity,gripper_velocity_limit=self.plan.gripper_velocity_limit)
            controller.velocity_limit[:6]=self.plan.velocity_limit
            for i,t in enumerate(np.arange(0,self.plan.duration+.02,.02)):
                q,v=self.plan.at(t);measured,_=self.plan.at(max(0,t-lag))
                command=controller.command(measured,1_000_000_000+i*20_000_000,q,v)
                self.assertTrue(np.all(np.abs(command['estimated_torque_nm'])<=CAPS))

    simulate_console=legacy.RealModelTests.simulate_console

    def test_default_normal_speed_reaches_runtime_and_logging(self):
        result,events,rows=self.simulate_console(trial_speed=None,bundle=ROOT/'record/dance_showcase/v9')
        self.assertTrue(result['success'],result);self.assertEqual(result['completed'],['21'])
        self.assertEqual(result['library']['actions']['21']['execution_speed'],1.)
        start=next(r['monotonic_ns'] for r in rows if r['event']=='segment_start' and r.get('phase','').startswith('21:'))
        commands=[r for r in rows if r['event']=='command' and r.get('phase','').startswith('21:')]
        for row in commands:
            elapsed=(row['target_sample_ns']-start)/1e9;q,v=self.plan.at(elapsed)
            np.testing.assert_allclose(row['target_rad'],q,atol=1e-8)
            np.testing.assert_allclose(row['velocity_rad_s'],v,atol=1e-8)
        self.assertLess((commands[-1]['target_sample_ns']-start)/1e9,self.plan.duration+.021)

    def test_feedback_fault_still_holds_before_explicit_exit(self):
        result,events,_=self.simulate_console(fail=True,trial_speed=None,bundle=ROOT/'record/dance_showcase/v9')
        self.assertEqual(result['completed'],[])
        self.assertLess(events.index('held'),events.index('operator_exit'))
        self.assertLess(events.index('operator_exit'),events.index('home'))
        self.assertLess(events.index('home'),events.index('disable'))


if __name__=='__main__':unittest.main()
