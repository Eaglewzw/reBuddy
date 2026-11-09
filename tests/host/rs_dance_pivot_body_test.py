"""Position-only fixed-TCP body sway: real-model and simulated execution tests."""
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT/'tools'))
import rs_dance_pivot_body as body
import rs_dance_showcase as showcase
import rs_dance_speed as speed
import rs_dance_pivot_test as legacy_pivot
from rs_gravity_record import DEFAULT_CALIBRATION,DEFAULT_URDF,CAPS
from rs_record_playback import LOWER,UPPER


class Tests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not DEFAULT_CALIBRATION.exists() or not DEFAULT_URDF.exists():
            raise unittest.SkipTest('Needs the calibrated local URDF; no CAN is opened')
        cls.manifest,cls.plans,cls.gravity,cls.identity,cls.reports=showcase.load_library(
            DEFAULT_CALIBRATION,DEFAULT_URDF,ROOT/'record/dance_showcase/v5')
        cls.old_manifest,cls.old_plans,_,_,cls.old_reports=showcase.load_library(
            DEFAULT_CALIBRATION,DEFAULT_URDF,ROOT/'record/dance_showcase/v4')
        cls.plan=cls.plans['21'];cls.segment=cls.plan.stages[0][1]
        cls.idle=np.asarray(cls.manifest['idle_rad']);cls.cfg=cls.segment.config
        cls.pin=cls.gravity.pin;cls.model=cls.gravity.model;cls.frame=cls.model.getFrameId('gripper_end')

    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup);self.root=Path(self.tmp.name)

    def fk(self,q):
        data=self.model.createData();self.pin.forwardKinematics(self.model,data,q[:6])
        self.pin.updateFramePlacements(self.model,data);pose=data.oMf[self.frame]
        return pose.translation+pose.rotation@np.asarray(self.cfg['tcp_offset_m']),pose.rotation.copy()

    def test_visible_base_sway_and_multiple_compensating_axes_keep_tcp_fixed(self):
        anchor=self.fk(self.idle)[0];positions=[];errors=[]
        for t in np.linspace(0.,self.plan.duration,1001):
            q,_=self.plan.at(t);positions.append(q)
            errors.append(np.linalg.norm(self.fk(q)[0]-anchor))
        ranges=np.degrees(np.ptp(positions,axis=0))
        self.assertGreaterEqual(ranges[0],20.)
        self.assertGreaterEqual(np.count_nonzero(ranges[1:6]>2.),2)
        self.assertLess(max(errors),self.cfg['position_tolerance_m'])
        self.assertLess(ranges[6],1e-8)
        self.assertLessEqual(self.plan.duration,10.)
        self.assertLess(self.plan.duration,self.old_plans['21'].duration*.5)
        self.assertEqual(self.reports['21']['stages'][0]['kind'],'fixed_tcp_body')
        design=self.reports['21']['body_design']
        self.assertTrue(design['position_fixed'])
        self.assertFalse(design['orientation_fixed'])
        self.assertFalse(design['physical_tip_calibrated'])
        self.assertLess(design['raw_end_joint_error_rad'],1e-6)

    def test_position_ik_uses_previous_five_axis_solution_and_rejects_bad_endpoint(self):
        solver_module=sys.modules[body._solve_samples.__module__]
        original=solver_module.least_squares;solves=[]
        count=self.reports['21']['body_design']['ik_samples']
        def solve(fun,seed,**kwargs):
            before=np.array(seed).copy();result=original(fun,seed,**kwargs)
            # The final roll perturbation preserves this model TCP's position
            # but must fail the joint-branch check before endpoint rounding.
            if len(solves)==count-1:
                result.x=result.x.copy();result.x[-1]+=1e-4
            solves.append((before,result.x.copy()))
            return result
        with patch.object(solver_module,'least_squares',side_effect=solve):
            with self.assertRaisesRegex(ValueError,'idle joint branch'):
                body._solve_samples(self.gravity,self.idle,self.cfg,self.segment.amplitude_scale)
        self.assertEqual(len(solves),count)
        np.testing.assert_allclose(solves[0][0],self.idle[1:6],atol=1e-12)
        for left,right in zip(solves,solves[1:]):
            np.testing.assert_allclose(right[0],left[1],atol=1e-12)

    def test_continuous_endpoints_and_original_joint_dynamic_limits(self):
        self.assertEqual(len(self.plan.stages),1)
        for t in (0.,self.plan.duration):
            np.testing.assert_allclose(self.plan.at(t)[0],self.idle,atol=1e-8)
            np.testing.assert_allclose(self.plan.at(t)[1],0.,atol=1e-8)
            np.testing.assert_allclose(self.plan.acceleration_at(t),0.,atol=1e-7)
        times=np.linspace(0.,self.plan.duration,1001)
        for t in times:
            q,v=self.plan.at(t);a=self.plan.acceleration_at(t)
            self.assertTrue(np.all(q>=LOWER-1e-8) and np.all(q<=UPPER+1e-8))
            self.assertLessEqual(max(abs(v[:6])),1.+1e-7)
            self.assertLessEqual(max(abs(a[:6])),2.+1e-7)
            self.assertAlmostEqual(q[6],0.)
        middle=np.linspace(.1*self.plan.duration,.9*self.plan.duration,1601)
        quiet=[max(abs(self.plan.at(t)[1][:6]))<math.radians(.4) for t in middle]
        run=longest=0
        for still in quiet:
            run=run+1 if still else 0;longest=max(longest,run)
        self.assertLess(longest*(middle[1]-middle[0]),.2)
        for t in np.unique(self.segment.curve.spline.t):
            if 0<t<self.plan.duration:
                np.testing.assert_allclose(self.segment.jerk_at(t-1e-7),
                                           self.segment.jerk_at(t+1e-7),atol=.002)

    def test_nominal_forty_and_sixty_ms_following_lag_keep_existing_torque_caps(self):
        from rs_gravity_play import Controller
        for lag in (.04,.06):
            with self.subTest(lag=lag):
                controller=Controller(self.gravity,gripper_velocity_limit=self.plan.gripper_velocity_limit)
                controller.velocity_limit[:6]=self.plan.velocity_limit
                for i,t in enumerate(np.arange(0.,self.plan.duration+.02,.02)):
                    target,v=self.plan.at(t);actual,_=self.plan.at(max(0.,t-lag))
                    command=controller.command(actual,1_000_000_000+i*20_000_000,target,v)
                    self.assertTrue(np.all(np.abs(command['estimated_torque_nm'])<=CAPS))

    def test_new_default_full_speed_and_legacy_thirty_percent_are_distinct(self):
        self.assertEqual(self.manifest['library_version'],'v5')
        self.assertEqual(self.old_manifest['library_version'],'v4')
        self.assertEqual(speed.trial_speed_for(self.plans),1.)
        self.assertEqual(speed.trial_speed_for(self.old_plans),.3)
        current=speed.apply_trial_speed(self.plans)
        previous=speed.apply_trial_speed(self.old_plans)
        self.assertAlmostEqual(current['21'].duration,self.plan.duration)
        self.assertAlmostEqual(previous['21'].duration,self.old_plans['21'].duration/.3)
        self.assertEqual(self.old_reports['21']['stages'][0]['kind'],'fixed_tcp_pivot')
        for code,plan in self.old_plans.items():
            if code=='21':continue
            self.assertAlmostEqual(self.plans[code].duration,plan.duration)
            for t in np.linspace(0.,plan.duration,31):
                np.testing.assert_allclose(self.plans[code].at(t),plan.at(t),atol=1e-8)

    def test_explicit_speed_override_only_changes_21(self):
        for plans in (self.plans,self.old_plans):
            for rate in (.3,.5,1.):
                with self.subTest(version=plans['21'].stages[0][1].__class__.__name__,rate=rate):
                    self.assertEqual(speed.trial_speed_for(plans,rate),rate)
                    scaled=speed.apply_trial_speed(plans,rate)
                    self.assertIsNot(scaled,plans)
                    for code in plans:
                        if code!='21':self.assertIs(scaled[code],plans[code])
                    original=plans['21'];trial=scaled['21']
                    self.assertAlmostEqual(trial.duration,original.duration/rate)
                    for t in np.linspace(0.,original.duration,101):
                        q,v=original.at(t);tq,tv=trial.at(t/rate)
                        np.testing.assert_allclose(tq,q,atol=1e-9)
                        np.testing.assert_allclose(tv,v*rate,atol=1e-9)
                        np.testing.assert_allclose(trial.acceleration_at(t/rate),original.acceleration_at(t)*rate**2,atol=1e-8)

    simulate_console=legacy_pivot.RealModelTests.simulate_console

    def test_runtime_uses_new_default_and_explicit_override_targets(self):
        for requested,rate in ((None,1.),(.5,.5)):
            with self.subTest(requested=requested):
                result,events,rows=self.simulate_console(trial_speed=requested,bundle=ROOT/'record/dance_showcase/v5')
                self.assertTrue(result['success'],result)
                self.assertEqual(result['completed'],['21'])
                start=next(r['monotonic_ns'] for r in rows if r['event']=='segment_start' and r.get('phase','').startswith('21:'))
                commands=[r for r in rows if r['event']=='command' and r.get('phase','').startswith('21:')]
                self.assertGreater(len(commands),100)
                for row in commands:
                    elapsed=(row['target_sample_ns']-start)/1e9
                    target,v=self.plan.at(min(elapsed*rate,self.plan.duration))
                    np.testing.assert_allclose(row['target_rad'],target,atol=1e-8)
                    np.testing.assert_allclose(row['velocity_rad_s'],v*rate,atol=1e-8)
                duration=(commands[-1]['target_sample_ns']-start)/1e9
                self.assertGreaterEqual(duration,self.plan.duration/rate)
                self.assertLess(duration,self.plan.duration/rate+.021)
                self.assertAlmostEqual(result['library']['actions']['21']['duration_s'],self.plan.duration/rate)
                self.assertLess(events.index('home'),events.index('disable'))

    def test_new_default_runtime_feedback_fault_still_holds_before_exit(self):
        result,events,rows=self.simulate_console(fail=True,trial_speed=None,bundle=ROOT/'record/dance_showcase/v5')
        self.assertEqual(result['completed'],[])
        self.assertIn('held',events)
        self.assertLess(events.index('held'),events.index('operator_exit'))
        self.assertLess(events.index('operator_exit'),events.index('home'))
        self.assertLess(events.index('home'),events.index('disable'))


if __name__=='__main__':unittest.main()
