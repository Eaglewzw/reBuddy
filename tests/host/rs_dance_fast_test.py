"""Versioned fast-dance regression, analytic continuity and offline playback."""
import json
import math
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

import numpy as np

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'tools'))
import rs_dance_design as design
import rs_dance_design_test as legacy
from rs_gravity_record import CAPS
from rs_record_playback import LOWER,UPPER


def stage_jerk(stage,t):
    if hasattr(stage,'jerk_at'):return stage.jerk_at(t)
    if hasattr(stage,'curve') and hasattr(stage.curve,'spline'):
        return stage.curve.spline(t,nu=3)
    raise AssertionError('fast motion requires an analytic jerk evaluator')


class Tests(unittest.TestCase):
    def setUp(self):
        legacy.Tests.setUp(self)
        self.legacy_output=self.output;self.legacy_manifest=self.manifest
        self.output=self.root/'fast'
        self.manifest=design.build_library(self.calibration,self.urdf,self.output,recipe_version='fast_v6')

    def load(self,number):
        return design.load_action(number,self.calibration,self.urdf,self.output)

    def test_legacy_manifest_without_version_still_loads_unchanged(self):
        path=self.legacy_output/'manifest.json'
        manifest=json.loads(path.read_text());manifest.pop('recipe_version',None)
        path.write_text(json.dumps(manifest))
        _,legacy,_,_,_=design.load_library(self.calibration,self.urdf,self.legacy_output)
        self.assertAlmostEqual(legacy['01'].duration,8.)
        self.assertAlmostEqual(legacy['02'].duration,10.)
        self.assertAlmostEqual(legacy['09'].duration,2.)
        self.assertEqual(len(legacy['09'].stages),3)
        self.assertEqual([row['number'] for row in self.manifest['actions']],list(range(12)))
        for number in (0,11):
            current,_,_=self.load(number)
            old=legacy[f'{number:02d}']
            self.assertAlmostEqual(current.duration,old.duration)
            for t in np.linspace(0.,old.duration,101):
                np.testing.assert_allclose(current.at(t),old.at(t),atol=1e-10)

    def test_unknown_version_and_recipe_mismatch_are_rejected(self):
        path=self.output/'manifest.json';original=path.read_text();manifest=json.loads(original)
        manifest['recipe_version']='not_an_approved_recipe'
        path.write_text(json.dumps(manifest))
        with self.assertRaises(ValueError):self.load(1)
        manifest['recipe_version']='legacy_v5'
        path.write_text(json.dumps(manifest))
        with self.assertRaises(ValueError):self.load(1)

    def test_fast_motion_is_shorter_without_internal_holds(self):
        _,legacy,_,_,_=design.load_library(self.calibration,self.urdf,self.legacy_output)
        for number in range(1,11):
            with self.subTest(action=number):
                plan,_,report=self.load(number)
                self.assertEqual(len(plan.stages),1)
                self.assertAlmostEqual(plan.duration,report['duration_s'])
                if number==9:self.assertAlmostEqual(plan.duration,2.)
                else:
                    self.assertLess(plan.duration,6.)
                    self.assertLess(plan.duration,legacy[f'{number:02d}'].duration*.8)
                times=np.linspace(.1*plan.duration,.9*plan.duration,1601)
                v=np.array([plan.at(t)[1] for t in times])
                slow=np.max(abs(v),axis=1)<math.radians(.4)
                run=longest=0
                for x in slow:
                    run=run+1 if x else 0;longest=max(longest,run)
                self.assertLess(longest*(times[1]-times[0]),.15)
                self.assertLess(np.mean(slow),.08)

    def test_fast_internal_jerk_continuity_and_c2_endpoints(self):
        for number in range(1,11):
            with self.subTest(action=number):
                plan,_,_=self.load(number);stage=plan.stages[0][1]
                for t in (0.,plan.duration):
                    np.testing.assert_allclose(plan.at(t)[0],self.idle,atol=1e-8)
                    np.testing.assert_allclose(plan.at(t)[1],0.,atol=1e-8)
                    np.testing.assert_allclose(plan.acceleration_at(t),0.,atol=1e-7)
                    # Finite endpoint jerk is recorded separately: these are
                    # C2 joins to rest, not a claim of zero-jerk/C3 endpoints.
                    self.assertTrue(np.all(np.isfinite(stage_jerk(stage,t))))
                seams=[plan.duration*.5]
                if hasattr(stage,'curve'):
                    seams.extend(np.unique(stage.curve.spline.t).tolist())
                for t in seams:
                    if not 0<t<plan.duration:continue
                    eps=1e-7
                    np.testing.assert_allclose(stage_jerk(stage,t-eps),stage_jerk(stage,t+eps),atol=2e-3)
                for t in np.linspace(.01*plan.duration,.99*plan.duration,101):
                    eps=1e-6
                    actual=(stage.acceleration_at(t+eps)-stage.acceleration_at(t-eps))/(2*eps)
                    np.testing.assert_allclose(actual,stage_jerk(stage,t),atol=2e-4)

    def test_positions_and_actual_speed_acceleration_keep_existing_budgets(self):
        for number in range(12):
            with self.subTest(action=number):
                plan,gravity,_=self.load(number)
                for _,stage in plan.stages:
                    self.assertEqual(stage.velocity_limit,plan.velocity_limit)
                    self.assertEqual(stage.gripper_velocity_limit,plan.gripper_velocity_limit)
                for t in np.linspace(0.,plan.duration,801):
                    q,v=plan.at(t);a=plan.acceleration_at(t)
                    self.assertTrue(np.all(q>=LOWER-1e-8) and np.all(q<=UPPER+1e-8))
                    self.assertLessEqual(max(abs(v[:6])),1.+1e-7)
                    self.assertLessEqual(max(abs(a[:6])),2.+1e-7)
                    self.assertGreaterEqual(q[6],-1e-9)
                    self.assertLessEqual(q[6],math.radians(45.)+1e-9)
                    self.assertLessEqual(abs(v[6]),1.6+1e-7)
                    self.assertLessEqual(abs(a[6]),5.1+1e-7)
                    self.assertTrue(np.all(abs(gravity(q[:6]))<=CAPS[:6]))

    def test_gripper_reverses_smoothly_without_open_hold(self):
        plan,_,_=self.load(9);stage=plan.stages[0][1]
        self.assertAlmostEqual(plan.duration,2.)
        self.assertAlmostEqual(plan.at(1.)[0][6],math.radians(45.))
        self.assertGreater(plan.at(.99)[1][6],0.)
        self.assertLess(plan.at(1.01)[1][6],0.)
        self.assertGreater(abs(plan.acceleration_at(1.)[6]),1.)
        self.assertAlmostEqual(plan.gripper_following_limit,math.radians(10.8))
        np.testing.assert_allclose(stage_jerk(stage,1.),0.,atol=1e-7)
        # Quantify the real C2-to-static endpoint rather than calling it C3.
        self.assertGreater(abs(stage_jerk(stage,0.)[6]),1.)
        np.testing.assert_allclose(stage_jerk(stage,0.)[:6],0.,atol=1e-10)

    def test_minor_support_reversals_removed_without_shrinking_wave_ranges(self):
        nod,_,_=self.load(1)
        q=np.array([nod.at(t)[0] for t in np.linspace(0.,nod.duration,1001)])
        axes=[0,1,2,4,5,6]
        np.testing.assert_allclose(q[:,axes],np.tile(self.idle[axes],(len(q),1)),atol=1e-10)
        turn,_,_=self.load(5)
        v=np.array([turn.at(t)[1] for t in np.linspace(0.,turn.duration,1001)])
        for joint in (1,2):
            signs=np.sign(v[abs(v[:,joint])>math.radians(.02),joint])
            self.assertEqual(np.count_nonzero(np.diff(signs)),1)
        for number in (6,10):
            current,_,_=self.load(number)
            old,_,_=design.load_action(number,self.calibration,self.urdf,self.legacy_output)
            new_range=np.ptp([current.at(t)[0] for t in np.linspace(0.,current.duration,2001)],axis=0)
            old_range=np.ptp([old.at(t)[0] for t in np.linspace(0.,old.duration,2001)],axis=0)
            np.testing.assert_allclose(np.degrees(new_range),np.degrees(old_range),atol=.05)

    def test_forty_ms_tracking_lag_stays_within_budgets(self):
        legacy.Tests.test_forty_ms_synthetic_tracking_lag_stays_within_control_budgets(self)

    def test_fast_sampler_feedback_fault_and_home_stop_regressions(self):
        # Reuse the actual runtime/console fake backend with the new library;
        # only fixture load selection changes, no CAN socket is constructed.
        legacy.Tests.test_curve_targets_are_sampled_after_variable_feedback_delay(self)
        legacy.Tests.test_console_feedback_fault_holds_before_explicit_home(self)
        legacy.Tests.test_terminal_stop_interrupts_home_and_holds_until_release(self)

    def test_session_logs_the_exact_version_and_action_hashes(self):
        result,_,_,_=self.simulate_console('sequence')
        self.assertTrue(result['success'],result)
        library=result['library']
        self.assertEqual(library['recipe_version'],'fast_v6')
        self.assertEqual(Path(library['bundle']),self.output)
        _,_,report=self.load(9)
        self.assertEqual(library['actions']['09']['sha256'],report['source_sha256'])
        self.assertAlmostEqual(library['actions']['09']['duration_s'],2.)
        rows=[json.loads(line) for line in (Path(result['folder'])/'control.jsonl').read_text().splitlines()]
        logged=next(row for row in rows if row['event']=='library_loaded')
        self.assertEqual(logged['recipe_version'],'fast_v6')
        self.assertEqual(logged['actions'],library['actions'])

    simulate_segment=legacy.Tests.simulate_segment
    simulate_console=legacy.Tests.simulate_console


if __name__=='__main__':unittest.main()
