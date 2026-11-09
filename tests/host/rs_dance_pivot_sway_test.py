"""Offline regression for the short left/right + up/down action."""
import copy
import json
from pathlib import Path
import sys
import unittest

import numpy as np
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'tools'))
import rs_dance_showcase as showcase
import rs_dance_pivot_sway as sway
import rs_dance_speed as speed
from rs_gravity_record import DEFAULT_CALIBRATION,DEFAULT_URDF,CAPS
from rs_gravity_play import Controller


class Tests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.manifest,cls.plans,cls.gravity,_,cls.reports=showcase.load_library(
            DEFAULT_CALIBRATION,DEFAULT_URDF,ROOT/'record/dance_showcase/v7')
        cls.plan=cls.plans['21'];cls.segment=cls.plan.stages[0][1]
        cls.idle=np.asarray(cls.manifest['idle_rad'])

    def test_point_fixed_while_body_moves_in_two_directions(self):
        pin=self.gravity.pin;model=self.gravity.model;data=model.createData();frame=model.getFrameId('gripper_end')
        anchor=np.asarray(self.reports['21']['pivot_sway_design']['P_world_m'])
        spans=[]
        for index,(_,curve) in enumerate(self.segment.stages):
            tips=[];joints=[]
            for t in np.linspace(0,curve.duration,301):
                q,_=curve.at(t);pin.forwardKinematics(model,data,q[:6]);pin.updateFramePlacements(model,data)
                pose=data.oMf[frame];p=pose.translation+.2*pose.rotation[:,0]
                self.assertLess(np.linalg.norm(p-anchor),.002)
                tips.append(pose.translation.copy());joints.append(q)
            spans.append(np.ptp(tips,axis=0))
            if index==0:self.assertGreater(np.degrees(np.ptp(joints,axis=0))[0],10.)
        self.assertGreater(spans[0][1],.01)
        self.assertLess(spans[0][2],1e-5)
        self.assertGreater(spans[1][2],.01)
        self.assertLess(spans[1][1],.001)

    def test_short_duration_stationary_seams_and_retained_limits(self):
        self.assertEqual(len(self.segment.stages),2)
        self.assertLess(self.plan.duration,4.8)
        for _,curve in self.segment.stages:
            for t in (0,curve.duration):
                np.testing.assert_allclose(curve.at(t)[0],self.idle,atol=1e-9)
                np.testing.assert_allclose(curve.at(t)[1],0,atol=1e-8)
                np.testing.assert_allclose(curve.acceleration_at(t),0,atol=1e-7)
        self.assertLessEqual(max(self.plan.peak_velocity),1.)
        self.assertLessEqual(max(self.plan.peak_acceleration),2.)
        scaled=speed.apply_trial_speed(self.plans)['21']
        self.assertEqual(scaled.speed,.3)
        self.assertLess(scaled.duration,16.)

    def test_actual_controller_lag_at_trial_and_nominal_speed(self):
        for factor in (.3,1.):
            plan=speed.apply_trial_speed(self.plans,factor)['21']
            for lag in (.04,.06):
                controller=Controller(self.gravity,gripper_velocity_limit=plan.gripper_velocity_limit)
                controller.velocity_limit[:6]=plan.velocity_limit
                for i,t in enumerate(np.arange(0,plan.duration+.02,.02)):
                    q,v=plan.at(t);observed,_=plan.at(max(0,t-lag))
                    command=controller.command(observed,1_000_000_000+i*20_000_000,q,v)
                    self.assertTrue(np.all(np.abs(command['estimated_torque_nm'])<=CAPS))

    def test_other_actions_preserved_and_changed_point_rejected(self):
        for entry in self.manifest['actions']:
            if entry['number']!=21:
                self.assertEqual((ROOT/'record/dance_showcase/v7'/entry['file']).read_bytes(),
                                 (ROOT/'record/dance_showcase/v5'/entry['file']).read_bytes())
        bad=copy.deepcopy(self.reports['21']);bad['pivot_sway_design']['P_world_m'][0]+=.01
        with self.assertRaisesRegex(ValueError,'point changed'):
            sway.validate_sway(self.plan,self.gravity,self.idle,bad)


class LoweredTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.manifest,cls.plans,cls.gravity,_,cls.reports=showcase.load_library(
            DEFAULT_CALIBRATION,DEFAULT_URDF,ROOT/'record/dance_showcase/v8')
        cls.plan=cls.plans['21'];cls.segment=cls.plan.stages[0][1]
        cls.idle=np.asarray(cls.manifest['idle_rad'])

    def test_lowered_point_stays_fixed_and_excursions_are_larger(self):
        pin=self.gravity.pin;model=self.gravity.model;data=model.createData();frame=model.getFrameId('gripper_end')
        def fk(q):
            pin.forwardKinematics(model,data,q[:6]);pin.updateFramePlacements(model,data)
            pose=data.oMf[frame]
            return pose.translation+.2*pose.rotation[:,0],pose.rotation.copy()
        old,reference=fk(self.idle);anchor=old-[0,0,.03]
        design=self.reports['21']['pivot_sway_design']
        np.testing.assert_allclose(design['P_world_m'],anchor,atol=1e-9)
        observed=[]
        for phase,(_,curve) in enumerate(self.segment.stages[1:3]):
            angles=[]
            for t in np.linspace(0,curve.duration,401):
                q,_=curve.at(t);p,r=fk(q);observed.append(q)
                self.assertLess(np.linalg.norm(p-anchor),.002)
                d=r[:,0]
                if phase==0:
                    angles.append(np.degrees(np.arctan2(d[1],d[0])-np.arctan2(reference[1,0],reference[0,0])))
                    self.assertAlmostEqual(d[2],reference[2,0],delta=1e-5)
                else:
                    angles.append(np.degrees(np.arctan2(-d@reference[:,2],d@reference[:,0])))
                    self.assertLess(abs(d@reference[:,1]),1e-5)
            target=(6.,10.)[phase]
            self.assertAlmostEqual(max(angles),target,delta=.01)
            self.assertAlmostEqual(min(angles),-target,delta=.01)
        ranges=np.degrees(np.ptp(observed,axis=0))
        self.assertGreater(ranges[0],35.)
        self.assertGreater(ranges[2],50.)

    def test_entry_exit_are_continuous_short_and_closed(self):
        self.assertEqual([n for n,_ in self.segment.stages],['entry','left_right','up_down','exit'])
        np.testing.assert_allclose(self.plan.start,self.idle,atol=1e-9)
        np.testing.assert_allclose(self.plan.end,self.idle,atol=1e-9)
        self.assertLess(self.plan.duration,11.)
        self.assertEqual(speed.trial_speed_for(self.plans),.3)
        for (_,left),(_,right) in zip(self.segment.stages,self.segment.stages[1:]):
            np.testing.assert_allclose(left.at(left.duration)[0],right.at(0)[0],atol=1e-9)
            np.testing.assert_allclose(left.at(left.duration)[1],right.at(0)[1],atol=1e-8)
            np.testing.assert_allclose(left.acceleration_at(left.duration),right.acceleration_at(0),atol=1e-7)
        self.assertLessEqual(max(self.plan.peak_velocity),1.)
        self.assertLessEqual(max(self.plan.peak_acceleration),2.)
        changed=copy.deepcopy(self.reports['21']);changed['pivot_sway_design']['dance_interval_s'][0]=0.
        with self.assertRaisesRegex(ValueError,'interval changed'):
            sway.validate_sway(self.plan,self.gravity,self.idle,changed)

    def test_original_controller_limits_with_following_delay(self):
        for factor in (.3,1.):
            plan=speed.apply_trial_speed(self.plans,factor)['21']
            for lag in (.04,.06):
                controller=Controller(self.gravity,gripper_velocity_limit=plan.gripper_velocity_limit)
                controller.velocity_limit[:6]=plan.velocity_limit
                for i,t in enumerate(np.arange(0,plan.duration+.02,.02)):
                    q,v=plan.at(t);observed,_=plan.at(max(0,t-lag))
                    command=controller.command(observed,1_000_000_000+i*20_000_000,q,v)
                    self.assertTrue(np.all(np.abs(command['estimated_torque_nm'])<=CAPS))


if __name__=='__main__':unittest.main()
