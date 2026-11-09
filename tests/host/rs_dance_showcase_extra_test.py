"""v2 showcase regression and independent real-model FK/lag checks; no CAN."""
import json
import math
import copy
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

import numpy as np

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'tools'))
import rs_dance_showcase as showcase
import rs_dance_showcase_extra as extra
import rs_dance_showcase_test as legacy
from rs_gravity_record import DEFAULT_CALIBRATION,DEFAULT_URDF,CAPS
from rs_record_playback import LOWER,UPPER


class FixtureTests(unittest.TestCase):
    def setUp(self):
        legacy.Tests.setUp(self)
        self.v1_output=self.output;self.output=self.root/'v2'
        def fake_points(gravity,idle):
            u=np.linspace(0.,1.,31);q=np.tile(idle,(len(u),1))
            q[:,0]+=math.radians(8.)*np.sin(2*math.pi*u)*np.sin(math.pi*u)**3
            q[:,4]-=math.radians(4.)*np.sin(2*math.pi*u)*np.sin(math.pi*u)**3
            q[0]=q[-1]=idle
            return q,dict(test_fixture=True,frame='gripper_end')
        patch.object(extra,'_stabilized_points',side_effect=fake_points).start()
        patch.object(extra,'_figure_eight_points',side_effect=fake_points).start()
        self.manifest=showcase.build_library(self.calibration,self.urdf,self.output,library_version='v2')

    def load(self,folder=None):
        return showcase.load_library(self.calibration,self.urdf,folder or self.output)

    def test_v2_adds_three_actions_and_preserves_versionless_v1(self):
        manifest,plans,_,_,reports=self.load()
        self.assertEqual(manifest['library_version'],'v2')
        self.assertEqual(list(plans),['00',*[str(n) for n in range(12,21)],'11'])
        v1_path=self.v1_output/'manifest.json'
        v1=json.loads(v1_path.read_text());v1.pop('library_version',None);v1_path.write_text(json.dumps(v1))
        _,old,_,_,old_reports=self.load(self.v1_output)
        self.assertEqual(list(old),['00',*[str(n) for n in range(12,18)],'11'])
        for number,old_plan in old.items():
            self.assertEqual(reports[number]['stages'],old_reports[number]['stages'])
            self.assertAlmostEqual(plans[number].duration,old_plan.duration)

    def test_extra_actions_are_one_c2_phrase_without_gripper_motion(self):
        _,plans,_,_,_=self.load()
        for number in ('18','19','20'):
            with self.subTest(action=number):
                plan=plans[number]
                self.assertEqual(len(plan.stages),1)
                self.assertFalse(plan.gripper_motion)
                for t in (0.,plan.duration):
                    np.testing.assert_allclose(plan.at(t)[0],self.idle,atol=1e-8)
                    np.testing.assert_allclose(plan.at(t)[1],0.,atol=1e-8)
                    np.testing.assert_allclose(plan.acceleration_at(t),0.,atol=1e-7)
                stage=plan.stages[0][1]
                for t in np.unique(stage.curve.spline.t):
                    if 0<t<plan.duration:
                        np.testing.assert_allclose(stage.curve.spline(t-1e-7,nu=3),
                                                   stage.curve.spline(t+1e-7,nu=3),atol=.002)
                for t in np.linspace(0.,plan.duration,501):
                    q,v=plan.at(t);a=plan.acceleration_at(t)
                    self.assertTrue(np.all(q>=LOWER-1e-8) and np.all(q<=UPPER+1e-8))
                    self.assertAlmostEqual(q[6],0.)
                    self.assertAlmostEqual(v[6],0.)
                    self.assertLessEqual(max(abs(v[:6])),1.+1e-7)
                    self.assertLessEqual(max(abs(a[:6])),2.+1e-7)

    def test_snake_knots_use_one_carrier_and_delayed_joint_phases(self):
        _,plans,_,_,reports=self.load()
        plan=plans['18'];stage=plan.stages[0][1]
        design=reports['18']['motion_design']
        self.assertEqual(design['carrier_cycles'],2)
        np.testing.assert_allclose(design['phase_deg'][:6],[60,0,-60,-120,-180,-240])
        phase=np.radians(design['phase_deg']);amplitude=np.radians(design['amplitude_deg'])
        u=stage.t_s/stage.duration
        expected=self.idle+np.sin(math.pi*u[:,None])**3*amplitude*np.sin(2*math.pi*2*u[:,None]+phase)
        np.testing.assert_allclose(stage.q_rad,expected,atol=1e-9)
        self.assertEqual(len(stage.t_s),31)

    def test_snake_phase_claim_rejects_nonuniform_knot_timing(self):
        _,_,gravity,_,reports=self.load()
        stages=copy.deepcopy(reports['18']['stages'])
        stages[0]['t_s'][10]+=stages[0]['duration_s']*.0001
        altered=showcase.ShowcasePlan(stages)
        with self.assertRaises(ValueError):
            extra.validate_extra(altered,gravity,18,self.idle,design=reports['18'])

    def test_unknown_library_version_and_tampered_new_action_rejected(self):
        path=self.output/'manifest.json';original=path.read_text()
        manifest=json.loads(original);manifest['library_version']='v999';path.write_text(json.dumps(manifest))
        with self.assertRaises(ValueError):self.load()
        path.write_text(original)
        action=next(r for r in self.manifest['actions'] if r['number']==19)
        source=self.output/action['file'];source.write_text(source.read_text()+' ')
        with self.assertRaises(ValueError):self.load()


class RealModelTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not DEFAULT_CALIBRATION.exists() or not DEFAULT_URDF.exists():
            raise unittest.SkipTest('Real-model tests need this robot calibration and URDF')
        cls.manifest,cls.plans,cls.gravity,cls.identity,cls.reports=showcase.load_library(
            DEFAULT_CALIBRATION,DEFAULT_URDF,ROOT/'record/dance_showcase/v2')
        cls.idle=np.array(cls.manifest['idle_rad'])
        cls.pin=cls.gravity.pin;cls.model=cls.gravity.model
        cls.frame=cls.model.getFrameId('gripper_end')
        if cls.frame>=cls.model.nframes:raise AssertionError('gripper_end frame missing')

    def fk(self,q):
        data=self.model.createData()
        self.pin.forwardKinematics(self.model,data,np.asarray(q[:6]))
        self.pin.updateFramePlacements(self.model,data)
        placement=data.oMf[self.frame]
        return placement.translation.copy(),placement.rotation.copy()

    def test_compensated_head_orientation_beats_frozen_wrists(self):
        plan=self.plans['19'];rotation=self.fk(self.idle)[1]
        errors=[];uncompensated=[];positions=[]
        for t in np.linspace(0.,plan.duration,501):
            q,_=plan.at(t);positions.append(q)
            baseline=q.copy();baseline[3:6]=self.idle[3:6]
            for pose,result in ((q,errors),(baseline,uncompensated)):
                R=self.fk(pose)[1]
                result.append(math.degrees(math.acos(np.clip((np.trace(rotation.T@R)-1)/2,-1.,1.))))
        self.assertLess(max(errors),.3)
        self.assertGreater(max(uncompensated),30.)
        self.assertLess(max(errors),max(uncompensated)/10.)
        q=np.array(positions)
        self.assertGreater(math.degrees(np.ptp(q[:,0])),65.)
        np.testing.assert_allclose(q[:,1:3],np.tile(self.idle[1:3],(len(q),1)),atol=1e-8)

    def test_cartesian_eight_has_two_lobes_and_moving_center_crossing(self):
        plan=self.plans['20'];times=np.linspace(0.,plan.duration,1001)
        points=np.array([self.fk(plan.at(t)[0])[0] for t in times])
        center=self.fk(self.idle)[0]
        design=self.reports['20']['motion_design']
        self.assertEqual(design['frame'],'gripper_end')
        axis_u=np.array(design['plane_u']);axis_v=np.array(design['plane_v'])
        np.testing.assert_allclose([np.linalg.norm(axis_u),np.linalg.norm(axis_v),axis_u@axis_v],
                                   [1.,1.,0.],atol=1e-10)
        self.assertAlmostEqual(design['half_width_m'],.10)
        self.assertAlmostEqual(design['half_height_m'],.055)
        u=times/plan.duration;theta=2*math.pi*(6*u**5-15*u**4+10*u**3)
        expected=center+.10*np.sin(theta[:,None])*axis_u+.055*np.sin(2*theta[:,None])*axis_v
        self.assertLess(np.max(np.linalg.norm(points-expected,axis=1)),.003)
        self.assertLess(np.linalg.norm(points[500]-center),.002)
        eps=1e-4
        speed=np.linalg.norm(self.fk(plan.at(plan.duration/2+eps)[0])[0]-
                             self.fk(plan.at(plan.duration/2-eps)[0])[0])/(2*eps)
        self.assertGreater(speed,.02)
        # Fit a plane to independently evaluated FK positions. Two closed
        # lobes must have opposite signed areas, not a joint-space drawing.
        _,_,basis=np.linalg.svd(points-center,full_matrices=False)
        xy=(points-center)@basis[:2].T
        self.assertLess(np.max(abs((points-center)@basis[2])),.003)
        areas=[]
        for lobe in (xy[:501],xy[500:]):
            areas.append(float(np.sum(lobe[:-1,0]*lobe[1:,1]-lobe[1:,0]*lobe[:-1,1])/2))
            self.assertLess(np.linalg.norm(lobe[-1]-lobe[0]),.003)
        self.assertLess(areas[0]*areas[1],0.)
        self.assertGreater(min(abs(x) for x in areas),.003)
        self.assertGreater(np.ptp(xy[:,0]),.15)
        self.assertGreater(np.ptp(xy[:,1]),.08)

    def test_real_model_40_and_60_ms_lag_preserves_control_budgets(self):
        from rs_gravity_play import Controller
        for code in ('18','19','20'):
            plan=self.plans[code]
            for lag in (.04,.06):
                with self.subTest(action=code,lag=lag):
                    controller=Controller(self.gravity,gripper_velocity_limit=plan.gripper_velocity_limit)
                    controller.velocity_limit[:6]=plan.velocity_limit
                    for i,t in enumerate(np.arange(0.,plan.duration+.02,.02)):
                        target,v=plan.at(t);actual,_=plan.at(max(0.,t-lag))
                        command=controller.command(actual,1_000_000_000+i*20_000_000,target,v)
                        self.assertTrue(np.all(np.abs(command['estimated_torque_nm'])<=CAPS))

    def test_v3_enlarges_only_twenty_and_preserves_continuous_cartesian_path(self):
        from rs_gravity_play import Controller
        folder=ROOT/'record/dance_showcase/v3'
        manifest,plans,gravity,_,reports=showcase.load_library(DEFAULT_CALIBRATION,DEFAULT_URDF,folder)
        self.assertEqual(manifest['library_version'],'v3')
        for item in manifest['actions']:
            if item['number']!=20:
                self.assertEqual((folder/item['file']).read_bytes(),
                                 (ROOT/'record/dance_showcase/v2'/item['file']).read_bytes())
        plan=plans['20'];design=reports['20']['motion_design']
        self.assertAlmostEqual(design['half_width_m'],.20)
        self.assertAlmostEqual(design['half_height_m'],.12)
        self.assertLess(plan.duration,9.)
        times=np.linspace(0.,plan.duration,1001);u=times/plan.duration
        theta=2*math.pi*(6*u**5-15*u**4+10*u**3)
        axis_u=np.array(design['plane_u']);axis_v=np.array(design['plane_v'])
        center=self.fk(self.idle)[0]
        points=np.array([self.fk(plan.at(t)[0])[0] for t in times])
        expected=center+.20*np.sin(theta[:,None])*axis_u+.12*np.sin(2*theta[:,None])*axis_v
        self.assertLess(np.max(np.linalg.norm(points-expected,axis=1)),.003)
        relative=points-center
        self.assertGreater(np.ptp(relative@axis_u),.395)
        self.assertGreater(np.ptp(relative@axis_v),.235)
        self.assertLess(np.linalg.norm(relative[500]),.002)
        eps=1e-4;middle=plan.duration/2
        speed=np.linalg.norm(self.fk(plan.at(middle+eps)[0])[0]-
                             self.fk(plan.at(middle-eps)[0])[0])/(2*eps)
        self.assertGreater(speed,.1)
        for lag in (.04,.06):
            controller=Controller(gravity,gripper_velocity_limit=plan.gripper_velocity_limit)
            controller.velocity_limit[:6]=plan.velocity_limit
            for i,t in enumerate(np.arange(0.,plan.duration+.02,.02)):
                target,v=plan.at(t);actual,_=plan.at(max(0.,t-lag))
                command=controller.command(actual,1_000_000_000+i*20_000_000,target,v)
                self.assertTrue(np.all(np.abs(command['estimated_torque_nm'])<=CAPS))

    def test_v4_eight_is_faster_without_reducing_its_spatial_size(self):
        from rs_gravity_play import Controller
        spec=json.loads((ROOT/'record/dance_showcase/v4/20_figure_eight.json').read_text())
        plan=showcase.ShowcasePlan(spec['stages'])
        self.assertLess(plan.duration,5.5)
        self.assertEqual(spec['motion_design']['timing'],{'profile':'smooth_cruise','ramp_fraction':.15})
        t=np.linspace(0,plan.duration,1001);u=t/plan.duration;r=.15
        def integral(z):return 2.5*z**4-3*z**5+z**6
        progress=np.where(u<r,r/(1-r)*integral(u/r),
                          np.where(u>1-r,1-r/(1-r)*integral((1-u)/r),(u-.5*r)/(1-r)))
        theta=2*math.pi*progress
        a=np.array(spec['motion_design']['plane_u']);b=np.array(spec['motion_design']['plane_v'])
        center=self.fk(self.idle)[0]
        actual=np.array([self.fk(plan.at(stamp)[0])[0] for stamp in t])
        expected=center+.20*np.sin(theta[:,None])*a+.12*np.sin(2*theta[:,None])*b
        self.assertLess(np.max(np.linalg.norm(actual-expected,axis=1)),.003)
        self.assertGreater(np.ptp((actual-center)@a),.395)
        self.assertGreater(np.ptp((actual-center)@b),.235)
        for lag in (.04,.06):
            controller=Controller(self.gravity,gripper_velocity_limit=plan.gripper_velocity_limit)
            controller.velocity_limit[:6]=plan.velocity_limit
            for i,stamp in enumerate(np.arange(0,plan.duration+.02,.02)):
                q,v=plan.at(stamp);measured,_=plan.at(max(0,stamp-lag))
                command=controller.command(measured,1_000_000_000+i*20_000_000,q,v)
                self.assertTrue(np.all(np.abs(command['estimated_torque_nm'])<=CAPS))


if __name__=='__main__':unittest.main()
