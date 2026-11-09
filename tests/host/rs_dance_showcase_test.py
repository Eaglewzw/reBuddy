"""Offline showcase contracts: no CAN backend or sockets are constructed."""
import io
import json
import copy
import math
from pathlib import Path
import queue
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

import numpy as np

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'tools'))
import rs_dance_design as base
import rs_dance_showcase as showcase
from rs_gravity_record import digest,CAPS
from rs_record_playback import LOWER,UPPER


class Gravity:
    lower=LOWER[:6]
    upper=UPPER[:6]
    def __call__(self,q):return np.array([0.,2.,3.,.5,0.,0.])


class Tests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.root=Path(self.tmp.name);self.daily=self.root/'daily';self.daily.mkdir()
        self.output=self.root/'showcase';self.calibration=self.root/'calibration.json';self.urdf=self.root/'model.urdf'
        self.calibration.write_text('{}');self.urdf.write_text('offline fixture')
        self.identity=dict(calibration_sha256=digest(self.calibration),urdf_sha256=digest(self.urdf))
        self.idle=np.radians([-1.635,44.893,66.682,-28.261,-3.082,-.700,0.])
        self.zero=np.zeros(7)
        manifest=dict(schema='rebuddy.daily.prepared.v1',idle_rad=self.idle.tolist(),zero_rad=self.zero.tolist(),actions={},**self.identity)
        for name,start,end in [('WAKE',self.zero,self.idle),('REST',self.idle,self.zero)]:
            p=self.daily/(name.lower()+'.json')
            p.write_text(json.dumps(dict(stages=[dict(kind='curve',label=name.lower(),t_s=[0.,1.],q_rad=[start.tolist(),end.tolist()],speed=1.,limits=dict(velocity_limit=.5,acceleration_limit=1.))])))
            manifest['actions'][name]=dict(plan_file=p.name,plan_sha256=digest(p))
        (self.daily/'manifest.json').write_text(json.dumps(manifest))
        self.addCleanup(patch.stopall)
        patch.object(base,'DAILY_DIR',self.daily).start()
        patch.object(showcase,'preflight',return_value=(Gravity(),self.identity)).start()
        # Exercise serialization/validation without inventing a Pinocchio
        # model. The real model's orbit/FK evidence is checked separately.
        def orbit_points(gravity,idle):
            angle=np.linspace(0.,2*math.pi,17)
            q=np.tile(idle,(len(angle),1))
            q[:,0]+=math.radians(5.)*np.sin(angle)
            q[:,4]+=math.radians(2.)*(1-np.cos(angle))
            return q,dict(test_fixture=True)
        patch.object(showcase,'_orbit_points',side_effect=orbit_points).start()
        self.manifest=showcase.build_library(self.calibration,self.urdf,self.output,
                                             library_version='v1')

    def load(self):
        return showcase.load_library(self.calibration,self.urdf,self.output)

    def test_catalog_identity_and_bookend_anchors(self):
        manifest,plans,gravity,identity,reports=self.load()
        self.assertEqual(manifest['schema'],'rebuddy.dance_showcase.v1')
        self.assertEqual(list(plans),['00','12','13','14','15','16','17','11'])
        self.assertFalse(manifest['hardware_accepted'])
        self.assertEqual(identity,self.identity)
        for number,plan in plans.items():
            with self.subTest(action=number):
                np.testing.assert_allclose(plan.start,self.zero if number=='00' else self.idle,atol=1e-9)
                np.testing.assert_allclose(plan.end,self.zero if number=='11' else self.idle,atol=1e-9)
                self.assertFalse(reports[number]['hardware_accepted'])
                self.assertEqual(reports[number]['source_sha256'],digest(Path(reports[number]['source'])))
        for row in manifest['actions']:
            self.assertIsNone(row['action_id'])

    def test_every_stage_c2_and_physical_budgets(self):
        _,plans,gravity,_,_=self.load()
        for number,plan in plans.items():
            with self.subTest(action=number):
                previous=None
                for _,stage in plan.stages:
                    self.assertEqual(stage.velocity_limit,plan.velocity_limit)
                    self.assertEqual(stage.gripper_velocity_limit,plan.gripper_velocity_limit)
                    if previous is not None:
                        pq,pv=previous.at(previous.duration);q,v=stage.at(0.)
                        np.testing.assert_allclose(pq,q,atol=1e-8)
                        np.testing.assert_allclose(pv,v,atol=1e-8)
                        np.testing.assert_allclose(previous.acceleration_at(previous.duration),stage.acceleration_at(0.),atol=1e-7)
                    previous=stage
                for t in (0.,plan.duration):
                    np.testing.assert_allclose(plan.at(t)[1],0.,atol=1e-8)
                    np.testing.assert_allclose(plan.acceleration_at(t),0.,atol=1e-7)
                for t in np.linspace(0.,plan.duration,501):
                    q,v=plan.at(t);a=plan.acceleration_at(t)
                    self.assertTrue(np.all(q>=LOWER-1e-8) and np.all(q<=UPPER+1e-8))
                    self.assertLessEqual(max(abs(v[:6])),1.+1e-7)
                    self.assertLessEqual(max(abs(a[:6])),2.+1e-7)
                    self.assertGreaterEqual(q[6],-1e-9)
                    self.assertLessEqual(q[6],math.radians(45.)+1e-9)
                    self.assertLessEqual(abs(v[6]),1.6+1e-7)
                    self.assertLessEqual(abs(a[6]),5.1+1e-7)
                    self.assertTrue(np.all(abs(gravity(q[:6]))<=CAPS[:6]))

    def test_deliberate_freeze_is_a_real_stationary_pose(self):
        _,plans,_,_,_=self.load();plan=plans['14']
        holds=[]
        for label,stage in plan.stages:
            samples=[stage.at(t) for t in np.linspace(0.,stage.duration,17)]
            q=np.array([x[0] for x in samples]);v=np.array([x[1] for x in samples])
            if np.max(np.ptp(q,axis=0))<1e-9:
                holds.append((label,stage))
                np.testing.assert_allclose(v,0.,atol=1e-9)
                for t in np.linspace(0.,stage.duration,17):
                    np.testing.assert_allclose(stage.acceleration_at(t),0.,atol=1e-8)
        self.assertTrue(holds,'spring/freeze must contain an intentional stationary beat')
        self.assertTrue(any(stage.duration>=.1 for _,stage in holds))

    def test_gripper_overlay_moves_with_body_and_inherits_scoped_limits(self):
        _,plans,_,_,_=self.load()
        for number,plan in plans.items():
            times=np.linspace(0.,plan.duration,1001)
            positions=np.array([plan.at(t)[0] for t in times])
            if number=='16':
                self.assertTrue(plan.gripper_motion)
                self.assertAlmostEqual(plan.gripper_following_limit,math.radians(10.8))
                self.assertAlmostEqual(plan.gripper_velocity_limit,1.6)
                self.assertGreater(np.ptp(positions[:,6]),math.radians(40.))
                self.assertGreater(np.max(np.ptp(positions[:,:6],axis=0)),math.radians(5.))
                velocities=np.array([plan.at(t)[1] for t in times])
                self.assertTrue(np.any((abs(velocities[:,6])>.1)&(np.max(abs(velocities[:,:6]),axis=1)>.02)))
                for _,stage in plan.stages:
                    self.assertAlmostEqual(stage.gripper_following_limit,math.radians(10.8))
            else:
                self.assertLess(np.ptp(positions[:,6]),1e-9)

    def test_modified_action_or_dependency_is_rejected(self):
        item=self.manifest['actions'][2];path=self.output/item['file'];original=path.read_bytes()
        path.write_bytes(original+b' ')
        with self.assertRaises(ValueError):self.load()
        path.write_bytes(original)
        manifest_path=self.output/'manifest.json';original_manifest=manifest_path.read_text()
        altered=json.loads(original_manifest);altered['calibration_sha256']='0'*64
        manifest_path.write_text(json.dumps(altered))
        with self.assertRaises(ValueError):self.load()
        manifest_path.write_text(original_manifest)
        wake=self.daily/'wake.json';wake.write_text(wake.read_text()+' ')
        with self.assertRaises(ValueError):self.load()

    def test_rehashed_action_still_must_match_the_current_entry_pose(self):
        manifest_path=self.output/'manifest.json'
        manifest=json.loads(manifest_path.read_text())
        item=next(row for row in manifest['actions'] if row['number']==12)
        path=self.output/item['file'];spec=json.loads(path.read_text())
        spec['stages'][0]['q_rad'][0][0]+=math.radians(.1)
        path.write_text(json.dumps(spec));item['sha256']=digest(path)
        manifest_path.write_text(json.dumps(manifest))
        with self.assertRaises(ValueError):self.load()

    def test_overlay_rejects_overlap_truncation_and_unapproved_pulses(self):
        *_,reports=self.load()
        original=reports['16']['stages'][0]
        for schedule in ([-.1,3.5],[.5,1.],[.5,5.],[3.5,.5]):
            with self.subTest(schedule=schedule):
                spec=copy.deepcopy(original);spec['pulse_starts_s']=schedule
                with self.assertRaises(ValueError):showcase.CompositeSegment(spec)
        spec=copy.deepcopy(original)
        spec['pulse_stages'][0]['duration_s']+=.01
        with self.assertRaises(ValueError):showcase.CompositeSegment(spec)

    def test_shorter_phrases_use_real_time_and_preserve_dynamic_limits(self):
        _,plans,_,_,reports=self.load()
        for number in ('12','13','14','15','16','17'):
            with self.subTest(action=number):
                plan=plans[number]
                self.assertLessEqual(plan.duration,10. if number=='17' else 8.)
                self.assertAlmostEqual(plan.duration,reports[number]['duration_s'])
                self.assertAlmostEqual(reports[number]['bpm'],reports[number]['beats']*60/plan.duration)
                self.assertLessEqual(np.max(plan.peak_velocity[:6]),1.+1e-7)
                self.assertLessEqual(np.max(plan.peak_acceleration[:6]),2.+1e-7)

    def test_forty_ms_tracking_lag_stays_inside_existing_controller_budgets(self):
        from rs_gravity_play import Controller
        _,plans,gravity,_,_=self.load()
        for number,plan in plans.items():
            with self.subTest(action=number):
                controller=Controller(gravity,gripper_velocity_limit=plan.gripper_velocity_limit)
                controller.velocity_limit[:6]=plan.velocity_limit
                if hasattr(plan,'gripper_following_limit'):
                    controller.following_limit[6]=plan.gripper_following_limit
                for i,t in enumerate(np.arange(0.,plan.duration+.02,.02)):
                    target,velocity=plan.at(t)
                    actual,_=plan.at(max(0.,t-.04))
                    command=controller.command(actual,1_000_000_000+i*20_000_000,target,velocity)
                    self.assertTrue(np.all(abs(np.asarray(command['estimated_torque_nm']))<=CAPS))

    def test_actual_session_sends_simultaneous_gripper_and_body_motion(self):
        import rs_daily_session as runtime
        from rs_dance_console import DanceSession
        from rs_gravity_play import Controller
        manifest,plans,gravity,identity,_=self.load()
        clock=[10.];events=[];sent=[]
        initial=self.idle.copy()
        class Backend:
            q=initial
            def heartbeat(self):pass
            def read(self):
                begin=int(clock[0]*1e9);clock[0]+=.001
                return self.q.copy(),begin,int(clock[0]*1e9)
            def send(self,command):
                sent.append(command);self.q=np.array(command['target_rad'])
        writer=types.SimpleNamespace(put=events.append,check=lambda:None,failure=None)
        args=types.SimpleNamespace(calibration=self.calibration,urdf=self.urdf)
        session=DanceSession(args,manifest,plans,gravity,identity,writer,writer,queue.Queue())
        session.backend=Backend();session.controller=Controller(gravity,initial=initial)
        original_limits=session.controller.following_limit.copy()
        with patch.object(runtime.time,'monotonic',side_effect=lambda:clock[0]), \
                patch.object(runtime.time,'monotonic_ns',side_effect=lambda:int(clock[0]*1e9)), \
                patch.object(runtime.time,'sleep',side_effect=lambda dt:clock.__setitem__(0,clock[0]+dt)):
            session.execute_plan('16','ATTENTIVE')
        q=np.array([r['target_rad'] for r in sent]);v=np.array([r['velocity_rad_s'] for r in sent])
        self.assertGreater(np.ptp(q[:,6]),math.radians(40.))
        self.assertTrue(np.any((abs(v[:,6])>.1)&(np.max(abs(v[:,:6]),axis=1)>.02)))
        self.assertEqual(session.state,'ATTENTIVE')
        np.testing.assert_allclose(session.target,self.idle,atol=1e-8)
        np.testing.assert_allclose(session.controller.following_limit,original_limits)

    def test_console_library_selection_is_offline_and_backwards_compatible(self):
        import rs_dance_console as console
        result=self.load()
        for flags,selected,other in [(['--showcase','--check'],showcase,base),(['--list'],base,showcase)]:
            with self.subTest(flags=flags),patch.object(selected,'load_library',return_value=result) as loader, \
                    patch.object(other,'load_library',side_effect=AssertionError('wrong library')), \
                    patch.object(console,'run',side_effect=AssertionError('offline CLI opened runtime')), \
                    patch('sys.stdout',io.StringIO()):
                self.assertEqual(console.main(flags),0)
                self.assertEqual(loader.call_args.args[2],selected.DEFAULT_OUTPUT)
        plans=result[1]
        self.assertEqual(console.number_for('16',plans),'16')
        with self.assertRaises(ValueError):console.number_for('09',plans)


if __name__=='__main__':unittest.main()
