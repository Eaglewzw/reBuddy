"""Fixed-model-TCP pivot validation and simulated 30% execution. Never opens CAN."""
import copy
import io
import json
import math
from pathlib import Path
import queue
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

import numpy as np
from scipy.spatial.transform import Rotation

ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT/'tools'))
import rs_dance_pivot as pivot
import rs_dance_showcase as showcase
import rs_dance_speed as speed
from rs_gravity_record import DEFAULT_CALIBRATION,DEFAULT_URDF,CAPS
from rs_record_playback import LOWER,UPPER


class ConfigurationTests(unittest.TestCase):
    def test_invalid_configuration_and_trial_speed_are_rejected(self):
        for config in ({'workspace_min_z_m':float('nan')},{'pointing_axis_local':[0,0,0]},
                       {'workspace_reach_fraction':.71},{'overshoot_ratio':.5},
                       {'tcp_offset_m':[float('inf'),0,0]}):
            with self.subTest(config=config),self.assertRaises(ValueError):pivot._config(config)
        for value in (float('nan'),0.,.09,1.01):
            with self.subTest(speed=value),self.assertRaises(ValueError):speed.checked_trial_speed(value)

    def test_orientation_reference_is_continuous_at_all_phase_boundaries(self):
        reference=np.eye(3);cfg=pivot._config()
        for beat in (1.,2.5,4.,8.,12.,14.):
            left=pivot.orientation_at(beat-1e-6,reference,cfg)
            right=pivot.orientation_at(beat+1e-6,reference,cfg)
            self.assertLess(np.linalg.norm(Rotation.from_matrix(left.T@right).as_rotvec()),1e-5)
        hold=pivot.orientation_at(13.,reference,cfg)
        self.assertGreater(np.linalg.norm(Rotation.from_matrix(reference.T@hold).as_rotvec()),.05)
        for beat in np.linspace(12.,14.,21):
            np.testing.assert_allclose(pivot.orientation_at(beat,reference,cfg),hold,atol=1e-12)
        np.testing.assert_allclose(pivot.orientation_at(16.,reference,cfg),reference,atol=1e-12)

    def test_configured_phase_beats_change_total_and_hold_location(self):
        cfg=pivot._config(dict(beat_sections=[3.,4.,4.,1.,2.]))
        self.assertEqual(cfg['beats'],14.)
        reference=np.eye(3);held=pivot.orientation_at(11.5,reference,cfg)
        for beat in np.linspace(11.,12.,21):
            np.testing.assert_allclose(pivot.orientation_at(beat,reference,cfg),held,atol=1e-12)
        np.testing.assert_allclose(pivot.orientation_at(14.,reference,cfg),reference,atol=1e-12)


class RealModelTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not DEFAULT_CALIBRATION.exists() or not DEFAULT_URDF.exists():
            raise unittest.SkipTest('Needs calibrated local RS URDF; no CAN is used')
        cls.manifest,cls.plans,cls.gravity,cls.identity,cls.reports=showcase.load_library(
            DEFAULT_CALIBRATION,DEFAULT_URDF,ROOT/'record/dance_showcase/v4')
        cls.plan=cls.plans['21'];cls.segment=cls.plan.stages[0][1]
        cls.idle=np.array(cls.manifest['idle_rad']);cls.cfg=cls.segment.config
        cls.pin=cls.gravity.pin;cls.model=cls.gravity.model;cls.frame=cls.model.getFrameId('gripper_end')

    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup);self.root=Path(self.tmp.name)

    def fk(self,q,offset=None):
        data=self.model.createData();self.pin.forwardKinematics(self.model,data,np.asarray(q[:6]))
        self.pin.updateFramePlacements(self.model,data);placement=data.oMf[self.frame]
        R=placement.rotation.copy();p=placement.translation.copy()
        return p+R@np.asarray(self.cfg['tcp_offset_m'] if offset is None else offset),R

    def test_fixed_tcp_cone_and_model_only_claims(self):
        anchor,reference=self.fk(self.idle);errors=[];orientation_errors=[];cone=[];azimuth=[]
        for t in np.linspace(0.,self.plan.duration,1001):
            q,v=self.plan.at(t);position,R=self.fk(q);errors.append(np.linalg.norm(position-anchor))
            beat=t/self.plan.duration*16
            desired=pivot.orientation_at(beat,reference,self.cfg,self.segment.amplitude_scale)
            orientation_errors.append(np.linalg.norm(Rotation.from_matrix(desired.T@R).as_rotvec()))
            self.assertAlmostEqual(q[6],0.)
            if 4<=beat<=8:
                axis=R@np.asarray(self.cfg['pointing_axis_local'])
                cone.append(math.degrees(math.acos(np.clip(axis[2],-1,1))))
                azimuth.append(math.atan2(axis[1],axis[0]))
        self.assertLess(max(errors),self.cfg['position_tolerance_m'])
        self.assertLess(math.degrees(max(orientation_errors)),self.cfg['orientation_tolerance_deg'])
        expected=self.cfg['cone_deg']*self.segment.amplitude_scale
        np.testing.assert_allclose(cone,expected,atol=.05)
        self.assertAlmostEqual(abs(np.unwrap(azimuth)[-1]-np.unwrap(azimuth)[0]),2*math.pi,places=3)
        design=self.reports['21']['pivot_design']
        self.assertEqual(design['model_frame'],'gripper_end')
        self.assertFalse(design['physical_tip_calibrated'])
        self.assertFalse(design['physical_pointing_axis_calibrated'])
        self.assertEqual(design['pointing_axis_local'],self.cfg['pointing_axis_local'])

    def test_full_six_dimensional_jacobian_and_declared_operational_envelope(self):
        workspace=self.reports['21']['pivot_design']['workspace']
        self.assertFalse(workspace['reachable_volume_fraction_claimed'])
        self.assertFalse(workspace['collision_checked'])
        self.assertAlmostEqual(workspace['sphere_outer_radius_m'],
                               workspace['chain_length_upper_bound_m']*self.cfg['workspace_reach_fraction'])
        self.assertLessEqual(self.cfg['workspace_reach_fraction'],.7)
        lower=np.maximum(LOWER[:6],self.gravity.lower);upper=np.minimum(UPPER[:6],self.gravity.upper)
        for t in np.linspace(0.,self.plan.duration,251):
            q,v=self.plan.at(t);position,R=self.fk(q)
            radius=np.linalg.norm(position-np.array(workspace['sphere_origin_m']))
            self.assertLessEqual(radius,workspace['sphere_outer_radius_m'])
            self.assertGreaterEqual(radius,workspace['sphere_inner_radius_m'])
            self.assertGreaterEqual(position[2],self.cfg['workspace_min_z_m'])
            self.assertGreaterEqual(np.min(np.minimum(q[:6]-lower,upper-q[:6])),
                                    math.radians(self.cfg['joint_margin_deg'])-1e-8)
            data=self.model.createData()
            J=self.pin.computeFrameJacobian(self.model,data,q[:6],self.frame,
                                            self.pin.ReferenceFrame.LOCAL_WORLD_ALIGNED).copy()
            J[:3]-=self.pin.skew(R@np.array(self.cfg['tcp_offset_m']))@J[3:]
            J[:3]/=self.cfg['jac_characteristic_length_m']
            singular=np.linalg.svd(J,compute_uv=False)
            self.assertEqual(len(singular),6)
            self.assertGreaterEqual(singular[-1],self.cfg['jac_sigma_min'])
            self.assertLessEqual(singular[0]/singular[-1],self.cfg['jac_condition_max'])

    def test_nonzero_offset_fk_and_point_jacobian_match_finite_differences(self):
        cfg=copy.deepcopy(self.cfg);cfg['tcp_offset_m']=[.01,-.006,.004]
        q=self.plan.at(self.plan.duration*.4)[0]
        actual,R=pivot.tcp_pose(self.gravity,q,cfg)
        expected,reference=self.fk(q,cfg['tcp_offset_m'])
        np.testing.assert_allclose(actual,expected,atol=1e-12)
        np.testing.assert_allclose(R,reference,atol=1e-12)
        J=pivot.tcp_jacobian(self.gravity,q,cfg);numeric=np.zeros((6,6));eps=1e-6
        for joint in range(6):
            left=q.copy();right=q.copy();left[joint]-=eps;right[joint]+=eps
            lp,lR=self.fk(left,cfg['tcp_offset_m']);rp,rR=self.fk(right,cfg['tcp_offset_m'])
            numeric[:3,joint]=(rp-lp)/(2*eps)
            numeric[3:,joint]=Rotation.from_matrix(rR@lR.T).as_rotvec()/(2*eps)
        np.testing.assert_allclose(J,numeric,atol=2e-7)

    def test_configured_offset_generation_uses_previous_ik_seed_and_clean_time_grid(self):
        config=dict(tcp_offset_m=[.005,0.,0.],bpm=75.,amplitude_scales=[.5])
        # Instrument the solver's defining module so a public re-export does
        # not accidentally bypass this previous-solution seed regression.
        solver_module=sys.modules[pivot._solve_samples.__module__]
        original=solver_module.least_squares;solves=[]
        def solve(fun,seed,**kwargs):
            saved=np.array(seed).copy();result=original(fun,seed,**kwargs)
            solves.append((saved,result.x.copy()));return result
        with patch.object(solver_module,'least_squares',side_effect=solve):
            stages,extra,_=pivot.build_recipe(self.gravity,self.idle,config)
        generated=pivot.PivotSegment(stages[0])
        self.assertGreater(len(solves),100)
        np.testing.assert_allclose(solves[0][0],self.idle[:6],atol=1e-12)
        for previous,current in zip(solves,solves[1:]):
            np.testing.assert_allclose(current[0],previous[1],atol=1e-12)
        for kind,curve in generated.stages:
            if kind=='dense_curve':self.assertGreater(np.min(np.diff(curve.t_s)),1e-8)
        anchor=self.fk(self.idle,config['tcp_offset_m'])[0]
        for t in np.linspace(0.,generated.duration,251):
            actual=self.fk(generated.at(t)[0],config['tcp_offset_m'])[0]
            self.assertLess(np.linalg.norm(actual-anchor),generated.config['position_tolerance_m'])

    def test_c2_stationary_hold_and_return_keep_budgets(self):
        start,end=self.segment.hold_interval_s
        self.assertAlmostEqual(end-start,self.plan.duration*2/16)
        held=self.plan.at((start+end)/2)[0]
        self.assertGreater(np.max(abs(held-self.idle)),math.radians(1.))
        for t in np.linspace(start,end,101):
            q,v=self.plan.at(t);np.testing.assert_allclose(q,held,atol=1e-10)
            np.testing.assert_allclose(v,0.,atol=1e-8)
            np.testing.assert_allclose(self.plan.acceleration_at(t),0.,atol=1e-7)
        for t in (0.,self.plan.duration):
            np.testing.assert_allclose(self.plan.at(t)[0],self.idle,atol=1e-8)
            np.testing.assert_allclose(self.plan.at(t)[1],0.,atol=1e-8)
            np.testing.assert_allclose(self.plan.acceleration_at(t),0.,atol=1e-7)
        for t in np.linspace(0.,self.plan.duration,1001):
            q,v=self.plan.at(t);a=self.plan.acceleration_at(t)
            self.assertLessEqual(max(abs(v[:6])),1.+1e-8)
            self.assertLessEqual(max(abs(a[:6])),2.+1e-8)

    def test_thirty_percent_changes_execution_only_including_hold_and_jerk(self):
        original_duration=self.plan.duration;scaled=speed.apply_trial_speed(self.plans)
        self.assertIsNot(scaled,self.plans);self.assertIs(scaled['20'],self.plans['20'])
        self.assertIsNot(scaled['21'],self.plan)
        trial=scaled['21'];trial_stage=trial.stages[0][1]
        self.assertAlmostEqual(trial.duration,original_duration/.3)
        np.testing.assert_allclose(trial_stage.hold_interval_s,np.array(self.segment.hold_interval_s)/.3)
        for nominal_t in np.linspace(0.,original_duration,501):
            q,v=self.plan.at(nominal_t);trial_q,trial_v=trial.at(nominal_t/.3)
            np.testing.assert_allclose(trial_q,q,atol=1e-9)
            np.testing.assert_allclose(trial_v,v*.3,atol=1e-9)
            np.testing.assert_allclose(trial.acceleration_at(nominal_t/.3),self.plan.acceleration_at(nominal_t)*.09,atol=1e-8)
            np.testing.assert_allclose(trial.jerk_at(nominal_t/.3),self.segment.jerk_at(nominal_t)*.027,atol=1e-7)
        self.assertEqual(self.plan.duration,original_duration)

    def test_nominal_and_trial_lag_preserve_original_controller_torque_budgets(self):
        from rs_gravity_play import Controller
        for rate in (1.,.3):
            plan=speed.ScaledPlan(self.plan,rate)
            for lag in (.04,.06):
                with self.subTest(rate=rate,lag=lag):
                    controller=Controller(self.gravity,gripper_velocity_limit=plan.gripper_velocity_limit)
                    controller.velocity_limit[:6]=plan.velocity_limit
                    for i,t in enumerate(np.arange(0.,plan.duration+.02,.02)):
                        target,v=plan.at(t);actual,_=plan.at(max(0.,t-lag))
                        command=controller.command(actual,1_000_000_000+i*20_000_000,target,v)
                        self.assertTrue(np.all(np.abs(command['estimated_torque_nm'])<=CAPS))

    def simulate_console(self,fail=False,trial_speed=.3,bundle=None):
        import rs_dance_console as console
        clock=[10.];events=[];state=dict(phase='',injected=False,held=False,hold_ticks=0,reads=0)
        commands=queue.Queue();initial=self.idle.copy()
        class Backend:
            q=initial.copy()
            def heartbeat(self):pass
            def read(self):
                if fail and state['phase'].startswith('21:') and not state['injected']:
                    state['injected']=True;raise RuntimeError('synthetic pivot feedback loss')
                begin=int(clock[0]*1e9);clock[0]+=[.012,.018,.014,.016][state['reads']%4];state['reads']+=1
                return self.q.copy(),begin,int(clock[0]*1e9)
            def enable(self,q):events.append('enable')
            def send(self,command):
                self.q=np.asarray(command['target_rad'])
                if state['held']:
                    state['hold_ticks']+=1
                    if state['hold_ticks']==20:events.append('operator_exit');commands.put('quit')
            def disable(self):events.append('disable');return True
            def close(self):events.append('close')
        def zero_factory(backend,gravity,identity,args,journal):
            put=journal.put
            def observe(row):
                if row['event']=='segment_start':state['phase']=row['phase']
                if row['event']=='action_completed':commands.put('quit')
                if row['event']=='protective_hold_entered':events.append('held');state['held']=True
                put(row)
            journal.put=observe
            def home(cancel,**kw):events.append('home');return not cancel()
            return types.SimpleNamespace(run=home,args=types.SimpleNamespace(exit_zero_tolerance_deg=3.))
        args=types.SimpleNamespace(supported=True,action='21',bundle=bundle or ROOT/'record/dance_showcase/v4',
            calibration=DEFAULT_CALIBRATION,urdf=DEFAULT_URDF,port='can0',pivot_speed=trial_speed)
        with patch.object(console,'ROOT',self.root), \
                patch.object(console.time,'monotonic',side_effect=lambda:clock[0]), \
                patch.object(console.time,'monotonic_ns',side_effect=lambda:int(clock[0]*1e9)), \
                patch.object(console.time,'sleep',side_effect=lambda dt:clock.__setitem__(0,clock[0]+dt)), \
                patch('sys.stdout',io.StringIO()):
            result=console.run(args,self.manifest,self.plans,self.gravity,self.identity,self.reports,
                commands=commands,backend_factory=Backend,zero_factory=zero_factory)
        rows=[json.loads(line) for line in (Path(result['folder'])/'control.jsonl').read_text().splitlines()]
        return result,events,rows

    def test_runtime_sends_scaled_targets_with_feedback_delay_and_preserves_hold(self):
        result,events,rows=self.simulate_console()
        self.assertTrue(result['success'],result)
        self.assertEqual(result['completed'],['21'])
        start=next(r['monotonic_ns'] for r in rows if r['event']=='segment_start' and r.get('phase','').startswith('21:'))
        motion=[r for r in rows if r['event']=='command' and r.get('phase','').startswith('21:')]
        self.assertGreater(len(motion),100)
        held=0;lo,hi=self.segment.hold_interval_s
        for row in motion:
            self.assertGreaterEqual(row['target_sample_ns'],row['feedback_end_ns'])
            t=min((row['target_sample_ns']-start)/1e9*.3,self.plan.duration)
            q,v=self.plan.at(t)
            np.testing.assert_allclose(row['target_rad'],q,atol=1e-8)
            np.testing.assert_allclose(row['velocity_rad_s'],v*.3,atol=1e-8)
            if lo+1e-6<t<hi-1e-6:
                held+=1;np.testing.assert_allclose(row['velocity_rad_s'],0.,atol=1e-8)
        elapsed=(motion[-1]['target_sample_ns']-start)/1e9
        self.assertGreaterEqual(elapsed,self.plan.duration/.3)
        self.assertLess(elapsed,self.plan.duration/.3+.021)
        self.assertGreaterEqual(held,(hi-lo)/.3/.02-2)
        self.assertEqual(events.count('enable'),1)
        self.assertLess(events.index('home'),events.index('disable'))
        self.assertAlmostEqual(result['library']['actions']['21']['duration_s'],self.plan.duration/.3)

    def test_runtime_feedback_failure_holds_until_explicit_exit(self):
        result,events,rows=self.simulate_console(fail=True)
        self.assertEqual(result['completed'],[])
        self.assertIn('held',events)
        self.assertLess(events.index('held'),events.index('operator_exit'))
        self.assertLess(events.index('operator_exit'),events.index('home'))
        self.assertLess(events.index('home'),events.index('disable'))

    def test_existing_showcase_versions_remain_loadable(self):
        for version,count in (('v1',8),('v2',11),('v3',11)):
            with self.subTest(version=version):
                manifest,plans,_,_,_=showcase.load_library(DEFAULT_CALIBRATION,DEFAULT_URDF,
                    ROOT/'record/dance_showcase'/version)
                self.assertEqual(len(plans),count);self.assertNotIn('21',plans)
        self.assertEqual(list(self.plans),['00',*[str(i) for i in range(12,22)],'11'])


if __name__=='__main__':unittest.main()
