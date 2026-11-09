"""Current dance motion semantics, integrity and simulated playback; never opens CAN."""
import io,json,hashlib
from pathlib import Path
import sys,tempfile,unittest
from unittest.mock import patch
import numpy as np
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT/'tools'))
import rs_dance_library as library
import rs_dance_console as console
import rs_dance_motion as basic
import rs_dance_plan as plans_module
import rs_dance_speed as speed
from dance_mock import MockPlayback
from rs_gravity_record import DEFAULT_CALIBRATION,DEFAULT_URDF,CAPS
from rs_gravity_play import Controller

class Tests(unittest.TestCase):
    simulate_console=MockPlayback.simulate_console

    @classmethod
    def setUpClass(cls):
        cls.loaded=library.load_library(DEFAULT_CALIBRATION,DEFAULT_URDF)
        cls.manifest,cls.plans,cls.gravity,cls.identity,cls.reports=cls.loaded
        cls.idle=np.asarray(cls.manifest['idle_rad'])


    def setUp(self):
        tmp=tempfile.TemporaryDirectory();self.addCleanup(tmp.cleanup);self.root=Path(tmp.name)


    def test_pivot_override_scales_selected_10_only(self):
        self.assertEqual(speed.pivot_code_for(self.plans),'10')
        self.assertEqual(speed.trial_speed_for(self.plans),1.)
        scaled=speed.apply_trial_speed(self.plans,.5)
        self.assertEqual(scaled['10'].duration,self.plans['10'].duration*2)
        for code,plan in self.plans.items():
            if code!='10':self.assertIs(scaled[code],plan)
        ordinary={'10':self.plans['01']}
        self.assertIs(speed.apply_trial_speed(ordinary,.5)['10'],ordinary['10'])


    def test_steady_world_orientation_and_controller_delay(self):
        plan=self.plans['08'];pin=self.gravity.pin;model=self.gravity.model
        data=model.createData();frame=model.getFrameId('gripper_end')
        def orientation(q):
            pin.framesForwardKinematics(model,data,q[:6]);return data.oMf[frame].rotation.copy()
        reference=orientation(self.idle)
        for t in np.linspace(0,plan.duration,1001):
            q,_=plan.at(t)
            self.assertLess(np.linalg.norm(pin.log3(reference.T@orientation(q))),np.radians(.03))
            np.testing.assert_allclose(q[[1,2,6]],self.idle[[1,2,6]],atol=1e-9)
        for t in (0.,plan.duration):
            q,v=plan.at(t)
            np.testing.assert_allclose(q,self.idle,atol=1e-9)
            np.testing.assert_allclose(v,0.,atol=1e-9)
            np.testing.assert_allclose(plan.acceleration_at(t),0.,atol=1e-8)
        for lag in (.04,.06):
            controller=Controller(self.gravity);controller.velocity_limit[:6]=plan.velocity_limit
            for i,t in enumerate(np.arange(0,plan.duration+.02,.02)):
                q,v=plan.at(t);measured,_=plan.at(max(0,t-lag))
                row=controller.command(measured,1_000_000_000+i*20_000_000,q,v)
                self.assertTrue(np.all(np.abs(row['estimated_torque_nm'])<=CAPS))


    def test_steady_fault_still_holds(self):
        result,events,_=self.simulate_console(action='08',fail=True,trial_speed=None,bundle=library.DEFAULT_OUTPUT)
        self.assertEqual(result['completed'],[])
        self.assertLess(events.index('held'),events.index('operator_exit'))
        self.assertLess(events.index('operator_exit'),events.index('home'))


    def test_cute_controller_limits_and_smooth_entry_exit(self):
        for code in ('02','05','06'):
            plan=self.plans[code]
            for t in (0.,plan.duration):
                q,v=plan.at(t)
                np.testing.assert_allclose(q,self.idle,atol=1e-9)
                np.testing.assert_allclose(v,0.,atol=1e-9)
                np.testing.assert_allclose(plan.acceleration_at(t),0.,atol=1e-8)
            for lag in (.04,.06):
                controller=Controller(self.gravity,gripper_velocity_limit=plan.gripper_velocity_limit)
                controller.velocity_limit[:6]=plan.velocity_limit
                if plan.gripper_motion:controller.following_limit[6]=plan.gripper_following_limit
                for i,t in enumerate(np.arange(0,plan.duration+.02,.02)):
                    q,v=plan.at(t);measured,_=plan.at(max(0,t-lag))
                    row=controller.command(measured,1_000_000_000+i*20_000_000,q,v)
                    self.assertTrue(np.all(np.abs(row['estimated_torque_nm'])<=CAPS))


    def test_cute_fault_still_holds(self):
        result,events,_=self.simulate_console(action='06',fail=True,trial_speed=None,bundle=library.DEFAULT_OUTPUT)
        self.assertEqual(result['completed'],[])
        self.assertLess(events.index('held'),events.index('operator_exit'))
        self.assertLess(events.index('operator_exit'),events.index('home'))


    def test_exactly_two_deep_nods_with_rebounds(self):
        from scipy.interpolate import PPoly
        curve=self.plans['01'].stages[0][1].curve.spline
        p=PPoly.from_spline((curve.t,curve.c[:,3],curve.k))
        roots=p.derivative().roots(extrapolate=False)
        roots=roots[np.isfinite(roots)&(roots>1e-5)&(roots<self.plans['01'].duration-1e-5)]
        troughs=[t for t in roots if p(t,nu=2)>0]
        peaks=[t for t in roots if p(t,nu=2)<0]
        self.assertEqual(len(troughs),2);self.assertEqual(len(peaks),2)
        for t in troughs:self.assertLess(np.degrees(p(t)-self.idle[3]),-27.)
        for t in peaks:self.assertGreater(np.degrees(p(t)-self.idle[3]),11.)
        # The center crossing between beats keeps moving, rather than stopping
        # at the idle position and restarting a second standalone clip.
        times=np.linspace(peaks[0],troughs[1],501)
        t=min(times,key=lambda t:abs(p(t)-self.idle[3]))
        self.assertGreater(abs(p(t,nu=1)),.5)


    def test_deep_nod_controller_with_following_delay(self):
        plan=self.plans['01']
        for lag in (.04,.06):
            controller=Controller(self.gravity)
            controller.velocity_limit[:6]=plan.velocity_limit
            for i,t in enumerate(np.arange(0,plan.duration+.02,.02)):
                q,v=plan.at(t);measured,_=plan.at(max(0,t-lag))
                row=controller.command(measured,1_000_000_000+i*20_000_000,q,v)
                self.assertTrue(np.all(np.abs(row['estimated_torque_nm'])<=CAPS))


    def test_list_uses_selected_menu_without_runtime(self):
        with patch.object(library,'load_library',return_value=self.loaded),patch.object(console,'run') as run,patch('sys.stdout',io.StringIO()) as output:
            self.assertEqual(console.main(['--list']),0)
        run.assert_not_called()
        self.assertIn('10组＋开场/收尾',output.getvalue())
        self.assertIn('10 大幅定点左右上下',output.getvalue())
        self.assertIn('./dance --supported',output.getvalue())


    def test_modified_source_pin_rejected(self):
        manifest=json.loads((library.DEFAULT_OUTPUT/'manifest.json').read_text())
        manifest['calibration_sha256']='0'*64
        (self.root/'manifest.json').write_text(json.dumps(manifest))
        with self.assertRaisesRegex(ValueError,'calibration/model changed'):
            library.load_library(DEFAULT_CALIBRATION,DEFAULT_URDF,self.root)


    def test_all_ten_execute_correct_alias_and_log_original(self):
        for code in [f'{i:02d}' for i in range(1,11)]:
            with self.subTest(code=code):
                result,events,rows=self.simulate_console(action=code,trial_speed=None,bundle=library.DEFAULT_OUTPUT)
                self.assertTrue(result['success'],result);self.assertEqual(result['completed'],[code])
                logged=result['library']['actions'][code]
                self.assertEqual(logged['source_number'],self.reports[code]['source_number'])
                self.assertEqual(logged['execution_speed'],1.)
                if code in ('02','06'):
                    limits=[r for r in rows if r['event']=='segment_limits' and r.get('phase','').startswith(code+':')]
                    self.assertEqual(len(limits),1)
                    self.assertEqual(limits[0]['velocity_rad_s'][6],2.8)
                    self.assertAlmostEqual(limits[0]['following_rad'][6],basic.GRIPPER_FOLLOWING_LIMIT)
                curves={code+':'+label:curve for label,curve in self.plans[code].stages}
                starts={}
                for row in rows:
                    phase=row.get('phase')
                    if row['event']=='segment_start' and phase in curves:
                        starts[phase]=row['monotonic_ns']
                    if row['event']=='command' and phase in curves:
                        curve=curves[phase]
                        t=min((row['target_sample_ns']-starts[phase])/1e9,curve.duration)
                        q,v=curve.at(t)
                        np.testing.assert_allclose(row['target_rad'],q,atol=1e-8)
                        np.testing.assert_allclose(row['velocity_rad_s'],v,atol=1e-8)


    def test_renumbered_pivot_fault_still_holds(self):
        result,events,_=self.simulate_console(action='10',fail=True,trial_speed=None,bundle=library.DEFAULT_OUTPUT)
        self.assertEqual(result['completed'],[])
        self.assertLess(events.index('held'),events.index('operator_exit'))
        self.assertLess(events.index('operator_exit'),events.index('home'))
        self.assertLess(events.index('home'),events.index('disable'))


    def test_deep_nod_fault_still_holds(self):
        result,events,_=self.simulate_console(action='01',fail=True,trial_speed=None,bundle=library.DEFAULT_OUTPUT)
        self.assertEqual(result['completed'],[])
        self.assertLess(events.index('held'),events.index('operator_exit'))
        self.assertLess(events.index('operator_exit'),events.index('home'))


    def test_migration_preserves_every_sampled_position_velocity_acceleration(self):
        fixture=json.loads((ROOT/'tests/host/fixtures/dance_motion_fingerprints.json').read_text())
        self.assertEqual(list(self.plans),[f'{i:02d}' for i in range(12)])
        for code,plan in self.plans.items():
            expected=fixture[code];self.assertEqual(plan.duration,expected['duration_s'])
            t=np.linspace(0,plan.duration,1001)
            for field,values in [('q',[plan.at(x)[0] for x in t]),('v',[plan.at(x)[1] for x in t]),('a',[plan.acceleration_at(x) for x in t])]:
                values=np.round(values,8);values[abs(values)<.5e-8]=0.
                self.assertEqual(hashlib.sha256(values.astype('<f8').tobytes()).hexdigest(),expected[field])

    def test_two_complete_sways_and_moving_center(self):
        from scipy.interpolate import PPoly
        plan=self.plans['08'];s=plan.stages[0][1].curve.spline
        p=PPoly.from_spline((s.t,s.c[:,0],s.k));roots=p.derivative().roots(extrapolate=False)
        roots=roots[np.isfinite(roots)&(roots>1e-5)&(roots<plan.duration-1e-5)]
        angles=np.degrees(p(roots)-self.idle[0])
        self.assertEqual(sum(angles>34.5),2);self.assertEqual(sum(angles<-34.5),2)
        self.assertGreater(abs(plan.at(plan.duration/2)[1][0]),.8)

    def test_two_second_90_degree_gripper_pulses(self):
        for code in ('02','06'):
            plan=self.plans[code];segment=plan.stages[0][1]
            self.assertEqual(segment.pulse.duration,2.)
            self.assertEqual(plan.gripper_velocity_limit,2.8)
            self.assertEqual(plan.gripper_acceleration_limit,10.)
            self.assertEqual(plan.gripper_following_limit,basic.GRIPPER_FOLLOWING_LIMIT)
            self.assertEqual(len(segment.pulse_starts_s),2)
            for start in segment.pulse_starts_s:
                self.assertAlmostEqual(np.degrees(plan.at(start+1.)[0][6]),90.)
                for t in (start,start+2.):
                    q,v=plan.at(t);self.assertAlmostEqual(q[6],0.);self.assertAlmostEqual(v[6],0.)
                    self.assertAlmostEqual(plan.acceleration_at(t)[6],0.)
            for t in np.linspace(0,plan.duration,501):
                q,v=plan.at(t);bq,bv=segment.body.at(t)
                self.assertGreaterEqual(q[6],-1e-9);self.assertLessEqual(q[6],np.radians(90.)+1e-9)
                np.testing.assert_array_equal(q[:6],bq[:6]);np.testing.assert_array_equal(v[:6],bv[:6])

    def test_pivot_handoff_and_fixed_point(self):
        plan=self.plans['10'];segment=plan.stages[0][1];bridge=segment.bridge
        pin=self.gravity.pin;model=self.gravity.model;data=model.createData();frame=model.getFrameId('gripper_end')
        point=np.asarray(self.reports['10']['pivot_sway_design']['P_world_m'])
        for t in np.linspace(*segment.dance_interval_s,1001):
            q,v=plan.at(t);pin.framesForwardKinematics(model,data,q[:6]);pose=data.oMf[frame]
            self.assertLess(np.linalg.norm(pose.translation+.2*pose.rotation[:,0]-point),.002)
            if bridge.start_s<=t<=bridge.end_s:self.assertGreater(max(abs(v[:6])),.05)
        for t in (bridge.start_s,bridge.end_s):
            np.testing.assert_allclose(segment.at(t),segment.source.at(t),atol=1e-9)
            np.testing.assert_allclose(segment.acceleration_at(t),segment.source.acceleration_at(t),atol=1e-8)

    def test_changed_action_bytes_rejected_before_playback(self):
        import shutil
        for path in library.DEFAULT_OUTPUT.glob('*.json'):shutil.copy2(path,self.root/path.name)
        path=self.root/'02_hello.json';path.write_text(path.read_text()+' ')
        with self.assertRaisesRegex(ValueError,'checksum'):
            library.load_library(DEFAULT_CALIBRATION,DEFAULT_URDF,self.root)

    def test_compatibility_flag_uses_the_same_library(self):
        with patch.object(library,'load_library',return_value=self.loaded),patch.object(console,'run') as run,patch('sys.stdout',io.StringIO()):
            self.assertEqual(console.main(['--selected','--check']),0)
        run.assert_not_called()

if __name__=='__main__':unittest.main()
