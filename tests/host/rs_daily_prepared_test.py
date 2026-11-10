"""Offline daily authoring and persistent power-state regression tests."""
import contextlib
import io
import json
import math
from pathlib import Path
import queue
import shutil
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT/'tools'))
import rs_daily_prepare as prepare
import rs_daily_session as runtime
import rs_gravity_play as playback
from rs_gravity_play import Controller,TorqueBudgetExceeded
from rs_gravity_record import DEFAULT_CALIBRATION,DEFAULT_URDF
from rs_recovery_policy import ArrivalTimeout, FollowingError


class Tests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.manifest,cls.plans,cls.gravity,cls.identity=prepare.load_bundle(DEFAULT_CALIBRATION,DEFAULT_URDF)

    def test_all_gestures_share_exact_idle_endpoints(self):
        idle=np.array(self.manifest['idle_rad'])
        for name in ('PROBE','NUZZLE','TILT','HAPPY'):
            plan=self.plans[name]
            np.testing.assert_allclose(plan.start,idle,atol=1e-10)
            np.testing.assert_allclose(plan.end,idle,atol=1e-10)
            for _,curve in plan.stages:
                np.testing.assert_allclose(curve.at(0)[1],0,atol=1e-8)
                np.testing.assert_allclose(curve.at(curve.duration)[1],0,atol=1e-8)
            self.assertLessEqual(self.manifest['actions'][name]['max_sample_fit_error_deg'],.5)

    def test_recording_provenance_is_preserved(self):
        for action in ('PROBE','HAPPY'):
            item=self.manifest['actions'][action]
            self.assertFalse(item['source_complete']);self.assertTrue(item['recovered_interrupted'])
            self.assertTrue(item['source'].endswith('.csv'))
            self.assertEqual(len(item['source_sha256']),64)

    def test_nuzzle_sways_only_after_each_of_two_descents(self):
        from scipy.signal import find_peaks
        from scipy.interpolate import PPoly
        from rs_daily_nuzzle import Nuzzle
        curve=self.plans['NUZZLE'].stages[1][1]
        self.assertIsInstance(curve,Nuzzle)
        spec=json.loads((prepare.BUNDLE/'nuzzle.json').read_text())['stages'][1]
        self.assertEqual(spec['design']['descents'],2)
        self.assertEqual(spec['design']['j5_cycles_per_descent'],1)
        self.assertEqual(spec['design']['sway_timing'],'after_descent_before_rise')
        center=curve.start[4]
        other=[0,1,2,3,5,6]
        for index,(begin,end) in enumerate(curve.sway_windows):
            self.assertEqual(begin,curve.descent_windows[index][1])
            roots=PPoly(curve.spline.c[:,:,4],curve.breakpoints).derivative().roots(extrapolate=False)
            roots=roots[np.isfinite(roots)&(roots>=begin)&(roots<=end)]
            stamps=np.unique(np.r_[np.linspace(begin,end,1001),roots])
            q,v=curve.at(stamps);offset=np.degrees(q[:,4]-center)
            low=np.asarray(spec['poses_rad'][1+2*index])
            np.testing.assert_allclose(curve.at(begin)[0],low,atol=1e-12)
            np.testing.assert_allclose(q[:,other],np.broadcast_to(low[other],(len(q),6)),atol=1e-12)
            np.testing.assert_allclose(v[:,other],0.,atol=1e-12)
            self.assertEqual(len(find_peaks(offset,prominence=1.)[0]),1)
            self.assertEqual(len(find_peaks(-offset,prominence=1.)[0]),1)
            self.assertAlmostEqual(end-begin,2.1)
            self.assertAlmostEqual(max(offset),20.,places=6)
            self.assertAlmostEqual(min(offset),-20.,places=6)
            np.testing.assert_allclose(offset[[0,-1]],0.,atol=1e-10)
            self.assertGreater(curve.at(begin+.25*(end-begin))[0][4],center)
            self.assertLess(curve.at(begin+.75*(end-begin))[0][4],center)
        # No sway during either descent or rise. Only the final rise retains
        # the tiny original J5 endpoint adjustment.
        for i in (0,2,3,5):
            stamps=np.linspace(curve.phase_times[i],curve.phase_times[i+1],101)
            q,v=curve.at(stamps)
            self.assertLess(max(abs(np.degrees(v[:,4]))),.1)
            self.assertLess(np.ptp(np.degrees(q[:,4])),.04)
        for stamp in curve.breakpoints:
            if 0<stamp<curve.duration:
                for k in (0,1,2):
                    np.testing.assert_allclose(curve.spline(stamp-1e-7,nu=k),
                                               curve.spline(stamp+1e-7,nu=k),atol=5e-5,rtol=0)
        for index,stamp in enumerate(curve.phase_times):
            np.testing.assert_allclose(curve.at(stamp)[1],0.,atol=1e-10)
            if index!=3:np.testing.assert_allclose(curve.spline(stamp,nu=2),0.,atol=1e-9)
        # Verify the physical effect from FK, independently of the phase names.
        model=self.gravity.model;data=model.createData();frame=model.getFrameId('gripper_end')
        stamps=np.linspace(0,curve.duration,2001);heights=[]
        for q in curve.at(stamps)[0]:
            self.gravity.pin.framesForwardKinematics(model,data,q[:6])
            heights.append(data.oMf[frame].translation[2])
        bottoms,_=find_peaks(-np.array(heights),prominence=.03)
        self.assertEqual(len(bottoms),2)
        for bottom,(begin,end) in zip(bottoms,curve.sway_windows):
            self.assertGreaterEqual(stamps[bottom],begin-.005)
            self.assertLessEqual(stamps[bottom],end+.005)
        for begin,end in curve.descent_windows:
            selected=(stamps>=begin)&(stamps<=end)
            self.assertLess(np.max(np.diff(np.array(heights)[selected])),1e-6)

    def test_nuzzle_rejects_excessive_speed_or_noncentered_j5(self):
        from rs_daily_nuzzle import Nuzzle
        spec=json.loads((prepare.BUNDLE/'nuzzle.json').read_text())['stages'][1]
        with self.assertRaisesRegex(ValueError,'dynamic budget'):
            Nuzzle(spec['poses_rad'],np.array(spec['durations_s'])*.3)
        poses=np.array(spec['poses_rad']);poses[2,4]+=.1
        with self.assertRaisesRegex(ValueError,'centered J5'):
            Nuzzle(poses,spec['durations_s'])

    def test_nuzzle_upper_turn_reverses_without_flat_stop(self):
        from rs_daily_nuzzle import Nuzzle
        spec=json.loads((prepare.BUNDLE/'nuzzle.json').read_text())['stages'][1]
        previous=Nuzzle(spec['poses_rad'],[1.1,2.1,1.15,1.1,2.1,1.35],20.,sway_scurve=spec['sway_scurve'])
        current=self.plans['NUZZLE'].stages[1][1]
        np.testing.assert_allclose(np.diff(current.phase_times),[1.1,2.1,.85,.9,2.1,1.35],atol=1e-12)
        self.assertAlmostEqual(previous.duration-current.duration,.5)
        fractions=np.linspace(0,1,501)
        for phase in range(6):
            old_t=previous.phase_times[phase]+fractions*np.diff(previous.phase_times)[phase]
            new_t=current.phase_times[phase]+fractions*np.diff(current.phase_times)[phase]
            old_q,old_v=previous.at(old_t);new_q,new_v=current.at(new_t)
            np.testing.assert_allclose(new_q[[0,-1]],old_q[[0,-1]],atol=1e-11,rtol=0)
            if phase not in (2,3):
                np.testing.assert_allclose(new_q,old_q,atol=1e-11,rtol=0)
                np.testing.assert_allclose(new_v,old_v,atol=1e-11,rtol=0)
            else:
                self.assertTrue(np.all(new_v[:,3]*(1 if phase==2 else -1)>=-1e-10))
        turn=current.phase_times[3]
        self.assertLess(current.spline(turn,nu=2)[3],-3.)
        self.assertGreater(current.at(turn-.05)[1][3],.15)
        self.assertLess(current.at(turn+.05)[1][3],-.15)
        def near_top_time(curve):
            t=np.linspace(curve.phase_times[2],curve.phase_times[4],10001)
            offset=curve.at(t)[0][:,3]-curve.at(curve.phase_times[3])[0][3]
            return np.sum(abs(offset)<math.radians(1))*(t[1]-t[0])
        self.assertLess(near_top_time(current),near_top_time(previous)*.6)

    def test_faster_j5_sway_preserves_descent_rise_and_other_axes(self):
        from rs_daily_nuzzle import Nuzzle
        spec=json.loads((prepare.BUNDLE/'nuzzle.json').read_text())['stages'][1]
        previous=Nuzzle(spec['poses_rad'],[1.1,2.25,.85,.9,2.25,1.35],20.,turnaround_factor=6.,
                        sway_peak_fraction=.28,sway_acceleration_factor=72.)
        current=self.plans['NUZZLE'].stages[1][1]
        self.assertAlmostEqual(previous.duration-current.duration,.3)
        fractions=np.linspace(0,1,501)
        for phase in range(6):
            old_t=previous.phase_times[phase]+fractions*np.diff(previous.phase_times)[phase]
            new_t=current.phase_times[phase]+fractions*np.diff(current.phase_times)[phase]
            old_q,old_v=previous.at(old_t);new_q,new_v=current.at(new_t)
            axes=[0,1,2,3,5,6] if phase in (1,4) else list(range(7))
            np.testing.assert_allclose(new_q[:,axes],old_q[:,axes],atol=1e-11,rtol=0)
            np.testing.assert_allclose(new_v[:,axes],old_v[:,axes],atol=1e-11,rtol=0)

    def test_tilt_variable_clock_preserves_path_and_c2_continuity(self):
        from rs_motion_timewarp import TimeWarpedCurve, author_tilt_clock
        curve=self.plans['TILT'].stages[1][1]
        self.assertIsInstance(curve,TimeWarpedCurve)
        rebuilt=TimeWarpedCurve(curve.source,author_tilt_clock(curve.source))
        self.assertAlmostEqual(rebuilt.duration,curve.duration)
        phase=np.linspace(0,curve.source.duration,2001)
        clock=curve.time_at(phase)
        self.assertTrue(np.all(np.diff(clock)>0))
        np.testing.assert_allclose(curve.at(clock)[0],curve.source.at(phase)[0],atol=1e-10,rtol=0)
        np.testing.assert_allclose(curve.source_at(clock),phase,atol=1e-10,rtol=0)
        for stamp in curve.breakpoints[1:-1]:
            before=curve.sample(stamp-1e-7);after=curve.sample(stamp+1e-7)
            for left,right in zip(before,after):np.testing.assert_allclose(left,right,atol=5e-5,rtol=0)
        for stamp in (0.,curve.duration):
            np.testing.assert_allclose(curve.at(stamp)[1],0.,atol=1e-10)
            np.testing.assert_allclose(curve.acceleration_at(stamp),0.,atol=1e-8)
        # Independent finite differences check the clock chain rule.
        for stamp in np.linspace(.001,curve.duration-.001,301):
            eps=1e-6
            finite=(curve.at(stamp+eps)[1]-curve.at(stamp-eps)[1])/(2*eps)
            np.testing.assert_allclose(curve.acceleration_at(stamp),finite,atol=1e-5,rtol=1e-5)

    def test_tilt_clock_rejects_reversal_and_excess_speed(self):
        from rs_motion_timewarp import TimeWarpedCurve
        curve=self.plans['TILT'].stages[1][1]
        spec=json.loads((prepare.BUNDLE/'tilt.json').read_text())['stages'][1]
        bad=json.loads(json.dumps(spec['time_warp']));bad['dt_ds'][10]=-.1
        with self.assertRaisesRegex(ValueError,'positive and bounded'):TimeWarpedCurve(curve.source,bad)
        bad=json.loads(json.dumps(spec['time_warp']));bad['dt_ds']=(np.array(bad['dt_ds'])*.7).tolist()
        with self.assertRaisesRegex(ValueError,'dynamic budget'):TimeWarpedCurve(curve.source,bad)

    def test_gif_timing_matches_only_feasible_actions_and_is_repeatable(self):
        from rs_daily_timing import align_to_gifs
        expected={'PROBE':5.,'NUZZLE':9.680262599376384,
                  'TILT':7.6225413485740745,'HAPPY':9.59247285029744,'REST':7.,'WAKE':5.5}
        with tempfile.TemporaryDirectory() as tmp:
            folder=Path(tmp);shutil.copytree(prepare.BUNDLE,folder,dirs_exist_ok=True)
            reports=align_to_gifs(folder,self.gravity)
            for name,duration in expected.items():
                self.assertAlmostEqual(self.plans[name].duration,duration)
                self.assertAlmostEqual(reports[name]['motion_s'],duration)
                self.assertEqual(reports[name]['matched'],name in ('PROBE','REST','WAKE'))
            # Fixed clap timing is not compressed to fit the eight-second GIF.
            before=json.loads((prepare.BUNDLE/'happy.json').read_text())['stages'][2]
            after=json.loads((folder/'happy.json').read_text())['stages'][2]
            self.assertEqual(before,after)

    def test_bundle_loads_without_authoring_directory(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);folder=root/'daily_ready'
            shutil.copytree(prepare.BUNDLE,folder)
            with patch.object(prepare,'ROOT',root):
                manifest,plans,_,_=prepare.load_bundle(DEFAULT_CALIBRATION,DEFAULT_URDF,folder)
            self.assertFalse((root/'record/gravity').exists())
            self.assertEqual(manifest,self.manifest)
            for name,plan in plans.items():
                for t in np.linspace(0,plan.duration,101):
                    for actual,expected in zip(plan.at(t),self.plans[name].at(t)):
                        np.testing.assert_array_equal(actual,expected)

    def test_corrupt_embedded_anchors_or_identity_rejected(self):
        cases=[('idle_rad',0,.05,'embedded anchors'),
               ('zero_rad',1,.01,'zero anchor'),
               ('idle_rad',2,float('nan'),'finite values'),
               ('calibration_sha256',None,'0'*64,'identity changed')]
        for key,index,value,message in cases:
            with self.subTest(key=key,value=value),tempfile.TemporaryDirectory() as tmp:
                folder=Path(tmp);shutil.copytree(prepare.BUNDLE,folder,dirs_exist_ok=True)
                manifest=json.loads((folder/'manifest.json').read_text())
                if index is None:manifest[key]=value
                else:manifest[key][index]=value
                (folder/'manifest.json').write_text(json.dumps(manifest))
                with self.assertRaisesRegex(ValueError,message):
                    prepare.load_bundle(DEFAULT_CALIBRATION,DEFAULT_URDF,folder)

    def test_corrupted_prepared_preview_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder=Path(tmp);shutil.copytree(prepare.BUNDLE,folder,dirs_exist_ok=True)
            (folder/'probe.csv').write_text('corrupt')
            with self.assertRaisesRegex(ValueError,'preview changed'):
                prepare.load_bundle(DEFAULT_CALIBRATION,DEFAULT_URDF,folder)

    def test_two_gripper_cycles_are_bounded_and_j2_through_j6_are_stationary(self):
        c=next(c for name,c in self.plans['HAPPY'].stages if name=='gripper_open_close_twice')
        self.assertAlmostEqual(c.half_s,1.)
        self.assertAlmostEqual(c.duration,5.4)
        for t,fraction in ((0,0),(.35,0),(1.35,1),(2.35,0),(3.05,0),(4.05,1),(5.05,0),(5.4,0)):
            q,v=c.at(t);np.testing.assert_allclose(q[1:6],c.start[1:6]);np.testing.assert_allclose(v[1:],0,atol=1e-9)
            self.assertAlmostEqual(q[6]-c.start[6],math.radians(90)*fraction)
        q=np.array([c.at(t)[0] for t in np.linspace(0,c.duration,1001)])
        self.assertLessEqual(q[:,6].max()-q[:,6].min(),math.radians(90)+1e-9)
        self.assertAlmostEqual(math.degrees(c.peak_velocity),168.75)
        self.assertLess(math.degrees(c.peak_acceleration),1200.)

    def test_j1_two_thirty_degree_sways_without_center_stop(self):
        c=next(c for _,c in self.plans['HAPPY'].stages if isinstance(c,prepare.Clap))
        self.assertAlmostEqual(math.degrees(c.j1_sway),30.)
        self.assertEqual(c.sway_cycles,2)
        self.assertEqual(c.start[6],0.)
        for elapsed,offset in ((0.,0.),(.9,30.),(2.1,-30.),(3.3,30.),(4.5,-30.),(5.4,0.)):
            q,v=c.at(elapsed)
            self.assertAlmostEqual(math.degrees(q[0]-c.start[0]),offset)
            self.assertAlmostEqual(v[0],0.)
        # Exactly four turning points, with no extra reversal or dwell.
        roots=c.sway_curve.derivative().roots(extrapolate=False)
        roots=sorted({round(float(t),5) for t in roots if 1e-5<t<c.duration-1e-5})
        np.testing.assert_allclose(roots,[.9,2.1,3.3,4.5],atol=1e-5)
        for t in (1.5,2.7,3.9):
            self.assertAlmostEqual(c.at(t)[0][0],c.start[0])
            self.assertGreater(abs(c.at(t)[1][0]),1.3)
        for t in c.sway_times[1:-1]:
            for order in (0,1,2):
                self.assertAlmostEqual(float(c.sway_curve(t-1e-7,nu=order)),
                                       float(c.sway_curve(t+1e-7,nu=order)),places=5)
        for t in (0.,c.duration):
            np.testing.assert_allclose(c.at(t)[0],c.start,atol=1e-10)
            np.testing.assert_allclose(c.at(t)[1],0,atol=1e-10)
            self.assertAlmostEqual(float(c.sway_curve(t,nu=2)),0.)
        for t in np.linspace(0,c.duration,1001):
            q,v=c.at(t)
            self.assertLessEqual(abs(q[0]-c.start[0]),math.radians(30.)+1e-10)
            self.assertLessEqual(abs(v[0]),c.j1_peak_velocity+1e-10)
        self.assertLessEqual(c.j1_peak_velocity,1.4)
        self.assertLessEqual(c.j1_peak_acceleration,4.)

    def test_j1_sway_rejects_outside_joint_bounds_and_excess_dynamics(self):
        from rs_record_playback import UPPER
        pose=np.zeros(7);pose[0]=UPPER[0]-math.radians(2.)
        with self.assertRaisesRegex(ValueError,'J1 sway outside'):prepare.Clap(pose,j1_sway_deg=5.)
        with self.assertRaisesRegex(ValueError,'J1 sway exceeds'):prepare.Clap(np.zeros(7),half_s=.8,j1_sway_deg=5.)
        with self.assertRaisesRegex(ValueError,'J1 sway exceeds'):
            prepare.Clap(np.zeros(7),**{**prepare.HAPPY_CLAP,'sway_duration_s':4.5})
        for angle in (float('nan'),-1.,31.):
            with self.assertRaisesRegex(ValueError,'J1 sway amplitude'):prepare.Clap(np.zeros(7),j1_sway_deg=angle)

    def test_gesture_profiles_and_sleep_wake_keep_authorized_limits(self):
        self.assertAlmostEqual(self.plans['HAPPY'].duration,9.59247285029744)
        for action,plan in self.plans.items():
            for label,curve in plan.stages:
                if not hasattr(curve,'velocity_limit'):continue
                velocity,acceleration=((1.,2.) if label in prepare.GESTURE_FAST_STAGES else (.5,1.)) if action in prepare.GESTURE_ACTIONS else (.5,1.)
                if action=='PROBE' and label=='recorded_gesture':velocity,acceleration=1.6,6.
                if action=='HAPPY' and label in prepare.HAPPY_FAST_STAGES:velocity,acceleration=1.4,4.
                if action in ('NUZZLE','TILT'):
                    if label=='recorded_gesture':velocity,acceleration=1.2,5.5
                    elif label=='generated_return_to_idle':velocity,acceleration=1.,3.
                self.assertEqual(curve.velocity_limit,velocity)
                self.assertEqual(curve.acceleration_limit,acceleration)
                self.assertLessEqual(curve.peak_velocity,velocity+1e-8)
                self.assertLessEqual(curve.peak_acceleration,acceleration+1e-8)

    def test_probe_matches_five_seconds_without_changing_path(self):
        from rs_motion import Curve
        spec=json.loads((prepare.BUNDLE/'probe.json').read_text())
        stage=next(s for s in spec['stages'] if s['label']=='recorded_gesture')
        old=Curve(stage['t_s'],stage['q_rad'],velocity_limit=1.25,acceleration_limit=3.125)
        current=next(c for label,c in self.plans['PROBE'].stages if label=='recorded_gesture')
        self.assertGreater(old.duration/current.duration,1.3)
        self.assertAlmostEqual(self.plans['PROBE'].duration,5.)
        for fraction in np.linspace(0,1,101):
            np.testing.assert_allclose(current.at(current.duration*fraction)[0],
                                       old.at(old.duration*fraction)[0],atol=1e-10)
        self.assertAlmostEqual(self.plans['PROBE'].stages[0][1].duration,.4052252232187142)
        self.assertAlmostEqual(self.plans['PROBE'].stages[-1][1].duration,.8581593997522535)

    def test_happy_clap_immediately_returns_monotonically_to_exact_idle(self):
        plan=self.plans['HAPPY'];labels=[name for name,_ in plan.stages]
        self.assertEqual(labels,['idle_to_recording','recorded_rise',
                                 'gripper_open_close_twice','direct_return_to_idle'])
        clap,returning=plan.stages[-2][1],plan.stages[-1][1]
        np.testing.assert_allclose(clap.end,returning.start,atol=1e-12)
        np.testing.assert_allclose(returning.end,self.manifest['idle_rad'],atol=1e-12)
        delta=returning.end-returning.start
        samples=np.array([returning.at(t)[0] for t in np.linspace(0,returning.duration,201)])
        self.assertTrue(np.all(np.diff(samples,axis=0)*delta>=-1e-12))
        # No initial dwell; smooth zero velocity/acceleration is retained.
        q,v=returning.at(.25)
        self.assertGreater(np.max(abs(np.degrees(q-returning.start))),.2)
        self.assertGreater(np.max(abs(v)),0.)
        for t in (0,returning.duration):
            np.testing.assert_allclose(returning.at(t)[1],0,atol=1e-9)
            np.testing.assert_allclose(returning.spline(t*returning.scale,nu=2),0,atol=1e-8)
        # Closed gripper smoothly returns to the tiny recorded idle offset.
        self.assertEqual(samples[0,6],0.)
        self.assertAlmostEqual(samples[-1,6],self.manifest['idle_rad'][6])
        self.assertTrue(np.all(np.diff(samples[:,6])>=-1e-12))

    def test_accelerated_gestures_with_forty_ms_lag_stay_within_budgets(self):
        # Stress fixture only; actual motor dynamics require operator playback.
        for action in ('PROBE','NUZZLE','TILT','REST','WAKE'):
            for label,curve in self.plans[action].stages:
                with self.subTest(action=action,stage=label):
                    ctrl=Controller(self.gravity)
                    ctrl.velocity_limit[:6]=curve.velocity_limit
                    for t in np.linspace(0,curve.duration,math.ceil(curve.duration/.02)+1):
                        actual=curve.at(max(0,t-.04))[0];target,v=curve.at(t)
                        command=ctrl.command(actual,1_000_000_000+int(t*1e9),target,v)
                        self.assertTrue(np.all(abs(np.array(command['estimated_torque_nm']))<=playback.CAPS))

    def test_fast_gripper_sixty_ms_lag_stays_within_current_budget(self):
        # A bounded lag fixture, not a hardware dynamics/acceptance claim.
        c=next(c for _,c in self.plans['HAPPY'].stages if isinstance(c,prepare.Clap))
        ctrl=Controller(self.gravity,gripper_velocity_limit=c.peak_velocity)
        ctrl.velocity_limit[0]=max(ctrl.velocity_limit[0],c.j1_peak_velocity)
        ctrl.following_limit[6]=c.gripper_following_limit
        peak=0.
        for i,t in enumerate(np.arange(0,c.duration+.001,.02)):
            actual=c.at(max(0,t-.06))[0];target,velocity=c.at(t)
            command=ctrl.command(actual,1_000_000_000+i*20_000_000,target,velocity)
            peak=max(peak,abs(command['estimated_torque_nm'][6]))
        self.assertLess(peak,playback.CAPS[6])

    def test_fast_interactive_actions_with_lag_and_inverse_dynamics(self):
        # Offline model/lag stress checks, not a real-arm acceptance claim.
        pin=self.gravity.pin;model=self.gravity.model;data=model.createData()
        for name in ('PROBE','NUZZLE','TILT','HAPPY'):
            for label,curve in self.plans[name].stages:
                for lag in (.04,.06):
                    with self.subTest(action=name,stage=label,lag=lag):
                        ctrl=Controller(self.gravity)
                        if isinstance(curve,prepare.Clap):
                            ctrl.velocity_limit[0]=curve.j1_peak_velocity
                            ctrl.velocity_limit[6]=curve.peak_velocity
                            ctrl.following_limit[6]=curve.gripper_following_limit
                        else:ctrl.velocity_limit[:6]=curve.velocity_limit
                        for i,t in enumerate(np.linspace(0,curve.duration,math.ceil(curve.duration/.02)+1)):
                            q,v=curve.at(t);actual=curve.at(max(0,t-lag))[0]
                            row=ctrl.command(actual,1_000_000_000+int(t*1e9),q,v)
                            self.assertTrue(np.all(np.abs(row['estimated_torque_nm'])<=playback.CAPS))
                for t in np.linspace(0,curve.duration,501):
                    q,v=curve.at(t);left=max(0,t-1e-5);right=min(curve.duration,t+1e-5)
                    a=(curve.at(right)[1]-curve.at(left)[1])/(right-left)
                    torque=pin.rnea(model,data,q[:6],v[:6],a[:6])
                    self.assertTrue(np.all(np.abs(torque)<=playback.CAPS[:6]))

    def test_gripper_still_rejects_excessive_lag(self):
        c=next(c for _,c in self.plans['HAPPY'].stages if isinstance(c,prepare.Clap))
        ctrl=Controller(self.gravity,gripper_velocity_limit=c.peak_velocity)
        ctrl.velocity_limit[0]=max(ctrl.velocity_limit[0],c.j1_peak_velocity)
        ctrl.following_limit[6]=c.gripper_following_limit
        with self.assertRaisesRegex(RuntimeError,'torque|following error'):
            for i,t in enumerate(np.arange(0,c.duration+.001,.02)):
                actual=c.at(max(0,t-.12))[0];target,velocity=c.at(t)
                ctrl.command(actual,1_000_000_000+i*20_000_000,target,velocity)

    def test_fast_03_04_do_not_bypass_fault_checks(self):
        for name in ('NUZZLE','TILT'):
            curve=next(c for label,c in self.plans[name].stages if label=='recorded_gesture')
            ctrl=Controller(self.gravity);ctrl.velocity_limit[:6]=curve.velocity_limit
            with self.subTest(action=name),self.assertRaisesRegex(RuntimeError,'torque|following error'):
                for t in np.arange(0,curve.duration,.02):
                    q,v=curve.at(t);actual=curve.at(max(0,t-.12))[0]
                    ctrl.command(actual,1_000_000_000+int(t*1e9),q,v)

    def test_rejected_gripper_command_logs_axis_and_pd_terms_before_send(self):
        # Reproduce the opening interval near the recorded 120-degree failure.
        from rs_gravity_play import TorqueBudgetExceeded
        pose=np.zeros(7);pose[6]=.0010715288443344773
        old=prepare.Clap(pose,120.,1.)
        target,velocity=old.at(.282)
        previous=pose.copy();previous[6]=.131615549325943
        actual=pose.copy();actual[6]=.16766409575939178
        ctrl=Controller(lambda q:np.zeros(6),gripper_velocity_limit=old.peak_velocity)
        ctrl.following_limit[6]=old.gripper_following_limit
        ctrl.previous=previous;ctrl.previous_ns=1_000_000_000
        ctrl.velocity[6]=.9664400340338748
        rows=[]
        session=object.__new__(runtime.Session);session.controller=ctrl;session.last_tick=None
        session.check_stop=lambda *a:None
        session.event=lambda name,**kw:rows.append(dict(event=name,**kw))
        session.backend=SimpleNamespace(heartbeat=lambda:None,read=lambda:(actual,1_020_000_000,1_020_000_000),
                                       send=lambda c:self.fail('rejected command sent'))
        # Keep this historical failure test at the old budget. New-budget
        # acceptance of the exact same sample is checked below.
        with patch.object(runtime.time,'monotonic',return_value=1.02), \
                patch.object(playback,'CAPS',np.array([12.,12.,12.,3.,3.,3.,1.])):
            with self.assertRaises(TorqueBudgetExceeded) as error:session.tick(target,velocity,label='HAPPY:gripper_open_close_twice')
        detail=error.exception.details[0]
        self.assertEqual(detail['motor_id'],7);self.assertEqual(detail['budget_nm'],1.)
        self.assertGreater(detail['estimated_nm'],1.)
        self.assertAlmostEqual(detail['estimated_nm'],detail['gravity_nm']+detail['position_pd_nm']+detail['velocity_pd_nm'])
        self.assertEqual(rows[-1]['event'],'command_rejected')
        self.assertEqual(rows[-1]['details'][0],detail)
        self.assertEqual(rows[-1]['phase'],'HAPPY:gripper_open_close_twice')
        current=Controller(lambda q:np.zeros(6),gripper_velocity_limit=old.peak_velocity)
        current.following_limit[6]=old.gripper_following_limit
        current.previous=previous;current.previous_ns=1_000_000_000
        current.velocity[6]=.9664400340338748
        accepted=current.command(actual,1_020_000_000,target,velocity)
        self.assertGreater(accepted['estimated_torque_nm'][6],1.)
        self.assertLess(accepted['estimated_torque_nm'][6],1.5)
        # Same moving sample can be braked within budget at a fresh hold anchor.
        # Preserve measured velocity: setting it to zero would hide braking torque.
        hold=ctrl.command(actual,1_040_000_000,actual,np.zeros(7))
        self.assertLess(hold['estimated_torque_nm'][6],0.)
        self.assertGreater(hold['estimated_torque_nm'][6],-1.)
        np.testing.assert_allclose(hold['target_rad'],actual)

    def test_segment_profile_is_scoped_and_restored_on_failure(self):
        session=object.__new__(runtime.Session)
        session.controller=Controller(lambda q:np.zeros(6),gripper_velocity_limit=math.radians(75))
        session.event=lambda *a,**k:None
        original_velocity=session.controller.velocity_limit.copy()
        original_following=session.controller.following_limit.copy()
        for _,curve in self.plans['HAPPY'].stages:
            def check(*args):
                if isinstance(curve,prepare.Clap):
                    self.assertAlmostEqual(session.controller.velocity_limit[6],curve.peak_velocity)
                    self.assertAlmostEqual(session.controller.velocity_limit[0],curve.j1_peak_velocity)
                    np.testing.assert_array_equal(session.controller.velocity_limit[1:6],original_velocity[1:6])
                    np.testing.assert_allclose(session.controller.following_limit[:6],np.radians(5.))
                    self.assertAlmostEqual(session.controller.following_limit[6],np.radians(10.8))
                    # Gripper window does not waive the shared torque budget.
                    target=np.zeros(7);target[6]=np.radians(10.7)
                    v=np.zeros(7);v[6]=curve.peak_velocity
                    with self.assertRaisesRegex(RuntimeError,'torque'):
                        session.controller.command(np.zeros(7),1,target,v)
                else:np.testing.assert_allclose(session.controller.velocity_limit[:6],curve.velocity_limit)
                raise RuntimeError('simulated segment stop')
            session._segment=check
            with self.assertRaisesRegex(RuntimeError,'simulated segment stop'):session.segment('test',curve=curve)
            np.testing.assert_array_equal(session.controller.velocity_limit,original_velocity)
            np.testing.assert_array_equal(session.controller.following_limit,original_following)

    def test_arm_velocity_limit_not_relaxed_for_gripper(self):
        control=Controller(lambda q:np.zeros(6),gripper_velocity_limit=math.radians(75))
        velocity=np.zeros(7);velocity[6]=1.
        control.command(np.zeros(7),1,np.zeros(7),velocity)
        velocity[0]=.13
        with self.assertRaisesRegex(RuntimeError,'velocity'):
            control.command(np.zeros(7),20_000_001,np.zeros(7),velocity)

    def test_arrival_separates_gripper_precision_without_waiving_arm_or_stillness(self):
        observed=np.array([.102,-.145,-.822,.092,.063,.092,2.118])
        cases=[('observed',observed,'ATTENTIVE',0.,None),
               ('gripper_outside',np.array([0.,0.,0.,0.,0.,0.,3.01]),'ATTENTIVE',0.,'J7'),
               ('arm_outside',np.array([0.,0.,1.01,0.,0.,0.,2.118]),'ATTENTIVE',0.,'J3'),
               ('moving',observed,'ATTENTIVE',2.,'尚未稳定'),
               ('zero_outside',np.array([0.,0.,2.1,0.,0.,0.,0.]),'ZERO',0.,'J3')]
        for name,error,state,speed,reject in cases:
            with self.subTest(name=name):
                clock=[10.];events=[]
                session=object.__new__(runtime.Session)
                session.controller=Controller(lambda q:np.zeros(6))
                session.controller.velocity[6]=math.radians(speed)
                session.home=SimpleNamespace(args=SimpleNamespace(exit_zero_tolerance_deg=2.))
                session.state='MOVING';session.event=lambda event,**kw:events.append(dict(event=event,**kw))
                session.tick=lambda *a,**kw:np.radians(error)
                with patch.object(runtime.time,'monotonic',side_effect=lambda:clock[0]), \
                     patch.object(runtime.time,'sleep',side_effect=lambda dt:clock.__setitem__(0,clock[0]+dt)):
                    if reject:
                        with self.assertRaisesRegex(RuntimeError,reject):session.settle(np.zeros(7),state)
                        self.assertEqual(session.state,'MOVING')
                    else:
                        session.settle(np.zeros(7),state)
                        self.assertEqual(session.state,state)
                        self.assertGreaterEqual(clock[0]-10.,.5)
                entry=next(e for e in events if e['event']=='segment_start')
                np.testing.assert_allclose(entry['arrival_tolerance_deg'],[2. if state=='ZERO' else 1.]*6+[3.])

    def test_corrupted_plan_rejected_before_hardware(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder=Path(tmp)
            import shutil
            shutil.copytree(prepare.BUNDLE,folder,dirs_exist_ok=True)
            (folder/'happy.json').write_text('{}')
            with self.assertRaisesRegex(ValueError,'plan changed'):
                prepare.load_bundle(DEFAULT_CALIBRATION,DEFAULT_URDF,folder)

    def test_rest_wake_are_opposite_zero_idle_paths(self):
        rest,wake=self.plans['REST'],self.plans['WAKE']
        np.testing.assert_allclose(rest.end[:6],0)
        np.testing.assert_allclose(wake.start,rest.end)
        np.testing.assert_allclose(wake.end,rest.start)
        self.assertAlmostEqual(rest.duration,7.)
        self.assertAlmostEqual(wake.duration,5.5)

    def test_daily_entry_and_sleep_wake_share_positioning_profile(self):
        idle=np.array(self.manifest['idle_rad']);zero=np.array(self.manifest['zero_rad'])
        entry=playback.prepare_entry(zero,idle,self.gravity,1.,hold_gripper=False,
                                     minimum_duration=self.plans['WAKE'].duration,
                                     **prepare.DAILY_POSITION_LIMITS)
        wake=self.plans['WAKE']
        self.assertAlmostEqual(entry.duration,wake.duration)
        for fraction in np.linspace(0,1,101):
            np.testing.assert_allclose(entry.at(entry.duration*fraction)[0],wake.at(wake.duration*fraction)[0],atol=1e-10)
        self.assertLessEqual(entry.peak_velocity,.5+1e-10)
        self.assertLessEqual(entry.peak_acceleration,1.+1e-10)
        # Standalone CSV entry retains its default speed and validation.
        previous=playback.prepare_entry(zero,idle,self.gravity,.5,hold_gripper=False)
        self.assertGreater(previous.duration,30.)

    def test_daily_zero_factory_passes_faster_limits_only_to_its_home_controller(self):
        with patch.object(runtime,'prepare_zero_return') as factory:
            result=runtime.prepare_daily_zero_return('backend','gravity','identity','args','journal')
            factory.assert_called_once_with('backend','gravity','identity','args','journal',
                                        exit_speed_rad_s=1.,exit_acceleration_rad_s2=2.,exit_tolerance_deg=3.)
            self.assertIs(result,factory.return_value)

    def simulate(self,mode='normal'):
        clock=[10.];events=[];rows=[];commands=queue.Queue();backends=[]
        idle=np.array([.01,.02,.03,-.02,0.,0.,.001]);zero=idle.copy();zero[:6]=0.
        class Backend:
            def __init__(self):self.q=zero.copy();self.enabled=False;self.returning_to_zero=False;backends.append(self)
            def read(self):
                clock[0]+=.002
                if mode=='fault' and self.enabled:raise RuntimeError('stale feedback')
                stamp=int(clock[0]*1e9);return self.q.copy(),stamp,stamp
            def enable(self,q):events.append('enable');self.enabled=True
            def send(self,c):self.q=np.array(c['target_rad'])
            def heartbeat(self):pass
            def disable(self):events.append('disable');self.enabled=False;return mode!='disable_failure'
            def close(self):events.append('close')
        class Journal:
            def put(self,row):rows.append(row)
            def check(self):pass
        def home_factory(*args):
            def home(*args,**kw):events.append('home');return True
            return SimpleNamespace(args=SimpleNamespace(exit_zero_tolerance_deg=2.),run=home)
        args=SimpleNamespace(port='can-test',calibration=DEFAULT_CALIBRATION,urdf=DEFAULT_URDF)
        from rs_motion import Curve
        plans={'REST':prepare.Plan([('rest',Curve([0,1],[idle,zero]))]),
               'WAKE':prepare.Plan([('wake',Curve([0,1],[zero,idle]))])}
        session=runtime.Session(args,dict(idle_rad=idle.tolist(),zero_rad=zero.tolist()),plans,
                                lambda q:np.zeros(6),self.identity,Journal(),Journal(),commands,Backend,home_factory)
        with patch.object(runtime.time,'monotonic',side_effect=lambda:clock[0]), \
             patch.object(runtime.time,'monotonic_ns',side_effect=lambda:int(clock[0]*1e9)), \
             patch.object(runtime.time,'sleep',side_effect=lambda dt:clock.__setitem__(0,clock[0]+dt)), \
             contextlib.redirect_stdout(io.StringIO()):
            if mode=='normal':
                session.execute('WAKE');self.assertEqual(session.state,'ATTENTIVE')
                session.execute('REST');self.assertEqual(session.state,'ZERO')
                self.assertTrue(session.enabled);self.assertNotIn('disable',events)
                for _ in range(100):clock[0]+=.02;session.tick(session.target)
                self.assertNotIn('disable',events)
                session.execute('SLEEP');self.assertEqual(session.state,'SLEEP');self.assertFalse(session.enabled)
                session.execute('WAKE');self.assertTrue(session.enabled);self.assertEqual(session.state,'ATTENTIVE')
                session.request_stop();session.home_and_exit();session.disconnect()
                self.assertEqual(events,['enable','disable','close','enable','home','disable','close'])
            elif mode=='fault':
                with self.assertRaisesRegex(RuntimeError,'stale'):session.execute('WAKE')
                session.disconnect();self.assertEqual(events,['enable','disable','close'])
            elif mode=='disable_failure':
                session.execute('REST')
                with self.assertRaisesRegex(RuntimeError,'disable not confirmed'):session.execute('SLEEP')
                self.assertNotEqual(session.state,'SLEEP')
            elif mode=='double_stop':
                session.execute('WAKE');session.request_stop();session.request_stop()
                session.home_and_exit();session.disconnect();self.assertNotIn('home',events)
            elif mode=='idle_command':
                session.execute('WAKE');commands.put('REST')
                clock[0]+=.02;session.tick(session.target,busy=False)
                self.assertEqual(commands.get_nowait(),'REST');session.disconnect()
            elif mode=='log_failure':
                session.execute('WAKE')
                session.journal.put=lambda row:(_ for _ in ()).throw(OSError('disk full'))
                with self.assertRaisesRegex(RuntimeError,'disk full'):session.disconnect()
                self.assertEqual(events,['enable','disable','close'])
                self.assertIsNone(session.backend)
            elif mode.startswith('protective'):
                session.execute('WAKE')
                error=(ArrivalTimeout('arrival timeout') if mode=='protective_arrival' else
                       FollowingError('following error') if mode=='protective_following' else
                       TorqueBudgetExceeded([dict(motor_id=7,estimated_nm=1.6,budget_nm=1.5)]))
                if mode=='protective_ramp':
                    session.last_gain=.5
                    with self.assertRaises(TorqueBudgetExceeded):session.enter_protective_hold(error,'WAKE')
                    self.assertEqual(session.protective_stops,[])
                    session.disconnect();return rows
                # Interrupt a real action before it completes. The next read has a
                # different position from both the previous command and its goal.
                def reject(*a,**kw):
                    session.backend.q[6]+=.002
                    raise error
                with patch.object(session,'segment',side_effect=reject):session.execute('REST')
                self.assertEqual(session.state,'PROTECTIVE_HOLD')
                self.assertTrue(session.enabled);self.assertEqual(events,['enable'])
                anchor=session.target.copy()
                self.assertAlmostEqual(anchor[6],idle[6]+.002)
                np.testing.assert_array_equal(session.last_command['velocity_rad_s'],np.zeros(7))
                self.assertFalse(any(r['event']=='action_completed' and r['action']=='REST' for r in rows))
                session.execute('WAKE');self.assertEqual(session.state,'PROTECTIVE_HOLD')
                # A small measured drift must not reset the fixed hold target.
                session.backend.q[6]+=.001
                for _ in range(20):clock[0]+=.02;session.hold_tick()
                np.testing.assert_array_equal(session.target,anchor)
                self.assertEqual(len(session.protective_stops),1)
                self.assertEqual(events,['enable'])
                if mode=='protective_failure':
                    with patch.object(session.controller,'command',side_effect=error):
                        with self.assertRaises(TorqueBudgetExceeded):session.hold_tick()
                    self.assertEqual(len(session.protective_stops),1)
                elif mode=='protective_feedback':
                    with patch.object(session.backend,'read',side_effect=RuntimeError('stale feedback')):
                        with self.assertRaisesRegex(RuntimeError,'stale feedback'):session.hold_tick()
                else:
                    session.request_stop();session.home_and_exit()
                session.disconnect()
                self.assertEqual(events,['enable','home','disable','close'] if mode in ('protective','protective_arrival','protective_following') else ['enable','disable','close'])
        return rows

    def test_rest_holds_sleep_disables_wake_reenables_and_quit_homes(self):self.simulate()
    def test_fault_does_not_home(self):self.simulate('fault')
    def test_failed_disable_cannot_claim_sleep(self):self.simulate('disable_failure')
    def test_second_stop_aborts_home(self):self.simulate('double_stop')
    def test_idle_hold_does_not_discard_next_action(self):self.simulate('idle_command')
    def test_logging_failure_cannot_skip_disable(self):self.simulate('log_failure')
    def test_budget_fault_latches_current_position_and_quit_homes(self):self.simulate('protective')
    def test_arrival_timeout_brakes_then_holds(self):self.simulate('protective_arrival')
    def test_following_error_brakes_then_holds(self):self.simulate('protective_following')
    def test_hold_budget_failure_is_not_reanchored(self):self.simulate('protective_failure')
    def test_hold_feedback_failure_propagates(self):self.simulate('protective_feedback')
    def test_partial_gain_ramp_cannot_enter_protective_hold(self):self.simulate('protective_ramp')

    def test_session_budget_fault_holds_until_quit_or_hold_failure(self):
        for failure in (None,'budget','feedback'):
            events=[];clock=[10.];commands=queue.Queue();injected=[False];sent_hold=[False]
            error=TorqueBudgetExceeded([dict(motor_id=7,estimated_nm=1.6,budget_nm=1.5)])
            class Backend:
                returning_to_zero=False
                def __init__(self):self.q=np.zeros(7)
                def read(self):
                    clock[0]+=.002
                    if injected[0] and failure=='feedback':raise RuntimeError('stale feedback')
                    stamp=int(clock[0]*1e9);return self.q.copy(),stamp,stamp
                def enable(self,q):events.append('enable')
                def send(self,c):
                    self.q=np.array(c['target_rad'])
                    if injected[0] and not sent_hold[0]:
                        sent_hold[0]=True
                        self_outer.assertNotIn('disable',events)
                        commands.put('quit')
                def heartbeat(self):pass
                def disable(self):events.append('disable');return True
                def close(self):events.append('close')
            self_outer=self
            def home_factory(*args):
                def home(*a,**kw):events.append('home');return True
                return SimpleNamespace(args=SimpleNamespace(exit_zero_tolerance_deg=2.),run=home)
            original=runtime.Session._execute_action
            def execute(session,action):
                original(session,action)
                command=session.controller.command
                def reject_once(*a,**kw):
                    if not injected[0] or failure=='budget':
                        injected[0]=True;raise error
                    return command(*a,**kw)
                session.controller.command=reject_once
                if failure=='budget':
                    def failed_recovery(*a,**kw):raise error
                    session.controller.recovery_command=failed_recovery
            with tempfile.TemporaryDirectory() as tmp,patch.object(runtime,'ROOT',Path(tmp)), \
                    patch.object(runtime,'load_bundle',return_value=(self.manifest,self.plans,self.gravity,self.identity)), \
                    patch.object(runtime.Session,'_execute_action',execute), \
                    patch.object(runtime.time,'monotonic',side_effect=lambda:clock[0]), \
                    patch.object(runtime.time,'monotonic_ns',side_effect=lambda:int(clock[0]*1e9)), \
                    patch.object(runtime.time,'sleep',side_effect=lambda dt:clock.__setitem__(0,clock[0]+dt)), \
                    contextlib.redirect_stdout(io.StringIO()):
                args=SimpleNamespace(supported=True,port='can-test',calibration=DEFAULT_CALIBRATION,
                                     urdf=DEFAULT_URDF,bundle=prepare.BUNDLE)
                result=runtime.run_session(args,'WAKE',commands=commands,backend_factory=Backend,zero_factory=home_factory)
            self.assertEqual(sent_hold[0],failure is None)
            self.assertEqual(result['success'],failure is None)
            self.assertEqual(len(result['protective_stops']),int(failure is None))
            self.assertEqual(events,['enable','home','disable','close'] if failure is None else ['enable','disable','close'])

    def test_session_thermal_and_storage_exit_home_but_partial_enable_does_not(self):
        for failure in ('temperature','storage','partial_enable'):
            events=[];clock=[10.]
            class Backend:
                enabled=False;returning_to_zero=False
                def read(self):
                    clock[0]+=.002
                    if self.enabled and not self.returning_to_zero:
                        if failure=='temperature':raise runtime.TemperatureReturnRequested([dict(motor_id=3,temperature_c=100.)])
                        raise runtime.JournalError('disk full')
                    stamp=int(clock[0]*1e9);return np.zeros(7),stamp,stamp
                def enable(self,q):
                    events.append('enable');self.enabled=True
                    if failure=='partial_enable':raise RuntimeError('partial enable failure')
                def heartbeat(self):pass
                def disable(self):events.append('disable');return True
                def close(self):events.append('close')
            def home_factory(backend,*args):
                def home(*a,**kw):events.append('home');backend.read();return True
                return SimpleNamespace(args=SimpleNamespace(exit_zero_tolerance_deg=2.),run=home)
            with tempfile.TemporaryDirectory() as tmp,patch.object(runtime,'ROOT',Path(tmp)), \
                    patch.object(runtime,'load_bundle',return_value=(self.manifest,self.plans,self.gravity,self.identity)), \
                    patch.object(runtime.time,'monotonic',side_effect=lambda:clock[0]), \
                    patch.object(runtime.time,'monotonic_ns',side_effect=lambda:int(clock[0]*1e9)), \
                    patch.object(runtime.time,'sleep',side_effect=lambda dt:clock.__setitem__(0,clock[0]+dt)), \
                    contextlib.redirect_stdout(io.StringIO()):
                args=SimpleNamespace(supported=True,port='can-test',calibration=DEFAULT_CALIBRATION,
                                     urdf=DEFAULT_URDF,bundle=prepare.BUNDLE)
                result=runtime.run_session(args,'WAKE',commands=queue.Queue(),backend_factory=Backend,zero_factory=home_factory)
            self.assertFalse(result['success'])
            self.assertEqual(events,['enable','disable','close'] if failure=='partial_enable' else ['enable','home','disable','close'])


if __name__=='__main__':unittest.main()
