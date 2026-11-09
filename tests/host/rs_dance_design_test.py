"""Dance library validation and simulated playback; never opens a CAN socket."""
import json
import math
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'tools'))
import rs_dance_design as design
from rs_gravity_record import digest, CAPS
from rs_record_playback import LOWER, UPPER


class Gravity:
    lower = LOWER[:6]
    upper = UPPER[:6]

    def __call__(self, q):
        return np.array([0., 2., 3., .5, 0., 0.])


class Tests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.daily = self.root/'daily'
        self.daily.mkdir()
        self.output = self.root/'designed'
        self.calibration = self.root/'calibration.json'
        self.urdf = self.root/'model.urdf'
        self.calibration.write_text('{}')
        self.urdf.write_text('synthetic model for offline testing')
        self.identity = dict(calibration_sha256=digest(self.calibration), urdf_sha256=digest(self.urdf))
        self.idle = np.radians([-1.635, 44.893, 66.682, -28.261, -3.082, -.700, .0614])
        self.zero = np.zeros(7); self.zero[6] = self.idle[6]
        self.daily_manifest = dict(schema='rebuddy.daily.prepared.v1', idle_rad=self.idle.tolist(),
                                   zero_rad=self.zero.tolist(), actions={}, **self.identity)
        for name, start, end in [('WAKE',self.zero,self.idle),('REST',self.idle,self.zero)]:
            p = self.daily/(name.lower()+'.json')
            spec = dict(stages=[dict(kind='curve', label=name.lower(), t_s=[0.,1.],
                                    q_rad=[start.tolist(),end.tolist()], speed=1.,
                                    limits=dict(velocity_limit=.5,acceleration_limit=1.))])
            p.write_text(json.dumps(spec))
            self.daily_manifest['actions'][name] = dict(plan_file=p.name,plan_sha256=digest(p))
        (self.daily/'manifest.json').write_text(json.dumps(self.daily_manifest))
        self.addCleanup(patch.stopall)
        patch.object(design, 'DAILY_DIR', self.daily).start()
        patch.object(design, 'preflight', return_value=(Gravity(),self.identity)).start()
        self.manifest = design.build_library(self.calibration,self.urdf,self.output,
                                              recipe_version='legacy_v5')
        # Daily's measured 0.0614-degree gripper offset is not a commanded
        # closed position; dance uses absolute motor zero without re-zeroing.
        self.idle[6]=self.zero[6]=0.

    def load(self, number):
        return design.load_action(number,self.calibration,self.urdf,self.output)

    def test_twelve_numbered_actions_have_expected_names_and_bookends(self):
        self.assertEqual(self.manifest['schema'],'rebuddy.dance_library.v1')
        entries = self.manifest['actions']
        self.assertEqual([int(a['number']) for a in entries],list(range(12)))
        self.assertEqual([a['key'] for a in entries],
                         ['start','nod','lift','dip','reach','base_turn','sway','elbow','wrist','grip','wave','end'])
        self.assertEqual([a['action_id'] for a in entries],[43,*range(33,43),44])
        for i in range(12):
            with self.subTest(action=i):
                plan, gravity, report = self.load(i)
                self.assertFalse(report['hardware_accepted'])
                np.testing.assert_allclose(plan.start,self.zero if i==0 else self.idle,atol=1e-9)
                np.testing.assert_allclose(plan.end,self.zero if i==11 else self.idle,atol=1e-9)
                self.assertEqual(report['source_sha256'],digest(Path(report['source'])))
                self.assertEqual(report['calibration_sha256'],self.identity['calibration_sha256'])
                self.assertEqual(report['urdf_sha256'],self.identity['urdf_sha256'])

    def test_limits_c2_endpoints_and_all_positions_stay_valid(self):
        for number in range(12):
            with self.subTest(action=number):
                plan, gravity, report = self.load(number)
                self.assertTrue(math.isfinite(plan.duration) and 0 < plan.duration <= 60)
                previous=None
                for _, stage in plan.stages:
                    self.assertEqual(stage.velocity_limit,plan.velocity_limit)
                    self.assertEqual(stage.gripper_velocity_limit,plan.gripper_velocity_limit)
                    if previous is not None:
                        # Internal motion need not stop: position, velocity and
                        # acceleration must agree on both sides of the seam.
                        left_q,left_v=previous.at(previous.duration)
                        right_q,right_v=stage.at(0.)
                        np.testing.assert_allclose(left_q,right_q,atol=1e-8)
                        np.testing.assert_allclose(left_v,right_v,atol=1e-8)
                        np.testing.assert_allclose(previous.acceleration_at(previous.duration),
                                                   stage.acceleration_at(0.),atol=1e-7)
                    previous=stage
                for stamp in (0.,plan.duration):
                    q,v=plan.at(stamp)
                    np.testing.assert_allclose(v,0.,atol=1e-8)
                    np.testing.assert_allclose(plan.acceleration_at(stamp),0.,atol=1e-7)
                for stamp in np.linspace(0.,plan.duration,501):
                    q,v=plan.at(stamp);acc=plan.acceleration_at(stamp)
                    self.assertTrue(np.isfinite(q).all() and np.isfinite(v).all() and np.isfinite(acc).all())
                    self.assertTrue(np.all(q>=LOWER-1e-8) and np.all(q<=UPPER+1e-8))
                    self.assertLessEqual(np.max(abs(v[:6])),1.+1e-6)
                    self.assertLessEqual(np.max(abs(acc[:6])),2.+1e-6)
                    self.assertLessEqual(abs(v[6]),plan.gripper_velocity_limit+1e-6)
                    self.assertTrue(np.all(abs(gravity(q[:6]))<=CAPS[:6]))

    def test_all_gestures_move_and_only_grip_moves_gripper(self):
        fingerprints=[]
        for number in range(1,11):
            plan,_,_=self.load(number)
            q=np.array([plan.at(t)[0] for t in np.linspace(0.,plan.duration,251)])
            ranges=np.ptp(q,axis=0)
            if number==9:
                self.assertTrue(plan.gripper_motion)
                self.assertAlmostEqual(plan.duration,2.)
                self.assertAlmostEqual(q[:,6].min(),0.)
                self.assertAlmostEqual(q[:,6].max(),math.radians(45.))
                np.testing.assert_allclose(q[:,:6],np.tile(self.idle[:6],(len(q),1)),atol=1e-10)
            else:
                self.assertFalse(getattr(plan,'gripper_motion',False))
                self.assertLess(ranges[6],1e-9)
                self.assertGreater(np.max(ranges[:6]),math.radians(5.))
            fingerprints.append(np.round(ranges,5).tobytes())
        self.assertEqual(len(set(fingerprints)),10)

    def test_v3_large_gestures_have_visible_joint_excursions(self):
        # Minimum useful ranges for the v3 choreography; substantially beyond
        # the small v2 profiles, without requiring one specific interpolator.
        expected={2:(2,23.),3:(2,28.),4:(1,24.),5:(0,68.),
                  6:(0,43.),7:(2,44.),8:(3,48.),10:(3,33.)}
        for number,(joint,min_range_deg) in expected.items():
            with self.subTest(action=number):
                plan,_,_=self.load(number)
                positions=np.array([plan.at(t)[0] for t in np.linspace(0.,plan.duration,2001)])
                self.assertGreaterEqual(math.degrees(np.ptp(positions[:,joint])),min_range_deg)
        nod,_,_=self.load(1)
        self.assertAlmostEqual(nod.duration,8.)  # Preserve duration and wrist envelope.
        q=np.array([nod.at(t)[0] for t in np.linspace(0.,nod.duration,1601)])
        self.assertAlmostEqual(math.degrees(q[:,3].min()-nod.start[3]),-14.,places=2)
        self.assertLessEqual(math.degrees(q[:,3].max()-nod.start[3]),4.+1e-6)
        self.assertGreater(math.degrees(q[:,3].max()-nod.start[3]),3.9)

    def test_nod_is_one_continuous_phrase_without_internal_holds(self):
        plan,_,report=self.load(1)
        self.assertAlmostEqual(plan.duration,8.)
        spec=json.loads(Path(report['source']).read_text())
        self.assertEqual(len(spec['stages']),1)
        self.assertEqual(spec['stages'][0]['kind'],'spline')
        self.assertEqual(len(plan.stages),1)
        stage=plan.stages[0][1]
        for stamp in stage.t_s[1:-1]:
            # No interior waypoint recreates the old 0.25-second accents or
            # the 0.75-second finish hold; only the action endpoints stop.
            self.assertGreater(np.max(abs(stage.at(stamp)[1][:6])),math.radians(.4))

    def test_v3_flowing_gestures_do_not_pause_at_every_pose(self):
        for number in (1,2,3,4,5,6,7,8,10):
            with self.subTest(action=number):
                plan,_,_=self.load(number)
                t=np.linspace(.1*plan.duration,.9*plan.duration,1601)
                speeds=np.array([np.max(abs(plan.at(stamp)[1][:6])) for stamp in t])
                stationary=speeds<math.radians(.4)
                longest=run=0
                for still in stationary:
                    run=run+1 if still else 0
                    longest=max(longest,run)
                self.assertLess(float(np.mean(stationary)),.08)
                self.assertLess(longest*(t[1]-t[0]),.2)
                # Differentiating the evaluated velocity should agree with
                # the supplied acceleration, including interior spline knots.
                for stamp in np.linspace(.01*plan.duration,.99*plan.duration,101):
                    eps=1e-5
                    finite=(plan.at(stamp+eps)[1]-plan.at(stamp-eps)[1])/(2*eps)
                    np.testing.assert_allclose(finite,plan.acceleration_at(stamp),atol=1e-5)

    def test_continuous_phrases_cross_interior_pose_knots_without_stopping(self):
        for number in (1,2,3,4,5,7,8):
            with self.subTest(action=number):
                plan,_,_=self.load(number)
                crossings=active=0
                for _,stage in plan.stages:
                    if not hasattr(stage,'t_s'):
                        continue
                    for stamp in stage.t_s[1:-1]:
                        eps=1e-6
                        lq,lv=stage.at(stamp-eps);rq,rv=stage.at(stamp+eps)
                        np.testing.assert_allclose(lq,rq,atol=3e-6)
                        np.testing.assert_allclose(lv,rv,atol=5e-6)
                        np.testing.assert_allclose(stage.acceleration_at(stamp-eps),
                                                   stage.acceleration_at(stamp+eps),atol=1e-4)
                        crossings+=1
                        active+=int(np.max(abs(stage.at(stamp)[1][:6]))>math.radians(.4))
                self.assertGreater(crossings,0)
                self.assertGreaterEqual(active,math.ceil(crossings*.75))

    def test_forty_ms_synthetic_tracking_lag_stays_within_control_budgets(self):
        from rs_gravity_play import Controller
        for number in range(12):
            with self.subTest(action=number):
                plan, gravity, report = self.load(number)
                controller=Controller(gravity,gripper_velocity_limit=plan.gripper_velocity_limit)
                controller.velocity_limit[:6]=plan.velocity_limit
                if hasattr(plan,'gripper_following_limit'):
                    controller.following_limit[6]=plan.gripper_following_limit
                for i,stamp in enumerate(np.arange(0.,plan.duration,.02)):
                    target,velocity=plan.at(stamp)
                    actual=plan.at(max(0.,stamp-.04))[0]
                    command=controller.command(actual,1_000_000_000+i*20_000_000,target,velocity)
                    self.assertTrue(np.all(abs(np.asarray(command['estimated_torque_nm']))<=CAPS))

    def simulate_segment(self, plan, gravity, read_delays=None):
        import queue
        import rs_daily_session as runtime
        from rs_gravity_play import Controller
        clock=[10.];sent=[];events=[];reads=[0]
        class Backend:
            q=plan.start.copy()
            def heartbeat(self):pass
            def read(self):
                begin=int(clock[0]*1e9)
                delay=.001 if read_delays is None else read_delays[reads[0]%len(read_delays)]
                reads[0]+=1
                clock[0]+=delay
                end=int(clock[0]*1e9)
                return self.q.copy(),begin,end
            def send(self,command):
                sent.append(command)
                self.q=np.asarray(command['target_rad'])
        writer=types.SimpleNamespace(put=events.append,check=lambda:None,failure=None)
        args=types.SimpleNamespace(calibration=self.calibration,urdf=self.urdf)
        manifest=dict(idle_rad=self.idle.tolist(),zero_rad=self.zero.tolist())
        session=runtime.Session(args,manifest,{},gravity,self.identity,writer,writer,queue.Queue())
        session.backend=Backend()
        session.controller=Controller(gravity,initial=plan.start)
        old_velocity=session.controller.velocity_limit.copy()
        old_follow=session.controller.following_limit.copy()
        with patch.object(runtime.time,'monotonic',side_effect=lambda:clock[0]), \
                patch.object(runtime.time,'monotonic_ns',side_effect=lambda:int(clock[0]*1e9)), \
                patch.object(runtime.time,'sleep',side_effect=lambda dt:clock.__setitem__(0,clock[0]+dt)):
            session.segment('designed_09',curve=plan)
        return session,sent,events,old_velocity,old_follow

    def test_curve_targets_are_sampled_after_variable_feedback_delay(self):
        plan,gravity,_=self.load(5)
        delays=[.012,.018,.014,.016]
        session,sent,events,old_velocity,old_follow=self.simulate_segment(plan,gravity,delays)
        start=next(e['monotonic_ns'] for e in events if e['event']=='segment_start')
        rows=[e for e in events if e['event']=='command']
        self.assertGreater(len(rows),100)
        for index,row in enumerate(rows):
            sampled=row['target_sample_ns']
            self.assertGreaterEqual(sampled,row['feedback_end_ns'])
            self.assertGreaterEqual(sampled,row['feedback_begin_ns'])
            self.assertAlmostEqual((row['feedback_end_ns']-row['feedback_begin_ns'])/1e9,
                                   delays[index%len(delays)],places=8)
            at=min((sampled-start)/1e9,plan.duration)
            target,velocity=plan.at(at)
            np.testing.assert_allclose(row['target_rad'],target,atol=2e-9)
            np.testing.assert_allclose(row['velocity_rad_s'],velocity,atol=2e-9)
        # Queries consume 12-18 ms inside a 20 ms tick, not an extra delay
        # appended to each target sample or to the total motion duration.
        elapsed=(rows[-1]['monotonic_ns']-start)/1e9
        self.assertGreaterEqual(elapsed,plan.duration)
        self.assertLess(elapsed,plan.duration+.021)
        # Distinguish this regression from merely logging a later timestamp:
        # moving targets actually differ from the pre-read trajectory sample.
        row=max(rows,key=lambda r:max(abs(x) for x in r['velocity_rad_s']))
        old_target,_=plan.at((row['feedback_begin_ns']-start)/1e9)
        self.assertGreater(np.max(abs(np.asarray(row['target_rad'])-old_target)),1e-4)

    def test_short_gripper_beat_is_two_seconds_with_independent_budget(self):
        plan,_,report=self.load(9)
        self.assertAlmostEqual(plan.duration,2.)
        self.assertAlmostEqual(plan.gripper_velocity_limit,1.6)
        self.assertAlmostEqual(plan.gripper_acceleration_limit,5.1)
        self.assertAlmostEqual(plan.velocity_limit,1.)
        self.assertAlmostEqual(plan.acceleration_limit,2.)
        samples=np.linspace(0.,plan.duration,1001)
        q=np.array([plan.at(t)[0] for t in samples])
        v=np.array([plan.at(t)[1] for t in samples])
        a=np.array([plan.acceleration_at(t) for t in samples])
        self.assertAlmostEqual(q[:,6].max(),math.radians(45.))
        self.assertLessEqual(np.max(abs(v[:,6])),1.6)
        self.assertLessEqual(np.max(abs(a[:,6])),5.1)
        spec=json.loads(Path(report['source']).read_text())
        self.assertFalse(spec['gripper_control']['full_open'])
        self.assertEqual(spec['gripper_control']['command_range_deg'],[0.,45.])

    def test_legacy_gripper_cruise_remains_c3_with_original_budgets(self):
        # Preserve the older slow full-stroke interpolator's seam contract;
        # current 09 deliberately uses a shorter 45-degree beat.
        opened=self.idle.copy();opened[6]=math.radians(250.)
        moving=[design.GripperCruiseSegment(dict(start_rad=start.tolist(),end_rad=end.tolist(),
                    ramp_s=1.,speed_rad_s=1.,duration_s=math.radians(250.)+1.))
                for start,end in ((self.idle,opened),(opened,self.idle))]
        for index,stage in enumerate(moving):
            sign=1. if index==0 else -1.
            self.assertAlmostEqual(stage.ramp_s,1.)
            self.assertAlmostEqual(stage.duration,math.radians(250.)+1.)
            for stamp in (0.,stage.duration):
                np.testing.assert_allclose(stage.at(stamp)[1],0.,atol=1e-10)
                np.testing.assert_allclose(stage.acceleration_at(stamp),0.,atol=1e-10)
                np.testing.assert_allclose(stage.jerk_at(stamp),0.,atol=1e-10)
            for stamp in (stage.ramp_s,stage.ramp_s+stage.cruise_s):
                eps=1e-6
                lq,lv=stage.at(stamp-eps);rq,rv=stage.at(stamp+eps)
                np.testing.assert_allclose(lq,rq,atol=3e-6)
                np.testing.assert_allclose(lv,rv,atol=3e-6)
                np.testing.assert_allclose(stage.acceleration_at(stamp-eps),
                                           stage.acceleration_at(stamp+eps),atol=1e-5)
                np.testing.assert_allclose(stage.jerk_at(stamp-eps),
                                           stage.jerk_at(stamp+eps),atol=1e-4)
            for stamp in np.linspace(stage.ramp_s,stage.ramp_s+stage.cruise_s,21):
                self.assertAlmostEqual(stage.at(stamp)[1][6],sign)
            acceleration=np.array([stage.acceleration_at(t) for t in np.linspace(0,stage.duration,1001)])
            self.assertLessEqual(np.max(abs(acceleration[:,6])),2.)
            self.assertTrue(np.all(acceleration[:,:6]==0.))

    def test_session_segment_sends_gripper_motion_and_restores_limits(self):
        plan,gravity,report=self.load(9)
        session,sent,events,old_velocity,old_follow=self.simulate_segment(plan,gravity)
        limits=next(e for e in events if e['event']=='segment_limits')
        np.testing.assert_allclose(limits['following_rad'][:6],np.full(6,math.radians(5.)))
        self.assertAlmostEqual(limits['following_rad'][6],math.radians(10.8))
        self.assertAlmostEqual(limits['velocity_rad_s'][6],plan.gripper_velocity_limit)
        grip=np.array([row['target_rad'][6] for row in sent])
        self.assertAlmostEqual(grip.min(),0.)
        self.assertAlmostEqual(grip.max(),math.radians(45.))
        self.assertTrue(any(abs(row['velocity_rad_s'][6])>.12 for row in sent))
        np.testing.assert_allclose(session.controller.velocity_limit,old_velocity)
        np.testing.assert_allclose(session.controller.following_limit,old_follow)
        np.testing.assert_allclose(session.target,plan.end,atol=1e-8)

    def simulate_closed_settle(self, *, target_degrees=0., gripper_error=4.5,
                               arm_error=0., oscillating=False, operation='settle',
                               moving=False, sampler=False):
        import queue
        import rs_daily_session as runtime
        from rs_dance_console import DanceSession
        from rs_gravity_play import Controller
        clock=[10.];events=[]
        target=self.idle.copy();target[6]=math.radians(target_degrees)
        q=target.copy();q[6]+=math.radians(gripper_error);q[0]+=math.radians(arm_error)
        class Backend:
            def heartbeat(self):pass
            def read(self):
                begin=int(clock[0]*1e9);clock[0]+=.001
                actual=q.copy()
                if oscillating:
                    actual[6]+=math.radians(.3)*math.sin(2*math.pi*3*clock[0])
                return actual,begin,int(clock[0]*1e9)
            def send(self,command):pass  # Fixed mechanical stop, not ideal tracking.
        writer=types.SimpleNamespace(put=events.append,check=lambda:None,failure=None)
        args=types.SimpleNamespace(calibration=self.calibration,urdf=self.urdf)
        manifest=dict(idle_rad=self.idle.tolist(),zero_rad=self.zero.tolist())
        session=DanceSession(args,manifest,{},Gravity(),self.identity,writer,writer,queue.Queue())
        session.backend=Backend();session.controller=Controller(Gravity(),initial=q)
        session.home=types.SimpleNamespace(args=types.SimpleNamespace(exit_zero_tolerance_deg=3.))
        original=session.controller.following_limit.copy()
        with patch.object(runtime.time,'monotonic',side_effect=lambda:clock[0]), \
                patch.object(runtime.time,'monotonic_ns',side_effect=lambda:int(clock[0]*1e9)), \
                patch.object(runtime.time,'sleep',side_effect=lambda dt:clock.__setitem__(0,clock[0]+dt)):
            try:
                if operation=='settle':session.settle(target,'ATTENTIVE')
                else:
                    velocity=np.zeros(7);velocity[6]=.01 if moving else 0.
                    if sampler:
                        session.tick(None,target_sampler=lambda:(target,velocity,1.))
                    else:session.tick(target,velocity)
            finally:
                # Even an exception must not leak the closed-stop margin into
                # another action or alter any of the six arm joints' checks.
                np.testing.assert_allclose(session.controller.following_limit,original)
        return session,events,clock[0]-10.

    def test_closed_settle_accepts_physical_stop_without_widening_other_targets(self):
        from rs_recovery_policy import ArrivalTimeout
        for error in (4.5,5.5):
            session,events,elapsed=self.simulate_closed_settle(gripper_error=error)
            self.assertEqual(session.state,'ATTENTIVE')
            self.assertGreaterEqual(elapsed,.5)
            limits=next(e['arrival_tolerance_deg'] for e in events if e['event']=='segment_start')
            np.testing.assert_allclose(limits,[1.,1.,1.,1.,1.,1.,6.])
        with self.assertRaises(ArrivalTimeout):
            self.simulate_closed_settle(target_degrees=20.,gripper_error=4.5)
        with self.assertRaises(ArrivalTimeout):
            self.simulate_closed_settle(arm_error=1.5)
        with self.assertRaises(ArrivalTimeout):
            self.simulate_closed_settle(oscillating=True)

    def test_closed_following_margin_is_static_j7_only_and_works_after_sampling(self):
        from rs_recovery_policy import FollowingError
        for sampler in (False,True):
            with self.subTest(sampler=sampler):
                self.simulate_closed_settle(operation='tick',gripper_error=5.5,sampler=sampler)
                with self.assertRaises(FollowingError):
                    self.simulate_closed_settle(operation='tick',gripper_error=6.1,sampler=sampler)
                with self.assertRaises(FollowingError):
                    self.simulate_closed_settle(operation='tick',target_degrees=20.,
                                                gripper_error=5.5,sampler=sampler)
                with self.assertRaises(FollowingError):
                    self.simulate_closed_settle(operation='tick',gripper_error=5.5,
                                                moving=True,sampler=sampler)
                with self.assertRaises(FollowingError):
                    self.simulate_closed_settle(operation='tick',arm_error=5.5,sampler=sampler)

    def test_gripper_follow_limit_is_scoped_and_still_rejects_excess_error(self):
        from rs_gravity_play import Controller, FOLLOW_TOLERANCE
        from rs_recovery_policy import FollowingError
        for number in range(12):
            plan,gravity,report=self.load(number)
            if number==9:
                self.assertAlmostEqual(plan.gripper_following_limit,math.radians(10.8))
                for _,stage in plan.stages:
                    self.assertAlmostEqual(stage.gripper_following_limit,math.radians(10.8))
            else:
                self.assertFalse(hasattr(plan,'gripper_following_limit'))
                self.assertTrue(all(not hasattr(stage,'gripper_following_limit') for _,stage in plan.stages))
        plan,gravity,report=self.load(9)
        controller=Controller(gravity)
        controller.following_limit[6]=plan.gripper_following_limit
        actual=plan.start.copy()
        goal=actual.copy();goal[6]+=math.radians(7.)
        command=controller.command(actual,1_000_000_000,goal,np.zeros(7))
        self.assertLess(abs(command['estimated_torque_nm'][6]),CAPS[6])
        for i,(joint,degrees) in enumerate(((6,11.),(0,7.))):
            goal=actual.copy();goal[joint]+=math.radians(degrees)
            with self.subTest(joint=joint),self.assertRaises(FollowingError):
                controller.command(actual,1_020_000_000+i*20_000_000,goal,np.zeros(7))
        self.assertEqual(CAPS[6],1.5)
        self.assertAlmostEqual(FOLLOW_TOLERANCE,math.radians(5.))

    def test_legacy_daily_clap_keeps_scalar_velocity_and_following_profile(self):
        from rs_daily_prepare import Clap
        clap=Clap(self.idle)
        self.assertFalse(hasattr(clap,'gripper_velocity_limit'))
        session,sent,events,old_velocity,old_follow=self.simulate_segment(clap,Gravity())
        limits=next(e for e in events if e['event']=='segment_limits')
        self.assertAlmostEqual(limits['velocity_rad_s'][6],clap.peak_velocity)
        self.assertAlmostEqual(limits['following_rad'][6],clap.gripper_following_limit)
        self.assertGreater(np.ptp([c['target_rad'][6] for c in sent]),math.radians(119.))
        np.testing.assert_allclose(session.controller.velocity_limit,old_velocity)
        np.testing.assert_allclose(session.controller.following_limit,old_follow)

    def simulate_console(self, case):
        import io
        import queue
        import rs_dance_console as console
        manifest,plans,gravity,identity,reports=design.load_library(self.calibration,self.urdf,self.output)
        commands=queue.Queue();clock=[10.];events=[];rows=[]
        state=dict(phase='',held=False,hold_ticks=0,injected=False)
        script=iter(['00','09','11','01','stop'])
        class Backend:
            q=self.zero.copy()
            def heartbeat(self):pass
            def read(self):
                if case=='feedback_fault' and state['phase'].startswith('09:') and not state['injected']:
                    state['injected']=True
                    raise RuntimeError('synthetic feedback fault')
                clock[0]+=.001
                n=int(clock[0]*1e9)
                return self.q.copy(),n,n
            def enable(self,q):events.append('enable')
            def send(self,command):
                self.q=np.asarray(command['target_rad'])
                if state['held']:
                    state['hold_ticks']+=1
                    if case=='sequence' and state['hold_ticks']==10:commands.put('01')
                    if state['hold_ticks']==20:
                        events.append('operator_exit')
                        commands.put('release' if case=='home_stop' else 'quit')
            def disable(self):events.append('disable');return True
            def close(self):events.append('close')
        def zero_factory(backend,gravity,identity,args,journal):
            put=journal.put
            def observe(row):
                rows.append(row)
                if row['event']=='segment_start':state['phase']=row['phase']
                if row['event']=='action_completed':
                    if case=='sequence':commands.put(next(script))
                    elif case=='home_stop':commands.put('quit')
                if row['event']=='protective_hold_entered':
                    events.append('held');state['held']=True;state['hold_ticks']=0
                put(row)
            journal.put=observe
            def home(cancel,**kw):
                events.append('home')
                if case=='home_stop':commands.put('stop')
                return not cancel()
            return types.SimpleNamespace(run=home,args=types.SimpleNamespace(exit_zero_tolerance_deg=3.))
        args=types.SimpleNamespace(supported=True,action='09' if case=='feedback_fault' else '00',
                                   bundle=self.output,calibration=self.calibration,urdf=self.urdf,port='can0')
        with patch.object(console,'ROOT',self.root), \
                patch.object(console.time,'monotonic',side_effect=lambda:clock[0]), \
                patch.object(console.time,'monotonic_ns',side_effect=lambda:int(clock[0]*1e9)), \
                patch.object(console.time,'sleep',side_effect=lambda dt:clock.__setitem__(0,clock[0]+dt)), \
                patch('sys.stdout',io.StringIO()) as output:
            result=console.run(args,manifest,plans,gravity,identity,reports,commands=commands,
                               backend_factory=Backend,zero_factory=zero_factory)
        return result,events,rows,output.getvalue()

    def test_console_sequences_bookends_grip_and_protective_stop(self):
        result,events,rows,output=self.simulate_console('sequence')
        self.assertTrue(result['success'],result)
        self.assertEqual(result['completed'],['00','00','09','11','01'])
        self.assertEqual(events.count('enable'),1)
        self.assertEqual(events.count('disable'),1)
        self.assertIn('跳过起身',output)
        self.assertTrue(any(r['event']=='action_rejected' and r.get('action')=='01' for r in rows))
        wakes=[r for r in rows if r['event']=='segment_start' and r.get('phase','').startswith('00:')]
        self.assertEqual(len(wakes),2)  # Initial opening and wake after closing; redundant 00 skipped.
        grip=[r for r in rows if r['event']=='command' and r.get('phase','').startswith('09:')]
        self.assertAlmostEqual(min(r['target_rad'][6] for r in grip),0.)
        self.assertAlmostEqual(max(r['target_rad'][6] for r in grip),math.radians(45.))
        self.assertLess(events.index('held'),events.index('operator_exit'))
        self.assertLess(events.index('operator_exit'),events.index('home'))
        self.assertLess(events.index('home'),events.index('disable'))

    def test_console_feedback_fault_holds_before_explicit_home(self):
        result,events,rows,output=self.simulate_console('feedback_fault')
        self.assertFalse(result['success'])
        self.assertTrue(result['protective_hold_requested'],result)
        self.assertEqual(result['completed'],[])
        self.assertLess(events.index('held'),events.index('operator_exit'))
        self.assertLess(events.index('operator_exit'),events.index('home'))
        self.assertLess(events.index('home'),events.index('disable'))

    def test_terminal_stop_interrupts_home_and_holds_until_release(self):
        result,events,rows,output=self.simulate_console('home_stop')
        self.assertFalse(result['success'])
        self.assertTrue(result['protective_hold_requested'],result)
        self.assertEqual(events.count('home'),1)
        self.assertLess(events.index('home'),events.index('held'))
        self.assertLess(events.index('held'),events.index('operator_exit'))
        self.assertLess(events.index('operator_exit'),events.index('disable'))

    def test_tampered_action_and_daily_source_rejected(self):
        entry=self.manifest['actions'][1]
        action_path=self.output/entry['file']
        saved=action_path.read_text()
        action_path.write_text(saved+'\n')
        with self.assertRaises(ValueError):self.load(1)
        action_path.write_text(saved)
        wake=self.daily/'wake.json';original=wake.read_text();wake.write_text(original+'\n')
        with self.assertRaises(ValueError):self.load(0)
        wake.write_text(original)
        bad_identity=dict(self.identity,calibration_sha256='f'*64)
        with patch.object(design,'preflight',return_value=(Gravity(),bad_identity)),self.assertRaises(ValueError):
            self.load(1)


if __name__=='__main__':unittest.main()
