"""Offline daily authoring and persistent power-state regression tests."""
import contextlib
import io
import json
import math
from pathlib import Path
import queue
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
from rs_gravity_play import Controller,TorqueBudgetExceeded,load_clip
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

    def test_recovered_sources_still_rejected_by_raw_player(self):
        for action in ('PROBE','HAPPY'):
            item=self.manifest['actions'][action]
            self.assertFalse(item['source_complete']);self.assertTrue(item['recovered_interrupted'])
            with self.assertRaisesRegex(ValueError,'完整结束'):
                load_clip(ROOT/item['source'],self.identity)
            summary=json.loads(((ROOT/item['source']).parent/'summary.json').read_text())
            self.assertFalse(next(c for c in summary['clips'] if c['file']==Path(item['source']).name)['complete'])

    def test_two_gripper_cycles_are_bounded_and_j2_through_j6_are_stationary(self):
        c=next(c for name,c in self.plans['HAPPY'].stages if name=='gripper_open_close_twice')
        self.assertAlmostEqual(c.half_s,.9)
        self.assertAlmostEqual(c.duration,3.6)
        for t,fraction in ((0,0),(.9,1),(1.8,0),(2.7,1),(3.6,0)):
            q,v=c.at(t);np.testing.assert_allclose(q[1:6],c.start[1:6]);np.testing.assert_allclose(v[1:],0,atol=1e-9)
            self.assertAlmostEqual(q[6]-c.start[6],math.radians(120)*fraction)
        q=np.array([c.at(t)[0] for t in np.linspace(0,c.duration,1001)])
        self.assertLessEqual(q[:,6].max()-q[:,6].min(),math.radians(120)+1e-9)
        self.assertAlmostEqual(math.degrees(c.peak_velocity),250.)
        self.assertLess(math.degrees(c.peak_acceleration),1200.)

    def test_j1_two_synchronized_sway_cycles_and_smooth_return(self):
        c=next(c for _,c in self.plans['HAPPY'].stages if isinstance(c,prepare.Clap))
        self.assertAlmostEqual(math.degrees(c.j1_sway),5.)
        for cycle in (0.,1.8):
            for elapsed,offset in ((0.,0.),(.54,5.),(.9,0.),(1.26,-5.),(1.8,0.)):
                q,v=c.at(cycle+elapsed)
                self.assertAlmostEqual(math.degrees(q[0]-c.start[0]),offset)
                if elapsed!=.9:self.assertAlmostEqual(v[0],0.)
        for t in (0.,.54,1.26,1.8,2.34,3.06,3.6):
            eps=1e-5
            acceleration=(c.at(t+eps)[1][0]-c.at(t-eps)[1][0])/(2*eps)
            self.assertLess(abs(acceleration),.001)
        for t in np.linspace(0,c.duration,1001):
            q,v=c.at(t)
            self.assertLessEqual(abs(q[0]-c.start[0]),math.radians(5.)+1e-10)
            self.assertLessEqual(abs(v[0]),c.j1_peak_velocity+1e-10)
        self.assertLessEqual(c.j1_peak_velocity,.5)
        self.assertLessEqual(c.j1_peak_acceleration,2.)
        np.testing.assert_allclose(c.at(c.duration)[0],c.start,atol=1e-10)
        np.testing.assert_allclose(c.at(c.duration)[1],0,atol=1e-10)

    def test_j1_sway_rejects_outside_joint_bounds_and_excess_dynamics(self):
        from rs_record_playback import UPPER
        pose=np.zeros(7);pose[0]=UPPER[0]-math.radians(2.)
        with self.assertRaisesRegex(ValueError,'J1 sway outside'):prepare.Clap(pose,j1_sway_deg=5.)
        with self.assertRaisesRegex(ValueError,'J1 sway exceeds'):prepare.Clap(np.zeros(7),half_s=.8,j1_sway_deg=5.)
        for angle in (float('nan'),-1.,6.):
            with self.assertRaisesRegex(ValueError,'J1 sway amplitude'):prepare.Clap(np.zeros(7),j1_sway_deg=angle)

    def test_gesture_profiles_and_sleep_wake_keep_authorized_limits(self):
        self.assertLess(self.plans['HAPPY'].duration,13.)
        for action,plan in self.plans.items():
            for label,curve in plan.stages:
                if not hasattr(curve,'velocity_limit'):continue
                velocity,acceleration=((1.,2.) if label in prepare.GESTURE_FAST_STAGES else (.5,1.)) if action in prepare.GESTURE_ACTIONS else (.5,1.)
                if action=='PROBE' and label=='recorded_gesture':velocity,acceleration=1.25,3.125
                self.assertEqual(curve.velocity_limit,velocity)
                self.assertEqual(curve.acceleration_limit,acceleration)
                self.assertLessEqual(curve.peak_velocity,velocity+1e-8)
                self.assertLessEqual(curve.peak_acceleration,acceleration+1e-8)

    def test_probe_body_is_twenty_five_percent_faster_without_changing_path(self):
        from rs_motion import Curve
        spec=json.loads((prepare.BUNDLE/'probe.json').read_text())
        stage=next(s for s in spec['stages'] if s['label']=='recorded_gesture')
        old=Curve(stage['t_s'],stage['q_rad'],velocity_limit=1.,acceleration_limit=2.)
        current=next(c for label,c in self.plans['PROBE'].stages if label=='recorded_gesture')
        self.assertAlmostEqual(old.duration/current.duration,1.25)
        for fraction in np.linspace(0,1,101):
            np.testing.assert_allclose(current.at(current.duration*fraction)[0],
                                       old.at(old.duration*fraction)[0],atol=1e-10)
        self.assertAlmostEqual(self.plans['PROBE'].stages[0][1].duration,2.)
        self.assertAlmostEqual(self.plans['PROBE'].stages[-1][1].duration,2.)

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
        # The gripper has already returned to its idle opening and stays there.
        np.testing.assert_allclose(samples[:,6],self.manifest['idle_rad'][6],atol=1e-10)

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

    def test_fast_gripper_forty_ms_lag_stays_within_current_budget(self):
        # A bounded lag fixture, not a hardware dynamics/acceptance claim.
        c=next(c for _,c in self.plans['HAPPY'].stages if isinstance(c,prepare.Clap))
        ctrl=Controller(self.gravity,gripper_velocity_limit=c.peak_velocity)
        ctrl.velocity_limit[0]=max(ctrl.velocity_limit[0],c.j1_peak_velocity)
        ctrl.following_limit[6]=c.gripper_following_limit
        peak=0.
        for i,t in enumerate(np.arange(0,c.duration+.001,.02)):
            actual=c.at(max(0,t-.04))[0];target,velocity=c.at(t)
            command=ctrl.command(actual,1_000_000_000+i*20_000_000,target,velocity)
            peak=max(peak,abs(command['estimated_torque_nm'][6]))
        self.assertLess(peak,playback.CAPS[6])

    def test_faster_gripper_still_rejects_sixty_ms_lag(self):
        c=next(c for _,c in self.plans['HAPPY'].stages if isinstance(c,prepare.Clap))
        ctrl=Controller(self.gravity,gripper_velocity_limit=c.peak_velocity)
        ctrl.velocity_limit[0]=max(ctrl.velocity_limit[0],c.j1_peak_velocity)
        ctrl.following_limit[6]=c.gripper_following_limit
        with self.assertRaisesRegex(RuntimeError,'torque|following error'):
            for i,t in enumerate(np.arange(0,c.duration+.001,.02)):
                actual=c.at(max(0,t-.06))[0];target,velocity=c.at(t)
                ctrl.command(actual,1_000_000_000+i*20_000_000,target,velocity)

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
        self.assertGreater(rest.duration,4.)
        self.assertLess(rest.duration,5.)

    def test_daily_entry_and_sleep_wake_share_positioning_profile(self):
        idle=np.array(self.manifest['idle_rad']);zero=np.array(self.manifest['zero_rad'])
        entry=playback.prepare_entry(zero,idle,self.gravity,1.,hold_gripper=False,
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
