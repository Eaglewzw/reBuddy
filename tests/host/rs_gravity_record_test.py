"""Offline gravity/recording/fault tests. Never opens a CAN controller."""
import importlib.util
import io
import json
from pathlib import Path
import queue
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tools'))
import rs_gravity_record as m


class Gravity:
    lower = np.full(6, -3.)
    upper = np.full(6, 3.)

    def __call__(self, q):
        return np.array([0., 2., 3., .5, 0., 0.])


class Tests(unittest.TestCase):
    def test_failed_startup_preserves_observer_before_close_without_enable(self):
        import rs_record_playback
        import rs_feedback_audit
        with patch.object(rs_record_playback,'Hardware') as hardware, \
                patch.object(rs_feedback_audit,'Monitor') as monitor:
            hardware.return_value.reports.side_effect=RuntimeError('missing reports')
            with self.assertRaisesRegex(RuntimeError,'missing reports'):
                m.LiveBackend('can0',None,supported=True)
            monitor.return_value.drain.assert_called_once_with()
            monitor.return_value.close.assert_called_once_with()
            hardware.return_value.enable_supported_trial.assert_not_called()
            hardware.return_value.close.assert_called_once_with()

    def test_journal_failure_latches_without_growing_queue_during_return(self):
        for allow in (False,True):
            j=object.__new__(m.Journal)
            j.path=Path('/tmp/test-session.jsonl');j.error=None;j.failure=None
            j.allow_failure=allow;j.queue=queue.Queue(maxsize=1)
            j.put({'first':True})
            if allow:
                j.put({'overflow':True})
                for _ in range(100):j.put({'discarded':True})
            else:
                with self.assertRaises(m.JournalError):j.put({'overflow':True})
            self.assertIn('queue full',j.failure)
            self.assertEqual(j.queue.qsize(),1)

    def test_storage_failures_return_without_masking_feedback_or_losing_result(self):
        import errno
        cases=('session_error','raw_error','queue_full','during_home','feedback_fault',
               'home_feedback_fault','summary_full','partial_enable')
        for case in cases:
            with self.subTest(case=case):
                writers={};events=[];phase=['setup'];commands=queue.Queue()
                class FailingJournal(m.Journal):
                    injected=False
                    def __init__(self,path):
                        super().__init__(path);writers[path.name]=self
                    def put(self,row):
                        target='can.jsonl' if case in ('raw_error','feedback_fault') else 'session.jsonl'
                        active=self.allow_failure and (phase[0]=='home' if case=='during_home' else True)
                        if case not in ('summary_full','partial_enable') and active and not self.injected and self.path.name==target:
                            self.injected=True
                            if case=='queue_full':self.fail('journal queue full')
                            else:
                                self.queue.put_nowait(None)  # Model a writer that has stopped on ENOSPC.
                                self.error=OSError(errno.ENOSPC,'No space left on device')
                        super().put(row)
                class Backend:
                    enabled=False
                    def read(self):
                        writers['can.jsonl'].put({'event':'raw_feedback'})
                        if self.enabled and (case=='feedback_fault' or (case=='home_feedback_fault' and phase[0]=='home')):
                            raise RuntimeError('invalid motor feedback')
                        stamp=m.time.monotonic_ns()
                        return np.zeros(7),stamp,stamp
                    def enable(self,q):
                        events.append('enable');self.enabled=True
                        if case=='partial_enable':
                            writers['session.jsonl'].queue.put_nowait(None)
                            writers['session.jsonl'].error=OSError(errno.ENOSPC,'disk full during enable')
                            writers['session.jsonl'].put({'event':'partial_enable'})
                    def heartbeat(self):pass
                    def send(self,command):commands.put('quit')
                    def disable(self):events.append('disable');return True
                    def close(self):events.append('close')
                backend=Backend()
                def prepare(backend,gravity,metadata,args,journal):
                    def home(cancel,**kwargs):
                        events.append('home');phase[0]='home'
                        for _ in range(3):
                            backend.read()
                            journal.put({'event':'zero_return_command'})
                        return not cancel()
                    return types.SimpleNamespace(run=home)
                original_write=Path.write_text
                def write(path,*a,**kw):
                    if case=='summary_full' and path.name=='summary.json':
                        raise OSError(errno.ENOSPC,'summary disk full')
                    return original_write(path,*a,**kw)
                with tempfile.TemporaryDirectory() as tmp, patch.object(m,'Journal',FailingJournal), \
                        patch.object(Path,'write_text',write),patch('sys.stdout',io.StringIO()):
                    folder=Path(tmp);cal=folder/'cal.json';cal.write_text('{}')
                    args=types.SimpleNamespace(mode='record',supported=True,seconds=0.,output=folder,calibration=cal)
                    r=m.run(args,Gravity(),dict(calibration_sha256=m.digest(cal)),
                            backend_factory=lambda:backend,commands=commands,zero_return_factory=prepare)
                self.assertFalse(r['success'],r)
                self.assertTrue(r['disable_confirmed'])
                self.assertEqual(events[-2:],['disable','close'])
                if case in ('feedback_fault','partial_enable'):
                    self.assertNotIn('home',events)
                    self.assertFalse(r['zero_return_requested'])
                elif case=='home_feedback_fault':
                    self.assertFalse(r['zero_return_completed'])
                    self.assertEqual(r['reason'],'zero_return_failed')
                else:
                    self.assertTrue(r['zero_return_completed'],r)
                    self.assertLess(events.index('home'),events.index('disable'))
                if case=='summary_full':self.assertTrue(any('summary save failed' in e for e in r['errors']))
                else:self.assertTrue(r['logging_errors'],r)
                if case in ('session_error','raw_error','queue_full'):
                    self.assertEqual(r['reason'],'logging_error')

    def test_async_pose_disk_failure_is_typed_and_latched(self):
        import errno
        with tempfile.TemporaryDirectory() as tmp:
            folder=Path(tmp);pose=folder/'idle.json';pose.write_text('previous pose')
            journal=m.Journal(folder/'session.jsonl');journal.allow_failure=True
            with patch.object(m.os,'replace',side_effect=OSError(errno.ENOSPC,'disk full')):
                future=journal.save_pose(pose,{'q_rad':[0.]*7})
                with self.assertRaises(m.JournalError) as caught:future.result(timeout=2)
                journal.fail(caught.exception)
                journal.put({'event':'zero_return_command'})
                with self.assertRaisesRegex(RuntimeError,'journal incomplete'):journal.close()
            self.assertIn('disk full',journal.failure)
            self.assertEqual(pose.read_text(),'previous pose')

    def test_record_requires_support_and_interactive_terminal(self):
        with patch.object(m, 'preflight') as preflight, patch.object(m, 'run') as run, \
                patch('sys.stderr', io.StringIO()), patch.object(m.sys.stdin, 'isatty', return_value=False):
            self.assertEqual(m.main(['record']), 2)
            self.assertEqual(m.main(['record', '--supported']), 2)
            preflight.assert_not_called()
            run.assert_not_called()
        with self.assertRaisesRegex(ValueError, '--supported'):
            m.LiveBackend('can0',None)
        backend = object.__new__(m.LiveBackend)
        backend.supported = False
        with self.assertRaisesRegex(ValueError, '--supported'):
            backend.enable(np.zeros(7))

    def test_supported_record_reaches_preflight_and_runner(self):
        gravity = Gravity()
        metadata = dict(calibration_sha256='test')
        with patch.object(m, 'preflight', return_value=(gravity, metadata)) as preflight, \
                patch.object(m, 'run', return_value=dict(success=True)) as run, \
                patch.object(m.sys.stdin, 'isatty', return_value=True), patch('sys.stdout', io.StringIO()):
            self.assertEqual(m.main(['record', '--supported']), 0)
            preflight.assert_called_once()
            args, actual_gravity, actual_metadata = run.call_args.args
            self.assertEqual(args.mode, 'record')
            self.assertEqual(args.seconds, 0.)
            self.assertTrue(args.supported)
            self.assertIs(actual_gravity, gravity)
            self.assertIs(actual_metadata, metadata)

    def test_supported_backend_constructs_and_enables_using_existing_checks(self):
        import rs_record_playback
        import rs_feedback_audit
        with patch.object(rs_record_playback, 'Hardware') as hardware, \
                patch.object(rs_feedback_audit, 'Monitor') as monitor:
            backend = m.LiveBackend('can0', None, supported=True)
            hardware.assert_called_once_with('can0')
            hardware.return_value.reports.assert_called_once()
            monitor.return_value.faults=[]
            q = np.zeros(7)
            backend.enable(q)
            hardware.return_value.enable_supported_trial.assert_called_once_with(
                q, fall_arrest_confirmed=True, progress=backend._startup_feedback)
            hardware.return_value.stop_reports.assert_called_once_with()
            self.assertLess(
                [c[0] for c in hardware.return_value.method_calls].index('enable_supported_trial'),
                [c[0] for c in hardware.return_value.method_calls].index('stop_reports'))
            self.assertEqual(backend.mode, 2)
            self.assertEqual(monitor.return_value.accepted_modes, {2})

    def test_startup_observer_drains_and_never_clears_faults(self):
        from unittest.mock import Mock
        backend=object.__new__(m.LiveBackend)
        backend.monitor=types.SimpleNamespace(drain=Mock(),faults=[])
        backend._startup_feedback()
        backend.monitor.drain.assert_called_once_with()
        backend.monitor.faults.append({'motor':3,'fault_bits':1})
        with self.assertRaisesRegex(RuntimeError,'during enable'):
            backend._startup_feedback()
        self.assertEqual(len(backend.monitor.faults),1)

    def test_live_session_continues_past_ten_minutes_until_quit(self):
        # Advance every 20 ms tick, without real waiting, CAN or large log files.
        for seconds in (0., 900.):
            clock=[10.];commands=queue.Queue();events=[];samples=[0]
            class Journal:
                failure=None;error=None
                def __init__(self,*_): pass
                def check(self): pass
                def put(self,row): pass
                def close(self): pass
            class Backend:
                def read(self):
                    stamp=int(clock[0]*1e9)
                    return np.zeros(7),stamp,stamp
                def enable(self,q): events.append('enable')
                def heartbeat(self): pass
                def send(self,command):
                    samples[0]+=1
                    if clock[0]>=613.: commands.put('quit')
                def disable(self): events.append('disable');return True
                def close(self): events.append('close')
            def sleep(dt): clock[0]+=dt
            def home(*_):
                return types.SimpleNamespace(run=lambda *a,**kw:events.append('home') or True)
            with tempfile.TemporaryDirectory() as tmp, patch.object(m,'Journal',Journal), \
                    patch.object(m,'export_clips',return_value=[]), patch('sys.stdout',io.StringIO()), \
                    patch.object(m.time,'monotonic',side_effect=lambda:clock[0]), \
                    patch.object(m.time,'monotonic_ns',side_effect=lambda:int(clock[0]*1e9)), \
                    patch.object(m.time,'sleep',side_effect=sleep):
                folder=Path(tmp);cal=folder/'cal.json';cal.write_text('{}')
                args=types.SimpleNamespace(mode='record',supported=True,seconds=seconds,
                                          output=folder,calibration=cal)
                result=m.run(args,Gravity(),dict(calibration_sha256=m.digest(cal)),
                             backend_factory=Backend,commands=commands,zero_return_factory=home)
                self.assertTrue(result['success'],result)
                self.assertEqual(result['reason'],'quit')
                self.assertGreater(samples[0],30000)
                self.assertEqual(events,['enable','home','disable','close'])
                meta=json.loads((Path(result['folder'])/'metadata.json').read_text())
                self.assertEqual(meta['session_limit_s'],seconds or None)

    def test_rejected_sdk_position_still_drains_raw_evidence(self):
        backend=object.__new__(m.LiveBackend);events=[]
        def bad_read():
            events.append('sdk');raise RuntimeError('invalid J4 value')
        backend.hw=types.SimpleNamespace(positions=bad_read)
        backend.monitor=types.SimpleNamespace(drain=lambda:events.append('drain'))
        with self.assertRaisesRegex(RuntimeError,'invalid J4'):
            backend.read()
        self.assertEqual(events,['drain','sdk','drain'])

    def test_temperature_exit_requires_valid_feedback_on_every_axis(self):
        for homing in (False, True):
            for problem in ('none', 'raw_mismatch', 'stale', 'motor_fault', 'latched_fault', 'invalid_temperature'):
                backend=object.__new__(m.LiveBackend)
                backend.mode=2;backend.returning_to_zero=homing
                backend.hw=types.SimpleNamespace(positions=lambda:np.zeros(7),check_status=lambda mode:None)
                backend.monitor=types.SimpleNamespace(drain=lambda:None,faults=[],
                    positions={i:dict(received_unix_ns=1_015_000_000,position_rad=0.) for i in range(1,8)},
                    status={i:dict(received_unix_ns=1_005_000_000,mode=2,fault_bits=0,
                                   temperature_c=100. if i==1 else 30.) for i in range(1,8)})
                if problem=='raw_mismatch':backend.monitor.positions[7]['position_rad']=.1
                if problem=='stale':backend.monitor.status[7]['received_unix_ns']=0
                if problem=='motor_fault':backend.monitor.status[7]['fault_bits']=1
                if problem=='latched_fault':backend.monitor.faults.append({'kind':'fault'})
                if problem=='invalid_temperature':backend.monitor.status[7]['temperature_c']=float('nan')
                with self.subTest(homing=homing,problem=problem), \
                        patch.object(m.time,'time_ns',side_effect=[1_000_000_000,1_020_000_000]):
                    if problem=='none' and homing:
                        np.testing.assert_array_equal(backend.read()[0],np.zeros(7))
                    elif problem=='none':
                        with self.assertRaises(m.TemperatureReturnRequested) as caught:backend.read()
                        self.assertEqual(caught.exception.motors,[dict(motor_id=1,temperature_c=100.)])
                    else:
                        with self.assertRaises(RuntimeError) as caught:backend.read()
                        self.assertNotIsInstance(caught.exception,m.TemperatureReturnRequested)

    def test_temperature_threshold_is_100_in_recording_and_playback(self):
        from rs_gravity_play import PlaybackBackend
        for backend_type in (m.LiveBackend,PlaybackBackend):
            for temperature in (65.,99.9,100.,100.1):
                backend=object.__new__(backend_type);backend.mode=2;backend.returning_to_zero=False
                backend.hw=types.SimpleNamespace(positions=lambda:np.zeros(7),check_status=lambda mode:None)
                backend.monitor=types.SimpleNamespace(drain=lambda:None,faults=[],
                    positions={i:dict(received_unix_ns=1_015_000_000,position_rad=0.) for i in range(1,8)},
                    status={i:dict(received_unix_ns=1_005_000_000,mode=2,fault_bits=0,
                                   temperature_c=temperature) for i in range(1,8)})
                with self.subTest(backend=backend_type.__name__,temperature=temperature), \
                        patch.object(m.time,'time_ns',side_effect=[1_000_000_000,1_020_000_000]):
                    if temperature<100.:
                        np.testing.assert_array_equal(backend.read()[0],np.zeros(7))
                    else:
                        with self.assertRaises(m.TemperatureReturnRequested):backend.read()

    def test_sdk_failure_batch_is_retained_before_exception_escapes(self):
        backend=object.__new__(m.LiveBackend)
        batch=dict(sdk_rad=[0.,0.,0.,1e26,0.,0.,0.],errors=[],read_intervals_ns={})
        backend.hw=types.SimpleNamespace(last_read=batch,
            positions=lambda:(_ for _ in ()).throw(RuntimeError('bad SDK position')))
        backend.monitor=types.SimpleNamespace(drain=lambda:None)
        rows=[];backend.raw_journal=types.SimpleNamespace(put=rows.append)
        with self.assertRaisesRegex(RuntimeError,'bad SDK position'):backend.read()
        self.assertEqual(rows[0]['event'],'sdk_position_batch')
        self.assertEqual(rows[0]['sdk_rad'][3],1e26)

    def test_disable_retains_raw_transition_frames(self):
        backend=object.__new__(m.LiveBackend);events=[]
        backend.hw=types.SimpleNamespace(disable=lambda:events.append('disable') or True)
        backend.monitor=types.SimpleNamespace(drain=lambda:events.append('drain'))
        self.assertTrue(backend.disable())
        self.assertEqual(events,['disable','drain'])
        self.assertEqual(backend.monitor.accepted_modes,{0,2})

    def pose_metadata(self):
        return dict(calibration_sha256='a'*64,urdf_sha256='b'*64,mock=False)

    def stable_pose(self):
        history=[(i*20_000_000,np.array([0.,.7,1.1,0.,0.,0.,0.])) for i in range(51)]
        return m.capture_idle_pose(history,self.pose_metadata(),Path('/example/session'))

    def test_idle_capture_requires_stable_contiguous_window(self):
        pose=self.stable_pose()
        np.testing.assert_allclose(pose['q_rad'],[0,.7,1.1,0,0,0,0])
        self.assertEqual(pose['samples'],51)
        self.assertFalse(pose['hardware_accepted'])
        for history in ([],[(0,np.zeros(7))],
                        [(0,np.zeros(7)),(1_000_000_000,np.zeros(7))],
                        [(i*20_000_000,np.ones(7)*i*.001) for i in range(51)]):
            with self.assertRaises(ValueError):
                m.capture_idle_pose(history,self.pose_metadata(),Path('/example'))

    def test_saved_idle_is_bound_to_calibration_model_and_mock_mode(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder=Path(tmp);path=folder/'presets/idle.json'
            journal=m.Journal(folder/'session.jsonl')
            pose=self.stable_pose()
            future=journal.save_pose(path,pose)
            journal.close()
            self.assertEqual(future.result(),pose)
            self.assertEqual(m.load_idle_pose(path,self.pose_metadata()),pose)
            for key in ('calibration_sha256','urdf_sha256','mock'):
                meta=self.pose_metadata();meta[key]='different'
                with self.assertRaisesRegex(ValueError,key):
                    m.load_idle_pose(path,meta)

    def test_idle_based_actions_require_anchor_and_return(self):
        pose=self.stable_pose();q=np.array(pose['q_rad']);away=q.copy();away[3]+=.1
        for name in ('idle','probe','nuzzle_2','tilt-3','happy'):
            with self.assertRaisesRegex(ValueError,'先设置'):
                m.clip_anchor(name,None,q)
            with self.assertRaisesRegex(ValueError,'J4'):
                m.clip_anchor(name,pose,away)
            anchor=m.clip_anchor(name,pose,q)
            self.assertTrue(anchor['entry_idle']);self.assertTrue(anchor['exit_idle'])
        self.assertFalse(m.clip_anchor('rest',pose,q)['exit_idle'])
        self.assertFalse(m.clip_anchor('wake',pose,away)['entry_idle'])
        self.assertIsNone(m.clip_anchor('freeform',None,away))
        with tempfile.TemporaryDirectory() as tmp:
            folder=Path(tmp);journal=m.Journal(folder/'session.jsonl');clips=m.Clips(journal)
            clips.start('probe',1,m.clip_anchor('probe',pose,q))
            with self.assertRaisesRegex(ValueError,'J4'):
                clips.stop(2,q=away)
            self.assertEqual(clips.active,'probe')
            clips.stop(3,q=q)
            self.assertIsNone(clips.active)
            clips.start('happy',4,m.clip_anchor('happy',pose,q))
            clips.stop(5,reason='fault')  # Fault exit never requires a return move.
            journal.close()
            exported=m.export_clips(folder)
            self.assertEqual(exported[0]['anchor']['idle_pose'],pose)
            self.assertEqual(exported[1]['end_reason'],'fault')

    def test_interactive_pose_and_recording_with_fake_hardware(self):
        # Real command loop/storage, synthetic backend only. Exercises capture,
        # async save, start/stop anchors, persistence and compensation continuity.
        commands=queue.Queue();events=[]
        class Backend(m.MockBackend):
            ticks=0
            def read(self):
                stamp=m.time.monotonic_ns()
                return np.array([0.,.7,1.1,0.,0.,0.,0.]),stamp,stamp
            def send(self,command):
                super().send(command);self.ticks+=1
                script={150:'pose idle',165:'start probe_1',175:'stop',560:'quit'}
                if self.ticks in script:commands.put(script[self.ticks])
            def disable(self):
                events.append(('disable',self.ticks));return super().disable()
        with tempfile.TemporaryDirectory() as tmp,patch('sys.stdout',io.StringIO()):
            folder=Path(tmp);cal=folder/'cal.json';cal.write_text('{}')
            meta=self.pose_metadata();meta['calibration_sha256']=m.digest(cal)
            args=types.SimpleNamespace(mode='record',supported=True,seconds=0.,output=folder,
                                       name='unused',calibration=cal)
            def zero_factory(*_):
                return types.SimpleNamespace(run=lambda cancel,**kw: events.append(('zero',560)) or True)
            result=m.run(args,Gravity(),meta,backend_factory=Backend,commands=commands,zero_return_factory=zero_factory)
            self.assertTrue(result['success'],result)
            self.assertEqual(events,[('zero',560),('disable',560)])
            self.assertTrue(result['zero_return_completed'])
            self.assertEqual(len(result['clips']),1)
            self.assertTrue(result['clips'][0]['complete'])
            self.assertEqual(result['clips'][0]['anchor']['idle_pose']['name'],'ATTENTIVE')
            self.assertEqual(m.load_idle_pose(folder/'presets/idle.json',meta)['q_rad'][0],0.)

    def test_observed_hand_drag_keeps_compensation_without_warning(self):
        # Final measured wrist samples from 20261031T092732Z. Old 0.7 rad/s
        # single-sample guard rejected the last (~48.46 deg/s) hand-guided step.
        rows = [(194004770450318, .01994176022708416),
                (194004789915195, .007784986402839422),
                (194004810362332, -.009510660544037819)]
        control=m.Control(Gravity(),np.zeros(7))
        for stamp,wrist in rows:
            q=np.zeros(7);q[3]=wrist
            command=control.step(q,stamp,1.)
        self.assertNotIn('speed_warning_motor_ids',command)
        self.assertAlmostEqual(abs(np.degrees(command['dq_raw_rad_s'][3])),48.4643,places=2)
        self.assertEqual(command['gravity_nm'][3],.5)
        self.assertGreater(command['feedforward_nm'][3],.5)
        self.assertEqual(command['q_rad'][3],rows[-1][1])

    def test_hand_guiding_has_no_speed_threshold_or_clipping(self):
        for degrees in (40.,90.1,180.,360.):
            control=m.Control(Gravity(),np.zeros(7))
            control.step(np.zeros(7),1_000_000_000,1.)
            q=np.zeros(7);q[3]=np.radians(degrees)*.02
            command=control.step(q,1_020_000_000,1.)
            self.assertAlmostEqual(np.degrees(command['dq_raw_rad_s'][3]),degrees)
            self.assertEqual(command['q_rad'][3],q[3])
            self.assertNotIn('speed_warning_motor_ids',command)

    def test_asynchronous_status_position_has_no_hidden_speed_gate(self):
        backend=object.__new__(m.LiveBackend);backend.mode=2
        q=np.zeros(7);q[3]=.04
        backend.hw=types.SimpleNamespace(positions=lambda:q,check_status=lambda mode:None)
        backend.monitor=types.SimpleNamespace(drain=lambda:None,faults=[],
            positions={i:dict(received_unix_ns=1_015_000_000,position_rad=q[i-1]) for i in range(1,8)},
            status={i:dict(received_unix_ns=1_005_000_000,position_rad=0.,mode=2,
                           fault_bits=0,temperature_c=30.) for i in range(1,8)})
        with patch.object(m.time,'time_ns',side_effect=[1_000_000_000,1_020_000_000]):
            np.testing.assert_array_equal(backend.read()[0],q)
        backend.monitor.positions[4]['position_rad']=.05
        with patch.object(m.time,'time_ns',side_effect=[1_000_000_000,1_020_000_000]):
            with self.assertRaisesRegex(RuntimeError,'SDK/raw position mismatch'):
                backend.read()

    def test_control_uses_same_feedback_tolerance_as_startup(self):
        gravity=Gravity()
        gravity.lower=np.zeros(6)
        gravity.upper=np.ones(6)*3.
        for joint in range(6):
            for edge,direction in ((0.,-1),(3.,1)):
                for excess,accepted in ((.1098635928,True),(.2001958861416885,True),(2.9999,True),(3.0001,False)):
                    q=np.zeros(7);q[joint]=edge+direction*np.radians(excess)
                    control=m.Control(gravity,q)
                    if accepted:
                        command=control.step(q,1,1.)
                        np.testing.assert_array_equal(command['q_rad'],q)
                    else:
                        with self.assertRaisesRegex(ValueError,'outside model bounds'):
                            control.step(q,1,1.)

    def test_gravity_and_damping_applied_once(self):
        c = m.Control(Gravity(), np.zeros(7))
        first = c.step(np.zeros(7), 1_000_000_000, 1.)
        np.testing.assert_allclose(first['feedforward_nm'], [0, 2, 3, .5, 0, 0, 0])
        q = np.zeros(7)
        q[1] = .01
        second = c.step(q, 1_020_000_000, 1.)
        self.assertEqual(second['kp'][:6], [0.] * 6)
        self.assertEqual(second['kd'], [0.] * 7)
        self.assertAlmostEqual(second['dq_est_rad_s'][1], 1 / 6)
        self.assertAlmostEqual(second['feedforward_nm'][1], 2 - .2 / 6)
        self.assertEqual(second['target_rad'][6], 0.)

    def test_invalid_feedback_and_torque(self):
        for q, timestamp in [(np.zeros(7), 1_101_000_000), (np.zeros(7), 1_000_000_000),
                             (np.ones(7), 1_020_000_000), (np.full(7, np.nan), 1_020_000_000)]:
            c = m.Control(Gravity(), np.zeros(7))
            c.step(np.zeros(7), 1_000_000_000, 1.)
            with self.assertRaises(ValueError):
                c.step(q, timestamp, 1.)
        g = Gravity()
        with patch.object(Gravity, '__call__', return_value=np.ones(6) * 13):
            with self.assertRaisesRegex(ValueError, 'gravity exceeds'):
                m.Control(g, np.zeros(7)).step(np.zeros(7), 0, 0.)

    def test_gripper_never_commanded_to_zero(self):
        q = np.zeros(7)
        q[6] = .03
        c = m.Control(Gravity(), q)
        self.assertEqual(c.step(q, 1, 1.)['target_rad'][6], .03)
        moved = q.copy()
        moved[6] += .04
        with self.assertRaisesRegex(ValueError, 'gripper moved'):
            c.step(moved, 20_000_001, 1.)

    def test_journal_multiple_clips_and_incomplete_marking(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp)
            j = m.Journal(folder / 'session.jsonl')
            clips = m.Clips(j)
            clips.start('idle', 100)
            with self.assertRaises(ValueError):
                clips.start('../bad', 100)
            for stamp in [110, 130]:
                j.put(dict(event='sample', clip=clips.active, monotonic_ns=stamp,
                           read_start_ns=stamp - 5, read_end_ns=stamp, q_rad=[.1] * 7,
                           dq_est_rad_s=[0.] * 7, feedforward_nm=[0.] * 7))
            clips.stop(140)
            with self.assertRaises(ValueError):
                clips.start('idle', 150)
            clips.start('rest', 150)
            clips.stop(160, 'fault')
            j.close()
            reports = m.export_clips(folder)
            self.assertTrue(reports[0]['complete'])
            self.assertFalse(reports[1]['complete'])
            self.assertEqual(reports[0]['samples'], 2)
            self.assertIn('q7_rad', (folder / 'idle.csv').read_text())
            self.assertEqual(reports[0]['sha256'], m.digest(folder / 'idle.csv'))

    def test_no_hardware_without_explicit_record_mode_and_support(self):
        with patch.object(m, 'LiveBackend') as backend, patch.object(m, 'preflight') as preflight:
            with patch('sys.stderr', io.StringIO()):
                self.assertEqual(m.main(['record']), 2)
            backend.assert_not_called()
            preflight.assert_not_called()
            preflight.return_value = (Gravity(), {})
            with patch('sys.stdout', io.StringIO()):
                self.assertEqual(m.main([]), 0)
            backend.assert_not_called()

    def test_fault_and_partial_enable_disable_before_export(self):
        for fail_enable in [True, False]:
            events = []
            class Backend:
                def read(self):
                    return np.zeros(7), 0, m.time.monotonic_ns()
                def enable(self, q):
                    events.append('enable')
                    if fail_enable:
                        raise RuntimeError('partial enable')
                def heartbeat(self):
                    raise RuntimeError('feedback fault')
                def send(self, command):
                    self.fail('must not send after fault')
                def disable(self):
                    events.append('disable')
                    return True
                def close(self):
                    events.append('close')
            with tempfile.TemporaryDirectory() as tmp:
                args = types.SimpleNamespace(mode='mock', supported=False, seconds=.1,
                                             output=Path(tmp), name='test')
                actual_export = m.export_clips
                def export(folder):
                    events.append('export')
                    return actual_export(folder)
                with patch.object(m, 'export_clips', side_effect=export):
                    r = m.run(args, Gravity(), {}, backend_factory=Backend, commands=queue.Queue())
                self.assertFalse(r['success'])
                self.assertTrue(r['disable_confirmed'])
                self.assertEqual(events, ['enable', 'disable', 'close', 'export'])

    def test_stop_keeps_compensation_running_until_quit(self):
        events = []
        commands = queue.Queue()
        class Backend(m.MockBackend):
            def read(self):
                n = m.time.monotonic_ns()
                return np.zeros(7), n, n
            def send(self, command):
                super().send(command)
                events.append('send')
            def disable(self):
                events.append('disable')
                return super().disable()
        commands.put('stop')
        commands.put('status')
        commands.put('quit')
        with tempfile.TemporaryDirectory() as tmp, patch('sys.stdout', io.StringIO()):
            args = types.SimpleNamespace(mode='mock', supported=False, seconds=1., output=Path(tmp), name='test')
            r = m.run(args, Gravity(), {}, backend_factory=Backend, commands=commands)
        self.assertTrue(r['success'])
        self.assertEqual(events, ['send', 'send', 'send', 'disable'])

    def test_failed_disable_is_not_success(self):
        class Backend(m.MockBackend):
            def read(self):
                n = m.time.monotonic_ns()
                return np.zeros(7), n, n
            def disable(self):
                return False
        commands = queue.Queue()
        commands.put('quit')
        with tempfile.TemporaryDirectory() as tmp:
            args = types.SimpleNamespace(mode='mock', supported=False, seconds=1., output=Path(tmp), name='test')
            r = m.run(args, Gravity(), {}, backend_factory=Backend, commands=commands)
        self.assertFalse(r['success'])
        self.assertFalse(r['disable_confirmed'])
        self.assertIn('disable not confirmed', r['errors'][0])

    def test_live_feedback_rejects_stale_raw_reply(self):
        backend = object.__new__(m.LiveBackend)
        backend.mode = 2
        backend.hw = types.SimpleNamespace(positions=lambda: np.zeros(7), check_status=lambda mode: None)
        backend.monitor = types.SimpleNamespace(drain=lambda: None, faults=[], status={},
            positions={1: dict(received_unix_ns=0, position_rad=0)})
        with self.assertRaisesRegex(RuntimeError, 'fresh raw position'):
            backend.read()

    @unittest.skipUnless(importlib.util.find_spec('pinocchio') and
                         importlib.util.find_spec('lerobot_teleoperator_rebot_vr') and m.DEFAULT_URDF.exists(),
                         'local VR package and Pinocchio model unavailable')
    def test_gravity_matches_potential_energy_gradient(self):
        gravity = m.Gravity(m.DEFAULT_URDF)
        pin = gravity.pin
        for q in (np.array([0., .7, 1.1, 0., 0., 0.]), np.array([.2, 1.2, .8, .1, -.2, .3])):
            numerical = []
            for i in range(6):
                plus, minus = q.copy(), q.copy()
                plus[i] += 1e-6
                minus[i] -= 1e-6
                numerical.append((pin.computePotentialEnergy(gravity.model, gravity.data, plus) -
                                  pin.computePotentialEnergy(gravity.model, gravity.data, minus)) / 2e-6)
            np.testing.assert_allclose(gravity(q), numerical, atol=1e-7)

    def test_calibration_identity_rejected_before_model_loading(self):
        with tempfile.TemporaryDirectory() as tmp:
            cal = Path(tmp) / 'cal.json'
            urdf = Path(tmp) / 'model.urdf'
            urdf.write_text('changed')
            data = dict(status='complete', robot_type='rebot_b601_rs_follower', zeroed_joints=m.NAMES,
                        motors={name: dict(id=i, host_id=253, model='rs-06' if i <= 3 else 'rs-00')
                                for i, name in enumerate(m.NAMES, 1)}, model_sha256='wrong')
            cal.write_text(json.dumps(data))
            with patch.object(m, 'Gravity') as gravity, self.assertRaisesRegex(ValueError, 'identity'):
                m.preflight(cal, urdf)
            gravity.assert_not_called()


if __name__ == '__main__':
    unittest.main()
