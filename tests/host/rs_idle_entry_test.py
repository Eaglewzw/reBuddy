"""Automatic idle entry and gravity handoff, with simulated feedback only."""
import io
import json
from pathlib import Path
import queue
import signal
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'tools'))
import rs_gravity_record as record
from rs_idle_entry import IdleEntry


class Gravity:
    lower=np.full(6,-3.);upper=np.full(6,3.)
    def __call__(self,q):return np.array([0.,2.,3.,.5,0.,0.])


class Tests(unittest.TestCase):
    def test_observed_static_zero_offset_passes_zero_but_not_idle_arrival(self):
        # 20261101T075452Z: J3 1.82 deg, J4 1.16 deg were stationary.
        initial=np.radians([.18,.12,1.82,1.16,-.14,-.12,.07])
        target=np.radians([-1.64,44.9,66.68,-28.26,-3.08,-.7,.06])
        entry=IdleEntry(Gravity(),initial,{'q_rad':target.tolist()},via_zero=True)
        for i in range(700):
            entry.step(initial,1_000_000_000+i*20_000_000)
            if entry.zero_completed:break
        self.assertTrue(entry.zero_completed)
        self.assertFalse(entry.done)
        # The same offset is still rejected at the final idle, with diagnostics.
        entry.phase='settle';entry.phase_start=20_000_000_000
        entry.stable_since=None
        q=target.copy();q[2]+=np.radians(1.82)
        from rs_gravity_play import Controller
        entry.position=Controller(Gravity(),initial=q)
        with self.assertRaisesRegex(RuntimeError,'自动待机.*J3=\\+1.820°.*±1°'):
            entry.step(q,25_000_000_000)

    def test_zero_failure_reports_stage_and_actual_axis_error(self):
        q=np.radians([0,0,2.1,1.16,0,0,0])
        entry=IdleEntry(Gravity(),q,{'q_rad':np.zeros(7).tolist()},via_zero=True)
        with self.assertRaisesRegex(RuntimeError,'启动回零.*J3=\\+2.100°.*±2°'):
            for i in range(700):entry.step(q,1_000_000_000+i*20_000_000)
        self.assertFalse(entry.zero_completed)
        self.assertFalse(entry.done)

    def test_startup_zero_precedes_idle_even_with_negative_zero_reading(self):
        initial=np.radians([5.,-.112,8.,-4.,1.,2.,.06])
        target=np.radians([-1.64,44.90,66.69,-28.26,-3.08,-.69,.06])
        entry=IdleEntry(Gravity(),initial,{'q_rad':target.tolist()},via_zero=True)
        q=initial.copy();zero_stamp=None
        for i in range(6000):
            command,phase=entry.step(q,1_000_000_000+i*20_000_000)
            if i==0:np.testing.assert_allclose(command['target_rad'],initial)
            if entry.zero_completed and zero_stamp is None:
                zero_stamp=i
                self.assertLessEqual(np.max(abs(q[:6])),np.radians(1.))
                self.assertFalse(entry.done)
            q=np.array(command['target_rad'])
            self.assertGreaterEqual(q[1],initial[1]-1e-10)
            if entry.done:break
        self.assertIsNotNone(zero_stamp)
        self.assertTrue(entry.done)
        np.testing.assert_allclose(q,target,atol=1e-9)

    def test_full_size_entry_preserves_compensation_through_release(self):
        target=np.radians([-1.64,44.90,66.69,-28.26,-3.08,-.69,.06])
        entry=IdleEntry(Gravity(),np.zeros(7),{'q_rad':target.tolist()})
        q=np.zeros(7);release=[];phases=set()
        for i in range(10000):
            command,phase=entry.step(q,1_000_000_000+i*20_000_000)
            phases.add(phase)
            q=np.array(command['target_rad'])
            if phase=='release':release.append(command)
            if entry.done:break
        self.assertTrue(entry.done)
        self.assertEqual(phases,{'hold','approach','settle','release'})
        np.testing.assert_allclose(q,target,atol=1e-10)
        kp=np.array([c['kp'] for c in release])
        self.assertTrue(np.all(np.diff(kp[:,:6],axis=0)<=1e-9))
        np.testing.assert_array_equal(kp[-1,:6],np.zeros(6))
        self.assertEqual(kp[-1,6],5.)
        for command in release:
            np.testing.assert_allclose(command['feedforward_nm'][:6],Gravity()(q[:6]),atol=.01)

    def simulate(self,case):
        clock=[10.];events=[];commands=queue.Queue();handlers={};entry_ref=[];sent=[]
        target=np.array([.05,.1,.12,-.05,.01,.02,.01])
        def handler(sig,fn):
            old=handlers.get(sig,signal.SIG_DFL);handlers[sig]=fn;return old
        def factory(gravity,initial,pose,**kw):
            entry=IdleEntry(gravity,initial,pose,**kw);entry_ref.append(entry);return entry
        class Backend:
            q=np.zeros(7);enabled=False;after_ready=0;stopped=False;return_ticks=0
            def read(self):
                if case=='return_fault' and len(entry_ref)>1 and entry_ref[-1].phase=='approach':
                    raise RuntimeError('bad return feedback')
                if self.enabled and entry_ref and entry_ref[0].phase=='approach':
                    if case=='feedback_fault':raise RuntimeError('bad approach feedback')
                    if case=='temperature':raise record.TemperatureReturnRequested([dict(motor_id=2,temperature_c=100.)])
                stamp=int(clock[0]*1e9);return self.q.copy(),stamp,stamp
            def heartbeat(self):pass
            def enable(self,q):
                events.append('enable');self.enabled=True
                if case=='early_start':commands.put('start too_early')
            def send(self,c):
                sent.append(c);self.q=np.array(c['target_rad'])
                if case.startswith('return'):
                    if len(entry_ref)==1 and entry_ref[0].done:
                        self.after_ready+=1
                        if self.after_ready==1:commands.put('start nuzzle')
                        if 2<=self.after_ready<=30:self.q[1]+=.012
                        if self.after_ready==31:commands.put('stop')
                    elif len(entry_ref)>1:
                        self.return_ticks+=1
                        if self.return_ticks==2:commands.put('stop')
                        if self.return_ticks==3:commands.put('start too_early')
                        if case=='return_cancel' and self.return_ticks==8:
                            handlers[signal.SIGINT](signal.SIGINT,None)
                        if case=='return_not_arrived':self.q[0]-=np.radians(2.)
                        if entry_ref[-1].done:commands.put('quit')
                    return
                if case=='not_arrived':self.q[0]-=np.radians(2.)
                if case=='ctrl_c' and entry_ref[0].phase=='approach' and not self.stopped:
                    self.stopped=True;handlers[signal.SIGINT](signal.SIGINT,None)
                ready=(entry_ref[0].done if entry_ref else clock[0]>11.7)
                if ready:
                    self.after_ready+=1
                    action={1:'start demo',6:'stop',7:'quit'}.get(self.after_ready)
                    if action:commands.put(action)
            def disable(self):events.append('disable');return True
            def close(self):events.append('close')
        def home_factory(*_):
            def home(cancel,**kw):
                events.append('home')
                np.testing.assert_allclose(kw['initial_kp'],sent[-1]['kp'][:6])
                return not cancel()
            return SimpleNamespace(run=home)
        with tempfile.TemporaryDirectory() as tmp, patch.object(record.time,'monotonic',side_effect=lambda:clock[0]), \
                patch.object(record.time,'monotonic_ns',side_effect=lambda:int(clock[0]*1e9)), \
                patch.object(record.time,'sleep',side_effect=lambda dt:clock.__setitem__(0,clock[0]+dt)), \
                patch.object(record.signal,'signal',side_effect=handler),patch('sys.stdout',io.StringIO()):
            folder=Path(tmp);cal=folder/'cal.json';cal.write_text('{}')
            metadata=dict(calibration_sha256=record.digest(cal),urdf_sha256='test',mock=False)
            pose=dict(schema='rebuddy.idle_pose.v1',name='ATTENTIVE',q_rad=target.tolist(),**metadata)
            (folder/'presets').mkdir();(folder/'presets/idle.json').write_text(json.dumps(pose))
            args=SimpleNamespace(mode='record',supported=True,seconds=0.,output=folder,
                                 calibration=cal,manual_start=case=='manual')
            result=record.run(args,Gravity(),metadata,backend_factory=Backend,commands=commands,
                              zero_return_factory=home_factory,idle_entry_factory=factory)
            rows=[json.loads(line) for line in (Path(result['folder'])/'session.jsonl').read_text().splitlines()]
            if case=='return_success':
                import rs_gravity_play
                times,positions,report=rs_gravity_play.load_clip(Path(result['folder'])/'nuzzle.csv',metadata)
                self.assertLess(np.max(abs(positions[-1]-target)),np.radians(1.))
                self.assertGreater(np.max(positions[:,1])-target[1],np.radians(15.))
        return result,events,rows

    def test_stop_appends_continuous_measured_return_and_only_then_finishes_clip(self):
        result,events,rows=self.simulate('return_success')
        self.assertTrue(result['success'],result)
        self.assertEqual([c['name'] for c in result['clips']],['nuzzle'])
        clip=result['clips'][0]
        self.assertTrue(clip['complete'])
        self.assertTrue(clip['auto_return'])
        tail=[r for r in rows if r.get('phase')=='auto_return']
        self.assertGreater(len(tail),100)
        self.assertGreater(tail[0]['monotonic_ns'],clip['manual_end_ns'])
        first_hold=next(r for r in rows if r['event']=='clip_return_command')
        np.testing.assert_allclose(first_hold['feedforward_nm'][:6],Gravity()(np.zeros(6)),atol=.2)
        samples=[r for r in rows if r['event']=='sample' and r.get('clip')=='nuzzle']
        self.assertLessEqual(max(np.diff([r['monotonic_ns'] for r in samples])),100_000_000)
        self.assertEqual(events,['enable','home','disable','close'])

    def test_interrupted_or_failed_return_never_marks_clip_complete(self):
        for case in ('return_cancel','return_fault','return_not_arrived'):
            result,events,rows=self.simulate(case)
            self.assertFalse(result['clips'][0]['complete'],result)
            self.assertNotEqual(result['clips'][0]['end_reason'],'operator_stop')
            self.assertTrue(result['disable_confirmed'])
            self.assertEqual(result['zero_return_completed'],case=='return_cancel')

    def test_automatic_motion_is_excluded_from_clip_and_handoff_is_draggable(self):
        for case in ('success','early_start'):
            result,events,rows=self.simulate(case)
            self.assertTrue(result['success'],result)
            self.assertTrue(result['idle_entry_completed'])
            self.assertTrue(result['startup_zero_completed'])
            self.assertEqual([c['name'] for c in result['clips']],['demo'])
            self.assertTrue(result['clips'][0]['complete'])
            self.assertEqual(events,['enable','home','disable','close'])
            samples=[r for r in rows if r['event']=='sample' and r['clip']=='demo']
            self.assertGreaterEqual(len(samples),2)
            for row in samples:np.testing.assert_array_equal(row['kp'][:6],np.zeros(6))
            phases=[r['event'] for r in rows]
            self.assertLess(phases.index('startup_zero_completed'),phases.index('idle_entry_completed'))
            self.assertLess(phases.index('idle_entry_completed'),phases.index('clip_start'))

    def test_cancel_and_temperature_during_entry_return_from_current_gains(self):
        for case in ('ctrl_c','temperature'):
            result,events,_=self.simulate(case)
            self.assertTrue(result['zero_return_completed'],result)
            self.assertFalse(result['idle_entry_completed'])
            self.assertEqual(result['clips'],[])
            self.assertEqual(events,['enable','home','disable','close'])

    def test_bad_feedback_or_unsettled_entry_never_enables_teaching(self):
        for case in ('feedback_fault','not_arrived'):
            result,events,rows=self.simulate(case)
            self.assertFalse(result['success'])
            self.assertFalse(result['idle_entry_completed'])
            self.assertEqual(result['clips'],[])
            self.assertEqual(events,['enable','disable','close'])

    def test_manual_start_preserves_original_workflow(self):
        result,_,_=self.simulate('manual')
        self.assertTrue(result['success'],result)
        self.assertFalse(result['idle_entry_requested'])


if __name__=='__main__':unittest.main()
