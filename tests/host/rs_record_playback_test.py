"""Host-only recording, interpolation and fail-closed command checks."""
import importlib.util
from pathlib import Path
import tempfile
import unittest
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
spec=importlib.util.spec_from_file_location('playback',ROOT/'tools/rs_record_playback.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)


def recording(points):
    count=len(points)
    extra=bytes([7,0])+(250).to_bytes(4,'little')+bytes(12)
    raw=bytearray(m.HEADER.pack(b'TCH1',2,64,2,1,6,1,127,18,20,0,1,0,count,
        points[-1][0],bytes(range(1,7)),extra,0)+b''.join(m.SAMPLE.pack(*p) for p in points))
    m.struct.pack_into('<I',raw,60,m.zlib.crc32(raw))
    return raw


class Tests(unittest.TestCase):
    def test_watchdog_start_drains_observer_while_waiting_for_child(self):
        from unittest.mock import Mock
        clock=[0.];drains=[]
        def poll(timeout):
            clock[0]+=timeout
            return clock[0]>=.6
        pipe=Mock();pipe.poll.side_effect=poll;pipe.recv.return_value='ready'
        child=Mock();ctx=Mock();ctx.Pipe.return_value=(pipe,child)
        hw=object.__new__(m.Hardware);hw.port='can-test';hw.armed=False
        with patch.object(m.mp,'get_context',return_value=ctx), \
                patch.object(m.time,'monotonic',side_effect=lambda:clock[0]):
            hw.start_guard(progress=lambda:drains.append(clock[0]))
        self.assertTrue(hw.armed)
        self.assertGreaterEqual(len(drains),29)
        self.assertLessEqual(max(np.diff([0.]+drains)),.020001)
        pipe.send.assert_called_once_with('arm')

    def test_enable_services_observer_through_mode_guard_and_enable(self):
        events=[];hw=object.__new__(m.Hardware);hw.states={}
        hw.check_status=lambda mode:events.append(('check',mode))
        hw.positions=lambda:np.zeros(7)
        hw.heartbeat=lambda:None
        hw.drain=lambda:None
        def guard(progress):
            events.append('guard');progress()
        hw.start_guard=guard
        def enabled(mid):
            events.append(('enable',mid));hw.states[mid]=(m.time.monotonic(),2,0)
        hw.motors=[SimpleNamespace(
            ensure_mode=lambda *a,i=i:events.append(('mode',i)),
            send_mit=lambda *a:None,enable=lambda i=i:enabled(i)) for i in range(1,8)]
        hw.enable_supported_trial(np.zeros(7),fall_arrest_confirmed=True,
                                  progress=lambda:events.append('observe'))
        for i,event in enumerate(events):
            if event=='guard' or isinstance(event,tuple) and event[0] in ('mode','enable'):
                self.assertEqual(events[i+1],'observe')

    def test_stop_periodic_reports_uses_sdk_and_preserves_guard_and_status(self):
        for failure in (None,4):
            hw=object.__new__(m.Hardware);events=[]
            hw.report_enabled=True;hw.states={1:'retained'}
            hw.heartbeat=lambda:events.append('heartbeat')
            def report(mid,enabled):
                events.append((mid,enabled))
                if mid==failure:raise RuntimeError('send failed')
            hw.motors=[SimpleNamespace(robstride_set_active_report=lambda enabled,i=i:report(i,enabled))
                       for i in range(1,8)]
            if failure:
                with self.assertRaisesRegex(RuntimeError,'send failed'):hw.stop_reports()
                self.assertTrue(hw.report_enabled)  # close must retry partial failure.
            else:
                hw.stop_reports()
                self.assertFalse(hw.report_enabled)
            count=failure or 7
            self.assertEqual(events,[e for i in range(1,count+1) for e in ('heartbeat',(i,False))])
            self.assertEqual(hw.states,{1:'retained'})

    def test_feedback_tolerance_preserves_measurement_and_target_limits(self):
        for joint in range(7):
            for edge, direction in ((m.LOWER[joint], -1), (m.UPPER[joint], 1)):
                for excess_deg, accepted in ((.1098635928, True), (.2001958861416885, True), (2.9999, True), (3.0001, False)):
                    q=np.zeros(7);q[joint]=edge+direction*np.radians(excess_deg)
                    results={f'joint{i}':(v,1,2) for i,v in enumerate(q)}
                    hw=SimpleNamespace(connection=SimpleNamespace(read_positions=lambda:results),drain=lambda:None)
                    if accepted:
                        np.testing.assert_array_equal(m.Hardware.positions(hw),q)
                    else:
                        with self.assertRaisesRegex(RuntimeError,'measurement_tolerance_deg'):
                            m.Hardware.positions(hw)
        self.assertEqual(m.LOWER[1],0.)
        self.assertEqual(m.UPPER[1],3.14)

    def test_pre_enable_cleanup_confirms_only_fresh_disabled_states(self):
        for state in ('disabled','missing','stale','enabled','faulted'):
            with self.subTest(state=state):
                hw=object.__new__(m.Hardware)
                hw.armed=False;hw.pipe=None;hw.guard=None
                now=m.time.monotonic()
                hw.states={i:(now-(1 if state=='stale' else 0),
                              2 if state=='enabled' else 0,
                              1 if state=='faulted' else 0) for i in range(1,8)}
                if state=='missing':hw.states.pop(7)
                hw.drain=lambda:None
                # No motor command interface is provided: this branch only reads status.
                self.assertEqual(hw.disable(),state=='disabled')

    def test_driver_version_before_hardware(self):
        for version in ('0.5.0','0.5.3rc1','1.0.0'):
            with patch.object(m,'package_version',return_value=version), \
                    patch.object(m.socket,'socket') as socket:
                with self.assertRaisesRegex(RuntimeError,'incompatible'):m.Hardware()
                socket.assert_not_called()
        with patch.object(m,'package_version',return_value='0.5.3'):
            self.assertEqual(m.validate_driver_version(),'0.5.3')

    def test_incident_lock_before_hardware(self):
        import sys
        with patch.object(sys,'argv',['playback','unused.tch','--mode','play','--site-confirmed']), \
                patch.object(m,'Hardware') as hardware:
            with self.assertRaisesRegex(RuntimeError,'observed arm drop'):m.main()
            hardware.assert_not_called()
        with self.assertRaisesRegex(RuntimeError,'observed arm drop'):
            m.Hardware.enable(None,None)

    def test_rejected_feedback_is_logged_without_command(self):
        import io
        class HW:
            motors=[]
            def positions(self):return np.zeros(7)
            def check_status(self,mode):pass
            def heartbeat(self):pass
        log=io.StringIO()
        target=np.zeros(7);target[2]=np.radians(7.54)
        with self.assertRaisesRegex(RuntimeError,'following error'):
            m.run_segment(HW(),lambda t:(target,np.zeros(7)),0,
                lambda q:(_ for _ in ()).throw(AssertionError('advanced after fault')),
                np.ones(7),np.ones(7),np.ones(6),log,'play')
        sample=m.json.loads(log.getvalue())
        self.assertEqual(sample['event'],'feedback')
        self.assertAlmostEqual(sample['joint_error_deg'][2],7.54)
        self.assertEqual(sample['q'],[0]*7)

    def test_codec(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'test.tch';p.write_bytes(recording([(0,0,0,0,0,0,0,0),(20,0,10,20,0,0,0,0)]))
            t,q,meta=m.load_record(p)
            self.assertEqual(meta['samples'],2);self.assertEqual(q[1,2],.002)
            b=bytearray(p.read_bytes());b[-1]^=1;p.write_bytes(b)
            with self.assertRaisesRegex(ValueError,'CRC'):m.load_record(p)
            for points in [[(0,0,0,0,0,0,0,0),(101,0,0,0,0,0,0,0)],
                           [(0,0,0,0,0,0,0,0),(20,0,-100,0,0,0,0,0)],
                           [(0,0,0,0,0,0,0,0),(20,0,0,0,0,0,0,1)]]:
                p.write_bytes(recording(points))
                with self.assertRaises(ValueError):m.load_record(p)

    def test_retime(self):
        t=np.array([0,.03,.06,.08,.12]);q=np.zeros((5,7));q[:,1]=[0,.08,.1,.02,0]
        plan=m.Plan(t,q,.5)
        ts=np.linspace(0,plan.duration,5000)
        pos,vel=plan.at(ts)
        self.assertLessEqual(np.max(np.abs(vel)),.3+1e-9)
        accel=plan.curve(np.minimum(ts*plan.speed,t[-1]),2)*plan.speed**2
        self.assertLessEqual(np.max(np.abs(accel)),.8+1e-9)
        np.testing.assert_allclose(plan.at(0)[1],0,atol=1e-12)
        np.testing.assert_allclose(plan.at(plan.duration)[1],0,atol=1e-12)
        self.assertGreaterEqual(np.min(pos[:,1]),-1e-9)
        self.assertLessEqual(np.max(pos[:,1]),.1+1e-9)
        for bad in [0,-1,1,float('nan')]:
            with self.assertRaises(ValueError):m.Plan(t,q,bad)

    def test_bridge(self):
        start=np.zeros(7);end=np.ones(7)*.1
        duration,curve=m.bridge(start,end)
        np.testing.assert_allclose(curve(0)[0],start)
        np.testing.assert_allclose(curve(duration)[0],end)
        np.testing.assert_allclose(curve(duration)[1],0)
        with self.assertRaises(ValueError):m.bridge(start,np.ones(7))

    def test_status(self):
        packet=m.FRAME.pack(0x80000000|(2<<24)|((2<<14|3)<<8)|0xFD,8,bytes(8))
        self.assertEqual(m.parse_status(packet),(3,2,0))
        self.assertIsNone(m.parse_status(packet[:8]))
        self.assertIsNone(m.parse_status(m.FRAME.pack(0x123,8,bytes(8))))
        fault=m.FRAME.pack(0x80000000|(24<<24)|((2<<14|1<<9|7)<<8)|0xFD,8,bytes(8))
        self.assertEqual(m.parse_status(fault),(7,2,2))

    def test_fault_prevents_targets(self):
        class HW:
            def positions(self):return np.zeros(7)
            def check_status(self,mode):raise RuntimeError('fault')
            def heartbeat(self):raise AssertionError('advanced after fault')
        import io
        with self.assertRaisesRegex(RuntimeError,'fault'):
            m.run_segment(HW(),lambda t:(np.zeros(7),np.zeros(7)),0,
                lambda q:np.zeros(6),np.ones(7),np.ones(7),np.ones(6),io.StringIO(),'test')

    def test_feedback_error_never_returns_zero(self):
        hw=m.Hardware.__new__(m.Hardware)
        hw.connection=SimpleNamespace(read_positions=lambda:{str(i):RuntimeError('offline') for i in range(7)})
        with self.assertRaisesRegex(RuntimeError,'feedback read failed'):hw.positions()
        self.assertEqual(hw.last_read['sdk_rad'],[None]*7)
        self.assertEqual(len(hw.last_read['errors']),7)

    def test_rejected_sdk_value_is_available_for_failure_journal(self):
        hw=m.Hardware.__new__(m.Hardware)
        hw.connection=SimpleNamespace(read_positions=lambda:{str(i):(1e26 if i==4 else 0.,1,2) for i in range(7)})
        with self.assertRaisesRegex(RuntimeError,'outside model limits'):hw.positions()
        self.assertEqual(hw.last_read['sdk_rad'][4],1e26)

    @unittest.skipUnless(importlib.util.find_spec('lerobot_teleoperator_rebot_vr'), 'VR reference package unavailable')
    def test_transport_open_and_cleanup_delegate_to_vr_connection(self):
        from lerobot_teleoperator_rebot_vr.hardware import rs
        with patch.object(rs,'RSConnection') as connection, patch.object(m.socket,'socket') as socket, \
                patch.object(m,'validate_driver_version',return_value='0.5.3'):
            instance=connection.return_value
            instance.motors={str(i):SimpleNamespace() for i in range(7)}
            hw=m.Hardware('can-test')
            connection.assert_called_once_with('can-test')
            instance.open.assert_called_once()
            self.assertIs(hw.bus,instance.bus)
            self.assertEqual(hw.motors,list(instance.motors.values()))
            hw.close()
            instance.close.assert_called_once()
            socket.return_value.close.assert_called_once()

    def test_guard_disable_packet(self):
        class Pipe:
            def __init__(self):self.calls=0
            def send(self,x):self.ready=x
            def poll(self,t):self.calls+=1;return self.calls==1
            def recv(self):return 'arm'
            def close(self):pass
        class Bus:
            def __init__(self):self.sent=[]
            def settimeout(self,t):pass
            def bind(self,p):pass
            def send(self,p):self.sent.append(p)
            def close(self):pass
        bus=Bus();pipe=Pipe()
        with patch.object(m.socket,'socket',return_value=bus),patch.object(m.time,'sleep'), \
                patch.object(m.signal,'signal') as handler:
            m.watchdog(pipe,'can0')
            handler.assert_any_call(m.signal.SIGINT,m.signal.SIG_IGN)
            handler.assert_any_call(m.signal.SIGTERM,m.signal.SIG_IGN)
        self.assertEqual(pipe.ready,'ready');self.assertEqual(len(bus.sent),21)
        for i,p in enumerate(bus.sent):
            ident,n,data=m.FRAME.unpack(p)
            self.assertEqual(ident,0x80000000|(4<<24)|(0xFD<<8)|(i%7+1))
            self.assertEqual((n,data),(8,bytes(8)))

if __name__=='__main__':unittest.main()
