"""Diagnostic must retain bad SDK values without enabling or commanding motors."""
from concurrent.futures import Future
from collections import deque
import io
import json
from pathlib import Path
import sys
import struct
import tempfile
import time
import types
import unittest
from unittest.mock import patch

sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'tools'))
import rs_feedback_audit as m


def submit(fn,*args):
    f=Future()
    try:f.set_result(fn(*args))
    except Exception as error:f.set_exception(error)
    return f


class Tests(unittest.TestCase):
    def test_latest_malformed_position_matches_rejected_sdk_value(self):
        # 20261031T104937Z: SDK accepted these trailing bytes as float32.
        frame=bytes.fromhex('fd05009108000000197000098013012c')
        row=m.decode(frame)
        self.assertEqual(row['kind'],'invalid_position')
        self.assertEqual(row['reason'],'invalid_position_payload')
        self.assertEqual(row['motor'],5)
        self.assertEqual(struct.unpack('<f',frame[-4:])[0],1.834282725710068e-12)
        self.assertNotIn('position_rad',row)

    def test_incident_unmatched_register_is_not_position_failure(self):
        # Exact raw frame from the 2026-10-31 18:07:42 CST session.
        frame=bytes.fromhex('fd058091080000007f7480027fb4012c')
        row=m.decode(frame)
        self.assertEqual(row['kind'],'other_parameter')
        self.assertEqual(row['motor'],5)
        self.assertEqual(row['register'],0x747f)
        self.assertEqual(row['read_status'],128)
        self.assertEqual(row['data'],'7f7480027fb4012c')
        self.assertNotIn('position_rad',row)

    def test_unmatched_frame_cannot_supply_missing_position(self):
        monitor=types.SimpleNamespace(positions={},faults=[],drain=lambda:None)
        def send(packet):
            row=m.decode(bytes.fromhex('fd058091080000007f7480027fb4012c'))
            if row['kind']=='position':
                monitor.positions[row['motor']]=dict(received_unix_ns=time.time_ns(),**row)
            return len(packet)
        monitor.sock=types.SimpleNamespace(send=send)
        values,errors=m.read_raw_batch(monitor,timeout=0)
        self.assertEqual(values,[None]*7)
        self.assertEqual(len(errors),7)

    def test_monitor_keeps_fresh_position_and_logs_unmatched_frame(self):
        good=bytes.fromhex('fd0500910800000019700000dacf5abd')
        unrelated=bytes.fromhex('fd058091080000007f7480027fb4012c')
        packets=deque([good,unrelated])
        stamp=time.time_ns()
        def recvmsg(*args):
            if not packets:raise BlockingIOError()
            anc=[(m.socket.SOL_SOCKET,m.SO_TIMESTAMPNS_NEW,
                  struct.pack('=qq',stamp//1000000000,stamp%1000000000))]
            return packets.popleft(),anc,0,None
        monitor=object.__new__(m.Monitor)
        monitor.sock=types.SimpleNamespace(recvmsg=recvmsg)
        monitor.log=io.StringIO();monitor.positions={};monitor.status={}
        monitor.frames=0;monitor.faults=[];monitor.accepted_modes={2}
        monitor.drain()
        self.assertEqual(monitor.faults,[])
        self.assertAlmostEqual(monitor.positions[5]['position_rad'],-.05342087894678116)
        self.assertEqual(monitor.frames,2)
        logged=[json.loads(line) for line in monitor.log.getvalue().splitlines()]
        self.assertEqual(logged[1]['packet'],unrelated.hex())
        # A failed reply to the actual position register still latches a fault.
        packets.append(m.FRAME.pack(0x80000000|(17<<24)|(1<<16)|(5<<8)|0xFD,
                                   8,struct.pack('<H2xf',0x7019,0.)))
        monitor.drain()
        self.assertEqual(monitor.faults[0]['reason'],'parameter_read_failed')
        self.assertAlmostEqual(monitor.positions[5]['position_rad'],-.05342087894678116)

    def test_failed_parameter_payload_never_becomes_position(self):
        payload=struct.pack('<H2xf',0x7019,1e26)
        frame=m.FRAME.pack(0x80000000|(17<<24)|(1<<16)|(4<<8)|0xFD,8,payload)
        row=m.decode(frame)
        self.assertEqual(row['kind'],'invalid_position')
        self.assertEqual(row['reason'],'parameter_read_failed')
        self.assertNotIn('position_rad',row)
        good=m.FRAME.pack(0x80000000|(17<<24)|(4<<8)|0xFD,8,struct.pack('<H2xf',0x7019,-.643))
        self.assertAlmostEqual(m.decode(good)['position_rad'],-.643,places=6)

    def test_raw_reader_sends_only_position_queries(self):
        calls=[]
        monitor=types.SimpleNamespace(positions={},faults=[],drain=lambda:None)
        def send(packet):
            ident,dlc,data=m.FRAME.unpack(packet);mid=ident&255
            self.assertEqual((ident>>24)&31,17)
            self.assertEqual((ident>>8)&65535,0xFD)
            self.assertEqual(data,struct.pack('<H6x',0x7019))
            calls.append(mid)
            monitor.positions[mid]=dict(received_unix_ns=time.time_ns(),position_rad=.1*mid)
            return len(packet)
        monitor.sock=types.SimpleNamespace(send=send)
        values,errors=m.read_raw_batch(monitor)
        self.assertEqual(calls,list(range(1,8)));self.assertEqual(errors,[])
        self.assertEqual(values,[.1*i for i in range(1,8)])

    def test_raw_reader_rejects_stale_replies(self):
        monitor=types.SimpleNamespace(positions={i:dict(received_unix_ns=0,position_rad=.1) for i in range(1,8)},
                                     faults=[],drain=lambda:None,sock=types.SimpleNamespace(send=len))
        values,errors=m.read_raw_batch(monitor,timeout=0)
        self.assertEqual(values,[None]*7);self.assertEqual(len(errors),7)

    def test_both_read_modes_preserve_bad_sdk_value(self):
        for mode in ('parallel','serial'):
            calls=[]
            def read(i,*args):
                calls.append((i,args))
                return 1e26 if i==4 else .1
            motors=[types.SimpleNamespace(robstride_get_param_f32_host_id=lambda *a,i=i:read(i,*a)) for i in range(1,8)]
            hw=types.SimpleNamespace(motors=motors,reader=types.SimpleNamespace(submit=submit))
            values,errors=m.read_sdk_batch(hw,mode)
            self.assertEqual(values[3],1e26);self.assertEqual(errors,[])
            self.assertEqual(len(calls),7)
            self.assertTrue(all(args==(0x7019,0xFD,30) for _,args in calls))

    def test_errors_are_recordable_not_fabricated_zero(self):
        motors=[types.SimpleNamespace(robstride_get_param_f32_host_id=lambda *args:float('nan'))]*7
        values,errors=m.read_sdk_batch(types.SimpleNamespace(motors=motors),'serial')
        self.assertEqual(values,[None]*7);self.assertEqual(len(errors),7)
        json.dumps(dict(values=values,errors=errors),allow_nan=False)

    def test_anomaly_is_saved_before_exit_without_motor_writes(self):
        calls=[]
        class HW:
            driver_version='test'
            def __init__(self,port):calls.append('open')
            def reports(self):calls.append('subscribe')
            def check_status(self,mode):self.assert_disabled=mode;calls.append(('status',mode))
            def close(self):calls.append('close')
        class Monitor:
            def __init__(self,port,log):
                self.positions={};self.status={};self.frames=0;self.faults=[]
            def drain(self):
                now=time.time_ns();self.frames+=1
                self.positions={i:dict(received_unix_ns=now,position_rad=.1) for i in range(1,8)}
                self.status={i:dict(received_unix_ns=now,position_rad=.1,mode=0,fault_bits=0) for i in range(1,8)}
            def close(self):calls.append('monitor_close')
        with tempfile.TemporaryDirectory() as tmp,patch.object(m,'ROOT',Path(tmp)),patch.object(m,'Hardware',HW),patch.object(m,'Monitor',Monitor),patch.object(m,'read_sdk_batch',return_value=([.1,.1,.1,1e26,.1,.1,.1],[])):
            report=m.audit(1,'fake','parallel')
            self.assertFalse(report['success']);self.assertFalse(report['enable_requested'])
            self.assertFalse(report['motion_requested']);self.assertEqual(report['sdk_raw_mismatch_count'],1)
            rows=[json.loads(x) for x in Path(report['raw_log']).read_text().splitlines()]
            sample=next(x for x in rows if x.get('event')=='sdk_raw_sample')
            self.assertEqual(sample['sdk_rad'][3],1e26)
            self.assertEqual(sample['raw_positions']['4']['position_rad'],.1)
            self.assertEqual(calls[-2:],['close','monitor_close'])


if __name__=='__main__':unittest.main()
