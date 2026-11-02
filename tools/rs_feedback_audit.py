#!/usr/bin/env python3
"""Read-only SDK/raw CAN comparison. Never enables, changes mode, or sends targets."""
from __future__ import annotations
import argparse
import json
import math
from pathlib import Path
import select
import socket
import struct
import time
from rs_record_playback import Hardware, FRAME, ROOT, LOWER, UPPER, MEASUREMENT_TOLERANCE_RAD

# Linux asm-generic/socket.h: new ABI uses two signed 64-bit timestamp fields.
SO_TIMESTAMPNS_NEW=64


def decode(packet):
    if len(packet)!=FRAME.size:return None
    ident,dlc,data=FRAME.unpack(packet)
    if not ident&0x80000000 or ident&0x60000000 or dlc!=8:return None
    ident &= 0x1fffffff
    kind=(ident>>24)&31;mid=(ident>>8)&255
    if ident&255!=0xFD or not 1<=mid<=7:return None
    if kind==17:
        register=struct.unpack_from('<H',data)[0]
        read_status=(ident>>16)&255
        # Match the echoed register BEFORE classifying a position failure.
        # An unrelated/malformed parameter frame cannot refresh mechPos, but it
        # also cannot invalidate a separate, fresh successful 0x7019 reply.
        # Keep the complete payload/status in the raw journal for diagnosis.
        if register!=0x7019:
            return dict(kind='other_parameter',motor=mid,register=register,
                        read_status=read_status,data=data.hex())
        if read_status:
            return dict(kind='invalid_position',motor=mid,register=register,
                        read_status=read_status,reason='parameter_read_failed',data=data.hex())
        value=struct.unpack_from('<f',data,4)[0]
        if data[2:4]!=bytes(2) or not math.isfinite(value):
            return dict(kind='invalid_position',motor=mid,register=register,
                        reason='invalid_position_payload',data=data.hex())
        return dict(kind='position',motor=mid,position_rad=value)
    if kind in (2,24):
        p,v,torque,temp=struct.unpack('>4H',data)
        return dict(kind='status',motor=mid,mode=(ident>>22)&3,fault_bits=(ident>>16)&63,
            position_rad=(p/32767.-1)*4*math.pi,
            velocity_rad_s=(v/32767.-1)*(50 if mid<=3 else 33),
            torque_nm=(torque/32767.-1)*(36 if mid<=3 else 14),temperature_c=temp/10)
    if kind==21:
        fault,warning=struct.unpack('<II',data)
        return dict(kind='fault',motor=mid,fault_raw=fault,warning_raw=warning)
    return None


class Monitor:
    def __init__(self,port,log):
        self.sock=socket.socket(socket.PF_CAN,socket.SOCK_RAW,socket.CAN_RAW)
        try:
            self.sock.setsockopt(socket.SOL_SOCKET,SO_TIMESTAMPNS_NEW,1)
            self.sock.setsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF,1024*1024)
            self.sock.bind((port,));self.sock.setblocking(False)
        except BaseException:
            self.sock.close();raise
        self.log=log;self.positions={};self.status={};self.frames=0;self.faults=[]
        self.accepted_modes={0}

    def drain(self):
        while True:
            try:packet,anc,flags,_=self.sock.recvmsg(16,128)
            except BlockingIOError:break
            if flags&(socket.MSG_TRUNC|socket.MSG_CTRUNC):raise RuntimeError('CAN/timestamp truncated')
            stamp=None
            for level,kind,payload in anc:
                if level==socket.SOL_SOCKET and kind==SO_TIMESTAMPNS_NEW and len(payload)>=16:
                    sec,nsec=struct.unpack_from('=qq',payload);stamp=sec*1000000000+nsec
            if stamp is None:raise RuntimeError('kernel CAN receive timestamp missing')
            decoded=decode(packet);self.frames+=1
            self.log.write(json.dumps(dict(received_unix_ns=stamp,packet=packet.hex(),decoded=decoded))+'\n')
            if decoded:
                row=dict(received_unix_ns=stamp,**decoded)
                if row['kind']=='position':self.positions[row['motor']]=row
                elif row['kind']=='status':
                    self.status[row['motor']]=row
                    if row['fault_bits'] or row['mode'] not in self.accepted_modes:self.faults.append(row)
                elif row['kind'] in ('fault','invalid_position'):self.faults.append(row)

    def close(self):self.sock.close()


def read_sdk_batch(hw, read_mode):
    """Diagnostic reads preserve rejected SDK values instead of throwing before logging."""
    values=[];errors=[]
    if read_mode=='parallel':
        futures=[hw.reader.submit(m.robstride_get_param_f32_host_id,0x7019,0xFD,30) for m in hw.motors]
        getters=[f.result for f in futures]
    elif read_mode=='serial':
        getters=[lambda m=m:m.robstride_get_param_f32_host_id(0x7019,0xFD,30) for m in hw.motors]
    else:
        raise ValueError('read_mode must be parallel or serial')
    for mid,get in enumerate(getters,1):
        try:
            value=float(get())
            if not math.isfinite(value):raise ValueError(f'nonfinite SDK value: {value!r}')
            values.append(value)
        except Exception as error:
            values.append(None);errors.append(dict(motor=mid,error=str(error)))
    return values,errors


def read_raw_batch(monitor, timeout=.05):
    """Read only 0x7019 using matched motor/register/timestamps, bypassing SDK cache."""
    monitor.drain()
    sent={}
    for mid in range(1,8):
        sent[mid]=time.time_ns()
        packet=FRAME.pack(0x80000000|(17<<24)|(0xFD<<8)|mid,8,struct.pack('<H6x',0x7019))
        if monitor.sock.send(packet)!=len(packet):raise RuntimeError('short CAN parameter request')
    deadline=time.monotonic()+timeout
    while True:
        monitor.drain()
        missing=[mid for mid in sent if mid not in monitor.positions or
                 monitor.positions[mid]['received_unix_ns']<sent[mid]]
        if not missing:break
        remaining=deadline-time.monotonic()
        if remaining<=0 or monitor.faults:break
        select.select([monitor.sock],[],[],min(remaining,.005))
    values=[];errors=[]
    for mid in range(1,8):
        raw=monitor.positions.get(mid)
        if raw is None or raw['received_unix_ns']<sent[mid]:
            values.append(None);errors.append(dict(motor=mid,error='fresh raw 0x7019 reply missing'))
        else:values.append(raw['position_rad'])
    return values,errors


def audit(seconds,port,read_mode='parallel'):
    if not 1<=seconds<=60:raise ValueError('audit duration must be 1..60 seconds')
    if read_mode not in ('parallel','serial','raw'):raise ValueError('read mode must be parallel, serial or raw')
    folder=ROOT/'build/record-review';folder.mkdir(parents=True,exist_ok=True)
    stem=folder/f'feedback-audit-{time.time_ns()}'
    report=dict(mode='read_only',read_mode=read_mode,sdk_compared=read_mode!='raw',enable_requested=False,motion_requested=False,success=False,
                samples=0,sdk_raw_mismatch_count=0,missing_raw_count=0,
                sdk_read_error_count=0,raw_read_error_count=0,out_of_range_count=0,
                max_status_difference_deg=0.,max_cycle_ms=0.,max_status_age_ms=0.)
    hw=None;monitor=None
    with stem.with_suffix('.jsonl').open('x') as log:
        try:
            monitor=Monitor(port,log);hw=Hardware(port);hw.reports();hw.check_status(0)
            start=time.monotonic();deadline=start
            while time.monotonic()-start<seconds:
                monitor.drain();begin=time.time_ns();tick=time.monotonic()
                # Do not use Hardware.positions(): its range check raises before
                # the diagnostic can persist the bad value and matching raw frame.
                try:
                    sdk,read_errors=read_raw_batch(monitor) if read_mode=='raw' else read_sdk_batch(hw,read_mode)
                finally:
                    monitor.drain()
                now=time.time_ns()
                sample=dict(event='raw_position_sample' if read_mode=='raw' else 'sdk_raw_sample',request_begin_unix_ns=begin,
                            received_end_unix_ns=now,read_mode=read_mode,
                            sdk_rad=None if read_mode=='raw' else sdk,position_rad=sdk,
                            sdk_errors=read_errors,raw_positions=dict(monitor.positions),
                            raw_status=dict(monitor.status))
                log.write(json.dumps(sample,allow_nan=False)+'\n');log.flush()
                report['samples']+=1
                report['raw_read_error_count' if read_mode=='raw' else 'sdk_read_error_count']+=len(read_errors)
                report['max_cycle_ms']=max(report['max_cycle_ms'],(time.monotonic()-tick)*1000)
                hw.check_status(0)
                for mid,value in enumerate(sdk,1):
                    raw=monitor.positions.get(mid);status=monitor.status.get(mid)
                    if raw is None or raw['received_unix_ns']<begin:
                        report['missing_raw_count']+=1
                    elif value is not None and abs(float(value)-raw['position_rad'])>1e-7:
                        report['sdk_raw_mismatch_count']+=1
                        log.write(json.dumps(dict(event='sdk_raw_mismatch',motor=mid,sdk_rad=float(value),raw=raw))+'\n')
                    if status is None:raise RuntimeError('missing independent status')
                    age=(now-status['received_unix_ns'])/1e6
                    if age<0 or age>150:raise RuntimeError('stale independent status')
                    report['max_status_age_ms']=max(report['max_status_age_ms'],age)
                    if value is not None:
                        report['max_status_difference_deg']=max(report['max_status_difference_deg'],abs(math.degrees(float(value)-status['position_rad'])))
                        if value<LOWER[mid-1]-MEASUREMENT_TOLERANCE_RAD or value>UPPER[mid-1]+MEASUREMENT_TOLERANCE_RAD:
                            report['out_of_range_count']+=1
                if monitor.faults:raise RuntimeError('unexpected state/fault/raw parameter response')
                if read_errors or report['sdk_raw_mismatch_count'] or report['missing_raw_count']:
                    raise RuntimeError('SDK/raw feedback anomaly captured; see raw log')
                deadline+=.02;time.sleep(max(0,deadline-time.monotonic()))
            report.update(success=not report['sdk_raw_mismatch_count'] and not report['missing_raw_count'] and not report['out_of_range_count'],
                          motorbridge=hw.driver_version,raw_frames=monitor.frames,
                          motor_status=monitor.status)
        except Exception as error:
            report['error']=str(error)
        finally:
            # Cleanup does not enable/disable or clear any motor fault.
            if monitor is not None:
                try:monitor.drain()
                except Exception as error:report['drain_error']=str(error);report['success']=False
                report['raw_frames']=monitor.frames
                report['motor_status']=monitor.status
            if hw is not None:
                try:hw.close()
                except Exception as error:report['close_error']=str(error);report['success']=False
            if monitor is not None:monitor.close()
    report['raw_log']=str(stem.with_suffix('.jsonl'));report['report']=str(stem.with_suffix('.json'))
    stem.with_suffix('.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    return report


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds',type=float,default=30)
    parser.add_argument('--port',default='can0')
    parser.add_argument('--read-mode',choices=['parallel','serial','raw'],default='parallel')
    args=parser.parse_args();result=audit(args.seconds,args.port,args.read_mode)
    print(json.dumps(result,ensure_ascii=False,indent=2))
    raise SystemExit(0 if result['success'] else 1)
