#!/usr/bin/env python3
"""Inspect/probe or explicitly play one RS TCH1 v2 recording over SocketCAN.

Transport/read/cleanup conventions and MIT gains/dynamics follow the user's
lerobot-teleoperator-rebot-vr project. No zero writes or fault clearing.
Default operation is offline inspection. This is a supervised commissioning
utility, not the ESP32 execution backend or a collision planner.

Live playback is quarantined after the 2026-10-31 drop incident. Inspect/probe
remain available; there is deliberately no command-line bypass for this lock.
"""
from __future__ import annotations
import argparse
import hashlib
from importlib.metadata import version as package_version
import json
import math
import multiprocessing as mp
from pathlib import Path
import signal
import socket
import struct
import sys
import time
import zlib

import numpy as np
from packaging.version import Version
from scipy.interpolate import PchipInterpolator, CubicHermiteSpline

ROOT = Path(__file__).resolve().parents[1]
REFERENCE = Path('/home/verser/Python/lerobot-teleoperator-rebot-vr')
HEADER = struct.Struct('<4sHHBBBBHHHHIIII6s18sI')
SAMPLE = struct.Struct('<I6hH')
LOWER = np.array([-2.8, 0, 0, -1.57, -1.57, -3.14, -.12])
UPPER = np.array([2.8, 3.14, 3.14, 1.57, 1.57, 3.14, math.radians(250)+.12])
# Feedback-only allowance; recording/trajectory target bounds stay unchanged.
MEASUREMENT_TOLERANCE_RAD = math.radians(3.)
FRAME = struct.Struct('=IB3x8s')
PLAYBACK_BLOCK_REASON = ('Live playback disabled after observed arm drop on 2026-10-31. '
    'Resolve feedback integrity and a mechanically supported/braked stop procedure '
    'before commissioning again; see assets/docs/RS_PLAYBACK_INCIDENT.md.')


def require_playback_clearance():
    raise RuntimeError(PLAYBACK_BLOCK_REASON)


def validate_driver_version():
    installed=package_version('motorbridge')
    if not Version('0.5.3') <= Version(installed) < Version('1'):
        raise RuntimeError(f'motorbridge {installed} is incompatible with the reference; '
            'requires >=0.5.3,<1. Use the calibrated lerobot environment.')
    return installed


def load_record(path):
    raw = Path(path).read_bytes()
    if not 100 <= len(raw) <= 540082:
        raise ValueError('invalid recording size')
    (magic, version, hsize, fmt, model, dof, reason, mask, ssize, period, flags,
     coordinate, calibration, count, duration, ids, extra, crc) = HEADER.unpack_from(raw)
    if ((magic, version, hsize, fmt, model, dof, reason, mask, ssize, period, flags)
            != (b'TCH1', 2, 64, 2, 1, 6, 1, 127, 18, 20, 0)):
        raise ValueError('requires normally stopped RS seven-channel TCH1 v2')
    if (coordinate != 1 or ids != bytes(range(1,7)) or extra[0] != 7 or
            extra[1] or struct.unpack_from('<I',extra,2)[0] != 250 or extra[6:] != bytes(12)):
        raise ValueError('coordinate/motor/gripper calibration identity mismatch')
    if count < 2 or count > 30001 or len(raw) != 64+18*count:
        raise ValueError('sample count mismatch')
    if zlib.crc32(raw[:60]+bytes(4)+raw[64:]) != crc:
        raise ValueError('CRC mismatch')
    rows = np.asarray(list(SAMPLE.iter_unpack(raw[64:])), dtype=float)
    t = rows[:,0]/1000
    q = rows[:,1:7]/10000
    q = np.column_stack((q,rows[:,7]/65535*math.radians(250)))
    if (t[0] != 0 or rows[-1,0] != duration or not 0 < duration <= 600000 or
            np.any(np.diff(rows[:,0]) <= 0) or np.any(np.diff(rows[:,0]) > 100)):
        raise ValueError('invalid timestamps')
    if np.any(q < LOWER) or np.any(q > UPPER):
        raise ValueError('recording exceeds reference RS joint limits')
    # First supervised release supports the supplied closed-gripper recordings.
    if np.any(rows[:,7] != 0):
        raise ValueError('moving-gripper recordings require separate commissioning')
    return t,q,dict(file=str(Path(path).resolve()),sha256=hashlib.sha256(raw).hexdigest(),
        samples=count,duration_s=t[-1],calibration_id=calibration)


class Plan:
    def __init__(self,t,q,speed=.25):
        if not math.isfinite(speed) or not 0 < speed <= .5:
            raise ValueError('commissioning speed must be in (0, 0.5]')
        tangents=PchipInterpolator(t,q).derivative()(t)
        tangents[0]=tangents[-1]=0
        self.curve=CubicHermiteSpline(t,q,tangents)
        vmax=amax=0.
        for i,h in enumerate(np.diff(t)):
            a,b,c,d=self.curve.c[:,i,:]
            for j in range(7):
                positions=[0.,float(h)]
                for r in np.roots([3*a[j],2*b[j],c[j]]):
                    if abs(r.imag)<1e-10 and 0<r.real<h: positions.append(float(r.real))
                for u in positions:
                    value=((a[j]*u+b[j])*u+c[j])*u+d[j]
                    if value<LOWER[j]-1e-9 or value>UPPER[j]+1e-9:
                        raise ValueError('interpolated joint limit violation')
                velocities=[abs(c[j]),abs(3*a[j]*h*h+2*b[j]*h+c[j])]
                if a[j] and 0 < -b[j]/(3*a[j]) < h:
                    u=-b[j]/(3*a[j]);velocities.append(abs(3*a[j]*u*u+2*b[j]*u+c[j]))
                vmax=max(vmax,*velocities)
                amax=max(amax,abs(2*b[j]),abs(6*a[j]*h+2*b[j]))
        self.speed=min(speed,.3/max(vmax,1e-12),math.sqrt(.8/max(amax,1e-12)))
        self.duration=float(t[-1]/self.speed)
        self.start=q[0].copy();self.end=q[-1].copy()
        self.peak_velocity=vmax*self.speed;self.peak_acceleration=amax*self.speed**2

    def at(self,elapsed):
        u=np.clip(elapsed*self.speed,0,self.curve.x[-1])
        return self.curve(u),self.curve(u,1)*self.speed


def bridge(start,end):
    delta=end-start
    if np.max(np.abs(delta))>math.radians(10):
        raise ValueError('start/return gap exceeds 10 degrees; reposition explicitly')
    duration=max(3.,1.875*np.max(np.abs(delta))/.15,
                 math.sqrt(5.774*np.max(np.abs(delta))/.4))
    def sample(t):
        u=np.clip(t/duration,0,1)
        return start+delta*(10*u**3-15*u**4+6*u**5), delta*(30*u**2-60*u**3+30*u**4)/duration
    return duration,sample


def parse_status(packet):
    if len(packet)!=16:return None
    ident,n,data=FRAME.unpack(packet)
    if not ident&0x80000000 or ident&0x60000000 or n!=8:return None
    ident &= 0x1fffffff
    if (ident>>24)&31 not in (2,24) or ident&255 != 0xFD:return None
    extra=(ident>>8)&65535
    mid=extra&255
    if not 1<=mid<=7:return None
    return mid,extra>>14,(extra>>8)&63


def watchdog(pipe,port):
    """Separate process: terminate output on lost heartbeat or parent death."""
    # Terminal Ctrl+C reaches the whole foreground process group. The parent
    # owns graceful return/abort; this guard must stay alive during that motion.
    signal.signal(signal.SIGINT,signal.SIG_IGN)
    signal.signal(signal.SIGTERM,signal.SIG_IGN)
    bus=socket.socket(socket.PF_CAN,socket.SOCK_RAW,socket.CAN_RAW)
    bus.settimeout(.1);bus.bind((port,));pipe.send('ready')
    armed=False
    try:
        while True:
            if pipe.poll(.4):
                try:message=pipe.recv()
                except EOFError:break
                if message=='quit':return
                if message=='arm':armed=True
            elif armed:break
        if armed:
            for _ in range(3):
                for mid in range(1,8):
                    try:bus.send(FRAME.pack(0x80000000|(4<<24)|(0xFD<<8)|mid,8,bytes(8)))
                    except OSError:pass
                time.sleep(.02)
    finally:bus.close();pipe.close()


class Hardware:
    def __init__(self,port='can0'):
        # Reject a different SDK before opening a socket or loading a controller.
        self.driver_version=validate_driver_version()
        from lerobot_teleoperator_rebot_vr.hardware.rs import RSConnection
        self.bus=None;self.motors=[];self.reader=None;self.rx=None
        self.connection=None;self.last_read=None
        self.states={};self.report_enabled=False;self.armed=False;self.guard=None;self.pipe=None
        try:
            self.rx=socket.socket(socket.PF_CAN,socket.SOCK_RAW,socket.CAN_RAW)
            self.rx.bind((port,));self.rx.setblocking(False)
            # Reuse the user's VR transport: motor identities, host ID, parallel
            # parameter reads, joined futures and resource cleanup live there.
            self.connection=RSConnection(port)
            self.connection.open()
            self.bus=self.connection.bus
            self.motors=list(self.connection.motors.values())
            self.reader=self.connection._reader  # Compatibility for diagnostic-only reads.
            self.port=port
        except BaseException:
            self.close();raise

    def drain(self):
        while True:
            try:packet=self.rx.recv(16)
            except BlockingIOError:break
            parsed=parse_status(packet)
            if parsed:self.states[parsed[0]]=(time.monotonic(),*parsed[1:])

    def positions(self):
        begin=time.monotonic()
        results=self.connection.read_positions()
        values=[];errors=[];intervals={}
        for name,result in results.items():
            if isinstance(result,Exception):
                values.append(None);errors.append(f'{name}: {result}')
            else:
                value,started,finished=result
                values.append(float(value));intervals[name]=[started,finished]
        # Keep even rejected SDK values for the caller's failure journal.
        self.last_read=dict(sdk_rad=[v if v is not None and math.isfinite(v) else None for v in values],
                            errors=errors,read_intervals_ns=intervals)
        if errors:raise RuntimeError('feedback read failed: '+str(errors))
        q=np.asarray(values)
        if q.shape!=(7,) or time.monotonic()-begin>.1 or not np.all(np.isfinite(q)):
            raise RuntimeError('feedback stale/nonfinite')
        outside=(q<LOWER-MEASUREMENT_TOLERANCE_RAD) | (q>UPPER+MEASUREMENT_TOLERANCE_RAD)
        if np.any(outside):
            details=[dict(motor_id=int(i+1),measured_deg=float(np.degrees(q[i])),
                lower_deg=float(np.degrees(LOWER[i])),upper_deg=float(np.degrees(UPPER[i])),
                measurement_tolerance_deg=math.degrees(MEASUREMENT_TOLERANCE_RAD))
                for i in np.flatnonzero(outside)]
            raise RuntimeError('measured position outside model limits: '+json.dumps(details))
        self.drain()
        return q

    def reports(self):
        self.report_enabled=True
        for m in self.motors:m.robstride_set_active_report(True)
        deadline=time.monotonic()+1
        while time.monotonic()<deadline:
            self.drain()
            if len(self.states)==7:return
            time.sleep(.01)
        raise RuntimeError('missing fresh seven-motor status reports; no enable')

    def stop_reports(self):
        # Periodic type-24 reports are needed for disabled startup checks only.
        # During MIT control, each command already returns a type-2 status.
        # Keep the SDK transport and existing freshness/fault checks; avoid an
        # additional 100 Hz stream per motor interleaving with parameter reads.
        for m in self.motors:
            self.heartbeat()
            m.robstride_set_active_report(False)
        # Leave this true on partial failure so close() retries all motors.
        self.report_enabled=False

    def check_status(self,mode):
        self.drain();now=time.monotonic()
        for mid in range(1,8):
            value=self.states.get(mid)
            if value is None or now-value[0]>.15 or value[1]!=mode or value[2]:
                raise RuntimeError(f'motor {mid}: stale/faulted/unexpected state {value}; expected mode {mode}')

    def start_guard(self, progress=None):
        ctx=mp.get_context('spawn');self.pipe,child=ctx.Pipe()
        self.guard=ctx.Process(target=watchdog,args=(child,self.port),daemon=True)
        self.guard.start();child.close()
        # Spawning Python can take hundreds of milliseconds. Keep independent
        # CAN observers draining while periodic startup reports are active.
        deadline=time.monotonic()+5
        while not self.pipe.poll(.02):
            if progress is not None:progress()
            if time.monotonic()>=deadline:raise RuntimeError('watchdog unavailable')
        if progress is not None:progress()
        if self.pipe.recv()!='ready':raise RuntimeError('watchdog unavailable')
        self.pipe.send('arm');self.armed=True

    def heartbeat(self):
        if self.armed:
            if not self.guard.is_alive():raise RuntimeError('watchdog stopped')
            self.pipe.send('heartbeat')

    def enable(self,q):
        require_playback_clearance()

    def enable_supported_trial(self,q,*,fall_arrest_confirmed=False,progress=None):
        # Only the bounded commissioning runner uses this path. The original
        # raw-record player stays quarantined. Loss-of-process disable requires
        # an actual mechanical fall arrest, never merely a cleared workspace.
        if fall_arrest_confirmed is not True:
            raise RuntimeError('mechanical fall arrest must be installed before commissioning')
        from motorbridge import Mode
        self.check_status(0)
        for m in self.motors:
            m.ensure_mode(Mode.MIT,300)
            if progress is not None:progress()
        q2=self.positions()
        if np.max(np.abs(q2-q))>math.radians(1):raise RuntimeError('arm moved during setup')
        self.check_status(0)
        # Seed measured positions at zero gains, matching RSFollower startup.
        for m,p in zip(self.motors,q2):m.send_mit(float(p),0.,0.,0.,0.)
        if progress is not None:progress()
        self.start_guard(progress=progress)
        for m in self.motors:
            self.heartbeat();m.enable()
            if progress is not None:progress()
        deadline=time.monotonic()+.25
        while time.monotonic()<deadline:
            self.heartbeat();self.drain()
            if progress is not None:progress()
            if len(self.states)==7 and all(x[1]==2 and not x[2] for x in self.states.values()):return
            time.sleep(.01)
        raise RuntimeError('enable feedback missing')

    def disable(self):
        confirmed=False
        if self.armed:
            for _ in range(3):
                try:self.heartbeat()
                except Exception:pass
                self.drain();self.states.clear()
                for m in self.motors:
                    try:m.disable()
                    except Exception:pass
                end=time.monotonic()+.12
                while time.monotonic()<end:
                    self.drain()
                    if len(self.states)==7 and all(s[1]==0 for s in self.states.values()):
                        confirmed=True;break
                    time.sleep(.005)
                if confirmed:break
        else:
            # Setup can fail before start_guard()/enable(). Confirm the current
            # seven-axis disabled state instead of reporting a fictitious stop
            # failure. Stale/missing/faulted feedback must still fail confirmation.
            try:
                self.check_status(0)
                confirmed=True
            except Exception:
                pass
        if self.pipe:
            # On failed confirmation, closing pipe lets the guard repeat disable.
            if confirmed or not self.armed:
                try:self.pipe.send('quit')
                except (BrokenPipeError, EOFError):pass
            self.pipe.close();self.pipe=None
        if self.guard:self.guard.join(timeout=2)
        self.armed=False
        return confirmed

    def close(self):
        try:
            if self.report_enabled:
                for m in self.motors:
                    try:m.robstride_set_active_report(False)
                    except Exception:pass
            if self.connection is not None:self.connection.close()
        finally:
            self.connection=None;self.bus=None;self.reader=None;self.motors=[]
            if self.rx:self.rx.close();self.rx=None


def run_segment(hw,sample,duration,gravity,kp,kd,caps,log,phase,gravity_ramp=False):
    start=time.monotonic();previous=start;deadline=start;peak=0.;last_print=-5
    while True:
        now=time.monotonic()
        if now-previous>.1:raise RuntimeError('control deadline exceeded 100 ms')
        elapsed=min(now-start,duration);previous=now
        actual=hw.positions();hw.check_status(2);hw.heartbeat()
        target,velocity=sample(elapsed)
        error=float(np.max(np.abs(target-actual)))
        # Persist the failing sample BEFORE validation/command dispatch. A
        # rejected measurement is diagnostic evidence, never a new setpoint.
        log.write(json.dumps(dict(event='feedback',monotonic_s=time.monotonic(),
            phase=phase,t=elapsed,q=actual.tolist(),target=target.tolist(),
            joint_error_deg=np.degrees(target-actual).tolist(),
            error_deg=math.degrees(error)))+'\n')
        log.flush()
        if error>math.radians(5):raise RuntimeError(f'following error {math.degrees(error):.2f} deg')
        torque=np.asarray(gravity(actual[:6]))
        if not np.all(np.isfinite(torque)) or np.any(np.abs(torque)>caps):
            raise RuntimeError('gravity torque exceeds reference budget')
        if gravity_ramp:torque*=min(elapsed/1.5,1)
        for i,m in enumerate(hw.motors):
            m.send_mit(float(target[i]),float(velocity[i]),float(kp[i]),float(kd[i]),float(torque[i]) if i<6 else 0.)
        peak=max(peak,error)
        if elapsed-last_print>=5:
            print(f'{phase}: {elapsed:.1f}/{duration:.1f}s, error={math.degrees(error):.2f}deg',flush=True);last_print=elapsed
        if elapsed>=duration:break
        deadline+=.02
        time.sleep(max(0.,deadline-time.monotonic()))
    return math.degrees(peak)


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('record',type=Path)
    ap.add_argument('--mode',choices=['inspect','probe','play'],default='inspect')
    ap.add_argument('--port',default='can0')
    ap.add_argument('--speed',type=float,default=.25)
    ap.add_argument('--reference-root',type=Path,default=REFERENCE)
    ap.add_argument('--site-confirmed',action='store_true')
    ap.add_argument('--log-dir',type=Path,default=ROOT/'build/record-review')
    args=ap.parse_args()
    if args.mode=='play':require_playback_clearance()
    t,q,meta=load_record(args.record);plan=Plan(t,q,args.speed)
    meta.update(effective_speed=plan.speed,playback_s=plan.duration,
        planned_peak_rad_s=plan.peak_velocity,planned_peak_rad_s2=plan.peak_acceleration)
    print(json.dumps(meta,ensure_ascii=False,indent=2),flush=True)
    if args.mode=='inspect':return
    if args.mode=='play' and not args.site_confirmed:raise ValueError('site confirmation required')
    # Load/validate dependencies before opening hardware or enabling motors.
    if args.mode=='play':
        import pinocchio as pin
        import yaml
        config=yaml.safe_load((args.reference_root/'config/rs_mit_split.yaml').read_text())
        control=config['mit']
        kp=np.array([*control['mit_kp'],5.]);kd=np.array([*control['mit_kd'],.3])
        caps=np.array(control['mit_torque_limit_nm'])
        if kp.shape!=(7,) or kd.shape!=(7,) or caps.shape!=(6,) or not np.all(np.isfinite(np.r_[kp,kd,caps])):
            raise ValueError('invalid reference gains')
        if np.any(kp<0) or np.any(kp>500) or np.any(kd<0) or np.any(kd>5) or np.any(caps<=0) or np.any(caps>np.array([36]*3+[14]*3)):
            raise ValueError('reference gains/torque outside hardware bounds')
        urdf=args.reference_root/'src/lerobot_teleoperator_rebot_vr/urdf/rebot_b601_rs.urdf'
        model=pin.buildModelFromUrdf(str(urdf));data=model.createData()
        if model.nq!=6 or list(model.names)[1:]!=[f'joint{i}' for i in range(1,7)]:raise ValueError('model mismatch')
        def gravity(x):return pin.computeGeneralizedGravity(model,data,x).copy()
        for row in q:
            if np.any(np.abs(gravity(row[:6]))>caps):raise ValueError('recording gravity exceeds budget')
        meta['reference_urdf_sha256']=hashlib.sha256(urdf.read_bytes()).hexdigest()
    hw=Hardware(args.port)
    result=dict(metadata=meta,success=False,disable_confirmed=False,events=[],
        runtime=dict(python=sys.executable,motorbridge=hw.driver_version),mode=args.mode)
    args.log_dir.mkdir(parents=True,exist_ok=True)
    prefix=args.log_dir/(time.strftime('%Y%m%d-%H%M%S')+'-'+args.record.stem)
    try:
        initial=hw.positions();print('actual_deg:',np.degrees(initial).round(3).tolist(),flush=True)
        result['initial_rad']=initial.tolist()
        if args.mode=='probe':
            hw.reports();hw.drain()
            result['motor_status']={str(k):{'mode':v[1],'fault_bits':v[2]} for k,v in hw.states.items()}
            print(json.dumps(result['motor_status']),flush=True)
            result['success']=True
            return
        hw.reports();hw.check_status(0)
        entrance=bridge(initial,plan.start);exit_segment=bridge(plan.end,initial)
        hw.enable(initial)
        def interrupted(*_):raise KeyboardInterrupt('stop requested')
        signal.signal(signal.SIGINT,interrupted);signal.signal(signal.SIGTERM,interrupted)
        hold=lambda t:(initial,np.zeros(7))
        with prefix.with_suffix('.jsonl').open('x') as log:
            run_segment(hw,hold,2.,gravity,kp,kd,caps,log,'hold',True)
            run_segment(hw,entrance[1],entrance[0],gravity,kp,kd,caps,log,'align')
            run_segment(hw,plan.at,plan.duration,gravity,kp,kd,caps,log,'play')
            run_segment(hw,exit_segment[1],exit_segment[0],gravity,kp,kd,caps,log,'return')
            run_segment(hw,hold,1.,gravity,kp,kd,caps,log,'settle')
        if np.max(np.abs(hw.positions()-initial))>math.radians(1):
            raise RuntimeError('return did not settle within one degree')
        result['success']=True
    except BaseException as e:
        result['events'].append(dict(event='exception',monotonic_s=time.monotonic(),detail=str(e)))
        result['error']=str(e);raise
    finally:
        try:
            if hw.armed:
                result['events'].append(dict(event='disable_requested',monotonic_s=time.monotonic()))
                result['disable_confirmed']=hw.disable()
                result['events'].append(dict(event='disable_finished',monotonic_s=time.monotonic(),
                    confirmed=result['disable_confirmed']))
                if not result['disable_confirmed']:result['success']=False
        finally:
            hw.close()
            prefix.with_suffix('.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
            print(json.dumps({'success':result['success'],'disable_confirmed':result['disable_confirmed'],'report':str(prefix.with_suffix('.json'))}),flush=True)

if __name__=='__main__':main()
