#!/usr/bin/env python3
"""Calibration-bound standing PROBE commissioning; mechanical fall arrest required."""
from __future__ import annotations
import hashlib
import json
import math
from pathlib import Path
import signal
import time

import numpy as np
from scipy.interpolate import make_interp_spline, PPoly
from rs_record_playback import ROOT, REFERENCE, LOWER, UPPER, Hardware, load_record
from rs_feedback_audit import Monitor

CALIBRATION=Path.home()/'.cache/huggingface/lerobot/calibration/robots/rebot_b601_rs_follower/rebot_b601_rs_vr.json'
CALIBRATION_LOG=CALIBRATION.parent/'rebot_b601_rs_vr_20261031_163055_890711.jsonl'
NAMES=['shoulder_pan','shoulder_lift','elbow_flex','wrist_flex','wrist_yaw','wrist_roll','gripper']
VMAX=.12
AMAX=.25
AWAITING_FRESH_RECORDING=True


def digest(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


class Curve:
    """C2 entry/exit, analytic per-axis extrema; retime, never clamp joint values."""
    def __init__(self,t,q,minimum_duration=0,measurement_margin=False,initial_position_margin=0.,
                 velocity_limit=VMAX,acceleration_limit=AMAX):
        t=np.asarray(t,float);q=np.asarray(q,float)
        if q.shape!=(len(t),7) or t[0]!=0 or np.any(np.diff(t)<=0) or not np.all(np.isfinite(q)):
            raise ValueError('invalid curve')
        if not math.isfinite(initial_position_margin) or initial_position_margin<0:
            raise ValueError('invalid initial position allowance')
        if not all(math.isfinite(x) and x>0 for x in (velocity_limit,acceleration_limit)):
            raise ValueError('invalid curve dynamic limits')
        self.velocity_limit=velocity_limit;self.acceleration_limit=acceleration_limit
        if initial_position_margin:
            if (len(t)!=2 or np.any(q[0]<LOWER-initial_position_margin)
                    or np.any(q[0]>UPPER+initial_position_margin)
                    or np.any(q[-1]<LOWER) or np.any(q[-1]>UPPER)):
                raise ValueError('initial feedback or destination outside joint bounds')
        zero=np.zeros(7)
        self.spline=make_interp_spline(t,q,k=5,bc_type=([(1,zero),(2,zero)],[(1,zero),(2,zero)]))
        vmax=amax=0.;self.lower=[];self.upper=[]
        margin=math.radians(.1) if measurement_margin else 1e-8
        for j in range(7):
            p=PPoly.from_spline((self.spline.t,self.spline.c[:,j],self.spline.k))
            for derivative in range(3):
                candidates=np.r_[t,p.derivative(derivative+1).roots(extrapolate=False)]
                candidates=candidates[np.isfinite(candidates)&(candidates>=t[0])&(candidates<=t[-1])]
                values=p(candidates,nu=derivative)
                if not np.all(np.isfinite(values)):raise ValueError('invalid curve extrema')
                if derivative==0:
                    lo=float(min(values));hi=float(max(values));self.lower.append(lo);self.upper.append(hi)
                    lower=min(LOWER[j],q[0,j]) if initial_position_margin else LOWER[j]-margin
                    upper=max(UPPER[j],q[0,j]) if initial_position_margin else UPPER[j]+margin
                    if lo<lower-1e-8 or hi>upper+1e-8:
                        raise ValueError(f'curve exceeds motor {j+1} bounds: {math.degrees(lo):.3f}..{math.degrees(hi):.3f}deg')
                elif derivative==1:vmax=max(vmax,float(max(abs(values))))
                else:amax=max(amax,float(max(abs(values))))
        self.scale=min(1.,velocity_limit/max(vmax,1e-12),math.sqrt(acceleration_limit/max(amax,1e-12)),t[-1]/max(minimum_duration,1e-12))
        self.duration=float(t[-1]/self.scale)
        self.start=q[0].copy();self.end=q[-1].copy()
        self.peak_velocity=vmax*self.scale;self.peak_acceleration=amax*self.scale**2

    def at(self,t):
        x=np.clip(t,0,self.duration)*self.scale
        return self.spline(x),self.spline(x,nu=1)*self.scale

    def reverse_at(self,t):
        q,v=self.at(self.duration-t)
        return q,-v


def prepare():
    if AWAITING_FRESH_RECORDING:
        raise RuntimeError('等待新录制；不再生成旧零点轨迹的转换方案。')
    evidence=json.loads((ROOT/'assets/verification/rs_recalibration.json').read_text())
    if digest(CALIBRATION)!=evidence['calibration_sha256'] or digest(CALIBRATION_LOG)!=evidence['calibration_log_sha256']:
        raise ValueError('标定文件已变化，需要重新生成动作；没有启动电机。')
    cal=json.loads(CALIBRATION.read_text())
    urdf=REFERENCE/'src/lerobot_teleoperator_rebot_vr/urdf/rebot_b601_rs.urdf'
    if cal['status']!='complete' or cal['zeroed_joints']!=NAMES or digest(urdf)!=cal['model_sha256']:
        raise ValueError('标定身份或模型不匹配')
    events=[json.loads(line) for line in CALIBRATION_LOG.read_text().splitlines()]
    before=next(r['positions_deg'] for r in events if r['event']=='before')
    settled=next(r['positions_deg'] for r in events if r['event']=='after_disable')
    if max(abs(before[n]-settled[n]) for n in NAMES)>.1:
        raise ValueError('校零前姿态不稳定，不能转移旧轨迹')
    if [r['joint'] for r in events if r['event']=='zero_acknowledged']!=NAMES:
        raise ValueError('七路零点写入证据不完整')
    source=ROOT/'record/trajectory-2.tch'
    record=next(r for r in json.loads((ROOT/'record/manifest.json').read_text())['clips'] if r['action_id']=='PROBE')
    if digest(source)!=record['sha256']:raise ValueError('抬头源录制已变化')
    t,q,_=load_record(source)
    offset=np.radians([settled[n] for n in NAMES[:6]])
    transformed=q.copy();transformed[:,:6]-=offset
    # TCH gripper is normalized, not an old absolute encoder angle. Closed stays 0.
    transformed[:,6]=0.
    end=int(np.argmin(abs(t-5.0)));peak=int(np.argmin(abs(t-7.0)))
    # Keep the recorded approach geometry and remove only sub-sample hand noise.
    indices=np.unique(np.r_[0,[int(np.argmin(abs(t-sec))) for sec in np.arange(.2,t[end],.2)],end])
    entry=Curve(t[indices],transformed[indices])
    # Reject path fitting that deviates appreciably from the actual taught approach.
    fitted=entry.spline(t[:end+1])
    fitting_error=float(np.max(np.abs(fitted-transformed[:end+1])))
    if fitting_error>math.radians(1.):raise ValueError(f'起身拟合偏差过大：{math.degrees(fitting_error):.3f}°')
    stand=entry.end.copy()
    look=stand+.2*(transformed[peak]-stand)
    look[6]=0.
    gesture=Curve([0.,1.],np.array([stand,look]),minimum_duration=3.)
    aligned=Curve([0.,1.],np.array([np.zeros(7),entry.start]),minimum_duration=3.)
    report=dict(schema='rs.standing_probe_trial.v1',status='prepared_not_hardware_accepted',
        source=str(source),source_sha256=digest(source),calibration_sha256=digest(CALIBRATION),
        calibration_log_sha256=digest(CALIBRATION_LOG),urdf_sha256=digest(urdf),
        coordinate_transfer=dict(method='new_arm_q = recorded_arm_q - old_reading_at_new_zero',
            old_zero_deg=[settled[n] for n in NAMES[:6]],gripper='normalized closed remains new calibrated zero',
            uncertainty='Pre-zero snapshot precedes individual zero writes; transformed path still requires supported physical validation.'),
        entry_source_end_s=float(t[end]),look_source_peak_s=float(t[peak]),expression_scale=.2,
        stand_deg=np.degrees(stand).tolist(),look_deg=np.degrees(look).tolist(),
        entry_t=t[indices].tolist(),entry_q_rad=transformed[indices].tolist(),
        max_entry_fit_error_deg=math.degrees(fitting_error),
        align_s=aligned.duration,entry_s=entry.duration,look_up_s=gesture.duration,look_hold_s=1.,
        nominal_total_s=2*aligned.duration+2*entry.duration+2*gesture.duration+2.,
        speed_limit_rad_s=VMAX,acceleration_limit_rad_s2=AMAX,
        mechanical_fall_arrest_required=True,gripper_motion=False)
    folder=ROOT/'record/prepared';folder.mkdir(exist_ok=True)
    path=folder/'standing-probe-v1.json';path.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    return report,entry,gesture,urdf


class CheckedFeedback:
    def __init__(self,hw,monitor):
        self.hw=hw;self.monitor=monitor

    def read(self,mode):
        self.monitor.drain();begin=time.time_ns()
        q=self.hw.positions();self.hw.check_status(mode);self.monitor.drain()
        now=time.time_ns()
        for mid,value in enumerate(q,1):
            raw=self.monitor.positions.get(mid);status=self.monitor.status.get(mid)
            if raw is None or raw['received_unix_ns']<begin or now-raw['received_unix_ns']>100_000_000:
                raise RuntimeError(f'motor {mid}: no fresh raw position reply')
            if abs(float(value)-raw['position_rad'])>1e-7:
                raise RuntimeError(f'motor {mid}: SDK/raw feedback mismatch')
            if status is None or not 0<=now-status['received_unix_ns']<=100_000_000:
                raise RuntimeError(f'motor {mid}: stale kernel-timestamped status')
            if status['mode']!=mode or status['fault_bits']:
                raise RuntimeError(f'motor {mid}: unexpected mode/fault')
            if abs(float(value)-status['position_rad'])>math.radians(.5):
                raise RuntimeError(f'motor {mid}: parameter/status position mismatch')
        if self.monitor.faults:raise RuntimeError('CAN status/fault anomaly recorded')
        return q


class Runner:
    def __init__(self,hw,feedback,gravity,log,cancel):
        self.hw=hw;self.feedback=feedback;self.gravity=gravity;self.log=log;self.cancel=cancel
        self.kp=np.array([50,50,50,30,50,50,5.])
        self.kd=np.array([3,5,5,3,4,4,.3])
        self.caps=np.array([12,12,12,3,3,3])

    def segment(self,label,at,duration,*,protective_hold=False):
        started=time.monotonic();previous=started;deadline=started
        while True:
            now=time.monotonic()
            if now-previous>.1:raise RuntimeError('control deadline exceeded 100 ms')
            previous=now
            if self.cancel[0] and not protective_hold:raise RuntimeError('operator stop requested')
            self.hw.heartbeat()
            actual=self.feedback.read(2)
            elapsed=min(time.monotonic()-started,duration)
            target,velocity=at(elapsed)
            error=float(max(abs(target-actual)))
            torque=self.gravity(actual[:6])
            sample=dict(event='control_sample',phase=label,t=elapsed,monotonic_s=time.monotonic(),
                actual_rad=actual.tolist(),target_rad=target.tolist(),velocity_rad_s=velocity.tolist(),
                gravity_nm=torque.tolist(),error_deg=math.degrees(error))
            self.log.write(json.dumps(sample)+'\n');self.log.flush()
            if error>math.radians(5):raise RuntimeError(f'following error {math.degrees(error):.3f}deg')
            if not np.all(np.isfinite(torque)) or np.any(abs(torque)>self.caps):
                raise RuntimeError('gravity feedforward exceeds reference budget')
            if np.any(abs(velocity)>VMAX+1e-8):raise RuntimeError('command velocity exceeds budget')
            for i,motor in enumerate(self.hw.motors):
                motor.send_mit(float(target[i]),float(velocity[i]),float(self.kp[i]),float(self.kd[i]),
                               float(torque[i]) if i<6 else 0.)
            if elapsed>=duration:return
            deadline+=.02;time.sleep(max(0,deadline-time.monotonic()))

    def hold(self,q,duration,label='hold',protective=False):
        self.segment(label,lambda t:(q,np.zeros(7)),duration,protective_hold=protective)


def play(*,port='can0',fall_arrest_confirmed=False):
    if AWAITING_FRESH_RECORDING:
        raise RuntimeError('等待重新录制的抬头动作；旧轨迹转换方案已停用。')
    # This is a real commissioning entry, not a bypass for the old raw player.
    # No CAN is opened until the operator confirms an installed fall arrest.
    if fall_arrest_confirmed is not True:
        raise RuntimeError('先装好防坠支撑或吊带，再用 ./daily --supported 启动。')
    plan,entry,gesture,urdf=prepare()
    import pinocchio as pin
    model=pin.buildModelFromUrdf(str(urdf));data=model.createData()
    if model.nq!=6 or list(model.names)[1:]!=[f'joint{i}' for i in range(1,7)]:
        raise ValueError('unexpected dynamics model')
    def gravity(q):return pin.computeGeneralizedGravity(model,data,q).copy()
    for curve in (entry,gesture):
        for t in np.linspace(0,curve.duration,500):
            if np.any(abs(gravity(curve.at(t)[0][:6]))>np.array([12,12,12,3,3,3])):
                raise ValueError('planned gravity exceeds budget')
    folder=ROOT/'build/daily-actions';folder.mkdir(parents=True,exist_ok=True)
    stem=folder/f'probe-trial-{time.time_ns()}'
    result=dict(success=False,hardware_accepted=False,plan=plan,fall_arrest_confirmed=True,
                events=[],disable_confirmed=False)
    hw=None;monitor=None;runner=None;cancel=[False];old_handlers={}
    def event(name,**detail):
        result['events'].append(dict(event=name,monotonic_s=time.monotonic(),**detail))
    def stop(*_):cancel[0]=True
    with stem.with_suffix('.jsonl').open('x') as control_log,Path(str(stem)+'-can.jsonl').open('x') as raw_log:
        try:
            # One SDK controller, plus an independent receive-only observer.
            monitor=Monitor(port,raw_log);hw=Hardware(port);hw.reports()
            feedback=CheckedFeedback(hw,monitor)
            initial=feedback.read(0)
            if np.max(abs(initial))>math.radians(.5):
                raise RuntimeError('请先在未使能状态下恢复本次校零的坐姿；当前不是起始位。')
            for _ in range(10):
                time.sleep(.02)
                if np.max(abs(feedback.read(0)-initial))>math.radians(.1):
                    raise RuntimeError('起始姿态仍在移动，没有使能。')
            align=Curve([0.,1.],np.array([initial,entry.start]),minimum_duration=3.,measurement_margin=True)
            result.update(initial_rad=initial.tolist(),motorbridge=hw.driver_version)
            # Bind the runtime to the very same calibration checked by prepare().
            if digest(CALIBRATION)!=plan['calibration_sha256']:raise RuntimeError('calibration changed during setup')
            for sig in (signal.SIGINT,signal.SIGTERM):old_handlers[sig]=signal.signal(sig,stop)
            monitor.accepted_modes={0,2}
            event('enable_requested')
            hw.enable_supported_trial(initial,fall_arrest_confirmed=True)
            monitor.drain();monitor.accepted_modes={2}
            runner=Runner(hw,feedback,gravity,control_log,cancel)
            event('enabled')
            runner.hold(initial,1.,'initial_hold')
            print(f'正在起身，约 {align.duration+entry.duration:.0f} 秒…',flush=True)
            runner.segment('align',align.at,align.duration)
            runner.segment('enter_standing',entry.at,entry.duration)
            runner.hold(entry.end,1.,'standing')
            print('抬头看你…',flush=True)
            runner.segment('look_up',gesture.at,gesture.duration)
            runner.hold(gesture.end,1.,'look_hold')
            runner.segment('look_return',gesture.reverse_at,gesture.duration)
            runner.hold(entry.end,1.,'standing_return')
            print('动作完成，正在归位…',flush=True)
            runner.segment('leave_standing',entry.reverse_at,entry.duration)
            runner.segment('park',align.reverse_at,align.duration)
            runner.hold(initial,1.,'park_settle')
            for _ in range(10):
                if np.max(abs(feedback.read(2)-initial))>math.radians(.5):raise RuntimeError('park not verified')
                hw.heartbeat();time.sleep(.02)
            event('park_verified');result['success']=True
        except BaseException as error:
            event('failure',reason=str(error));result['error']=str(error)
            if hw is not None and hw.armed and runner is not None:
                # A fresh, mutually consistent measurement is required to hold.
                # Never drive back along a guessed recovery path after a fault.
                try:
                    q=runner.feedback.read(2)
                    event('protective_hold_started')
                    runner.hold(q,.5,'protective_hold',protective=True)
                    event('protective_hold_completed')
                except BaseException as hold_error:event('hold_unavailable',reason=str(hold_error))
            print('试播已停止。请保持防坠支撑，查看日志。',flush=True)
        finally:
            # Unlike the legacy player, this exit path is permitted only under
            # installed mechanical fall arrest. The watchdog uses the same rule.
            try:
                if hw is not None and hw.armed:
                    monitor.accepted_modes={0,2}
                    event('disable_requested_after_park_or_supported_fault')
                    result['disable_confirmed']=hw.disable()
                    event('disable_result',confirmed=result['disable_confirmed'])
                    if not result['disable_confirmed']:result['success']=False
            finally:
                try:
                    if hw is not None:hw.close()
                except Exception as close_error:
                    result['success']=False;result['close_error']=str(close_error)
                if monitor is not None:monitor.close()
                for sig,handler in old_handlers.items():signal.signal(sig,handler)
                result['report']=str(stem.with_suffix('.json'))
                stem.with_suffix('.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    if result['success']:print('已归位并失能。',flush=True)
    else:print('未完成试播；'+str(result.get('error','请检查失能反馈。')),flush=True)
    print('日志：'+result['report'],flush=True)
    return result


if __name__=='__main__':
    report,*_=prepare()
    print(json.dumps(report,ensure_ascii=False,indent=2))
