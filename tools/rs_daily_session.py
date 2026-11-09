"""Persistent PC daily-action trial; reuse checked RS transport and controllers."""
import json
import math
import queue
import signal
import threading
import time

import numpy as np

from rs_daily_prepare import DAILY_POSITION_LIMITS, load_bundle
from rs_gravity_record import (Journal, JournalError, TemperatureReturnRequested,
                               CAPS, MAX_GAP, PERIOD, ROOT, digest, read_commands)
from rs_gravity_play import Controller, PlaybackBackend, TorqueBudgetExceeded, prepare_entry
from rs_zero_return import prepare_zero_return
from rs_recovery_policy import RecoverableControlError, ArrivalTimeout
from rs_record_playback import LOWER, UPPER

GRIPPER_SETTLE_TOLERANCE = math.radians(3.)


def prepare_daily_zero_return(backend,gravity,identity,args,journal):
    return prepare_zero_return(backend,gravity,identity,args,journal,
                               exit_speed_rad_s=1.,exit_acceleration_rad_s2=2.,exit_tolerance_deg=3.)


class StopRequested(RuntimeError):
    pass


class Session:
    def __init__(self,args,manifest,plans,gravity,identity,journal,raw,commands,
                 backend_factory=None,zero_factory=prepare_daily_zero_return):
        self.args,self.manifest,self.plans,self.gravity,self.identity=args,manifest,plans,gravity,identity
        self.journal,self.raw,self.commands=journal,raw,commands
        self.backend_factory=backend_factory or (lambda:PlaybackBackend(args.port,raw,supported=True))
        self.zero_factory=zero_factory
        self.backend=None;self.enabled=False;self.attempted=False;self.home=None
        self.state='DISABLED';self.stop=None;self.homing=False
        self.idle=np.array(manifest['idle_rad']);self.zero=np.array(manifest['zero_rad'])
        self.target=None;self.controller=None;self.last=None;self.next_identity=0.
        self.last_tick=None;self.last_command=None
        self.protective_stops=[]

    def event(self,name,**values):
        self.journal.put(dict(event=name,monotonic_ns=time.monotonic_ns(),**values))

    def request_stop(self,*_):
        self.stop='abort' if self.stop or self.homing else 'quit'

    def check_stop(self,busy=True):
        if busy:
            while True:
                try:word=self.commands.get_nowait().strip()
                except queue.Empty:break
                if word in ('quit','exit','eof','退出'):self.request_stop()
                elif word=='status':print(f'当前状态：{self.state}',flush=True)
                elif word:print('动作进行中，请完成后再输入编号。',flush=True)
        if self.stop:raise StopRequested(self.stop)

    def verify(self):
        now=time.monotonic()
        if now<self.next_identity:return
        for path,key in ((self.args.calibration,'calibration_sha256'),(self.args.urdf,'urdf_sha256')):
            if digest(path)!=self.identity[key]:raise RuntimeError('calibration/model changed')
        self.next_identity=now+1.

    def tick(self,target,velocity=None,gain=1.,label='hold',busy=True,capture_hold=False,recovery=None,
             target_sampler=None):
        try:self.check_stop(busy)
        except StopRequested:
            if recovery is None or self.stop=='abort':raise
        now=time.monotonic()
        if self.last_tick is not None and now-self.last_tick>MAX_GAP:
            raise RuntimeError('daily control deadline exceeded 100 ms')
        self.last_tick=now
        self.backend.heartbeat()
        q,begin,end=self.backend.read()
        # CAN reads take about 14 ms on this setup. Evaluate moving targets
        # after the read, against the same wall clock as the imminent send,
        # rather than sending the pose calculated before blocking on feedback.
        if target_sampler is not None:
            target,velocity,gain=target_sampler()
        target_sample_ns=time.monotonic_ns()
        if capture_hold:
            # Anchor once to this newly verified sample, not the rejected goal.
            # Keep the velocity estimator intact: damping must account for motion.
            target=q.copy();velocity=np.zeros(7)
        requested_velocity=np.zeros(7) if velocity is None else velocity
        try:
            command=(self.controller.command(q,end,target,requested_velocity,gain) if recovery is None else
                     self.controller.recovery_command(q,end,**recovery))
        except RuntimeError as error:
            # Persist the rejected sample, not only the previous successful send.
            # A logging failure must not replace the original control fault.
            try:self.event('command_rejected',phase=label,actual_rad=q.tolist(),
                           target_rad=np.asarray(target).tolist(),velocity_rad_s=np.asarray(requested_velocity).tolist(),
                           error=str(error),details=getattr(error,'details',None))
            except Exception:pass
            raise
        self.verify();self.journal.check();self.raw.check()
        failures=[writer.failure for writer in (self.journal,self.raw) if getattr(writer,'failure',None)]
        if failures:raise JournalError('; '.join(failures))
        if time.monotonic_ns()-begin>100_000_000:raise RuntimeError('daily feedback expired before send')
        self.backend.send(command);self.last_command=command;self.last_gain=gain;self.last=q
        self.event('command',phase=label,target_sample_ns=target_sample_ns,feedback_begin_ns=begin,
                   feedback_end_ns=end,**command)
        return q

    def segment(self,label,curve=None,target=None,duration=None,ramp=False,tolerance=None):
        # Apply the planned speed profile only within this segment, restoring
        # the controller defaults before holding or starting another segment.
        old_velocity=self.controller.velocity_limit.copy()
        old_following=self.controller.following_limit.copy()
        try:
            if curve is not None:
                if hasattr(curve,'velocity_limit'):
                    self.controller.velocity_limit[:6]=curve.velocity_limit
                if hasattr(curve,'gripper_velocity_limit'):
                    self.controller.velocity_limit[6]=curve.gripper_velocity_limit
                if hasattr(curve,'gripper_following_limit'):
                    if not hasattr(curve,'gripper_velocity_limit'):
                        self.controller.velocity_limit[6]=curve.peak_velocity
                    self.controller.following_limit[6]=curve.gripper_following_limit
                if getattr(curve,'j1_peak_velocity',0.)>0:
                    self.controller.velocity_limit[0]=max(old_velocity[0],curve.j1_peak_velocity)
            self.event('segment_limits',phase=label,velocity_rad_s=self.controller.velocity_limit.tolist(),
                       following_rad=self.controller.following_limit.tolist())
            return self._segment(label,curve,target,duration,ramp,tolerance)
        finally:
            self.controller.velocity_limit=old_velocity
            self.controller.following_limit=old_following

    def _segment(self,label,curve=None,target=None,duration=None,ramp=False,tolerance=None):
        duration=curve.duration if curve is not None else duration
        limits=None if tolerance is None else np.broadcast_to(np.asarray(tolerance,float),(7,))
        if limits is not None and (not np.all(np.isfinite(limits)) or np.any(limits<=0)):
            raise ValueError('invalid arrival tolerances')
        started=time.monotonic();deadline=started;stable=None
        self.event('segment_start',phase=label,duration_s=duration,
                   arrival_tolerance_deg=None if limits is None else np.degrees(limits).tolist())
        def sample_after_feedback():
            nonlocal elapsed,goal,velocity
            elapsed=min(time.monotonic()-started,duration)
            goal,velocity=curve.at(elapsed)
            return goal,velocity,min(1.,elapsed/1.5) if ramp else 1.
        while True:
            if curve is not None:
                q=self.tick(None,label=label,target_sampler=sample_after_feedback)
            else:
                elapsed=min(time.monotonic()-started,duration)
                goal,velocity=target,np.zeros(7)
                q=self.tick(goal,velocity,min(1.,elapsed/1.5) if ramp else 1.,label)
            if tolerance is not None:
                settled=np.all(abs(q-goal)<=limits) and np.max(abs(self.controller.velocity))<=math.radians(1.)
                stable=(stable if stable is not None else time.monotonic()) if settled else None
                if stable is not None and time.monotonic()-stable>=.5:break
            if elapsed>=duration:
                if tolerance is not None:
                    outside=np.flatnonzero(abs(q-goal)>limits)
                    detail=', '.join(f'J{i+1}偏差 {math.degrees(q[i]-goal[i]):+.3f}°（允许 ±{math.degrees(limits[i]):g}°）' for i in outside)
                    speed=math.degrees(float(np.max(abs(self.controller.velocity))))
                    raise ArrivalTimeout(f'{label}未到位：{detail or "位置已在容差内，但尚未稳定"}；'
                                       f'最大估算速度 {speed:.3f}°/s（需≤1°/s持续0.5秒）')
                break
            deadline=max(deadline+PERIOD,time.monotonic());time.sleep(max(0,deadline-time.monotonic()))
        self.target=np.array(goal).copy()
        self.event('segment_end',phase=label)

    def connect(self,target,state):
        # All model/config/path work happens while still disabled.
        self.backend=self.backend_factory()
        self.home=self.zero_factory(self.backend,self.gravity,self.identity,self.args,self.journal)
        q,_,_=self.backend.read()
        for _ in range(3):
            time.sleep(PERIOD);measured,_,_=self.backend.read()
            if np.max(abs(measured-q))>math.radians(.2):raise RuntimeError('启动姿态仍在移动')
        opening=self.plans.get('WAKE',self.plans.get('00'))
        minimum=opening.duration if opening is not None and state=='ATTENTIVE' else 2.
        entry=prepare_entry(q,target,self.gravity,1.,hold_gripper=False,
                            minimum_duration=minimum,**DAILY_POSITION_LIMITS)
        self.event('positioning_plan',state=state,duration_s=entry.duration,**DAILY_POSITION_LIMITS)
        print(f'正在前往{ "待机" if state=="ATTENTIVE" else "零位" }，计划运动约 {entry.duration:.1f} 秒。',flush=True)
        measured,_,_=self.backend.read()
        if np.max(abs(measured-q))>math.radians(.2):raise RuntimeError('规划期间姿态发生变化')
        self.controller=Controller(self.gravity,initial=q,gripper_velocity_limit=math.radians(75.))
        self.controller.command(q,time.monotonic_ns(),q,np.zeros(7))
        if self.stop:raise StopRequested(self.stop)
        self.attempted=True;self.event('enable_requested');self.backend.enable(q);self.enabled=True
        # Let feedback/homing continue if storage alone fails after full enable.
        # tick() explicitly notices the failure before another ordinary command.
        self.journal.allow_failure=self.raw.allow_failure=True
        self.last_tick=None;self.last_gain=0.;self.last_command=None
        self.controller=Controller(self.gravity,initial=q,gripper_velocity_limit=math.radians(75.))
        self.segment('startup',target=q,duration=1.5,ramp=True)
        self.segment('approach_'+state,curve=entry)
        self.settle(target,state)

    def settle(self,target,state):
        arm_tolerance=math.radians(self.home.args.exit_zero_tolerance_deg) if state=='ZERO' else math.radians(1.)
        # Gripper closure repeatability is separate from six-axis pose accuracy.
        # Do not change commanded opening, encoder zero, or moving follow limits.
        tolerance=np.r_[np.full(6,arm_tolerance),GRIPPER_SETTLE_TOLERANCE]
        self.segment('settle_'+state,target=target,duration=8.,tolerance=tolerance)
        self.state=state;self.event('state',state=state)

    def execute_plan(self,action,state):
        self.state='MOVING_'+action
        for label,curve in self.plans[action].stages:
            self.segment(action+':'+label,curve=curve)
        self.settle(self.plans[action].end,state)

    def ensure_idle(self):
        if not self.enabled:self.connect(self.idle,'ATTENTIVE')
        elif self.state=='ZERO':self.execute_plan('WAKE','ATTENTIVE')
        elif self.state!='ATTENTIVE':raise RuntimeError('cannot start gesture in current state')

    def execute(self,action):
        if self.state=='PROTECTIVE_HOLD':
            self.event('action_rejected',action=action,reason='protective_hold_latched')
            print('保护保持中，动作不会自动恢复。可输入 status；quit 或 Ctrl+C 回零退出。',flush=True)
            return
        try:
            self._execute_action(action)
        except RecoverableControlError as error:
            self.enter_protective_hold(error,action)

    def enter_protective_hold(self,error,action):
        # Healthy-feedback recovery is one bounded brake, then a fixed anchor.
        # Never repeatedly re-anchor a failed hold or reuse a rejected target.
        if (not self.enabled or self.state in ('BRAKING','HOLD_RAMP','PROTECTIVE_HOLD') or self.homing
                or getattr(self,'last_gain',0.)<1. or self.last_command is None):
            raise error
        interrupted_state=self.state
        self.event('protective_hold_requested',action=action,interrupted_state=interrupted_state,
                   details=error.details)
        try:
            self.state='BRAKING';started=time.monotonic();stable=None
            while True:
                q=self.tick(None,label='protective_braking',recovery={})
                still=np.max(abs(self.controller.velocity))<=math.radians(1.)
                stable=(stable if stable is not None else time.monotonic()) if still else None
                if stable is not None and time.monotonic()-stable>=.5:break
                if time.monotonic()-started>5.:raise RuntimeError('recovery did not stop within 5s')
                time.sleep(PERIOD)
            anchor=q.copy();brake_kd=np.array(self.last_command['kd'])
            self.controller.target_lower=np.minimum(LOWER,anchor)-1e-8
            self.controller.target_upper=np.maximum(UPPER,anchor)+1e-8
            self.state='HOLD_RAMP';started=time.monotonic()
            while True:
                gain=min(1.,(time.monotonic()-started)/.5)
                self.tick(anchor,label='protective_hold_ramp',recovery=dict(anchor=anchor,gain=gain,brake_kd=brake_kd))
                if gain>=1:break
                time.sleep(PERIOD)
        except Exception as hold_error:
            try:self.event('protective_hold_failed',error=str(hold_error))
            except Exception:pass
            raise
        self.target=anchor.copy();self.state='PROTECTIVE_HOLD'
        stop=dict(action=action,interrupted_state=interrupted_state,target_rad=anchor.tolist(),
                  reason=str(error),details=error.details)
        self.protective_stops.append(stop)
        self.event('action_aborted',action=action,reason=str(error))
        self.event('protective_hold_entered',**stop)
        print('当前动作已中止：已完成阻尼减速并进入位置保护保持，电机继续使能。'
              '输入 status 查看；quit 或 Ctrl+C 回零退出。',flush=True)

    def hold_tick(self):
        try:
            self.tick(self.target,label='protective_hold' if self.state=='PROTECTIVE_HOLD' else 'hold',busy=False)
        except RecoverableControlError as error:
            self.enter_protective_hold(error,'hold')

    def _execute_action(self,action):
        self.event('action_requested',action=action)
        if action in ('IDLE','WAKE'):
            self.ensure_idle()
        elif action=='REST':
            if not self.enabled:self.connect(self.zero,'ZERO')
            elif self.state!='ZERO':self.execute_plan('REST','ZERO')
        elif action=='SLEEP':
            if self.enabled:
                if self.state!='ZERO':self.execute_plan('REST','ZERO')
                self.settle(self.zero,'ZERO')
                self.disconnect()
            else:
                # Verify a disabled arm really is at zero; never enable just to sleep.
                self.backend=self.backend_factory()
                q,_,_=self.backend.read()
                if np.max(abs(q[:6]))>math.radians(3.):raise ValueError('当前未在零位；先执行入睡')
                self.disconnect()
            self.state='SLEEP'
        elif action in self.plans:
            self.ensure_idle();self.execute_plan(action,'ATTENTIVE')
        else:raise ValueError('unknown daily action')
        self.event('action_completed',action=action,state=self.state)
        from daily_actions import NAMES
        number=f'{tuple(NAMES).index(action)+1:02d}'
        print(f'{number} {NAMES[action][0]} 完成，'+
              ('已失能。输入 08 唤醒；quit 退出。' if self.state=='SLEEP' else
               '保持位置。输入下个编号；quit 回零退出。'),flush=True)

    def disconnect(self):
        if self.backend is None:return
        errors=[]
        try:
            if self.attempted:
                # Storage failures must not skip the physical disable call.
                try:self.event('disable_requested')
                except Exception as error:errors.append(f'log: {error}')
                try:
                    confirmed=self.backend.disable()
                    if not confirmed:errors.append('disable not confirmed; use independent stop')
                    try:self.event('disable_completed',confirmed=confirmed)
                    except Exception as error:errors.append(f'log: {error}')
                except BaseException as error:errors.append(f'disable: {error}')
        finally:
            try:self.backend.close()
            except BaseException as error:errors.append(f'close: {error}')
            finally:
                self.backend=None;self.enabled=False;self.attempted=False;self.last_tick=None
        if errors:raise RuntimeError('; '.join(errors))

    def home_and_exit(self):
        if not self.enabled:return
        if self.stop=='abort':return
        self.homing=True;self.backend.returning_to_zero=True
        print('正在回零退出；再次 Ctrl+C 中断并失能。',flush=True)
        command=self.last_command or dict(kp=[0.]*7,kd=[0.]*7)
        try:
            completed=self.home.run(lambda:self.stop=='abort',initial_gravity_gain=self.last_gain,
                                    initial_kp=command['kp'][:6],initial_kd=command['kd'][:6])
            self.event('zero_return_completed',confirmed=bool(completed))
            if not completed:raise RuntimeError('zero return not completed')
        finally:self.backend.returning_to_zero=False


def run_session(args,initial_action=None,*,commands=None,backend_factory=None,zero_factory=prepare_daily_zero_return):
    if args.supported is not True:raise ValueError('试播需要 --supported')
    manifest,plans,gravity,identity=load_bundle(args.calibration,args.urdf,args.bundle)
    folder=ROOT/'build/daily-actions'/f'session-{time.time_ns()}';folder.mkdir(parents=True)
    journal=Journal(folder/'control.jsonl')
    try:raw=Journal(folder/'can.jsonl')
    except BaseException:journal.close();raise
    result=dict(success=False,hardware_accepted=False,folder=str(folder),errors=[],
                torque_budget_nm=CAPS.tolist())
    if commands is None:
        commands=queue.Queue();threading.Thread(target=read_commands,args=(commands,),daemon=True).start()
    session=Session(args,manifest,plans,gravity,identity,journal,raw,commands,backend_factory,zero_factory)
    handlers={}
    from daily_actions import catalog,resolve_action,NAMES
    _,actions=catalog()
    try:
        for sig in (signal.SIGINT,signal.SIGTERM):handlers[sig]=signal.signal(sig,session.request_stop)
        print('日常动作 · 8组',flush=True)
        for action in sorted(actions.values(),key=lambda item:item['file_order']):
            name=action['id'];duration=manifest['actions'][name]['duration_s']
            timing=f'{duration:.1f}s' if duration is not None else ('待机保持' if name=='IDLE' else '熟睡失能')
            print(f'{action["file_order"]:02d} {NAMES[name][0]}  {timing}',flush=True)
        print('输入编号播放；完成后保持。status 查看状态；quit/Ctrl+C 回零退出。',flush=True)
        session.event('torque_budget',torque_budget_nm=CAPS.tolist())
        if initial_action:session.execute(initial_action)
        while True:
            if session.stop:raise StopRequested(session.stop)
            try:word=commands.get_nowait().strip()
            except queue.Empty:word=''
            if word in ('quit','exit','eof','退出'):session.request_stop();raise StopRequested('quit')
            if word=='status':print(f'当前状态：{session.state}',flush=True)
            elif word:
                try:action=resolve_action(word,actions)['id']
                except ValueError as error:print(str(error),flush=True)
                else:session.execute(action)
            if session.enabled:session.hold_tick()
            time.sleep(PERIOD)
    except StopRequested:
        try:session.home_and_exit();result['success']=session.stop!='abort'
        except BaseException as error:result['errors'].append(str(error))
    except (TemperatureReturnRequested,JournalError) as error:
        result['reason']='temperature' if isinstance(error,TemperatureReturnRequested) else 'logging_error'
        result['errors'].append(str(error))
        if session.enabled:
            try:session.home_and_exit()
            except BaseException as home_error:result['errors'].append(f'zero return: {home_error}')
    except BaseException as error:result['errors'].append(f'{type(error).__name__}: {error}')
    finally:
        try:session.disconnect()
        except BaseException as error:result['errors'].append(str(error))
        for sig,handler in handlers.items():signal.signal(sig,handler)
        for writer in (journal,raw):
            try:writer.close()
            except BaseException as error:result['errors'].append(str(error))
        result['success']=result['success'] and not result['errors']
        result['protective_stops']=session.protective_stops
        (folder/'summary.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    return result
