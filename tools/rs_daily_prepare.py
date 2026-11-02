"""Build reviewable daily plans from explicit CSV sources; never opens CAN."""
from pathlib import Path
import csv
import json
import math

import numpy as np

from rs_gravity_record import ROOT, CAPS, PERIOD, digest, load_idle_pose, preflight
from rs_gravity_play import load_clip, prepare_entry
from rs_probe_trial import Curve
from rs_record_playback import LOWER, UPPER

BUNDLE = ROOT/'record/daily_ready'
SOURCES = {
    'PROBE': '20261101T072951Z-1793518191706012636/probe.csv',
    'NUZZLE': '20261101T074356Z-1793519036406134122/nuzzle.csv',
    'TILT': '20261101T074356Z-1793519036406134122/tilt.csv',
    'HAPPY': '20261101T080255Z-1793520175597405200/happy.csv',
}
FIT_TOLERANCE = math.radians(.5)
HAPPY_LIMITS = dict(velocity_limit=.5,acceleration_limit=1.)
DAILY_POSITION_LIMITS = dict(velocity_limit=.5,acceleration_limit=1.)
HAPPY_FAST_LIMITS = dict(velocity_limit=1.,acceleration_limit=2.)
# PROBE body runs 25% faster; acceleration scales with speed squared.
PROBE_FAST_LIMITS = dict(velocity_limit=1.25,acceleration_limit=3.125)
HAPPY_FAST_STAGES = {'recorded_rise','recorded_return','direct_return_to_idle'}
GESTURE_ACTIONS = {'PROBE','NUZZLE','TILT','HAPPY'}
GESTURE_FAST_STAGES = HAPPY_FAST_STAGES | {'recorded_gesture'}
HAPPY_CLAP = dict(amplitude_deg=120.,half_s=.9,j1_sway_deg=5.)


class Clap:
    """Two C2 opening pulses with optional synchronized J1 sway."""
    def __init__(self, pose, amplitude_deg=120., half_s=.9,j1_sway_deg=0.):
        self.start=np.asarray(pose,float).copy();self.end=self.start.copy()
        self.amplitude=math.radians(amplitude_deg);self.half_s=half_s
        if not math.isfinite(half_s) or half_s<=0 or not 0<amplitude_deg<=120:
            raise ValueError('invalid gripper gesture')
        if self.start[6]<0 or self.start[6]+self.amplitude>UPPER[6]:
            raise ValueError('gripper gesture outside calibrated command range')
        self.duration=4*half_s
        # Reference rs_mit_split.yaml: independent gripper target window 12 deg,
        # controller uses 90%; torque is independently checked against CAPS.
        self.gripper_following_limit=math.radians(10.8)
        self.peak_velocity=1.875*self.amplitude/half_s
        self.peak_acceleration=10/math.sqrt(3)*self.amplitude/half_s**2
        # Below the reference RS gripper budgets (300 deg/s, 1200 deg/s²).
        if self.peak_velocity>math.radians(300) or self.peak_acceleration>math.radians(1200):
            raise ValueError('gripper gesture exceeds reference dynamic budget')
        if not math.isfinite(j1_sway_deg) or not 0<=j1_sway_deg<=5.:
            raise ValueError('invalid J1 sway amplitude')
        self.j1_sway=math.radians(j1_sway_deg)
        if self.start[0]-self.j1_sway<LOWER[0] or self.start[0]+self.j1_sway>UPPER[0]:
            raise ValueError('J1 sway outside joint bounds')
        # One center -> +A -> -A -> center cycle per open/close pair.
        # 30/40/30% timing bounds speed/acceleration without lengthening the clap.
        self.sway_times=np.array([0.,.6,1.4,2.])*half_s
        self.sway_levels=np.array([0.,1.,-1.,0.])*self.j1_sway
        intervals=np.diff(self.sway_times);steps=abs(np.diff(self.sway_levels))
        self.j1_peak_velocity=float(max(1.875*steps/intervals))
        self.j1_peak_acceleration=float(max(10/math.sqrt(3)*steps/intervals**2))
        if self.j1_peak_velocity>.5 or self.j1_peak_acceleration>2.:
            raise ValueError('J1 sway exceeds dynamic budget')

    def at(self, stamp):
        stamp=float(np.clip(stamp,0,self.duration))
        phase=min(3,int(stamp/self.half_s));u=(stamp-phase*self.half_s)/self.half_s
        s=10*u**3-15*u**4+6*u**5;v=30*u**2*(1-u)**2/self.half_s
        q=self.start.copy();dq=np.zeros(7)
        q[6]+=self.amplitude*(s if phase%2==0 else 1-s)
        dq[6]=self.amplitude*v*(1 if phase%2==0 else -1)
        elapsed=stamp%(2*self.half_s)
        leg=min(2,int(np.searchsorted(self.sway_times,elapsed,side='right'))-1)
        duration=self.sway_times[leg+1]-self.sway_times[leg]
        u=(elapsed-self.sway_times[leg])/duration
        delta=self.sway_levels[leg+1]-self.sway_levels[leg]
        q[0]+=self.sway_levels[leg]+delta*(10*u**3-15*u**4+6*u**5)
        dq[0]=delta*30*u**2*(1-u)**2/duration
        return q,dq


class Plan:
    def __init__(self, stages):
        self.stages=stages
        self.ends=np.cumsum([c.duration for _,c in stages])
        self.duration=float(self.ends[-1])
        self.start=stages[0][1].start;self.end=stages[-1][1].end
        for (_,left),(_,right) in zip(stages,stages[1:]):
            if np.max(abs(left.end-right.start))>1e-8:
                raise ValueError('daily stages are discontinuous')

    def at(self, stamp):
        index=min(len(self.stages)-1,int(np.searchsorted(self.ends,stamp,side='right')))
        return self.stages[index][1].at(stamp-(self.ends[index-1] if index else 0.))


def fit_recording(t,q,**limits):
    """Reduce knots, then bound reconstruction error at every measured sample."""
    t=np.asarray(t)-t[0];q=np.asarray(q)
    # Start with temporal anchors; add the worst residual until <=0.5 degrees.
    selected=set(np.searchsorted(t,np.arange(0,t[-1],.5)).tolist())|{0,len(t)-1}
    for _ in range(len(t)):
        indices=sorted(selected)
        curve=Curve(t[indices],q[indices],**limits)
        residual=np.max(abs(curve.spline(t)-q),axis=1)
        worst=int(np.argmax(residual))
        if residual[worst]<=FIT_TOLERANCE:
            return curve,dict(t_s=t[indices].tolist(),q_rad=q[indices].tolist(),
                              max_sample_fit_error_deg=math.degrees(float(residual[worst])),
                              input_samples=len(t),knots=len(indices))
        if worst in selected:raise ValueError('cannot fit recording within tolerance')
        selected.add(worst)
    raise ValueError('fit failed')


def validate_gravity(plan,gravity):
    for stamp in np.linspace(0,plan.duration,math.ceil(plan.duration/PERIOD)+1):
        q,_=plan.at(stamp)
        if np.any(abs(gravity(q[:6]))>CAPS[:6]):
            raise ValueError('daily path gravity exceeds torque budget')


def load_bundle(calibration,urdf,folder=BUNDLE):
    gravity,identity=preflight(calibration,urdf)
    manifest=json.loads((folder/'manifest.json').read_text())
    if manifest.get('schema')!='rebuddy.daily.prepared.v1':raise ValueError('invalid daily bundle')
    for key in ('calibration_sha256','urdf_sha256'):
        if manifest[key]!=identity[key]:raise ValueError('daily bundle identity changed')
    if digest(ROOT/manifest['idle_source'])!=manifest['idle_sha256']:
        raise ValueError('saved idle changed; prepare daily bundle again')
    plans={}
    for action,item in manifest['actions'].items():
        if item.get('source') and digest(ROOT/item['source'])!=item['source_sha256']:
            raise ValueError('daily source changed')
        if not item.get('plan_file'):continue
        path=folder/item['plan_file']
        if path.parent!=folder or digest(path)!=item['plan_sha256']:raise ValueError('daily plan changed')
        spec=json.loads(path.read_text());stages=[]
        for stage in spec['stages']:
            if stage['kind']=='clap':
                curve=Clap(stage['pose_rad'],stage['amplitude_deg'],stage['half_s'],
                           stage.get('j1_sway_deg',0.))
            elif stage['kind']=='curve':
                limits=stage.get('limits',{})
                # Interactive gestures and positioning use separate profiles.
                expected = (HAPPY_FAST_LIMITS if stage['label'] in GESTURE_FAST_STAGES
                            else HAPPY_LIMITS)
                if action=='PROBE' and stage['label']=='recorded_gesture':
                    expected=PROBE_FAST_LIMITS
                positioning=action in ('REST','WAKE') and stage['label']==action.lower()
                if positioning:expected=DAILY_POSITION_LIMITS
                if limits and (not (action in GESTURE_ACTIONS or positioning) or limits!=expected):
                    raise ValueError('unrecognized daily speed profile')
                curve=Curve(stage['t_s'],stage['q_rad'],**limits)
                factor=stage.get('speed',1.)
                if not 0<factor<=1:raise ValueError('invalid speed')
                curve.scale*=factor;curve.duration/=factor
            else:raise ValueError('unknown daily stage')
            stages.append((stage['label'],curve))
        plan=Plan(stages)
        if abs(plan.duration-item['duration_s'])>1e-6:raise ValueError('daily plan duration changed')
        validate_gravity(plan,gravity);plans[action]=plan
    return manifest,plans,gravity,identity


def prepare_bundle(calibration,urdf,folder=BUNDLE):
    gravity,identity=preflight(calibration,urdf)
    idle_path=ROOT/'record/gravity/presets/idle.json'
    idle=load_idle_pose(idle_path,{**identity,'mock':False})
    anchor=np.array(idle['q_rad']);zero=anchor.copy();zero[:6]=0.
    folder.mkdir(parents=True,exist_ok=True)
    manifest=dict(schema='rebuddy.daily.prepared.v1',hardware_accepted=False,
                  calibration_sha256=identity['calibration_sha256'],urdf_sha256=identity['urdf_sha256'],
                  idle_source=str(idle_path.relative_to(ROOT)),idle_sha256=digest(idle_path),
                  idle_rad=anchor.tolist(),zero_rad=zero.tolist(),
                  screen_sync=False,actions={})

    def save(action,stages,specs,details):
        plan=Plan(stages);validate_gravity(plan,gravity)
        path=folder/(action.lower()+'.json')
        path.write_text(json.dumps(dict(stages=specs),ensure_ascii=False,indent=2)+'\n')
        # Derived command samples, explicitly different from raw teaching CSVs.
        csv_path=folder/(action.lower()+'.csv')
        with csv_path.open('w',newline='') as stream:
            writer=csv.writer(stream);writer.writerow(['t_s']+[f'target_q{i}_rad' for i in range(1,8)])
            for stamp in np.r_[np.arange(0,plan.duration,PERIOD),plan.duration]:
                writer.writerow([stamp,*plan.at(stamp)[0]])
        manifest['actions'][action]=dict(**details,duration_s=plan.duration,
            plan_file=path.name,plan_sha256=digest(path),preview_csv=csv_path.name,
            preview_sha256=digest(csv_path),stages=[dict(name=n,duration_s=c.duration) for n,c in stages],
            hardware_accepted=False)

    for action,relative in SOURCES.items():
        source=ROOT/'record/gravity'/relative
        t,q,info=load_clip(source,identity,recover_interrupted=action in ('PROBE','HAPPY'))
        summary=json.loads((source.parent/'summary.json').read_text())
        clip=next(c for c in summary['clips']if c['file']==source.name)
        if clip.get('manual_end_ns'):
            # Source t was normalized by load_clip. Read matching monotonic stamps.
            stamps=np.array([int(r['monotonic_ns']) for r in csv.DictReader(source.open())])
            keep=stamps<=clip['manual_end_ns'];t,q=t[keep],q[keep]
        # Trim only leading/trailing stillness, preserve .5 seconds at each end.
        moving_from_start=np.flatnonzero(np.max(abs(q-q[0]),axis=1)>math.radians(.3))
        moving_to_end=np.flatnonzero(np.max(abs(q-q[-1]),axis=1)>math.radians(.3))
        if not len(moving_from_start) or not len(moving_to_end):raise ValueError('gesture has no motion')
        lo=max(0,int(np.searchsorted(t,t[moving_from_start[0]]-.5)))
        hi=min(len(t),int(np.searchsorted(t,t[moving_to_end[-1]]+.5))+1)
        kept_range=[float(t[lo]),float(t[hi-1])];t,q=t[lo:hi],q[lo:hi].copy();t-=t[0]
        q[:,6]=anchor[6]
        stages=[];specs=[]
        limits=HAPPY_LIMITS if action in GESTURE_ACTIONS else {}
        def bridge(label,a,b):
            c=(Curve([0.,1.],[a,b],minimum_duration=2.,**limits) if limits else
               prepare_entry(a,b,gravity,1.,hold_gripper=False))
            stages.append((label,c));specs.append(dict(kind='curve',label=label,t_s=[0.,1.],q_rad=[a.tolist(),b.tolist()],
                                                     speed=c.scale/Curve([0.,1.],[a,b],**limits).scale))
            if limits:specs[-1]['limits']=limits
        bridge('idle_to_recording',anchor,q[0])
        parts=[(t,q,'recorded_gesture')]
        if action=='HAPPY':
            peak=int(np.argmax(np.linalg.norm(q[:,:6]-anchor[:6],axis=1)))
            if not 2<=peak<len(t)-2:raise ValueError('happy peak is at an endpoint')
            # The taught return begins with several seconds of holding/noise.
            # After the clap, return directly to idle instead of replaying it.
            parts=[(t[:peak+1],q[:peak+1],'recorded_rise')]
        fit_errors=[]
        for i,(part_t,part_q,label) in enumerate(parts):
            body_limits=HAPPY_FAST_LIMITS if action in GESTURE_ACTIONS else limits
            if action=='PROBE':body_limits=PROBE_FAST_LIMITS
            c,fit=fit_recording(part_t,part_q,**body_limits);stages.append((label,c))
            specs.append(dict(kind='curve',label=label,**fit));fit_errors.append(fit['max_sample_fit_error_deg'])
            if body_limits:specs[-1]['limits']=body_limits
            if action=='HAPPY' and i==0:
                clap=Clap(part_q[-1],**HAPPY_CLAP);stages.append(('gripper_open_close_twice',clap))
                specs.append(dict(kind='clap',label=stages[-1][0],pose_rad=part_q[-1].tolist(),**HAPPY_CLAP))
        if action=='HAPPY':
            start=stages[-1][1].end
            returning=Curve([0.,1.],[start,anchor],**HAPPY_FAST_LIMITS)
            stages.append(('direct_return_to_idle',returning))
            specs.append(dict(kind='curve',label='direct_return_to_idle',t_s=[0.,1.],
                              q_rad=[start.tolist(),anchor.tolist()],limits=HAPPY_FAST_LIMITS))
            info['return_policy']='C2 direct return immediately after clap; recorded return omitted'
            info['fitted_source_seconds']=[kept_range[0],kept_range[0]+float(t[peak])]
        else:
            bridge('generated_return_to_idle',q[-1],anchor)
        info['source']=str(source.relative_to(ROOT))
        save(action,stages,specs,dict(**info,kept_source_seconds=kept_range,
             max_sample_fit_error_deg=max(fit_errors),entry='ATTENTIVE',exit='ATTENTIVE_ENABLED',
             generated_return=True,gripper=f"120 motor degrees, two {2*HAPPY_CLAP['half_s']:g}-second open-close cycles" if action=='HAPPY' else 'idle opening'))
    for action,a,b in [('REST',anchor,zero),('WAKE',zero,anchor)]:
        curve=prepare_entry(a,b,gravity,1.,hold_gripper=False,**DAILY_POSITION_LIMITS)
        base=Curve([0.,1.],[a,b],**DAILY_POSITION_LIMITS)
        save(action,[(action.lower(),curve)],[dict(kind='curve',label=action.lower(),t_s=[0.,1.],q_rad=[a.tolist(),b.tolist()],speed=curve.scale/base.scale,limits=DAILY_POSITION_LIMITS)],
             dict(source=None,entry='ATTENTIVE' if action=='REST' else 'ZERO_DISABLED',
                  exit='ZERO_ENABLED' if action=='REST' else 'ATTENTIVE_ENABLED',recording_required=False))
    manifest['actions']['IDLE']=dict(source=None,recording_required=False,exit='ATTENTIVE_ENABLED',duration_s=None)
    manifest['actions']['SLEEP']=dict(source=None,recording_required=False,entry='ZERO_ENABLED',exit='ZERO_DISABLED',duration_s=None)
    (folder/'manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n')
    return manifest
