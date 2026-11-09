"""Virtual-point choreography generation, sampled envelope scan, and real audit.

Default geometry is tool +X orbiting world vertical. An alternative axis is
available only through an explicit configuration and is never silently chosen.
"""
from __future__ import annotations
import argparse
import json
import math
from pathlib import Path
import numpy as np
import rs_dance_design as base
import rs_dance_pivot_kinematics as kin
from rs_dance_pivot_trajectory import _DenseCurve
from rs_dance_virtual_pivot_scan import VIRTUAL_CONFIG,config_dict,VirtualIK,scan_envelope,desired_rotation
from rs_dance_virtual_pivot_primitives import orientation_at


class VirtualPivotInfeasible(ValueError):
    def __init__(self,message,scan):super().__init__(message);self.scan=scan


class VirtualPivotSegment:
    def __init__(self,spec):
        self.config=config_dict(spec['config']);self.P_world_m=np.asarray(spec['P_world_m'],float)
        self.selected=dict(spec['selected'])
        if not all(math.isfinite(float(self.selected[k])) for k in ('cone_deg','pendulum_deg','spiral_inner_deg','roll_deg','sampled_envelope_cone_deg','envelope_fraction')):raise ValueError('non-finite virtual-pivot angular metadata')
        if self.selected['sampled_envelope_cone_deg']<=0 or min(self.selected['cone_deg'],self.selected['pendulum_deg'],self.selected['spiral_inner_deg'])<=0:raise ValueError('invalid virtual-pivot angles')
        self.velocity_limit=1.;self.acceleration_limit=2.
        self.gripper_velocity_limit=1.;self.gripper_acceleration_limit=2.;self.gripper_motion=False
        self.stages=[];self.roles=[];self.phase_ids=[]
        for chunk in spec['chunks']:
            if chunk['kind']=='dense_curve':curve=_DenseCurve(chunk)
            elif chunk['kind']=='quintic':curve=base.PoseSegment(chunk)
            elif chunk['kind']=='hold':curve=base.PoseSegment(dict(start_rad=chunk['q_rad'],end_rad=chunk['q_rad'],duration_s=chunk['duration_s']))
            else:raise ValueError('invalid virtual-pivot chunk')
            if self.stages:
                left=self.stages[-1][1]
                if max(np.max(abs(left.end-curve.start)),np.max(abs(left.at(left.duration)[1]-curve.at(0)[1])),
                       np.max(abs(left.acceleration_at(left.duration)-curve.acceleration_at(0))))>1e-7:
                    raise ValueError('virtual-pivot chunk seam is not C2')
            self.stages.append((chunk['role'],curve));self.roles.append(chunk['role']);self.phase_ids.append(chunk.get('phase_index'))
        if len(self.roles)<3 or self.roles[0]!='entry' or self.roles[-1]!='exit' or any(r not in ('entry','dance','freeze','exit') for r in self.roles):raise ValueError('invalid virtual-pivot phase roles')
        self.ends=np.cumsum([c.duration for _,c in self.stages]);self.duration=float(self.ends[-1])
        if abs(self.duration-spec['duration_s'])>1e-8:raise ValueError('virtual-pivot duration mismatch')
        self.phase_ranges_s=[];beat_start=0.
        if [p for p in self.phase_ids[1:-1] if p is not None]!=sorted(p for p in self.phase_ids[1:-1] if p is not None):raise ValueError('virtual phases are out of order')
        for index,(name,beats) in enumerate(zip(('two_cones','planar_pendulum','spiral','beat_freezes'),self.config['beat_sections'])):
            members=[i for i,p in enumerate(self.phase_ids) if p==index]
            if not members:raise ValueError('virtual pivot is missing a phase')
            first,last=members[0],members[-1];start=float(self.ends[first-1]);end=float(self.ends[last])
            self.phase_ranges_s.append(dict(name=name,start_s=start,end_s=end,start_beat=beat_start,end_beat=beat_start+beats,
                actual_bpm=beats*60/(end-start)))
            beat_start+=beats
        self.dance_interval_s=(float(self.ends[0]),float(self.ends[-2]));self.start=self.stages[0][1].start.copy();self.end=self.stages[-1][1].end.copy()
        self.lower=np.min([c.lower for _,c in self.stages],axis=0);self.upper=np.max([c.upper for _,c in self.stages],axis=0)
        self.peak_velocity=np.max([c.peak_velocity for _,c in self.stages],axis=0);self.peak_acceleration=np.max([c.peak_acceleration for _,c in self.stages],axis=0)
        if max(abs(self.lower[6]),abs(self.upper[6]))>1e-10:raise ValueError('virtual pivot must keep gripper closed')
        if max(self.peak_velocity)>1+1e-8 or max(self.peak_acceleration)>2+1e-8:raise ValueError('virtual pivot exceeds dynamics')
        for stamp in (0,self.duration):
            if max(np.max(abs(self.at(stamp)[1])),np.max(abs(self.acceleration_at(stamp))))>1e-7:raise ValueError('virtual pivot must stop smoothly at endpoints')
    def beat_at(self,t):
        t=float(np.clip(t,*self.dance_interval_s))
        phase=next((p for p in self.phase_ranges_s if t<=p['end_s']),self.phase_ranges_s[-1])
        return phase['start_beat']+(phase['end_beat']-phase['start_beat'])*(t-phase['start_s'])/(phase['end_s']-phase['start_s'])
    def _chunk(self,t):
        t=float(np.clip(t,0,self.duration));i=min(len(self.stages)-1,int(np.searchsorted(self.ends,t,side='right')))
        return self.stages[i][1],t-(self.ends[i-1] if i else 0.)
    def at(self,t):curve,t=self._chunk(t);return curve.at(t)
    def acceleration_at(self,t):curve,t=self._chunk(t);return curve.acceleration_at(t)
    def jerk_at(self,t):
        curve,t=self._chunk(t)
        if hasattr(curve,'jerk_at'):return curve.jerk_at(t)
        u=t/curve.duration
        return 60*(1-6*u+6*u*u)*curve.delta/curve.duration**3


def _transition(start,end,minimum,role):
    delta=abs(np.asarray(end)-start)
    duration=max(minimum,float(max(1.875*delta)),math.sqrt(float(max(10/math.sqrt(3)*delta/2))))*1.005
    return dict(kind='quintic',role=role,duration_s=duration,start_rad=np.asarray(start).tolist(),end_rad=np.asarray(end).tolist())


def _build_candidate(gravity,idle,cfg,scan,point,ring):
    solver=VirtualIK(gravity,idle,cfg);P=np.asarray(point['P_xyz_m']);cone=min(cfg['requested_cone_deg'],cfg['envelope_fraction']*ring['cone_deg'])
    selected=dict(cone_deg=cone,pendulum_deg=min(cfg['requested_pendulum_deg'],cfg['envelope_fraction']*ring['cone_deg']),
        spiral_inner_deg=min(cone,cfg['requested_spiral_inner_deg']*cone/cfg['requested_cone_deg']),
        roll_deg=ring['roll_deg'],sampled_envelope_cone_deg=ring['cone_deg'],envelope_fraction=cfg['envelope_fraction'])
    prep=solver.solve(P,orientation_at(0,solver.reference,cfg,selected),solver.seeds())
    if not prep['accepted']:raise ValueError('selected envelope fraction preparation failed: '+prep['reason'])
    prepared=np.asarray(prep['q_rad']);previous=prepared.copy();seconds_per_beat=60/cfg['bpm'];ends=np.cumsum(cfg['beat_sections'])
    # Continuous first three phrases, then four exact move-and-freeze beats.
    intervals=[]
    for phase_index,(start,end) in enumerate(zip(np.r_[0,ends[:2]],ends[:3])):
        pause_start=float(end-cfg['phase_pause_beats'])
        intervals.extend([(float(start),pause_start,'dance',phase_index),(pause_start,float(end),'freeze',phase_index)])
    quarter=cfg['beat_sections'][3]/4
    for n in range(4):
        start=ends[2]+n*quarter;move_end=start+quarter*(1-cfg['freeze_fraction'])
        intervals.extend([(float(start),float(move_end),'dance',3),(float(move_end),float(start+quarter),'freeze',3)])
    chunks=[];raw_end_error=0.;samples_count=0
    for start,end,role,phase_index in intervals:
        if role=='freeze':
            chunks.append(dict(kind='hold',role=role,phase_index=phase_index,duration_s=(end-start)*seconds_per_beat,q_rad=previous.tolist()));continue
        duration=(end-start)*seconds_per_beat;times=np.linspace(0,duration,max(4,math.ceil(duration/.02)+1));positions=[]
        for local in times:
            beat=start+local/seconds_per_beat;target=orientation_at(beat,solver.reference,cfg,selected)
            solved=solver.solve(P,target,[previous[:6]])
            if not solved['accepted']:raise ValueError(f'continuous IK at beat {beat:.3f}: '+solved['reason'])
            q=np.asarray(solved['q_rad'])
            if np.max(abs(q-previous))>.2:raise ValueError('continuous virtual IK jumped joint branch')
            if local==0 and np.max(abs(q-previous))<1e-5:q=previous.copy()
            previous=q;positions.append(q);samples_count+=1
        if abs(end-(ends[-1]-quarter*cfg['freeze_fraction']))<1e-9:
            raw_end_error=float(np.max(abs(previous-prepared)))
            if raw_end_error>1e-5:raise ValueError('virtual choreography did not close its initial joint branch')
            previous=prepared.copy();positions[-1]=prepared.copy()
        chunks.append(dict(kind='dense_curve',role=role,phase_index=phase_index,t_s=times.tolist(),q_rad=np.asarray(positions).tolist()))
    phase_factors=[]
    for phase_index in range(4):
        required=max(1.,*(1/base.Curve(c['t_s'],c['q_rad'],velocity_limit=1.,acceleration_limit=2.).scale
            for c in chunks if c['kind']=='dense_curve' and c['phase_index']==phase_index))
        phase_factors.append(required*1.005)
    if cfg['timing_mode']=='uniform':phase_factors=[max(phase_factors)]*4
    for chunk in chunks:
        factor=phase_factors[chunk['phase_index']]
        if chunk['kind']=='dense_curve':chunk['t_s']=(np.asarray(chunk['t_s'])*factor).tolist()
        else:chunk['duration_s']*=factor
    chunks=[_transition(idle,prepared,cfg['entry_min_duration_s'],'entry'),*chunks,_transition(prepared,idle,cfg['exit_min_duration_s'],'exit')]
    duration=sum(c['t_s'][-1] if c['kind']=='dense_curve' else c['duration_s'] for c in chunks)
    if duration>cfg['max_duration_s']:raise ValueError('dynamic retiming exceeds configured maximum duration')
    stage=dict(kind='virtual_pivot',label='virtual_point_orientation_phrases',config=cfg,P_world_m=P.tolist(),selected=selected,duration_s=duration,chunks=chunks)
    plan=VirtualPivotSegment(stage);dance_duration=plan.dance_interval_s[1]-plan.dance_interval_s[0]
    extra=dict(virtual_pivot_design=dict(config=cfg,P_world_m=P.tolist(),virtual_offset_m=cfg['tcp_offset_m'],model_tcp_offset_m=[0.,0.,0.],
        model_frame='gripper_end',pointing_axis_local=[1.,0.,0.],cone_axis_reference=cfg['cone_axis'],
        cone_axis_world=[0.,0.,1.] if cfg['cone_axis']=='world_z' else solver.reference[:,0].tolist(),
        reference_rotation=solver.reference.tolist(),selected=selected,requested_cone_deg=cfg['requested_cone_deg'],actual_cone_deg=cone,
        requested_bpm=cfg['bpm'],actual_bpm=cfg['beats']*60/dance_duration,actual_bpm_kind='average_over_dance',
        variable_bpm=cfg['timing_mode']=='per_phrase',phase_ranges_s=plan.phase_ranges_s,phase_time_factors=phase_factors,
        actual_duration_s=duration,dance_duration_s=dance_duration,
        dance_interval_s=list(plan.dance_interval_s),entry_exit_outside_beats=True,recommended_playback_speed=.3,
        raw_dance_end_joint_error_rad=raw_end_error,ik_samples=samples_count,ik_seed='previous sample during dance; deterministic multiple seeds for preparation',
        collision_identity=solver.collision.identity,collision_checked=True,collision_scope='triangle-mesh robot self-collision; environment not represented',
        physical_tool_calibrated=False,hardware_accepted=False,scan_summary=scan['points']))
    validate_virtual_pivot(plan,gravity,idle,extra)
    return [stage],extra,['进入准备姿态后，工具轴围绕声明的锥轴完成两圈','保持虚拟P，做过冲钟摆和扩张收回的螺旋','四次定格，舞段末回准备姿态，再返回idle']


def build_recipe(gravity,idle,config=None,*,scan=None):
    cfg=config_dict(config)
    if scan is None and cfg['cone_axis']=='world_z':
        raise VirtualPivotInfeasible('Strict world-Z generation requires an explicit verified scan. Run the offline --scan command; the recorded finite search is build/dance-virtual-pivot/world-z-scan.json. No alternate axis is selected automatically.',dict(schema='rebuddy.virtual_pivot_scan.v1',config=cfg,continuous_path_verified=False,hardware_accepted=False))
    scan=scan if scan is not None else scan_envelope(gravity,idle,cfg)
    if config_dict(scan['config'])!=cfg:raise ValueError('scan configuration differs from requested choreography')
    candidates=[(p,r) for p in scan['points'] for r in p['complete_sampled_rings']]
    candidates.sort(key=lambda item:item[1]['cone_deg'],reverse=True);failures=[]
    for point,ring in candidates:
        try:return _build_candidate(gravity,np.asarray(idle,float),cfg,scan,point,ring)
        except ValueError as error:failures.append(dict(point_id=point['point_id'],ring=ring,reason=str(error)))
    scan['trajectory_candidate_failures']=failures
    raise VirtualPivotInfeasible('No complete verified choreography found for the configured cone axis/P/offset/grid. This is a finite search, not a proof of global unreachability; inspect the saved scan or explicitly change configuration.',scan)


def validate_virtual_pivot(plan,gravity,idle,spec):
    design=spec['virtual_pivot_design'];cfg=config_dict(design['config']);segment=plan if isinstance(plan,VirtualPivotSegment) else plan.stages[0][1]
    if not isinstance(segment,VirtualPivotSegment) or segment.config!=cfg or segment.selected!=design['selected']:raise ValueError('virtual pivot metadata differs from executable segment')
    chosen=segment.selected
    if (chosen['envelope_fraction']!=cfg['envelope_fraction']
        or abs(chosen['cone_deg']-min(cfg['requested_cone_deg'],cfg['envelope_fraction']*chosen['sampled_envelope_cone_deg']))>1e-9
        or abs(chosen['pendulum_deg']-min(cfg['requested_pendulum_deg'],cfg['envelope_fraction']*chosen['sampled_envelope_cone_deg']))>1e-9):raise ValueError('virtual pivot amplitude is inconsistent with sampled-envelope fraction')
    dance_duration=segment.dance_interval_s[1]-segment.dance_interval_s[0]
    expected=dict(requested_cone_deg=cfg['requested_cone_deg'],actual_cone_deg=chosen['cone_deg'],requested_bpm=cfg['bpm'],actual_bpm=cfg['beats']*60/dance_duration,
        actual_duration_s=plan.duration,dance_duration_s=dance_duration,recommended_playback_speed=.3)
    if any(not math.isfinite(float(design[k])) or abs(float(design[k])-value)>1e-8 for k,value in expected.items()):raise ValueError('virtual pivot display/timing metadata is inconsistent')
    if design['virtual_offset_m']!=cfg['tcp_offset_m'] or design['model_tcp_offset_m']!=[0.,0.,0.] or design['cone_axis_reference']!=cfg['cone_axis']:
        raise ValueError('virtual pivot offset or axis metadata is inconsistent')
    if not np.allclose(plan.start,idle,rtol=0,atol=1e-8) or not np.allclose(plan.end,idle,rtol=0,atol=1e-8):raise ValueError('virtual choreography must start/end at idle')
    solver=VirtualIK(gravity,idle,cfg);solver.collision.validate_identity(design['collision_identity'])
    expected_factors=[(phase['end_s']-phase['start_s'])/((phase['end_beat']-phase['start_beat'])*60/cfg['bpm']) for phase in segment.phase_ranges_s]
    if not np.allclose(design['phase_time_factors'],expected_factors,rtol=0,atol=1e-8):raise ValueError('virtual-pivot timing factors differ from actual phase durations')
    if design['phase_ranges_s']!=segment.phase_ranges_s or design['variable_bpm']!=(cfg['timing_mode']=='per_phrase') or design['actual_bpm_kind']!='average_over_dance':raise ValueError('virtual-pivot phrase timing metadata mismatch')
    P=np.asarray(design['P_world_m']);dance_start,dance_end=segment.dance_interval_s
    if not np.array_equal(P,segment.P_world_m) or not np.allclose(design['dance_interval_s'],[dance_start,dance_end],rtol=0,atol=1e-8):raise ValueError('virtual pivot fixed point or dance interval mismatch')
    errors=[];angles=[];positions=[];all_q=[];checks=[];polars=[];pendulum_angles=[];pendulum_planes=[];count=0
    times=np.unique(np.r_[np.linspace(0,plan.duration,math.ceil(plan.duration/.02)+1),segment.ends])
    for stamp in times:
        q,_=plan.at(float(stamp));collision=solver.collision.check(q);count+=1
        if collision['colliding']:raise ValueError('virtual trajectory self collision: '+str(collision['pairs']))
        checks.append(kin._check_workspace(solver.parts,q,cfg,solver.workspace,solver.lower,solver.upper));all_q.append(q)
        if dance_start<=stamp<=dance_end:
            position,rotation=kin._pose(solver.parts,q,cfg);errors.append(float(np.linalg.norm(position-P)))
            beat=segment.beat_at(stamp);desired=orientation_at(beat,solver.reference,cfg,segment.selected)
            angles.append(math.degrees(float(np.linalg.norm(solver.pin.log3(desired.T@rotation)))))
            positions.append(kin._pose(solver.parts,q,dict(cfg,tcp_offset_m=[0.,0.,0.]))[0])
            direction=rotation[:,0];axis=np.array([0.,0.,1.]) if cfg['cone_axis']=='world_z' else solver.reference[:,0]
            transverse=np.array([1.,0.,0.]) if cfg['cone_axis']=='world_z' else solver.reference[:,1]
            polars.append(math.degrees(math.acos(float(np.clip(direction@axis,-1,1)))))
            bounds=np.cumsum(cfg['beat_sections'])
            if bounds[0]<=beat<=bounds[1]:
                pendulum_angles.append(math.degrees(math.atan2(float(direction@transverse),float(direction@axis))))
                pendulum_planes.append(abs(float(direction@np.cross(axis,transverse))))
    if max(errors)>cfg['position_tolerance_m'] or max(angles)>cfg['orientation_tolerance_deg']:raise ValueError('virtual choreography interpolation violates fixed-point or orientation tolerance')
    if max(polars)>segment.selected['sampled_envelope_cone_deg']+.01:raise ValueError('overshoot exceeds sampled angular envelope')
    for role,curve in segment.stages:
        if role=='freeze' and (np.max(curve.peak_velocity)>1e-10 or np.max(curve.peak_acceleration)>1e-10):raise ValueError('virtual freeze is not stationary')
    dynamics=base._validate_plan(plan,gravity);checks=np.asarray(checks)
    return dict(virtual_pivot=dict(P_world_m=P.tolist(),virtual_offset_m=cfg['tcp_offset_m'],model_tcp_offset_m=[0.,0.,0.],
        cone_axis_reference=cfg['cone_axis'],requested_cone_deg=cfg['requested_cone_deg'],actual_cone_deg=segment.selected['cone_deg'],
        position_error_max_m=max(errors),orientation_error_max_deg=max(angles),model_tcp_span_m=np.ptp(positions,axis=0).tolist(),
        joint_ranges_deg=np.degrees(np.ptp(all_q,axis=0)).tolist(),dance_interval_s=[dance_start,dance_end],
        variable_bpm=cfg['timing_mode']=='per_phrase',phase_ranges_s=segment.phase_ranges_s,
        max_polar_excursion_deg=max(polars),peak_sampled_envelope_fraction=max(polars)/segment.selected['sampled_envelope_cone_deg'],
        pendulum_signed_min_deg=min(pendulum_angles),pendulum_signed_max_deg=max(pendulum_angles),pendulum_plane_error_max=max(pendulum_planes),
        total_duration_s=plan.duration,dance_duration_s=dance_end-dance_start,actual_bpm=cfg['beats']*60/(dance_end-dance_start),
        minimum_joint_margin_deg=math.degrees(float(min(checks[:,1]))),scaled_6d_sigma_min=float(min(checks[:,2])),
        collision_checked=True,collision_samples=count,environment_collision_checked=False,
        inverse_dynamics_peak_nm=dynamics['inverse_dynamics_peak_nm'],physical_tool_calibrated=False))


def main(argv=None):
    from rs_gravity_record import DEFAULT_CALIBRATION,DEFAULT_URDF,preflight
    parser=argparse.ArgumentParser(description='Offline virtual-pivot scan/generation; no CAN')
    parser.add_argument('--scan',action='store_true');parser.add_argument('--scan-input',type=Path);parser.add_argument('--config',type=Path);parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--calibration',type=Path,default=DEFAULT_CALIBRATION);parser.add_argument('--urdf',type=Path,default=DEFAULT_URDF)
    args=parser.parse_args(argv);cfg=config_dict(json.loads(args.config.read_text()) if args.config else None)
    gravity,identity=preflight(args.calibration,args.urdf);idle,*_=base._anchors(identity)
    scan=None
    if args.scan or (not args.scan_input and cfg['cone_axis']!='world_z'):
        scan=scan_envelope(gravity,idle,cfg,identity,progress=lambda p:print(json.dumps(p,ensure_ascii=False),flush=True))
    elif args.scan_input:
        scan=json.loads(args.scan_input.read_text())
        if scan.get('schema')!='rebuddy.virtual_pivot_scan.v1':raise ValueError('invalid saved scan schema')
        for key in ('calibration_sha256','urdf_sha256'):
            if scan.get('identity',{}).get(key)!=identity[key]:raise ValueError('saved scan model/calibration identity changed')
        VirtualIK(gravity,idle,cfg).collision.validate_identity(scan['collision_identity'])
    exit_code=0
    if args.scan:payload=scan
    else:
        try:
            stages,extra,intentions=build_recipe(gravity,idle,cfg,scan=scan)
            payload=dict(stages=stages,**extra,intentions=intentions,identity=identity,scan=scan)
        except VirtualPivotInfeasible as error:payload=error.scan;payload['generation_error']=str(error);exit_code=2
    args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(payload,ensure_ascii=False,indent=2)+'\n')
    print(f'Saved offline result: {args.output}');return exit_code


if __name__=='__main__':raise SystemExit(main())
