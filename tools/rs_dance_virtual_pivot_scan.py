"""Offline, sampled reachability audit for a virtual point beyond tool +X.

A failed grid sample means the listed deterministic seeds did not yield an
accepted solution; it is not a proof of global unreachability. Collision tests
cover the robot's original collision meshes, not its surrounding environment.
"""
from __future__ import annotations
import copy
import math
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
import rs_dance_pivot_kinematics as kin
from rs_dance_virtual_collision import CollisionAudit
from rs_record_playback import LOWER,UPPER

VIRTUAL_CONFIG=dict(bpm=100.,beats=16,timing_mode='per_phrase',beat_sections=[4.,4.,4.,4.],sample_period_s=.02,
    tcp_offset_m=[.20,0.,0.],cone_axis='world_z',pointing_axis_local=[1.,0.,0.],
    P_world_m=None,P_candidates_m=[[.1,0.,.75],[.2,0.,.75],[0.,0.,.70],[-.1,0.,.70]],
    scan_cone_deg=[20.,30.,40.,50.,60.,70.],scan_azimuth_deg=list(np.arange(0.,360.,45.)),
    scan_roll_deg=[-30.,0.,30.],max_seed_attempts=7,preparation_seeds_rad=[],
    requested_cone_deg=50.,envelope_fraction=.75,overshoot_ratio=.03,
    phase_ramp_fraction=.15,requested_pendulum_deg=60.,requested_spiral_inner_deg=10.,phase_pause_beats=.15,freeze_fraction=.25,
    entry_min_duration_s=2.,exit_min_duration_s=2.,
    workspace_reach_fraction=.7,workspace_inner_fraction=.15,workspace_min_z_m=.15,
    joint_margin_deg=10.,jac_characteristic_length_m=.3,jac_sigma_min=.08,jac_condition_max=40.,
    position_tolerance_m=.002,orientation_tolerance_deg=.2,max_duration_s=180.)


def config_dict(config=None):
    cfg=copy.deepcopy(VIRTUAL_CONFIG)
    if config:
        if set(config)-set(cfg):raise ValueError('unknown virtual-pivot configuration field')
        cfg.update(copy.deepcopy(config))
    if cfg['timing_mode'] not in ('uniform','per_phrase'):raise ValueError('unknown virtual pivot timing mode')
    if cfg['position_tolerance_m']>.002:raise ValueError('virtual point tolerance cannot exceed 2 mm')
    if cfg['cone_axis'] not in ('world_z','idle_tool'):raise ValueError('cone_axis must explicitly name world_z or idle_tool')
    if cfg['pointing_axis_local']!=[1.,0.,0.]:raise ValueError('virtual pivot uses the confirmed model tool +X axis')
    if np.asarray(cfg['tcp_offset_m']).shape!=(3,) or not np.isfinite(cfg['tcp_offset_m']).all():raise ValueError('invalid virtual point offset')
    if abs(cfg['tcp_offset_m'][1])+abs(cfg['tcp_offset_m'][2])>1e-12 or cfg['tcp_offset_m'][0]<0:raise ValueError('virtual point offset must lie on tool +X; zero selects model TCP')
    if cfg['sample_period_s']!=.02:raise ValueError('RS controller cadence remains 50 Hz')
    for field in ('bpm','requested_cone_deg','joint_margin_deg','jac_characteristic_length_m','jac_sigma_min','jac_condition_max','position_tolerance_m','orientation_tolerance_deg','max_duration_s'):
        if not math.isfinite(cfg[field]) or cfg[field]<=0:raise ValueError('invalid virtual-pivot numeric setting')
    if not 0<cfg['workspace_inner_fraction']<cfg['workspace_reach_fraction']<=.7:raise ValueError('invalid operational envelope')
    if not math.isfinite(cfg['workspace_min_z_m']) or cfg['workspace_min_z_m']<0:raise ValueError('invalid workspace floor')
    if not 0<cfg['phase_ramp_fraction']<=.5 or not 0<cfg['freeze_fraction']<1:raise ValueError('invalid phase timing or spiral fraction')
    if any(not math.isfinite(cfg[k]) or cfg[k]<=0 for k in ('requested_pendulum_deg','requested_spiral_inner_deg','phase_pause_beats')):raise ValueError('invalid pendulum/spiral/pause setting')
    if cfg['phase_pause_beats']>=min(cfg['beat_sections'][:3]):raise ValueError('phase pause must be shorter than motion section')
    if any(not math.isfinite(cfg[k]) or cfg[k]<=0 for k in ('entry_min_duration_s','exit_min_duration_s')):raise ValueError('invalid preparation timing')
    if not 0<cfg['envelope_fraction']<1 or not 0<=cfg['overshoot_ratio']<=.3:raise ValueError('invalid amplitude fraction')
    if not isinstance(cfg['max_seed_attempts'],int) or not 1<=cfg['max_seed_attempts']<=32:raise ValueError('invalid seed count')
    sections=np.asarray(cfg['beat_sections'],float)
    if sections.shape!=(4,) or not np.isfinite(sections).all() or np.any(sections<=0):raise ValueError('four positive beat sections required')
    cfg['beats']=float(sum(sections))
    for field in ('scan_cone_deg','scan_azimuth_deg','scan_roll_deg'):
        values=np.asarray(cfg[field],float)
        if values.ndim!=1 or not len(values) or not np.isfinite(values).all():raise ValueError('invalid scan grid')
    if min(cfg['scan_cone_deg'])<0 or max(cfg['scan_cone_deg'])>=150:raise ValueError('scan cone angles must be within 0..150 degrees')
    for seed in cfg['preparation_seeds_rad']:
        if np.asarray(seed).shape not in ((6,),(7,)) or not np.isfinite(seed).all():raise ValueError('invalid preparation IK seed')
    for point in [cfg['P_world_m'],*cfg['P_candidates_m']]:
        if point is not None and (np.asarray(point).shape!=(3,) or not np.isfinite(point).all()):raise ValueError('invalid candidate P')
    return cfg


def desired_rotation(reference,cone_deg,azimuth_deg,roll_deg=0.,axis='world_z'):
    """Minimal swing of tool +X, followed by explicitly requested tool-X roll."""
    alpha,azimuth,roll=np.radians([cone_deg,azimuth_deg,roll_deg]);origin=reference[:,0]
    if axis=='world_z':target=np.array([math.sin(alpha)*math.cos(azimuth),math.sin(alpha)*math.sin(azimuth),math.cos(alpha)])
    elif axis=='idle_tool':target=reference@np.array([math.cos(alpha),math.sin(alpha)*math.cos(azimuth),math.sin(alpha)*math.sin(azimuth)])
    else:raise ValueError('unknown cone axis')
    dot=float(origin@target);cross=np.cross(origin,target)
    if dot<-.9999:raise ValueError('minimal swing is ambiguous at the opposite pointing direction')
    skew=np.array([[0,-cross[2],cross[1]],[cross[2],0,-cross[0]],[-cross[1],cross[0],0]])
    swing=np.eye(3)+skew+skew@skew/(1+dot)
    return swing@reference@Rotation.from_rotvec([roll,0,0]).as_matrix()


class VirtualIK:
    def __init__(self,gravity,idle,cfg):
        self.gravity,self.idle,self.cfg=gravity,np.asarray(idle,float),cfg
        self.parts=kin._model(gravity);self.pin=self.parts[0]
        self.idle_P,self.reference=kin._pose(self.parts,idle,cfg)
        self.workspace=kin._workspace(self.parts,idle,cfg)
        self.lower=np.maximum(LOWER[:6],gravity.lower);self.upper=np.minimum(UPPER[:6],gravity.upper)
        margin=math.radians(cfg['joint_margin_deg']);self.bounds=(self.lower+margin,self.upper-margin)
        self.collision=CollisionAudit(gravity)

    def seeds(self,preferred=None):
        seeds=[]
        if preferred is not None:seeds.append(np.asarray(preferred)[:6])
        seeds.extend(np.asarray(q)[:6] for q in self.cfg['preparation_seeds_rad'])
        seeds.append(self.idle[:6])
        for q in ([0,25,90,60,0,0],[0,25,150,-60,0,0],[0,85,150,-60,0,0],
                  [0,145,90,60,0,0],[120,90,120,-30,0,0],[-120,90,120,-30,0,0],
                  [0,145,150,-60,0,0],[0,85,25,60,0,0]):seeds.append(np.radians(q))
        unique=[]
        for seed in seeds:
            seed=np.clip(seed,*self.bounds)
            if not any(np.max(abs(seed-other))<1e-8 for other in unique):unique.append(seed)
        return unique[:self.cfg['max_seed_attempts']]

    def solve(self,P,rotation,seeds):
        cfg=self.cfg;P=np.asarray(P,float);best=None
        for count,seed in enumerate(seeds,1):
            def residual(q):
                position,current=kin._pose(self.parts,q,cfg)
                return np.r_[(position-P)/cfg['jac_characteristic_length_m'],self.pin.log3(rotation.T@current)]
            result=least_squares(residual,np.asarray(seed)[:6],bounds=self.bounds,
                                 ftol=1e-10,xtol=1e-10,gtol=1e-10,max_nfev=80)
            q=np.r_[result.x,0.];position,current=kin._pose(self.parts,q,cfg)
            error=float(np.linalg.norm(position-P));angle=math.degrees(float(np.linalg.norm(self.pin.log3(rotation.T@current))))
            row=dict(accepted=False,reason='IK residual or convergence',position_error_m=error,
                     orientation_error_deg=angle,seed_attempts=count,q_rad=q.tolist())
            cost=error/cfg['jac_characteristic_length_m']+math.radians(angle)
            if result.success and error<=cfg['position_tolerance_m'] and angle<=cfg['orientation_tolerance_deg']:
                try:
                    _,margin,sigma,condition=kin._check_workspace(self.parts,q,cfg,self.workspace,self.lower,self.upper)
                    row.update(joint_margin_deg=math.degrees(margin),scaled_6d_sigma_min=sigma,scaled_6d_condition=condition)
                    check=self.collision.check(q)
                    if check['colliding']:
                        row.update(reason='self_collision',collision_pairs=check['pairs'])
                    else:row.update(accepted=True,reason='accepted geometric sample; continuous path not yet certified')
                except ValueError as error_message:row['reason']=str(error_message)
            if row['accepted']:return row
            if best is None or cost<best[0]:best=(cost,row)
        best[1]['seed_attempts']=len(seeds)
        return best[1]


def scan_envelope(gravity,idle,config=None,identity=None,progress=None):
    cfg=config_dict(config);solver=VirtualIK(gravity,idle,cfg)
    requested=cfg['P_world_m'];candidates=[requested] if requested is not None else [solver.idle_P.tolist(),*cfg['P_candidates_m']]
    samples=[];summaries=[]
    for point_id,P in enumerate(candidates):
        point_rows=[]
        for roll in cfg['scan_roll_deg']:
            for azimuth in cfg['scan_azimuth_deg']:
                previous=None
                for cone in sorted(cfg['scan_cone_deg'],reverse=cfg['cone_axis']=='world_z'):
                    try:
                        rotation=desired_rotation(solver.reference,cone,azimuth,roll,cfg['cone_axis'])
                        row=solver.solve(P,rotation,solver.seeds(previous))
                    except ValueError as error:
                        row=dict(accepted=False,reason=str(error),position_error_m=None,orientation_error_deg=None,seed_attempts=0)
                    row.update(point_id=point_id,P_xyz_m=list(P),cone_deg=float(cone),azimuth_deg=float(azimuth),roll_deg=float(roll))
                    samples.append(row);point_rows.append(row)
                    if row['accepted']:previous=np.asarray(row['q_rad'])[:6]
        rings=[]
        for cone in cfg['scan_cone_deg']:
            for roll in cfg['scan_roll_deg']:
                selected=[r for r in point_rows if r['cone_deg']==cone and r['roll_deg']==roll]
                if len(selected)==len(cfg['scan_azimuth_deg']) and all(r['accepted'] for r in selected):
                    rings.append(dict(cone_deg=float(cone),roll_deg=float(roll)))
        summary=dict(point_id=point_id,P_xyz_m=list(P),accepted_samples=sum(r['accepted'] for r in point_rows),
            max_accepted_sample_cone_deg=max((r['cone_deg'] for r in point_rows if r['accepted']),default=None),
            max_complete_sampled_ring_cone_deg=max((r['cone_deg'] for r in rings),default=None),
            total_samples=len(point_rows),complete_sampled_rings=rings,
            continuous_path_verified=False)
        summaries.append(summary)
        if progress:progress(summary)
    return dict(schema='rebuddy.virtual_pivot_scan.v1',config=cfg,identity=identity or {},
        collision_identity=solver.collision.identity,collision_scope='original URDF triangle-mesh self-collision only; no environment model',
        cone_axis_world=[0.,0.,1.] if cfg['cone_axis']=='world_z' else solver.reference[:,0].tolist(),
        cone_axis_reference=cfg['cone_axis'],pointing_axis_local=[1.,0.,0.],model_tcp_offset_m=[0.,0.,0.],
        tcp_offset_m=cfg['tcp_offset_m'],virtual_offset_m=cfg['tcp_offset_m'],
        initial_model_tcp_world_m=kin._pose(solver.parts,idle,dict(cfg,tcp_offset_m=[0.,0.,0.]))[0].tolist(),
        initial_virtual_P_world_m=solver.idle_P.tolist(),points=summaries,samples=samples,
        scope='Finite deterministic seed/grid search; a rejected sample is not a proof of global unreachability',
        max_accepted_sample_cone_deg=max((r['cone_deg'] for r in samples if r['accepted']),default=None),
        max_complete_sampled_ring_cone_deg=max((r['cone_deg'] for p in summaries for r in p['complete_sampled_rings']),default=None),
        continuous_path_verified=False,hardware_accepted=False)
