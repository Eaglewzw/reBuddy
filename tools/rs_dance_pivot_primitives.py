"""Fixed-TCP configuration, orientation primitives and beat timeline."""
from __future__ import annotations
import copy
import math
from functools import lru_cache
import numpy as np
from scipy.optimize import brentq
from scipy.spatial.transform import Rotation

CONFIG=dict(bpm=100.,beats=16,beat_sections=[4.,4.,4.,2.,2.],sample_period_s=.02,tcp_offset_m=[0.,0.,0.],fixed_point_xyz_m=None,
    pointing_axis_local=[0.,0.,1.],nod_deg=15.,overshoot_ratio=.08,cone_deg=20.,figure8_deg=[12.,10.],
    nod_transition_fractions=[.29,.43,.28],angular_progress='smooth_cruise',angular_ramp_fraction=.15,
    amplitude_scales=[1.,.85,.7,.5],workspace_reach_fraction=.7,workspace_inner_fraction=.15,
    workspace_min_z_m=.15,joint_margin_deg=10.,jac_characteristic_length_m=.3,
    jac_sigma_min=.08,jac_condition_max=40.,position_tolerance_m=.0005,
    orientation_tolerance_deg=.2,max_duration_s=60.)


def _config(config=None):
    result=copy.deepcopy(CONFIG)
    if config:
        if set(config)-set(result):raise ValueError('unknown pivot configuration field')
        result.update(copy.deepcopy(config))
    sections=np.asarray(result['beat_sections'],float)
    if sections.shape!=(5,) or not np.isfinite(sections).all() or np.any(sections<=0):raise ValueError('five positive beat sections are required')
    total=float(np.sum(sections))
    if config and 'beats' in config and abs(float(config['beats'])-total)>1e-9:raise ValueError('beats must equal sum(beat_sections)')
    result['beats']=total
    if result['sample_period_s']!=.02:raise ValueError('pivot uses the existing 50 Hz control cadence')
    if not math.isfinite(float(result['workspace_min_z_m'])) or result['workspace_min_z_m']<0:raise ValueError('invalid minimum workspace height')
    if result['fixed_point_xyz_m'] is not None:
        if np.asarray(result['fixed_point_xyz_m']).shape!=(3,) or not np.isfinite(result['fixed_point_xyz_m']).all():raise ValueError('invalid fixed TCP point')
    numeric=('bpm','nod_deg','cone_deg','joint_margin_deg','jac_characteristic_length_m',
             'jac_sigma_min','jac_condition_max','position_tolerance_m','orientation_tolerance_deg','max_duration_s')
    if any(not math.isfinite(float(result[k])) or float(result[k])<=0 for k in numeric):raise ValueError('invalid pivot limits')
    fractions=np.asarray(result['nod_transition_fractions'],float)
    if fractions.shape!=(3,) or not np.isfinite(fractions).all() or min(fractions)<=0 or abs(sum(fractions)-1)>1e-9:raise ValueError('invalid nod timing fractions')
    if result['angular_progress'] not in ('smoothstep5','smooth_cruise'):raise ValueError('unknown angular progress')
    if not math.isfinite(result['angular_ramp_fraction']) or not 0<result['angular_ramp_fraction']<=.5:raise ValueError('invalid angular ramp fraction')
    if not 0<=result['overshoot_ratio']<=.3:raise ValueError('overshoot must be between 0 and 0.3')
    if not 0<result['workspace_inner_fraction']<result['workspace_reach_fraction']<=.7:
        raise ValueError('invalid conservative reach fractions')
    for key in ('tcp_offset_m','pointing_axis_local'):
        if np.asarray(result[key]).shape!=(3,) or not np.isfinite(result[key]).all():raise ValueError('invalid TCP vector')
    if abs(np.linalg.norm(result['pointing_axis_local'])-1)>1e-8:raise ValueError('pointing axis must be a unit vector')
    if not result['amplitude_scales'] or any(not 0<float(s)<=1 for s in result['amplitude_scales']):raise ValueError('invalid amplitude scale list')
    if len(result['figure8_deg'])!=2 or not all(math.isfinite(a) and 0<a<=30 for a in result['figure8_deg']):raise ValueError('invalid figure-eight amplitudes')
    if total*60/result['bpm']>result['max_duration_s']:raise ValueError('requested pivot duration exceeds configured maximum')
    return result


def _ease(u):
    u=float(np.clip(u,0,1));return u**3*(10-15*u+6*u*u)


@lru_cache(maxsize=64)
def _back_coefficient(ratio):
    from numpy.polynomial import Polynomial
    base_curve=Polynomial([0,0,0,10,-15,6]);bump=Polynomial([0,0,0,1,-3,3,-1])
    def excess(coefficient):
        curve=base_curve+coefficient*bump;roots=curve.deriv().roots()
        candidates=[0.,1.]+[float(r.real) for r in roots if abs(r.imag)<1e-9 and 0<r.real<1]
        return float(max(curve(candidates)))-1-ratio
    return brentq(excess,0.,100.)


def _progress(u,cfg):
    if cfg['angular_progress']=='smoothstep5':return _ease(u)
    ramp=cfg['angular_ramp_fraction'];u=float(np.clip(u,0,1))
    def enter(t):
        x=t/ramp;return ramp/(1-ramp)*(2.5*x**4-3*x**5+x**6)
    if u<ramp:return enter(u)
    if u>1-ramp:return 1-enter(1-u)
    return (u-.5*ramp)/(1-ramp)


def _slerp(a,b,u,overshoot_ratio=0.):
    fraction=_ease(u)
    if overshoot_ratio:
        # A degree-six C2 back curve overshoots, then settles at one.
        # Its coefficient is solved from the actual polynomial maximum.
        u=float(np.clip(u,0,1));fraction+=_back_coefficient(overshoot_ratio)*u**3*(1-u)**3
    return a@Rotation.from_rotvec(Rotation.from_matrix(a.T@b).as_rotvec()*fraction).as_matrix()


def _swing(reference,pointing_axis,target):
    origin=reference@pointing_axis;cross=np.cross(origin,target);dot=float(origin@target)
    if dot < -.9999:raise ValueError('pointing-axis swing approaches an ambiguous half turn')
    skew=np.array([[0,-cross[2],cross[1]],[cross[2],0,-cross[0]],[-cross[1],cross[0],0]])
    return (np.eye(3)+skew+skew@skew/(1+dot))@reference


def orientation_at(beat,reference_rotation,config=None,scale=1.):
    """Desired rotation on configurable section beats, independent of retiming."""
    cfg=_config(config);reference=np.asarray(reference_rotation);axis=np.asarray(cfg['pointing_axis_local'])
    initial_axis=reference@axis;azimuth=math.atan2(initial_axis[1],initial_axis[0])
    half=math.radians(cfg['cone_deg']*scale)
    def cone(theta):
        target=np.array([math.sin(half)*math.cos(theta+azimuth),math.sin(half)*math.sin(theta+azimuth),math.cos(half)])
        return _swing(reference,axis,target)
    center=cone(0);ends=np.cumsum(cfg['beat_sections']);beat=float(np.clip(beat,0,ends[-1]))
    if beat<=ends[0]:
        amplitude=math.radians(cfg['nod_deg']*scale)
        nodes=[reference,reference@Rotation.from_rotvec([0,amplitude,0]).as_matrix(),
               reference@Rotation.from_rotvec([0,-amplitude,0]).as_matrix(),center]
        knots=np.r_[0,np.cumsum(cfg['nod_transition_fractions'])]*ends[0]
        index=min(2,int(np.searchsorted(knots,beat,side='right')-1))
        return _slerp(nodes[index],nodes[index+1],(beat-knots[index])/(knots[index+1]-knots[index]),cfg['overshoot_ratio'] if index<2 else 0.)
    if beat<=ends[1]:return cone(2*math.pi*_progress((beat-ends[0])/cfg['beat_sections'][1],cfg))
    if beat<=ends[2]:
        theta=2*math.pi*_progress((beat-ends[1])/cfg['beat_sections'][2],cfg);a,b=np.radians(cfg['figure8_deg'])*scale
        return center@Rotation.from_rotvec([0,a*math.sin(theta),0]).as_matrix()@Rotation.from_rotvec([b*math.sin(2*theta),0,0]).as_matrix()
    if beat<=ends[3]:return center
    return _slerp(center,reference,(beat-ends[3])/cfg['beat_sections'][4])


