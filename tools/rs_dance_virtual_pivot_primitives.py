"""Two conical circles, planar pendulum, expanding spiral, four beat freezes."""
import math
import numpy as np
from scipy.spatial.transform import Rotation
from rs_dance_pivot_primitives import _back_coefficient
from rs_dance_virtual_pivot_scan import desired_rotation


def ease(u):
    u=float(np.clip(u,0,1));return u**3*(10-15*u+6*u*u)


def progress(u,cfg):
    ramp=cfg['phase_ramp_fraction'];u=float(np.clip(u,0,1))
    def enter(t):
        x=t/ramp;return ramp/(1-ramp)*(2.5*x**4-3*x**5+x**6)
    if u<ramp:return enter(u)
    if u>1-ramp:return 1-enter(1-u)
    return (u-ramp/2)/(1-ramp)


def back(u,ratio):
    u=float(np.clip(u,0,1));return ease(u)+_back_coefficient(ratio)*u**3*(1-u)**3 if ratio else ease(u)


def interpolate(a,b,u):return a@Rotation.from_rotvec(Rotation.from_matrix(a.T@b).as_rotvec()*ease(u)).as_matrix()


def orientation_at(beat,reference,cfg,selected):
    cone=selected['cone_deg'];roll=selected['roll_deg'];axis=cfg['cone_axis']
    pendulum=selected['pendulum_deg'];inner=selected['spiral_inner_deg']
    rotation=lambda alpha,az:desired_rotation(reference,alpha,az,roll,axis)
    ends=np.cumsum(cfg['beat_sections']);beat=float(np.clip(beat,0,ends[-1]));pause=cfg['phase_pause_beats']
    if beat<=ends[0]:
        u=min(1.,beat/(cfg['beat_sections'][0]-pause));return rotation(cone,720*progress(u,cfg))
    if beat<=ends[1]:
        u=min(1.,(beat-ends[0])/(cfg['beat_sections'][1]-pause))
        # Signed inclination at fixed azimuth is a planar pendulum, not an
        # azimuthal sweep around a cone. Its back curve exceeds the negative
        # target then returns to that target before the final inward move.
        if abs(cone-pendulum)<1e-9:
            knots=[0.,.6,1.];angles=[cone,-pendulum,inner]
        else:knots=[0.,.2,.65,1.];angles=[cone,pendulum,-pendulum,inner]
        index=min(len(knots)-2,int(np.searchsorted(knots,u,side='right')-1))
        local=(u-knots[index])/(knots[index+1]-knots[index])
        amount=back(local,cfg['overshoot_ratio']) if index<len(knots)-2 else ease(local)
        return rotation(angles[index]+(angles[index+1]-angles[index])*amount,0.)
    if beat<=ends[2]:
        u=min(1.,(beat-ends[1])/(cfg['beat_sections'][2]-pause));envelope=64*u**3*(1-u)**3
        return rotation(inner+(cone-inner)*envelope,720*progress(u,cfg))
    length=cfg['beat_sections'][3]/4;position=(beat-ends[2])/length;index=min(3,int(position));u=position-index
    azimuth=[90.,180.,270.,0.];source=rotation(inner,0) if index==0 else rotation(cone,azimuth[index-1]);target=rotation(cone,azimuth[index])
    return interpolate(source,target,min(1.,u/(1-cfg['freeze_fraction'])))
