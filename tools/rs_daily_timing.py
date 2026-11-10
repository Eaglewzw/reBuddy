"""Offline GIF timing alignment; preserve existing paths and motor budgets."""
import csv
import io
import json
from pathlib import Path

import numpy as np

from rs_daily_prepare import Clap, Curve, Plan, validate_gravity
from rs_motion_timewarp import TimeWarpedCurve
from rs_daily_nuzzle import Nuzzle
from rs_gravity_record import ROOT, PERIOD, digest


def stage_curve(stage):
    if stage['kind']=='nuzzle':
        return Nuzzle(stage['poses_rad'],stage['durations_s'],stage['amplitude_deg'],
                      sway_scurve=stage.get('sway_scurve'),
                      sway_peak_fraction=stage.get('sway_peak_fraction',.25),
                      sway_acceleration_factor=stage.get('sway_acceleration_factor',4*np.pi**2),
                      turnaround_factor=stage.get('turnaround_factor',0.),**stage['limits'])
    if stage['kind']=='clap':
        return Clap(stage['pose_rad'],stage['amplitude_deg'],stage['half_s'],
                    stage.get('j1_sway_deg',0.),stage.get('sway_cycles',2),stage.get('sway_duration_s'))
    curve=Curve(stage['t_s'],stage['q_rad'],**stage.get('limits',{}))
    factor=stage.get('speed',1.)
    curve.scale*=factor;curve.duration/=factor
    curve.peak_velocity*=factor;curve.peak_acceleration*=factor**2
    if 'time_warp' in stage:curve=TimeWarpedCurve(curve,stage['time_warp'])
    return curve


def align_to_gifs(folder, gravity):
    """Match GIF time where feasible; otherwise use bounded faster transitions.

    Fixed gripper/sway phrases keep their timing. Other segments retain every
    joint knot and only change their time axis. New accelerations retain 2%
    time headroom; already constrained segments are not accelerated further.
    """
    folder=Path(folder)
    manifest=json.loads((folder/'manifest.json').read_text())
    gifs=json.loads((ROOT/'assets/gifs/daily/manifest.json').read_text())['clips']
    durations={clip['id']:clip['duration_ms']/1000 for clip in gifs}
    pending=[];reports={}
    for name,item in manifest['actions'].items():
        if not item.get('plan_file'):continue
        spec=json.loads((folder/item['plan_file']).read_text())
        curves=[stage_curve(stage) for stage in spec['stages']]
        old=np.array([c.duration for c in curves])
        adjustable=np.array([s['kind']=='curve' and 'time_warp' not in s for s in spec['stages']])
        fastest=old.copy()
        for i,c in enumerate(curves):
            if adjustable[i]:
                factor=max(c.peak_velocity/c.velocity_limit,
                           np.sqrt(c.peak_acceleration/c.acceleration_limit))
                fastest[i]=min(old[i],max(.02,old[i]*factor*1.02))
        target=durations[name]
        if target>=sum(old):
            times=old.copy()
            times[adjustable]*=(target-sum(old[~adjustable]))/sum(old[adjustable])
        elif target>=sum(fastest):
            fraction=(target-sum(fastest))/(sum(old)-sum(fastest))
            times=fastest+fraction*(old-fastest)
        else:times=fastest
        stages=[]
        for stage,curve,duration in zip(spec['stages'],curves,times):
            if stage['kind']=='curve' and abs(duration-curve.duration)>1e-9:
                stage['t_s']=(np.asarray(stage['t_s'])*duration/stage['t_s'][-1]).tolist()
                stage['speed']=1.
            updated=stage_curve(stage)
            if abs(updated.duration-duration)>1e-7:
                raise ValueError(f'{name}: requested timing exceeds dynamic limits')
            # Compare the complete shape, not only endpoints, before replacing data.
            for fraction in np.linspace(0,1,101):
                np.testing.assert_allclose(updated.at(fraction*updated.duration)[0],
                                           curve.at(fraction*curve.duration)[0],rtol=0,atol=1e-10)
            stages.append((stage['label'],updated))
        plan=Plan(stages);validate_gravity(plan,gravity)
        text=json.dumps(spec,ensure_ascii=False,indent=2)+'\n'
        stream=io.StringIO(newline='');writer=csv.writer(stream)
        writer.writerow(['t_s']+[f'target_q{i}_rad' for i in range(1,8)])
        for t in np.r_[np.arange(0,plan.duration,PERIOD),plan.duration]:writer.writerow([t,*plan.at(t)[0]])
        report=dict(gif_s=target,motion_s=plan.duration,matched=abs(plan.duration-target)<1e-7,
                    path_preserved=True,fixed_gripper_timing=True,
                    stages=[dict(name=n,duration_s=c.duration) for n,c in stages])
        item.update(duration_s=plan.duration,stages=report['stages'],gif_timing=report,
                    hardware_accepted=False)
        pending.append((item,text,stream.getvalue()))
        reports[name]=report
    # All paths and constraints passed before any library file is replaced.
    for item,text,preview in pending:
        path=folder/item['plan_file'];path.write_text(text)
        path_csv=folder/item['preview_csv'];path_csv.write_text(preview)
        item.update(plan_sha256=digest(path),preview_sha256=digest(path_csv))
    (folder/'manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n')
    return reports
