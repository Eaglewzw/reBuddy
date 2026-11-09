"""Load the sole, self-contained twelve-entry dance library before enabling CAN."""
import json
from pathlib import Path

import numpy as np

import rs_dance_motion as motion
from rs_dance_plan import DancePlan
from rs_gravity_record import ROOT, digest, preflight

DEFAULT_OUTPUT=ROOT/'record/dance_library'
NAMES=('开场','点头打拍','歪头招手','深蹲回升','前探伸展','探头撒娇',
       '开心欢呼','蛇形波浪','稳定摇摆','空间8字','大幅定点左右上下','收尾')


def _check_authored(number,spec,gravity,idle,identity):
    if number==1:
        from rs_dance_nod import make_spec
        expected=make_spec(gravity,idle,identity)
    elif number in (2,5,6):
        from rs_dance_cute import make_spec
        expected=make_spec(gravity,idle,identity,f'{number:02d}')
    elif number==8:
        from rs_dance_steady import make_spec
        expected=make_spec(gravity,idle,identity)
    else:return {}
    for key in ('stages','name','gripper_motion','collision_identity'):
        if spec.get(key)!=expected[key]:raise ValueError(f'{number:02d} authored {key} changed')
    return expected['validation']


def load_library(calibration,urdf,folder=DEFAULT_OUTPUT):
    folder=Path(folder);path=folder/'manifest.json'
    manifest=json.loads(path.read_text())
    if manifest.get('schema')!='rebuddy.dance_library.v2':
        raise ValueError('请使用当前 dance_library 动作库')
    gravity,identity=preflight(Path(calibration),Path(urdf))
    idle,zero,sources,bookends=motion._anchors(identity)
    for key in ('calibration_sha256','urdf_sha256'):
        if manifest.get(key)!=identity[key]:raise ValueError('dance calibration/model changed')
    if manifest.get('dependencies')!=sources:raise ValueError('Daily dance anchors changed')
    if not np.array_equal(manifest.get('idle_rad'),idle) or not np.array_equal(manifest.get('zero_rad'),zero):
        raise ValueError('dance entry/exit anchors changed')
    if [e.get('number') for e in manifest['actions']]!=list(range(12)):
        raise ValueError('dance requires 00, ten motions, and 11')
    plans={};reports={}
    for number,entry in enumerate(manifest['actions']):
        source=folder/entry['file']
        if source.parent.resolve()!=folder.resolve() or digest(source)!=entry['sha256']:
            raise ValueError('dance action file checksum changed')
        spec=json.loads(source.read_text());gripper=number in (2,6)
        if (spec.get('schema')!='rebuddy.dance_action.v2' or spec.get('number')!=number
            or spec.get('name')!=NAMES[number] or entry.get('name')!=NAMES[number]
            or spec.get('gripper_motion')!=gripper or entry.get('gripper_motion')!=gripper
            or spec.get('action_id') is not None or spec.get('firmware_integrated') is not False):
            raise ValueError('dance action identity changed')
        for key in ('calibration_sha256','urdf_sha256'):
            if spec.get(key)!=identity[key]:raise ValueError('dance action model changed')
        if number in (0,11) and spec['stages']!=[bookends['WAKE' if number==0 else 'REST']]:
            raise ValueError('Daily bookend motion changed')
        if number==10 and (len(spec['stages'])!=1 or spec['stages'][0].get('kind')!='virtual_pivot_joined'):
            raise ValueError('pivot must preserve continuous horizontal/vertical handoff')
        plan=DancePlan(spec['stages'],bookend=number in (0,11),gripper_motion=gripper)
        start,end=(zero,idle) if number==0 else (idle,zero) if number==11 else (idle,idle)
        if (not np.allclose(plan.start,start,rtol=0,atol=1e-9)
            or not np.allclose(plan.end,end,rtol=0,atol=1e-9)
            or not np.allclose(spec['start_rad'],plan.start,rtol=0,atol=1e-9)
            or not np.allclose(spec['end_rad'],plan.end,rtol=0,atol=1e-9)):
            raise ValueError('dance motion endpoints changed')
        if abs(plan.duration-spec['duration_s'])>1e-9 or abs(plan.duration-entry['duration_s'])>1e-9:
            raise ValueError('dance duration changed')
        checks=motion._validate_plan(plan,gravity)
        checks.update(_check_authored(number,spec,gravity,idle,identity))
        if number in (7,9):
            from rs_dance_spatial import validate_extra
            checks.update(validate_extra(plan,gravity,18 if number==7 else 20,idle,spec))
        if number==10:
            from rs_dance_pivot_join import validate_join
            checks.update(validate_join(plan,gravity,idle,spec))
            if spec.get('default_trial_speed')!=plan.default_trial_speed:
                raise ValueError('pivot default speed changed')
        code=f'{number:02d}';plans[code]=plan
        reports[code]=dict(spec,validation=checks,source=str(source.resolve()),source_sha256=digest(source),
            library_manifest=str(path.resolve()),library_manifest_sha256=digest(path),playback_s=plan.duration,
            source_library='dance_library',source_number=code)
    return manifest,plans,gravity,identity,reports
