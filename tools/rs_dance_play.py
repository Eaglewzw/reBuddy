#!/usr/bin/env python3
"""Prepare a separate, reduced-amplitude dance trial; preserve recorded timing."""
import argparse
import json
import math
from pathlib import Path

import numpy as np

from rs_daily_prepare import Plan, fit_recording, validate_gravity, HAPPY_FAST_LIMITS
from rs_gravity_record import ROOT, DEFAULT_CALIBRATION, DEFAULT_URDF, digest, preflight, vector
from rs_gravity_play import load_clip, play
from rs_motion import Curve


def derive(source, calibration, urdf):
    source = Path(source).resolve()
    gravity, identity = preflight(calibration, urdf)
    t, q, audit = load_clip(source, identity, recover_endpoint_failure=True)
    summary = json.loads((source.parent / 'summary.json').read_text())
    clip = next(c for c in summary['clips'] if c['file'] == source.name)
    anchor = clip.get('anchor', {})
    pose = anchor.get('idle_pose', {})
    if not anchor.get('entry_verified') or not anchor.get('exit_idle') or not clip.get('auto_return'):
        raise ValueError('需要带起点验证及自动回位记录的舞蹈片段')
    if not anchor.get('action', '').startswith('DANCE_'):
        raise ValueError('需要基础舞蹈录制')
    for key in ('calibration_sha256', 'urdf_sha256'):
        if pose.get(key) != identity[key]:
            raise ValueError('待机姿态标定不匹配')
    idle = vector(pose['q_rad'])
    # t was normalized to the first actual sample, which may follow clip_start.
    import csv
    with source.open(newline='') as f:
        first = next(csv.DictReader(f))
    manual_end = (clip['manual_end_ns'] - int(first['monotonic_ns'])) / 1e9
    count = int(np.searchsorted(t, manual_end + 1e-9, side='right'))
    if count < 4 or count >= len(t) or abs(t[count-1] - manual_end) > .1:
        raise ValueError('手动段结束时间缺失或不连续')
    if np.max(abs(q[count-1] - vector(clip['manual_end_rad']))) > math.radians(.1):
        raise ValueError('手动段结束位置不匹配')
    original, fit = fit_recording(t[:count], q[:count], **HAPPY_FAST_LIMITS)
    raw_v = original.peak_velocity / original.scale
    raw_a = original.peak_acceleration / original.scale**2
    # Reduce amplitude, not time. Leave 5% headroom under existing Daily budgets.
    amplitude = min(1., .95 * HAPPY_FAST_LIMITS['velocity_limit'] / max(raw_v, 1e-12),
                    .95 * HAPPY_FAST_LIMITS['acceleration_limit'] / max(raw_a, 1e-12))
    reduced = idle + amplitude * (np.asarray(fit['q_rad']) - idle)
    reduced[:, 6] = idle[6]  # Gripper is held, never replay encoder noise.
    body = Curve(fit['t_s'], reduced, **HAPPY_FAST_LIMITS)
    tail_s = float(t[-1] - t[count-1])
    tail = Curve([0., tail_s], [body.end, idle], **HAPPY_FAST_LIMITS)
    if body.scale < 1. - 1e-8 or tail.scale < 1. - 1e-8:
        raise ValueError('保留时长仍超出动态限值；未生成实机版本')
    plan = Plan([('recorded_gesture', body), ('prepared_return', tail)])
    plan.velocity_limit = HAPPY_FAST_LIMITS['velocity_limit']
    validate_gravity(plan, gravity)
    spec = dict(schema='rebuddy.dance_trial.v1', source=str(source),
                source_sha256=digest(source), metadata_sha256=digest(source.parent/'metadata.json'),
                summary_sha256=digest(source.parent/'summary.json'),
                calibration_sha256=identity['calibration_sha256'], urdf_sha256=identity['urdf_sha256'],
                source_complete=audit['source_complete'],
                recovered_endpoint_failure=audit['recovered_endpoint_failure'],
                hardware_accepted=False, policy='preserve timing; reduce amplitude; replace automatic return',
                recorded_duration_s=float(t[-1]), manual_duration_s=float(t[count-1]),
                playback_s=plan.duration, amplitude_scale=amplitude,
                original_peak_velocity_rad_s=raw_v, original_peak_acceleration_rad_s2=raw_a,
                limits=HAPPY_FAST_LIMITS, max_fit_error_deg=fit['max_sample_fit_error_deg'],
                fit_knots=fit['knots'],
                original_range_deg=np.degrees(np.ptp(q[:count, :6], axis=0)).tolist(),
                prepared_range_deg=np.degrees(np.array(body.upper[:6])-body.lower[:6]).tolist(),
                body=dict(t_s=fit['t_s'], q_rad=reduced.tolist()),
                return_segment=dict(t_s=[0., tail_s], q_rad=[body.end.tolist(), idle.tolist()]))
    return plan, gravity, spec


def load_prepared(path, calibration, urdf):
    path = Path(path).resolve()
    saved = json.loads(path.read_text())
    if saved.get('schema') != 'rebuddy.dance_trial.v1':
        raise ValueError('不是已整理的舞蹈试播文件')
    plan, gravity, checked = derive(saved['source'], calibration, urdf)
    # Re-derive all limits and coefficients; never trust edited executable knots.
    if saved != checked:
        raise ValueError('源数据、标定或处理结果变化，请重新 prepare')
    report = dict(checked, source_recording=checked['source'], source=str(path),
                  source_sha256=digest(path), exit_policy='completion/stop/fault: checked hold; home/quit: zero return; release: disable')
    return plan, gravity, report


def main(argv=None):
    parser = argparse.ArgumentParser(description='舞蹈试播：保留原时长，缩小幅度以满足已有动态限值')
    parser.add_argument('mode', choices=['prepare', 'inspect', 'play'])
    parser.add_argument('path', type=Path, help='prepare: 原始 CSV；inspect/play: 处理后的 JSON')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--supported', action='store_true')
    parser.add_argument('--port', default='can0')
    parser.add_argument('--calibration', type=Path, default=DEFAULT_CALIBRATION)
    parser.add_argument('--urdf', type=Path, default=DEFAULT_URDF)
    parser.add_argument('--log-dir', type=Path, default=ROOT/'build/record-review')
    args = parser.parse_args(argv)
    args.protective_hold = True
    args.speed = .5  # Approach only; the prepared body keeps original timing.
    try:
        if args.mode == 'prepare':
            plan, gravity, report = derive(args.path, args.calibration, args.urdf)
            output = args.output or ROOT/'record/dance_prepared'/args.path.parent.name/(args.path.stem+'.json')
            output.parent.mkdir(parents=True, exist_ok=True)
            with output.open('x') as f:
                f.write(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
            print(f'已生成：{output.resolve()}')
        else:
            plan, gravity, report = load_prepared(args.path, args.calibration, args.urdf)
        print(f'整段 {plan.duration:.2f} 秒；手动动作段 {report["manual_duration_s"]:.2f} 秒；'
              f'幅度为原动作 {report["amplitude_scale"]:.1%}。')
        print('原始录制保持不变；处理版本尚未经实机验收。')
        if args.mode == 'play':
            result = play(plan, gravity, report, args)
            print(json.dumps(result, ensure_ascii=False, indent=2))
            return 0 if result['success'] else 1
        return 0
    except Exception as error:
        print(f'舞蹈试播：{error}')
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
