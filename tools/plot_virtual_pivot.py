#!/usr/bin/env python3
"""Plot normalized virtual-pivot preview samples; no robot or CAN access.

Input: virtual-pivot-preview.json alongside render_dance_design.py outputs.
All coordinates and derivatives describe the offline model, not measurements.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
import numpy as np

FONT = FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
COLORS = ['#38866b', '#4d79a0', '#c26954', '#9a77a4', '#b69237', '#638e91']


def prepare_preview(folder, library, scan_file):
    """Join validated renderer samples to their exact library and scan evidence."""
    folder, library = Path(folder), Path(library)
    manifest_path = library/'manifest.json'
    index = json.loads((folder/'index.json').read_text())
    if index['manifest_sha256'] != hashlib.sha256(manifest_path.read_bytes()).hexdigest():
        raise ValueError('preview and library manifest hashes differ')
    manifest = json.loads(manifest_path.read_text())
    action = next(item for item in manifest['actions'] if item['number'] == 21)
    spec = json.loads((library/action['file']).read_text())
    design = spec['virtual_pivot_design']
    data = json.loads((folder/'21-tip-samples.json').read_text())
    if not np.allclose(data['fixed_point_m'], design['P_world_m'], rtol=0, atol=1e-12):
        raise ValueError('preview fixed P differs from the action')
    if not np.allclose(data['virtual_offset_m'], design['virtual_offset_m'], rtol=0, atol=1e-12):
        raise ValueError('preview virtual offset differs from the action')
    data['design'] = design
    data['dance_interval_s'] = design['dance_interval_s']
    start, end = design['dance_interval_s']
    names = ['锥形两圈', '过冲钟摆', '螺旋扩张与收回', '逐拍定格']
    if 'phase_ranges_s' in design:
        if len(design['phase_ranges_s']) != 4:
            raise ValueError('virtual pivot must identify all four plotted phrases')
        data['phase_ranges_s'] = [dict(phase, name=names[i], source_name=phase['name'])
                                  for i, phase in enumerate(design['phase_ranges_s'])]
    else:
        # Legacy virtual recipes retime the whole dance uniformly.
        if design.get('config', {}).get('timing_mode') == 'per_phrase':
            raise ValueError('per-phrase trajectory requires explicit phase_ranges_s')
        sections = np.asarray(design['config']['beat_sections'])
        boundaries = start+np.r_[0., np.cumsum(sections)]/sum(sections)*(end-start)
        data['phase_ranges_s'] = [dict(name=name, start_s=float(boundaries[i]), end_s=float(boundaries[i+1]))
                                  for i, name in enumerate(names)]
    data['envelope_samples'] = []
    if scan_file is not None:
        scan_file = Path(scan_file)
        scan = json.loads(scan_file.read_text())
        if scan['config']['cone_axis'] != design['config']['cone_axis']:
            raise ValueError('scan and action use different cone axes')
        data['scan_sha256'] = hashlib.sha256(scan_file.read_bytes()).hexdigest()
        data['envelope_samples'] = [dict(tilt_deg=s['cone_deg'], azimuth_deg=s['azimuth_deg'],
                                          roll_deg=s['roll_deg'], reachable=s['accepted'])
                                    for s in scan['samples'] if np.allclose(s['P_xyz_m'],design['P_world_m'],rtol=0,atol=1e-10)]
        if not data['envelope_samples']:
            raise ValueError('scan contains no samples for the selected P')
    (folder/'virtual-pivot-preview.json').write_text(json.dumps(data, ensure_ascii=False, indent=2)+'\n')


def render_envelope(samples, output, point_note=''):
    """A local solver scan is evidence at its samples, not a reachable volume."""
    if not samples:
        raise ValueError('the envelope scan contains no samples')
    tilt = np.array([s['tilt_deg'] for s in samples])
    azimuth = np.array([s['azimuth_deg'] for s in samples])
    roll = np.array([s['roll_deg'] for s in samples])
    valid = np.array([bool(s['reachable']) for s in samples])
    fig = plt.figure(figsize=(13, 7))
    fig.subplots_adjust(left=.04, right=.95, bottom=.10, top=.79, wspace=.24)
    ax = fig.add_subplot(1, 2, 1, projection='3d')
    ax.scatter(azimuth[~valid], tilt[~valid], roll[~valid], color='#bfc4c7', s=18, alpha=.5, label='No accepted solution')
    ax.scatter(azimuth[valid], tilt[valid], roll[valid], color=COLORS[0], s=24, label='Accepted sample')
    ax.set_xlabel('Azimuth / deg'); ax.set_ylabel('Tilt / deg'); ax.set_zlabel('Roll / deg')
    if np.ptp(roll) == 0.:
        ax.set_zlim(roll[0]-1., roll[0]+1.)
        ax.set_zticks([roll[0]])
    ax.set_title('角度 / 方位 / Roll 的离散采样', fontproperties=FONT)
    ax.legend(loc='upper left')
    ax = fig.add_subplot(1, 2, 2)
    ax.scatter(azimuth[~valid], tilt[~valid], color='#bfc4c7', s=18, marker='x')
    if valid.any():
        if np.ptp(roll[valid]) == 0.:
            ax.scatter(azimuth[valid], tilt[valid], color=COLORS[0], s=30,
                       edgecolors='#253539', linewidths=.3, label=f'Roll = {roll[valid][0]:g} deg')
            ax.legend()
        else:
            scatter = ax.scatter(azimuth[valid], tilt[valid], c=roll[valid], cmap='viridis', s=30,
                                 edgecolors='#253539', linewidths=.3)
            fig.colorbar(scatter, ax=ax, label='Roll / deg')
    else:
        ax.text(.5, .85, '本次采样未找到通过解', transform=ax.transAxes,
                ha='center', fontproperties=FONT)
    ax.set_xlabel('方位角 / °', fontproperties=FONT)
    ax.set_ylabel('倾角 / °', fontproperties=FONT)
    ax.set_title(f'通过 {valid.sum()} / {len(valid)} 样本；不同 Roll 的投影可能重合', fontproperties=FONT)
    ax.grid(alpha=.2)
    fig.suptitle(('局部求解扫描；失败不证明全局不可达，未作实体碰撞验收\n'+point_note).strip(),
                 fontproperties=FONT, fontsize=13, y=.98, va='top')
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=150)
    plt.close(fig)
    print(output)


def render_scan(source, output):
    source, output = Path(source), Path(output)
    scan = json.loads(source.read_text())
    if scan.get('schema') != 'rebuddy.virtual_pivot_scan.v1':
        render_envelope(scan['envelope_samples'], output)
        return
    groups = {}
    for item in scan['samples']:
        groups.setdefault(int(item['point_id']), []).append(item)
    index = dict(scan_file=str(source.resolve()), scan_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                 local_solver_samples_only=True, collision_acceptance_claimed=False, points=[])
    for point_id, raw in sorted(groups.items()):
        point = raw[0]['P_xyz_m']
        if any(item['P_xyz_m'] != point for item in raw):
            raise ValueError('one scan point_id contains different P coordinates')
        samples = [dict(tilt_deg=s['cone_deg'], azimuth_deg=s['azimuth_deg'],
                        roll_deg=s['roll_deg'], reachable=s['accepted']) for s in raw]
        target = output if len(groups)==1 else output.with_name(f'{output.stem}-point-{point_id:02d}{output.suffix}')
        cone_axis = scan.get('cone_axis_reference', scan.get('config', {}).get('cone_axis', 'unspecified'))
        axis_note = {'world_z': '世界 +Z', 'idle_tool': '入口工具轴（不是世界竖直）'}.get(cone_axis, cone_axis)
        point_text = '['+', '.join(f'{coordinate:.4f}' for coordinate in point)+']'
        render_envelope(samples, target, f'锥轴：{axis_note}；P{point_id}: {point_text} m')
        index['points'].append(dict(point_id=point_id, P_xyz_m=point, total=len(raw),
                                    accepted=sum(bool(s['accepted']) for s in raw), image=target.name))
    output.with_suffix('.index.json').write_text(json.dumps(index, ensure_ascii=False, indent=2)+'\n')


def render(folder):
    folder = Path(folder)
    data = json.loads((folder/'virtual-pivot-preview.json').read_text())
    plt.rcParams.update({'axes.spines.top': False, 'axes.spines.right': False,
                         'axes.unicode_minus': False, 'font.size': 10})
    t = np.asarray(data['time_s'])
    q = np.asarray(data['joint_deg'])
    v = np.asarray(data['joint_velocity_deg_s'])
    a = np.asarray(data['joint_acceleration_deg_s2'])
    phases = data['phase_ranges_s']
    reference = data.get('design', {}).get('cone_axis_reference', 'unspecified')
    motion_tag = '工具轴小幅备选' if reference == 'idle_tool' else '世界竖直轴方案'
    for number, phase in enumerate(phases, start=1):
        selected = (t >= phase['start_s']-1e-9) & (t <= phase['end_s']+1e-9)
        if selected.sum() < 2:
            raise ValueError(f'insufficient plot samples for {phase["name"]}')
        fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True, constrained_layout=True)
        for axis, values, ylabel in zip(axes, (q, v, a), ('角度 / °', '速度 / °/s', '加速度 / °/s²')):
            for joint in range(6):
                axis.plot(t[selected], values[selected, joint], color=COLORS[joint], label=f'J{joint+1}')
            axis.set_ylabel(ylabel, fontproperties=FONT)
            axis.grid(alpha=.2)
            axis.legend(ncol=6, loc='upper right')
        axes[-1].set_xlabel('名义轨迹时间 / 秒', fontproperties=FONT)
        bpm = f' · {phase["actual_bpm"]:.2f} BPM' if 'actual_bpm' in phase else ''
        fig.suptitle(f'{motion_tag} · 第 {number} 段：{phase["name"]}{bpm} · 模型 q/v/a（名义时间）',
                     fontproperties=FONT, fontsize=13)
        path = folder/f'21-phase-{number:02d}-qva.png'
        fig.savefig(path, dpi=150)
        plt.close(fig)
        print(path)

    if data['envelope_samples']:
        axis = data.get('design', {}).get('cone_axis_reference', 'unspecified')
        axis_note = {'world_z': '世界 +Z', 'idle_tool': '入口工具轴（不是世界竖直）'}.get(axis, axis)
        render_envelope(data['envelope_samples'], folder/'21-reach-envelope.png', f'锥轴：{axis_note}')

    tcp = np.asarray(data['tip_xyz_m'])*100.
    pivots = np.asarray(data['virtual_p_xyz_m'])*100.
    anchor = np.asarray(data['fixed_point_m'])*100.
    fig = plt.figure(figsize=(12, 6), constrained_layout=True)
    ax = fig.add_subplot(1, 2, 1, projection='3d')
    ax.plot(*(tcp-anchor).T, color=COLORS[0], linewidth=2, label='Model TCP path')
    for index in np.linspace(0, len(t)-1, 16).astype(int):
        endpoints = np.array([tcp[index], pivots[index]])-anchor
        ax.plot(*endpoints.T, color='#b69237', alpha=.35, linewidth=1)
    ax.scatter(0., 0., 0., color='#c26954', s=70, marker='x', label='Fixed virtual P')
    extents = np.ptp(np.vstack((tcp, pivots)), axis=0)
    ax.set_box_aspect(np.maximum(extents, max(extents)*.3))
    ax.set_xlabel('X / cm'); ax.set_ylabel('Y / cm'); ax.set_zlabel('Z / cm')
    ax.set_title('TCP 围绕虚拟 P 运动', fontproperties=FONT)
    ax.legend()
    ax = fig.add_subplot(1, 2, 2)
    errors_mm = np.linalg.norm(pivots-anchor, axis=1)*10.
    ax.plot(t, errors_mm, color=COLORS[0])
    if 'dance_interval_s' in data:
        start, end = data['dance_interval_s']
        ax.axvspan(t[0], start, color='#d7d7d7', alpha=.6)
        ax.axvspan(end, t[-1], color='#d7d7d7', alpha=.6)
    ax.set_xlabel('名义轨迹时间 / 秒', fontproperties=FONT)
    ax.set_ylabel('P 位置误差范数 / mm', fontproperties=FONT)
    ax.yaxis.set_label_position('right')
    ax.yaxis.tick_right()
    ax.set_title('舞段固定 P；灰区为进入/退出过渡', fontproperties=FONT)
    ax.grid(alpha=.2)
    fig.suptitle(f'{motion_tag}；模型 TCP + 工具轴偏移；非已标定工具或实测误差',
                 fontproperties=FONT, fontsize=13)
    path = folder/'21-virtual-p-path.png'
    fig.savefig(path, dpi=150)
    plt.close(fig)
    print(path)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('folder', type=Path, nargs='?')
    parser.add_argument('--scan', type=Path, help='independent JSON with envelope_samples; no trajectory required')
    parser.add_argument('--output', type=Path, help='scan image output path')
    parser.add_argument('--library', type=Path, help='join this exact action library to renderer samples before plotting')
    args = parser.parse_args()
    if args.folder is not None:
        if args.library is not None:
            prepare_preview(args.folder, args.library, args.scan)
        render(args.folder)
    elif args.scan is not None:
        render_scan(args.scan, args.output or args.scan.with_suffix('.png'))
    else:
        parser.error('provide a preview folder or --scan JSON')
