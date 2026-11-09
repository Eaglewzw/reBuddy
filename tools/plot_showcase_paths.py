#!/usr/bin/env python3
"""Plot offline tip samples exported by render_dance_design.py (no CAN)."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
import numpy as np

FONT = FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
COLORS = ['#38866b', '#4d79a0', '#c26954', '#9a77a4', '#b69237', '#638e91']


def label(axis, title, xlabel, ylabel):
    axis.set_title(title, fontproperties=FONT, fontsize=13)
    axis.set_xlabel(xlabel, fontproperties=FONT)
    axis.set_ylabel(ylabel, fontproperties=FONT)
    axis.grid(alpha=.2)


def render_fixed_tcp(folder, compare=None):
    """Plot the configured TCP, which may differ from the URDF frame origin."""
    path = folder/'21-tip-samples.json'
    if not path.exists():
        return
    data = json.loads(path.read_text())
    t = np.asarray(data['time_s'])
    xyz = np.asarray(data['tip_xyz_m'])
    fixed = np.asarray(data['fixed_point_m'])
    virtual = bool(data.get('virtual_p_xyz_m'))
    if virtual:
        # The dedicated plotter knows where entry/exit end and P becomes fixed.
        return
    constrained_positions = np.asarray(data['virtual_p_xyz_m']) if virtual else xyz
    error = (constrained_positions-fixed)*1000.
    rotation = np.asarray(data['world_rotation_vector_deg'])
    joints = np.asarray(data['joint_deg'])
    fig, axes = plt.subplots(3, 1, figsize=(11, 10), constrained_layout=True)
    for j, name in enumerate(('World X', 'World Y', 'World Z')):
        axes[0].plot(t, error[:, j], color=COLORS[j], label=name)
        axes[1].plot(t, rotation[:, j], color=COLORS[j], label=name)
    axes[0].plot(t, np.linalg.norm(error, axis=1), color='#253539', ls='--', label='Error norm')
    label(axes[0], '21 虚拟 P 相对固定支点的模型偏差' if virtual else '21 定点转动：配置 TCP 相对固定点的模型偏差',
          '名义轨迹时间 / 秒', '偏差 / mm')
    label(axes[1], '末端姿态变化（相对起始的世界旋转向量）', '名义轨迹时间 / 秒', '角度 / °')
    for j in range(6):
        axes[2].plot(t, joints[:, j]-joints[0, j], color=COLORS[j], label=f'J{j+1}')
    label(axes[2], '各关节配合保持虚拟 P 位置' if virtual else '各关节配合保持同一 TCP 位置',
          '名义轨迹时间 / 秒', '相对起始关节角 / °')
    for axis in axes:
        axis.legend(ncol=6)
    fig.suptitle('URDF + 配置 TCP 偏移；非实测漂移或已标定指尖；动画为名义速度',
                 fontproperties=FONT, fontsize=13)
    fig.savefig(folder/'21-fixed-tcp.png', dpi=150)
    plt.close(fig)
    print(folder/'21-fixed-tcp.png')
    if not virtual and compare is not None and (Path(compare)/'21-tip-samples.json').exists():
        previous = Path(compare)
        old = json.loads((previous/'21-tip-samples.json').read_text())
        old_joints = np.asarray(old['joint_deg'])
        old_error = (np.asarray(old['tip_xyz_m'])-np.asarray(old['fixed_point_m']))*1000.
        ranges = np.ptp(joints, axis=0)
        old_ranges = np.ptp(old_joints, axis=0)
        fig, axes = plt.subplots(2, 1, figsize=(11, 8), constrained_layout=True)
        x = np.arange(6)
        axes[0].bar(x-.18, old_ranges, width=.36, color=COLORS[1], label=previous.name)
        axes[0].bar(x+.18, ranges, width=.36, color=COLORS[0], label=folder.name)
        axes[0].set_xticks(x, [f'J{j+1}' for j in x])
        for j in range(6):
            axes[0].text(j-.18, old_ranges[j]+1., f'{old_ranges[j]:.1f}°', ha='center', fontsize=9)
            axes[0].text(j+.18, ranges[j]+1., f'{ranges[j]:.1f}°', ha='center', fontsize=9)
        axes[0].set_ylim(0., max(ranges.max(), old_ranges.max())*1.16)
        label(axes[0], '21 六轴运动范围：最大角减最小角', '关节', '峰峰幅度 / °')
        axes[0].legend()
        axes[1].plot(np.asarray(old['time_s'])/old['time_s'][-1]*100., np.linalg.norm(old_error, axis=1),
                     color=COLORS[1], label=f'{previous.name} / {old["time_s"][-1]:.2f} s nominal')
        axes[1].plot(t/t[-1]*100., np.linalg.norm(error, axis=1), color=COLORS[0],
                     label=f'{folder.name} / {t[-1]:.2f} s nominal')
        label(axes[1], '各版本配置 TCP 与各自固定点的模型偏差', '动作进度 / %', '位置误差范数 / mm')
        axes[1].legend()
        fig.suptitle('URDF 计划轨迹对比；不代表实测漂移或碰撞验证',
                     fontproperties=FONT, fontsize=13)
        fig.savefig(folder/'21-body-comparison.png', dpi=150)
        plt.close(fig)
        print(folder/'21-body-comparison.png')


def render(folder, compare=None):
    folder = Path(folder)
    plt.rcParams.update({'axes.spines.top': False, 'axes.spines.right': False,
                         'axes.unicode_minus': False, 'font.size': 10})
    data = json.loads((folder/'19-tip-samples.json').read_text())
    t = np.asarray(data['time_s'])
    joints = np.asarray(data['joint_deg'])
    rotation = np.asarray(data['world_rotation_vector_deg'])
    fig, axes = plt.subplots(2, 1, figsize=(11, 7), constrained_layout=True)
    for j in range(6):
        axes[0].plot(t, joints[:, j]-joints[0, j], color=COLORS[j], label=f'J{j+1}')
    label(axes[0], '19 稳定摇摆：各关节主动配合', '时间 / 秒', '相对起始关节角 / °')
    axes[0].legend(ncol=6)
    for j, name in enumerate(('World X', 'World Y', 'World Z')):
        axes[1].plot(t, rotation[:, j], color=COLORS[j], label=name)
    error = np.linalg.norm(rotation, axis=1)
    axes[1].plot(t, error, color='#253539', ls='--', label='Angle magnitude')
    label(axes[1], f'末端世界朝向变化：模型最大 {error.max():.4f}°',
          '时间 / 秒', '相对起始旋转向量 / °')
    axes[1].legend(ncol=4)
    fig.suptitle('URDF 计划轨迹；不代表实测稳定精度或碰撞验证',
                 fontproperties=FONT, fontsize=13)
    fig.savefig(folder/'19-orientation.png', dpi=150)
    plt.close(fig)

    data = json.loads((folder/'20-tip-samples.json').read_text())
    xyz = np.asarray(data['tip_xyz_m'])*100.
    relative = xyz-xyz[0]
    fig = plt.figure(figsize=(12, 9), constrained_layout=True)
    ax = fig.add_subplot(2, 2, 1, projection='3d')
    ax.plot(*relative.T, color=COLORS[0], linewidth=2)
    ax.scatter(*relative[0], color='#c26954', s=40)
    ax.set_xlabel('X / cm'); ax.set_ylabel('Y / cm'); ax.set_zlabel('Z / cm')
    ax.set_title('20 空间 8 字：末端轨迹', fontproperties=FONT)
    extent = np.ptp(relative, axis=0)
    ax.set_box_aspect(np.maximum(extent, max(extent)*.3))
    for slot, (a, b) in enumerate(((0, 1), (0, 2), (1, 2)), start=2):
        ax = fig.add_subplot(2, 2, slot)
        ax.plot(relative[:, a], relative[:, b], color=COLORS[0], linewidth=2)
        ax.scatter(relative[0, a], relative[0, b], color='#c26954', s=40, zorder=4)
        # Direction marks distinguish the two loops at their shared crossing.
        for at in (90, 210, 390, 510):
            ax.annotate('', xy=relative[at+6, [a, b]], xytext=relative[at, [a, b]],
                        arrowprops=dict(arrowstyle='->', color=COLORS[1]))
        names = 'XYZ'
        label(ax, f'{names[a]}{names[b]} 投影', f'{names[a]} / cm', f'{names[b]} / cm')
        ax.set_aspect('equal', adjustable='datalim')
    fig.suptitle('URDF 末端相对起始位置；红点为首尾；不是实机光轨或碰撞验证',
                 fontproperties=FONT, fontsize=13)
    fig.savefig(folder/'20-tip-path.png', dpi=150)
    plt.close(fig)
    if compare is not None:
        previous = Path(compare)
        old = json.loads((previous/'20-tip-samples.json').read_text())
        old_xyz = np.asarray(old['tip_xyz_m'])*100.
        old_relative = old_xyz-old_xyz[0]
        fig, ax = plt.subplots(figsize=(10, 7), constrained_layout=True)
        ax.plot(old_relative[:, 1], old_relative[:, 2], color=COLORS[1],
                ls='--', linewidth=2, label=f'{previous.name} / {old["time_s"][-1]:.2f} s')
        ax.plot(relative[:, 1], relative[:, 2], color=COLORS[0], linewidth=2,
                label=f'{folder.name} / {data["time_s"][-1]:.2f} s')
        ax.scatter(0., 0., color='#c26954', s=40, zorder=4)
        label(ax, '20 空间 8 字：同一比例下的 YZ 投影对比', 'Y / cm', 'Z / cm')
        ax.set_aspect('equal', adjustable='datalim')
        ax.legend()
        fig.suptitle('URDF 计划轨迹；红点为首尾；不是实测光轨或碰撞验证',
                     fontproperties=FONT, fontsize=13)
        fig.savefig(folder/'20-tip-path-comparison.png', dpi=150)
        plt.close(fig)
        print(folder/'20-tip-path-comparison.png')
    print(folder/'19-orientation.png')
    print(folder/'20-tip-path.png')
    render_fixed_tcp(folder, compare)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('folder', type=Path)
    parser.add_argument('--compare', type=Path, help='previous preview folder for 20 path and 21 joint/TCP comparisons')
    args = parser.parse_args()
    render(args.folder, args.compare)
