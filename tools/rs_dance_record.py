#!/usr/bin/env python3
"""Simple dance teaching UI over the shared gravity recorder; no playback."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import re
import sys

import numpy as np

import rs_gravity_record as recorder

ACTIONS = (
    ('nod', '点头打拍', 33), ('lift', '抬臂舒展', 34),
    ('dip', '深蹲回升', 35), ('reach', '前探伸展', 36),
    ('base_turn', '底盘回转', 37), ('sway', '大幅摇摆', 38),
    ('elbow', '折肘展开', 39), ('wrist', '抬头俯看', 40),
    ('grip', '夹爪开合', 41), ('wave', '大幅波浪', 42),
)
DEFAULT_OUTPUT = recorder.ROOT / 'record/dance'
DAILY_MANIFEST = recorder.ROOT / 'record/daily_ready/manifest.json'


def daily_idle_pose(metadata, gravity, path=None):
    path = DAILY_MANIFEST if path is None else path
    data = json.loads(path.read_text())
    if data.get('schema') != 'rebuddy.daily.prepared.v1':
        raise ValueError('Daily 待机数据格式不匹配，未连接机械臂。')
    for key in ('calibration_sha256', 'urdf_sha256'):
        if not metadata.get(key) or data.get(key) != metadata[key]:
            raise ValueError(f'Daily 待机姿态与当前标定不一致：{key}；未连接机械臂。')
    q = recorder.vector(data['idle_rad'])
    from rs_record_playback import LOWER, UPPER
    if (np.any(q < LOWER) or np.any(q > UPPER) or
            np.any(q[:6] < gravity.lower) or np.any(q[:6] > gravity.upper)):
        raise ValueError('Daily 待机姿态超出关节限位，未连接机械臂。')
    return dict(schema='rebuddy.idle_pose.v1', name='ATTENTIVE', mock=False,
                calibration_sha256=metadata['calibration_sha256'],
                urdf_sha256=metadata['urdf_sha256'], joint_names=recorder.NAMES,
                q_rad=q.tolist(), source_manifest=str(path),
                source_sha256=recorder.digest(path), hardware_accepted=False)


class DanceProfile:
    shared_daily_pose = False

    def metadata(self):
        return dict(schema='rebuddy.dance_record.v1',
                    actions=[dict(key=k, name=n, action_id=i, recording_required=k != 'grip',
                                  source='procedural' if k == 'grip' else 'recorded')
                             for k, n, i in ACTIONS],
                    endpoint_tolerance_deg=2., stable_window_s=1., stable_spread_deg=.5,
                    gripper='fixed opening during teaching; DANCE_GRIP reserved for later procedural control',
                    bookend_recording_required=False,
                    bookends=[
                        dict(action_id=43, file='00_start.gif', source='daily',
                             daily_action='WAKE', target_pose='ATTENTIVE',
                             recording_required=False,
                             entry_policy='verify current pose; reuse daily approach to ATTENTIVE; skip if already there'),
                        dict(action_id=44, file='11_end.gif', source='daily',
                             daily_action='REST', name='慢慢入睡',
                             recording_required=False),
                    ],
                    motion_binding_status='planned_not_connected',
                    startup_pose_source='daily_ready' if self.shared_daily_pose else 'manual_preset',
                    endpoint_policy='stop automatically returns to shared idle; record measured return and verify stable endpoint',
                    hardware_accepted=False)

    def help(self, has_pose=False):
        rows = ['基础舞蹈录制']
        rows += [f'{i:02d} {name}' + ('（程序生成，无需录制）' if key == 'grip' else '')
                 for i, (key, name, _) in enumerate(ACTIONS, 1)]
        rows += ['录制 01～08、10；开场复用 Daily 起始姿态，收尾复用慢慢入睡。',
                 '输入编号开始；stop 自动回位并结束片段；estop 制动保持；quit 回零退出。',
                 '开始前在待机姿态保持 1 秒；结束时无需手动回位。' if has_pose else
                 '启动后自动到 Daily 待机姿态。' if self.shared_daily_pose else
                 '先摆好互动姿态，保持 1 秒，输入 pose 保存。']
        return '\n'.join(rows)

    def command(self, line, used):
        line = line.strip()
        if self.shared_daily_pose and line.startswith('pose'):
            raise ValueError('当前共用 Daily 待机姿态，无需重新保存 pose。')
        aliases = {'pose': 'pose idle', 'pose replace': 'pose idle replace',
                   '结束': 'stop', '退出': 'quit', '帮助': 'help', 'list': 'help'}
        if line in aliases:
            return aliases[line]
        for index, (key, name, _) in enumerate(ACTIONS, 1):
            if line in (str(index), f'{index:02d}', key, name):
                if key == 'grip':
                    raise ValueError('09 夹爪开合由后续程序生成，无需录制；请选择 01～08 或 10。')
                take = 1
                while f'dance_{key}_{take:02d}' in used:
                    take += 1
                return f'start dance_{key}_{take:02d}'
        # Keep advanced start syntax, but anchor() rejects unknown/unsupported names.
        return line or 'help'

    def stable_at_pose(self, pose, q, history, metadata, folder):
        if pose is None:
            raise ValueError('先摆好互动姿态，保持 1 秒，输入 pose。')
        try:
            recorder.require_idle_match(q, pose)
            measured = recorder.capture_idle_pose(history, metadata, folder)
            for _, sample in history:
                recorder.require_idle_match(sample, pose)
        except ValueError as error:
            raise ValueError('请回到统一姿态并保持 1 秒，再重试。' + str(error)) from error
        return measured

    def anchor(self, name, pose, q, history, metadata, folder):
        for key, chinese, action_id in ACTIONS:
            if re.fullmatch(r'dance_' + key + r'_\d{2,}', name):
                if key == 'grip':
                    raise ValueError('09 夹爪开合由后续程序生成，无需录制；请选择 01～08 或 10。')
                measured = self.stable_at_pose(pose, q, history, metadata, folder)
                return dict(action=f'DANCE_{key.upper()}', action_id=action_id, name=chinese,
                            idle_pose=pose, entry_idle=True, exit_idle=True,
                            entry_verified=measured)
        raise ValueError('请输入 01～10 的动作编号。')

    def validate_end(self, anchor, q, history, metadata, folder):
        if not anchor:
            raise ValueError('片段缺少首尾姿态信息，不能标记完成。')
        return self.stable_at_pose(anchor['idle_pose'], q, history, metadata, folder)


def main(argv=None):
    parser = argparse.ArgumentParser(description='基础舞蹈拖动录制：编号开始，stop 自动回位并结束片段，quit 回零退出。')
    parser.add_argument('--check', action='store_true', help='仅离线检查环境，不连接机械臂')
    parser.add_argument('--list', action='store_true', help='只显示动作菜单')
    parser.add_argument('--supported', action='store_true', help='确认已安装能独立承重的防坠支撑')
    parser.add_argument('--manual-start', action='store_true', help='高级手动模式：跳过 Daily 姿态，手动设置独立预设')
    parser.add_argument('--port', default='can0')
    parser.add_argument('--output', type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument('--calibration', type=Path, default=recorder.DEFAULT_CALIBRATION)
    parser.add_argument('--urdf', type=Path, default=recorder.DEFAULT_URDF)
    args = parser.parse_args(argv)
    args.recording_profile = DanceProfile()
    args.recording_profile.shared_daily_pose = not args.manual_start
    if args.list:
        print(args.recording_profile.help())
        return 0
    args.mode = 'check' if args.check else 'record'
    args.seconds = 0.
    try:
        if not args.check:
            if not args.supported:
                raise ValueError('安装防坠支撑后，用 record_dance --supported 开始。查看菜单用 --list。')
            if not sys.stdin.isatty():
                raise ValueError('请在交互终端运行。')
        gravity, metadata = recorder.preflight(args.calibration, args.urdf)
        if not args.manual_start:
            args.startup_pose = daily_idle_pose(metadata, gravity)
        if args.check:
            print('离线检查通过，待机姿态已核对，未连接 CAN。')
            return 0
        print(f'保存目录：{args.output}')
        print('将先回零，再到 Daily 待机姿态；到位后等待动作编号。' if not args.manual_start else
              '手动模式：等待就绪后设置 pose。')
        result = recorder.run(args, gravity, metadata)
        print(f'数据目录：{result["folder"]}')
        print('录制会话已结束。' if result['success'] else '会话异常结束，请查看 summary.json。')
        for error in result['errors']:
            print(error, file=sys.stderr)
        return 0 if result['success'] else 1
    except Exception as error:
        print(f'舞蹈录制：{error}', file=sys.stderr)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
