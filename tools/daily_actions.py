#!/usr/bin/env python3
"""Daily action terminal: offline inspection, read-only CAN probe, guarded play entry."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import shlex
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
NAMES = {
    'IDLE': ('眨眼', '待机', '待机眨眼'),
    'PROBE': ('抬头看你', '抬头'),
    'NUZZLE': ('轻蹭',),
    'TILT': ('疑惑歪头', '歪头'),
    'HAPPY': ('开心回应', '开心'),
    'REST': ('慢慢入睡', '入睡'),
    'SLEEP': ('熟睡呼吸与z', '熟睡', '睡眠'),
    'WAKE': ('迷糊唤醒', '唤醒'),
}
HELP = 'prepare 整理本地动作 | session --supported 持续试播终端\nlist 列表 | check 动作名 查看详情 | play 动作名 --supported 试播并保持\n实机终端 quit / Ctrl+C 回零后失能；熟睡失能后可继续唤醒。'
BLOCKER_LABELS = {
    'Observed drop': '上次突然下落的原因尚未查清。',
    'No confirmed brake': '防坠条件，以及正常、故障和进程退出时的停机方式尚未验收。',
    'Common interaction': '统一互动姿态、休息姿态、运动路径和夹爪开度尚未验收。',
    'All four recordings': '四段旧录制早于本次重新校零，坐标兼容性需要重新验证。',
}


def catalog():
    design = json.loads((ROOT/'record/daily_design.json').read_text())
    actions = {a['id']: a for a in design['actions']}
    if set(actions) != set(NAMES) or len(design['actions']) != 8:
        raise ValueError('日常动作设计清单不完整')
    return design, actions


def resolve_action(name, actions):
    key = name.strip().casefold()
    for id, aliases in NAMES.items():
        if key in (id.casefold(), *aliases, str(actions[id]['file_order']), f"{actions[id]['file_order']:02d}"):
            return actions[id]
    raise ValueError(f'未知动作：{name}。使用 list 查看名称；文件序号与内部动作 ID 不同。')


def inspect_action(name):
    design, actions = catalog()
    action = resolve_action(name, actions)
    path = (ROOT/action['visual_gif']).resolve()
    if not path.is_relative_to(ROOT/'assets/gifs/daily'):
        raise ValueError('动画路径不属于 daily 目录')
    if hashlib.sha256(path.read_bytes()).hexdigest() != action['visual_sha256']:
        raise ValueError('动画资源校验失败')
    bundle=ROOT/'record/daily_ready/manifest.json'
    if bundle.exists():
        manifest=json.loads(bundle.read_text());prepared=manifest['actions'][action['id']]
        for filename,expected in ((prepared.get('plan_file'),prepared.get('plan_sha256')),
                                  (prepared.get('preview_csv'),prepared.get('preview_sha256'))):
            if filename and hashlib.sha256((bundle.parent/filename).read_bytes()).hexdigest()!=expected:
                raise ValueError('整理后的动作校验失败；请重新 prepare')
        return dict(action=action['id'],name=NAMES[action['id']][0],file_order=action['file_order'],
            intent=action['intent'],entry_pose=prepared.get('entry','ATTENTIVE'),exit_pose=prepared['exit'],
            screen_duration_s=action['screen_duration_ms']/1000,physical_duration_s=prepared['duration_s'],
            phases=action['phases'],gif=str(path),recording_reference=prepared.get('source'),
            motion_ready=True,hardware_accepted=False,hardware_opened=False,prepared=prepared,
            transition_note='慢慢入睡到零位保持使能；熟睡在零位失能；唤醒使能后慢慢回待机。',
            note='PC 本地实机试播；C5 可选原生 CAN 联动固件已接入，尚未进行 C5 实机验收。',blockers=[])
    return dict(action=action['id'], name=NAMES[action['id']][0],
        file_order=action['file_order'], intent=action['intent'],
        entry_pose=action['entry_pose'], exit_pose=action['exit_pose'],
        screen_duration_s=action['screen_duration_ms']/1000,
        physical_duration_s=None if action['physical_duration_ms'] is None else action['physical_duration_ms']/1000,
        phases=action['phases'], gif=str(path), recording_reference=None,
        motion_ready=False, hardware_opened=False,
        note='当前为动作设计检查，不播放 GIF，也不驱动机械臂。眨眼和 Z 仅由屏幕表现。',
        transition_note=('从正常互动状态需要先完成已验收的入睡过渡。' if action['id']=='SLEEP' else
                         '需要先确认实机位于已验收的休息姿态，不能从正常姿态直接套用唤醒轨迹。' if action['id']=='WAKE' else
                         '入口姿态必须由实测反馈验证，不能凭命令名假定。'),
        blockers=[next((label for prefix,label in BLOCKER_LABELS.items() if reason.startswith(prefix)),reason)
                  for reason in design['release_blockers']])


def probe(port):
    # Lazy import: listing/design checks need only Python's standard library.
    from rs_record_playback import Hardware
    hw = None
    report = dict(mode='read_only_probe', success=False, motion_requested=False,
                  enable_requested=False, port=port, python=sys.executable)
    try:
        hw = Hardware(port)
        report['motorbridge'] = hw.driver_version
        report['positions_deg'] = (hw.positions()*180/3.141592653589793).tolist()
        hw.reports()
        hw.drain()
        report['motor_status'] = {str(k): dict(mode=v[1], fault_bits=v[2]) for k,v in hw.states.items()}
        hw.check_status(0)
        report['success'] = True
        report['note'] = '七路反馈检查通过且为未使能状态；这不等于动作或停机安全验收。'
    except Exception as error:
        report['error'] = str(error)
    finally:
        if hw is not None:
            try:
                hw.close()
            except Exception as error:
                report['success'] = False
                report['close_error'] = str(error)
    folder = ROOT/'build/daily-actions'
    folder.mkdir(parents=True, exist_ok=True)
    path = folder/f'probe-{time.time_ns()}.json'
    report['log'] = str(path)
    path.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    return report


def emit(data, as_json=False):
    if as_json:
        print(json.dumps(data,ensure_ascii=False,indent=2))
    elif 'action' in data:
        print(f"{data['file_order']:02d} {data['name']} ({data['action']})")
        print(data['intent'])
        print(f"入口 {data['entry_pose']} → 出口 {data['exit_pose']}")
        duration=data['physical_duration_s']
        print(f"屏幕时长 {data['screen_duration_s']:g} 秒；" +
              (f'计划运动 {duration:.1f} 秒，待实机试播。' if duration is not None else '持续状态，无固定结束时间。'))
        print(data['transition_note'])
        print(data['note'])
        for reason in data['blockers']:
            print('  - '+reason)
    else:
        print(json.dumps(data,ensure_ascii=False,indent=2))


def trial(port,supported,initial_action=None):
    if not supported or not sys.stdin.isatty():raise ValueError('实机试播需要交互终端和 --supported')
    from types import SimpleNamespace
    from rs_gravity_record import DEFAULT_CALIBRATION,DEFAULT_URDF
    from rs_daily_prepare import BUNDLE
    from rs_daily_session import run_session
    args=SimpleNamespace(port=port,supported=True,calibration=DEFAULT_CALIBRATION,urdf=DEFAULT_URDF,bundle=BUNDLE)
    result=run_session(args,initial_action)
    emit(result,True)
    return 0 if result['success'] else 1


def dispatch(command, action=None, *, port='can0', as_json=False, supported=False):
    if command in ('help','帮助'):
        print(HELP)
        return 0
    if command in ('list','列表'):
        _, actions = catalog()
        rows = [dict(number=a['file_order'],id=a['id'],name=NAMES[a['id']][0],
                     screen_s=a['screen_duration_ms']/1000,motion_ready=(ROOT/'record/daily_ready/manifest.json').exists())
                for a in sorted(actions.values(),key=lambda a:a['file_order'])]
        if as_json:emit(rows,True)
        else:
            for r in rows:
                print(f"{r['number']:02d}  {r['name']}")
        return 0
    if command == 'probe':
        report = probe(port)
        emit(report,as_json)
        return 0 if report['success'] else 1
    if command == 'prepare':
        from rs_daily_prepare import prepare_bundle
        from rs_gravity_record import DEFAULT_CALIBRATION,DEFAULT_URDF
        result=prepare_bundle(DEFAULT_CALIBRATION,DEFAULT_URDF)
        emit({k:dict(seconds=v['duration_s'],source=v.get('source')) for k,v in result['actions'].items()},True)
        return 0
    if command == 'session':return trial(port,supported)
    if command in ('check','play'):
        if not action:raise ValueError(f'{command} 后需要动作名称')
        result = inspect_action(action)
        if command == 'play':
            if supported and result['motion_ready']:
                return trial(port,supported,result['action'])
            result['note'] = ('动作已整理；使用交互终端和 --supported 试播。未打开 CAN。' if result['motion_ready'] else
                             '等待重新录制的抬头动作；旧轨迹已停用，未打开 CAN。'
                              if result['action']=='PROBE' else '这个动作尚未接入实机播放；未打开 CAN。')
            if as_json:
                emit(result,True)
            else:
                print(result['note'])
            return 3
        emit(result,as_json)
        return 0
    if action is not None:raise ValueError('未知命令；使用 help 查看格式')
    return dispatch('play',command,port=port,as_json=as_json,supported=supported)


def interactive(port,supported=False):
    print('reBuddy 日常动作')
    dispatch('list')
    print('输入编号或名称；help 帮助，quit 退出。')
    while True:
        try:
            words = shlex.split(input('daily> ').strip())
            if not words:continue
            if words[0] in ('quit','exit','退出'):
                return 0
            if len(words)>2:raise ValueError('每次只输入一个命令和一个动作名')
            dispatch(*words,port=port,supported=supported)
        except (EOFError,KeyboardInterrupt):
            print('\n已退出命令终端。')
            return 0
        except (ValueError,OSError,RuntimeError) as error:
            print(f'错误：{error}',file=sys.stderr)


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__,epilog=HELP)
    parser.add_argument('command',nargs='?',help='list/check/play/probe，或动作名称；省略进入交互终端')
    parser.add_argument('action',nargs='?',help='中文动作名、英文 ID 或 01–08 文件序号')
    parser.add_argument('--port',default='can0')
    parser.add_argument('--json',action='store_true',dest='as_json')
    parser.add_argument('--supported',action='store_true',help='仅在固定支撑或防坠吊带已安装且不妨碍路径时使用；不是仅清空桌面')
    args=parser.parse_args(argv)
    try:
        if args.command is None:
            if args.as_json:raise ValueError('--json 需要指定命令')
            if args.supported:return trial(args.port,True)
            return interactive(args.port,args.supported)
        if args.action is not None and args.command not in ('check','play'):
            raise ValueError('仅 check/play 接受第二个动作参数')
        return dispatch(args.command,args.action,port=args.port,as_json=args.as_json,supported=args.supported)
    except (ValueError,OSError,RuntimeError) as error:
        print(f'错误：{error}',file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        return 130


if __name__=='__main__':
    raise SystemExit(main())
