#!/usr/bin/env python3
"""Compile/check/preview a continuous dance sequence before optional playback."""
import argparse
import csv
import json
import os
import random
from pathlib import Path
import subprocess
import sys

import numpy as np
from rs_dance_library import load_library, DEFAULT_OUTPUT
from rs_dance_sequence import SequencePlan, connect, state, audit_samples
from rs_dance_speed import ScaledPlan, checked_trial_speed
from rs_dance_virtual_collision import CollisionAudit, validate_identity
from rs_gravity_record import ROOT, DEFAULT_CALIBRATION, DEFAULT_URDF, digest


def select_numbers(numbers, random_count=None, seed=None):
    """Select once before compilation; playback never draws another action."""
    pool=[f'{i:02}' for i in range(1,11)]
    if random_count is not None:
        if numbers:raise ValueError('--random 不能与手动动作编号同时使用')
        if not 1<=random_count<=10:raise ValueError('随机动作数量需在 1～10 之间')
        if seed is None:seed=random.SystemRandom().getrandbits(32)
        if not 0<=seed<=0xffffffff:raise ValueError('种子需在 0～4294967295 之间')
        selected=random.Random(seed).sample(pool,random_count)
        return selected,dict(mode='random',seed=seed,count=random_count,
                             replacement=False,pool=pool,algorithm='python-random-sample-v1')
    if seed is not None:raise ValueError('--seed 需要与 --random 一起使用')
    selected=[f'{int(n):02}' for n in numbers] if numbers else pool
    if not 1<=len(selected)<=32 or any(n not in pool for n in selected):
        raise ValueError('请输入 1～32 个动作编号（01～10），开场与回零由播放器处理')
    return selected,dict(mode='explicit' if numbers else 'all',count=len(selected))


def write_preview(plan, folder):
    times = np.linspace(0., plan.duration, int(np.ceil(plan.duration/.02))+1)
    with (folder/'trajectory.csv').open('w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(['t_s']+[f'J{j}_{unit}' for unit in ('rad','rad_s','rad_s2') for j in range(1,8)])
        for t in times:writer.writerow([t,*state(plan,t).ravel()])


def main(argv=None):
    parser = argparse.ArgumentParser(description='机械臂动作组合；check/preview 不连接 CAN')
    parser.add_argument('mode',choices=('check','preview','play'))
    parser.add_argument('numbers',nargs='*',help='动作编号 01～10；省略时按顺序组合全部十组')
    parser.add_argument('--random',dest='random_count',nargs='?',const=5,type=int,
                        help='随机选取 1～10 个不同动作；省略数量时选 5 个')
    parser.add_argument('--seed',type=int,help='复现同一随机顺序，例如 --random 5 --seed 42')
    parser.add_argument('--speed',type=float,default=1.,help='整段速度倍率 0.1～1；默认保持原速度')
    parser.add_argument('--all-pairs',action='store_true',help='check 时检查全部 100 种有向连接，含自身重复')
    parser.add_argument('--supported',action='store_true',help='实机已安装防坠支撑')
    parser.add_argument('--bundle',type=Path,default=DEFAULT_OUTPUT)
    parser.add_argument('--calibration',type=Path,default=DEFAULT_CALIBRATION)
    parser.add_argument('--urdf',type=Path,default=DEFAULT_URDF)
    parser.add_argument('--port',default='can0')
    parser.add_argument('--output',type=Path,default=ROOT/'build/dance-sequence')
    args=parser.parse_args(argv)
    try:
        try:speed=checked_trial_speed(args.speed)
        except ValueError:raise ValueError('整段速度倍率需在 0.1～1.0 之间') from None
        numbers,selection=select_numbers(args.numbers,args.random_count,args.seed)
        if args.all_pairs and args.mode!='check':raise ValueError('--all-pairs 仅用于 check')
        if args.mode=='play' and (not args.supported or not sys.stdin.isatty()):
            raise ValueError('实机播放需交互终端及 --supported；离线使用 check 或 preview')
        if selection['mode']=='random':
            print(f'随机 {len(numbers)} 个动作（不重复），种子 {selection["seed"]}：'+ ' → '.join(numbers),flush=True)
        print('正在离线加载并验证动作库……',flush=True)
        manifest,plans,gravity,identity,reports=load_library(args.calibration,args.urdf,args.bundle)
        collision=CollisionAudit(gravity)
        cache={};matrix=[]
        if args.all_pairs:
            for left in (f'{i:02}' for i in range(1,11)):
                for right in (f'{i:02}' for i in range(1,11)):
                    try:
                        cache[left,right]=connect(plans[left],plans[right],gravity,collision)
                        matrix.append(dict(from_number=left,to_number=right,passed=True,**cache[left,right][1]))
                    except ValueError as error:
                        matrix.append(dict(from_number=left,to_number=right,passed=False,error=str(error)))
                print(f'已检查 {left} → 01～10',flush=True)
        sequence=SequencePlan(numbers,plans,gravity,collision,cache)
        checks=sequence.audit(gravity,collision)
        executable=ScaledPlan(sequence,speed)
        execution_checks = checks if speed==1. else audit_samples(
            executable,gravity,collision,extra_times=sequence.ends/speed)
        folder=args.output.resolve()
        if selection['mode']=='random':folder=folder/f'random-{selection["seed"]}'
        folder.mkdir(parents=True,exist_ok=True)
        report=dict(schema='rebuddy.dance_sequence.v1',numbers=numbers,selection=selection,speed=speed,
                    source_total_s=sum(plans[n].duration for n in numbers),nominal_duration_s=sequence.duration,
                    playback_duration_s=executable.duration,layout=sequence.layout,connections=sequence.connections,
                    validation=checks,execution_validation=execution_checks,hardware_tested=False,animation_included=False,
                    collision_limitations='No environment or open-finger collision verification; no added payload assumed.',
                    identity=identity,library_manifest_sha256=digest(args.bundle/'manifest.json'),
                    source_hashes={n:reports[n]['source_sha256'] for n in sorted(set(numbers))},
                    runtime_period_s=.02,bookends='Measured entry to idle before sequence; hold after completion; quit returns to zero.',
                    playback_velocity_upper_bound_rad_s=executable.peak_velocity.tolist(),
                    playback_acceleration_upper_bound_rad_s2=executable.peak_acceleration.tolist())
        if matrix:
            report['pair_checks']=matrix
            report['all_pairs_passed']=all(x['passed'] for x in matrix)
        report_path=folder/'report.json'
        report_path.write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
        write_preview(executable,folder)
        print('组合：'+' → '.join(numbers))
        print(f'{len(numbers)} 个动作，{len(sequence.connections)} 段五次过渡；原速 {sequence.duration:.2f}s，'
              f'本次 {speed:.0%} / {executable.duration:.2f}s（不含起身和回零）。')
        for row in sequence.connections:
            print(f'  {row["from_number"]} → {row["to_number"]}：过渡 {row["duration_s"]:.2f}s')
        print(f'离线检查通过，报告：{report_path}')
        print('碰撞检查范围：闭合夹爪模型的自碰撞；未覆盖张开手指和周围环境。')
        if matrix and not report['all_pairs_passed']:
            print('有连接未通过，详见 pair_checks；未连接 CAN。',file=sys.stderr)
            return 2
        if args.mode=='preview':
            # Use the system plotting environment, isolated from the robot SDK's
            # NumPy environment. Neither plotting process imports a CAN backend.
            env=dict(os.environ,MPLCONFIGDIR=str(folder/'matplotlib-cache'))
            subprocess.run(['/usr/bin/python3','-I',str(ROOT/'tools/plot_dance_sequence.py'),str(folder)],check=True,env=env)
            print(f'曲线预览：{folder/"trajectory.png"}')
        if args.mode!='play':
            print('未连接 CAN，未使能电机。')
            return 0
        # All planning, reporting and geometry work ends before opening CAN.
        validate_identity(collision.identity)
        from rs_dance_console import run
        args.action='12';args.pivot_speed=None
        runtime_plans={'00':plans['00'],'11':plans['11'],'12':executable}
        runtime_reports={n:reports[n] for n in ('00','11')}
        runtime_reports['12']=dict(name='连续组合',duration_s=sequence.duration,source=str(report_path),
                                   source_sha256=digest(report_path),source_library='dance_sequence',source_number='12')
        print('开始整段播放；stop/Ctrl+C 制动保持；完成后保持待机。12 重播；status 状态；quit 回零退出。',flush=True)
        result=run(args,manifest,runtime_plans,gravity,identity,runtime_reports)
        print(f'实机日志：{result["folder"]}')
        for error in result['errors']:print(error,file=sys.stderr)
        return 0 if result['success'] else 1
    except (Exception, KeyboardInterrupt) as error:
        print(f'动作组合：{type(error).__name__}: {error}',file=sys.stderr)
        return 2


if __name__=='__main__':raise SystemExit(main())
