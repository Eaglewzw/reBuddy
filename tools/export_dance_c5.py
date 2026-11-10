#!/usr/bin/env python3
"""Export checked Dance polynomials and 100 connection windows; never opens CAN."""
import argparse
import ctypes as C
import hashlib
import json
import math
from pathlib import Path
import subprocess
import tempfile
import numpy as np
from rs_dance_library import load_library, DEFAULT_OUTPUT
from rs_dance_sequence import QuinticBridge, state, connect, audit_samples
from rs_dance_virtual_collision import CollisionAudit
from rs_gravity_record import ROOT, DEFAULT_CALIBRATION, DEFAULT_URDF, digest
from export_daily_c5 import f

OUTPUT=ROOT/'components/rebuddy_core/src/rb_dance_data.c'
REPORT=ROOT/'assets/verification/dance_c5_mapping.json'
ERROR=np.array([2e-5,3e-4,5e-3])  # rad, rad/s, rad/s^2; includes native float evaluation


def check_polynomial(c,duration,bridge=False):
    """Interior extrema, including overshoot between the collision samples."""
    from rs_record_playback import LOWER, UPPER
    for j,row in enumerate(c):
        poly=np.polynomial.Polynomial(row)
        for order in range(4 if bridge else 3):
            roots=poly.deriv(order+1).roots()
            points=[0.,1.]+[float(r.real) for r in roots if abs(r.imag)<1e-8 and 0<r.real<1]
            values=poly.deriv(order)(points)/duration**order
            if order==0:
                if min(values)<LOWER[j]-1e-6 or max(values)>UPPER[j]+1e-6:raise ValueError('exported joint extrema')
            else:
                limit=([0,2.8,10.,20.] if j==6 else [0,1.,2.,20.])[order]
                if max(abs(values))>limit+1e-4:raise ValueError('exported derivative extrema')


def pieces(plan):
    result=[]
    def interval(begin,end):
        curve=QuinticBridge(state(plan,begin),state(plan,end),end-begin)
        curve.coefficients=curve.coefficients.astype(np.float32).astype(float)
        errors=np.max([np.max(abs(state(curve,(end-begin)*u)-state(plan,begin+(end-begin)*u)),axis=1)
                       for u in np.linspace(0,1,17)],axis=0)
        if np.any(errors>ERROR*.25):
            if end-begin<.001:raise ValueError(f'Cannot approximate [{begin},{end}]: {errors}')
            middle=float(np.float32((begin+end)*.5));interval(begin,middle);interval(middle,end)
        else:
            check_polynomial(curve.coefficients.T,end-begin)
            result.append((begin,end,curve.coefficients.T))
    boundaries=[0.,plan.duration];offset=0.
    for _,curve in plan.stages:
        boundaries.extend([offset,offset+curve.duration])
        if hasattr(curve,'t_s'):boundaries.extend(offset+np.asarray(curve.t_s))
        if hasattr(curve,'pulse_starts_s'):
            for start in curve.pulse_starts_s:boundaries.extend([offset+start,offset+start+curve.pulse.duration])
        if hasattr(curve,'dance_interval_s'):boundaries.extend(offset+np.asarray(curve.dance_interval_s))
        if hasattr(curve,'bridge'):boundaries.extend([offset+curve.bridge.start_s,offset+curve.bridge.end_s])
        offset+=curve.duration
    times=np.unique(np.r_[np.linspace(0,plan.duration,math.ceil(plan.duration/.2)+1),boundaries].astype(np.float32))
    for a,b in zip(times,times[1:]):interval(float(a),float(b))
    return result


class Part(C.Structure):
    _fields_=[('begin',C.c_float),('end',C.c_float),('source',C.c_float),('number',C.c_uint8),('bridge',C.c_uint8)]
class NativePlan(C.Structure):
    _fields_=[('count',C.c_uint8),('part_count',C.c_uint8),('numbers',C.c_uint8*32),
              ('duration',C.c_float),('speed',C.c_float),('nominal',C.c_float),
              ('parts',Part*63),('bridges',((C.c_float*6)*7)*31)]
class NativeCurve:
    def __init__(self,lib,plan,start=0.,duration=None):
        self.lib,self.plan,self.offset=lib,plan,start
        self.duration=plan.duration if duration is None else duration
    def sample(self,t):
        q,v,a=(np.zeros(7,dtype=np.float32) for _ in range(3))
        ptr=C.POINTER(C.c_float)
        if not self.lib.rb_dance_sample(C.byref(self.plan),float(t+self.offset),
                *[x.ctypes.data_as(ptr) for x in (q,v,a)],None,None):raise ValueError('C5 evaluator rejected sample')
        return np.array([q,v,a],float)
    def at(self,t):return tuple(self.sample(t)[:2])
    def acceleration_at(self,t):return self.sample(t)[2]


def validate_native(code,plans,gravity,collision,pairs):
    with tempfile.TemporaryDirectory() as temp:
        temp=Path(temp);(temp/'data.c').write_text(code)
        include=ROOT/'components/rebuddy_core/include'
        subprocess.run(['gcc','-std=c11','-O2','-Wall','-Wextra','-Werror','-shared','-fPIC',f'-I{include}',
                        str(ROOT/'components/rebuddy_core/src/rb_dance.c'),str(temp/'data.c'),'-lm','-o',str(temp/'dance.so')],check=True)
        lib=C.CDLL(str(temp/'dance.so'));ptr=C.POINTER(C.c_float)
        lib.rb_dance_compile.argtypes=[C.POINTER(NativePlan),C.POINTER(C.c_uint8),C.c_uint,C.c_float]
        lib.rb_dance_compile.restype=C.c_bool
        lib.rb_dance_sample.argtypes=[C.POINTER(NativePlan),C.c_float,ptr,ptr,ptr,C.c_void_p,C.c_void_p]
        lib.rb_dance_sample.restype=C.c_bool
        maxima=np.zeros(3);rows=[]
        for number in range(1,11):
            source=plans[f'{number:02}']
            for speed in (1.,.5):
                native=NativePlan()
                if not lib.rb_dance_compile(C.byref(native),(C.c_uint8*1)(number),1,speed):raise ValueError('C compile failed')
                curve=NativeCurve(lib,native)
                for t in np.linspace(0,curve.duration,math.ceil(curve.duration/.005)+1):
                    expected=state(source,t*speed);expected[1]*=speed;expected[2]*=speed**2
                    maxima=np.maximum(maxima,np.max(abs(state(curve,t)-expected),axis=1))
                rows.append(dict(number=number,speed=speed,validation=audit_samples(curve,gravity,collision)))
        join_errors=np.zeros(3);join_checks=0
        for left in range(1,11):
            for right in range(1,11):
                original=pairs[left-1][right-1][0]
                for speed in (1.,.5):
                    native=NativePlan()
                    if not lib.rb_dance_compile(C.byref(native),(C.c_uint8*2)(left,right),2,speed):raise ValueError('C join failed')
                    part=native.parts[1];start=part.begin/speed;duration=(part.end-part.begin)/speed
                    check_polynomial(np.array(native.bridges[0]),duration,bridge=True)
                    curve=NativeCurve(lib,native,start,duration)
                    audit_samples(curve,gravity,collision,step=.01)
                    for t in np.linspace(0,duration,41):
                        expected=state(original,t*speed);expected[1]*=speed;expected[2]*=speed**2
                        join_errors=np.maximum(join_errors,np.max(abs(state(curve,t)-expected),axis=1))
                    join_checks+=1
        if np.any(maxima>ERROR) or np.any(join_errors>ERROR*2):
            raise ValueError(f'C float approximation too large: body={maxima}; joins={join_errors}')
        return dict(max_action_error_q_v_a=maxima.tolist(),max_join_error_q_v_a=join_errors.tolist(),
                    tolerance_q_v_a=ERROR.tolist(),validated_native_joins=join_checks,
                    validated_speeds=[.5,1.],native_plan_ram_bytes=C.sizeof(NativePlan),actions=rows)


def render():
    _,plans,gravity,identity,_=load_library(DEFAULT_CALIBRATION,DEFAULT_URDF)
    collision=CollisionAudit(gravity);lines=['/* Generated offline by tools/export_dance_c5.py. */','#include "rb_dance.h"',
        'const char rb_dance_calibration_sha256[]="'+identity['calibration_sha256']+'";',
        'const char rb_dance_urdf_sha256[]="'+identity['urdf_sha256']+'";']
    entries=[];counts=[];pairs=[]
    for number in range(1,11):
        code=f'{number:02}';plan=plans[code];exported=pieces(plan);counts.append(len(exported))
        lines.append(f'static const rb_dance_piece_t action_{code}[]={{')
        for a,b,c in exported:lines.append('{'+f(a)+','+f(b)+',{'+','.join('{'+','.join(map(f,row))+'}' for row in c)+'}},')
        lines.append('};');entries.append('{'+f(exported[-1][1])+f',action_{code},{len(exported)}'+'}')
        print(f'Export {code}: {len(exported)} pieces',flush=True)
    lines.append('const rb_dance_action_t rb_dance_actions[10]={'+','.join(entries)+'};')
    lines.append('const rb_dance_join_t rb_dance_joins[10][10]={')
    for left in range(1,11):
        row=[];pair_row=[]
        for right in range(1,11):
            pair=connect(plans[f'{left:02}'],plans[f'{right:02}'],gravity,collision);pair_row.append(pair)
            row.append('{'+','.join(f(pair[1][k]) for k in ('left_trim_s','right_trim_s','duration_s'))+'}')
        pairs.append(pair_row);lines.append('{'+','.join(row)+'},')
    lines.append('};');code='\n'.join(lines)+'\n'
    checks=validate_native(code,plans,gravity,collision,pairs)
    sources=[DEFAULT_OUTPUT/'manifest.json',*sorted(DEFAULT_OUTPUT.glob('[0-9]*.json')),
             ROOT/'tools/rs_dance_sequence.py',Path(__file__),ROOT/'components/rebuddy_core/src/rb_dance.c']
    report=dict(schema='rebuddy.dance.c5.v1',hardware_tested=False,motor_output_enabled=False,
                data_sha256=hashlib.sha256(code.encode()).hexdigest(),
                sources={str(p.relative_to(ROOT)):digest(p) for p in sources},
                calibration_sha256=identity['calibration_sha256'],urdf_sha256=identity['urdf_sha256'],
                piece_counts=counts,polynomial_data_bytes=sum(counts)*176+1200,
                connection_count=100,checks=checks,collision_identity=collision.identity,
                limitations=['Sampled self-collision with closed-finger geometry only; no environment or payload validation.',
                             'On-board random seed algorithm differs from Python; save the printed sequence to reproduce across platforms.'])
    return code,json.dumps(report,ensure_ascii=False,indent=2)+'\n'


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--check',action='store_true')
    args=parser.parse_args();code,report=render()
    for path,value in ((OUTPUT,code),(REPORT,report)):
        if args.check:
            if not path.exists() or path.read_text()!=value:raise SystemExit(f'Stale export: {path}')
        else:path.write_text(value)
    print('Dance C5 export checked; no CAN I/O')
