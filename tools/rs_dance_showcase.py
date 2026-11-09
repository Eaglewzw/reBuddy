"""Independent stage phrases: authored offline, never connects to CAN.

Arm time scaling uses analytic spline extrema. The spatial loop is solved using
this robot's URDF, then stored as joint knots; loading never invents new IK.
"""
from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares

import rs_dance_design as base
from rs_gravity_record import ROOT, CAPS, digest, preflight
from rs_record_playback import LOWER, UPPER

DEFAULT_OUTPUT = ROOT / 'record/dance_showcase/v10'
LEGACY_CATALOG = (
    (0,'start','起身入场'), (12,'lookback','回眸亮相'),
    (13,'ripple_reverse','反向涟漪'), (14,'spring_freeze','弹簧定格'),
    (15,'orbit','画圈绕场'), (16,'claw_beat','夹爪鼓点'),
    (17,'finale','谢幕连招'), (11,'end','慢慢入睡'),
)
V3_CATALOG = LEGACY_CATALOG[:-1] + (
    (18,'snake_wave','蛇形波浪'), (19,'steady_sway','稳定摇摆'),
    (20,'figure_eight','空间8字'),
) + LEGACY_CATALOG[-1:]
CATALOG=V3_CATALOG[:-1]+((21,'fixed_tcp_pivot','定点转动'),)+V3_CATALOG[-1:]


def _catalog(version):
    if version == 'v1':return LEGACY_CATALOG
    if version in ('v2','v3'):return V3_CATALOG
    if version in ('v4','v5','v6','v7','v8','v9','v10'):return CATALOG
    raise ValueError('unknown showcase library version')


def _spline(points, label, duration=None):
    q=np.asarray(points,float)
    t=np.linspace(0,1,len(q))
    curve=base.Curve(t,q,velocity_limit=1.,acceleration_limit=2.)
    # Scale the actual extrema, rather than stretching an arbitrary eight beats.
    shortest=curve.duration*1.005
    duration=shortest if duration is None else max(float(duration),shortest)
    return dict(kind='spline',label=label,duration_s=duration,
                t_s=(t*duration).tolist(),q_rad=q.tolist())


def _transition(a,b,label,duration=None):
    delta=abs(np.asarray(b)-a)
    minimum=max(float(np.max(1.875*delta)),math.sqrt(float(np.max(10/math.sqrt(3)*delta/2))))*1.005
    return dict(kind='quintic',label=label,duration_s=max(minimum,float(duration or 0)),
                start_rad=np.asarray(a).tolist(),end_rad=np.asarray(b).tolist())


def _c2(plan):
    for (_,left),(_,right) in zip(plan.stages,plan.stages[1:]):
        for a,b in ((left.at(left.duration)[0],right.at(0)[0]),
                    (left.at(left.duration)[1],right.at(0)[1]),
                    (left.acceleration_at(left.duration),right.acceleration_at(0))):
            if np.max(abs(a-b))>1e-7:raise ValueError('showcase stage seam is not C2')
    for stamp in (0,plan.duration):
        if max(np.max(abs(plan.at(stamp)[1])),np.max(abs(plan.acceleration_at(stamp))))>1e-7:
            raise ValueError('showcase endpoints must stop smoothly')


class CompositeSegment:
    """A continuous arm phrase with two unchanged approved gripper pulses."""
    def __init__(self,spec):
        self.body=base.DancePlan(spec['body_stages'])
        self.pulse=base.DancePlan(spec['pulse_stages'],gripper_motion=True)
        self.duration=float(spec['duration_s'])
        self.pulse_starts_s=np.asarray(spec['pulse_starts_s'],float)
        if (abs(self.duration-self.body.duration)>1e-9 or len(self.pulse_starts_s)!=2
            or not np.all(np.isfinite(self.pulse_starts_s))
            or self.pulse_starts_s[0]<0 or self.pulse_starts_s[-1]+self.pulse.duration>self.duration
            or np.any(np.diff(self.pulse_starts_s)<self.pulse.duration)):
            raise ValueError('invalid gripper pulse schedule')
        if spec['pulse_stages']!=base._gripper_cycle(self.body.start):
            raise ValueError('showcase gripper must use the approved 45 degree / 2 second pulse')
        _c2(self.body);_c2(self.pulse)
        self.start,self.end=self.body.start.copy(),self.body.end.copy()
        self.lower=np.min([c.lower for _,c in self.body.stages],axis=0)
        self.upper=np.max([c.upper for _,c in self.body.stages],axis=0)
        self.lower[6]=0;self.upper[6]=base.GRIPPER_BEAT_OPEN
        self.peak_velocity=self.body.peak_velocity.copy();self.peak_velocity[6]=self.pulse.peak_velocity[6]
        self.peak_acceleration=self.body.peak_acceleration.copy();self.peak_acceleration[6]=self.pulse.peak_acceleration[6]
        self.velocity_limit=1.;self.acceleration_limit=2.
        self.gripper_velocity_limit=base.GRIPPER_VMAX
        self.gripper_acceleration_limit=base.GRIPPER_AMAX
        self.gripper_following_limit=base.GRIPPER_FOLLOWING_LIMIT
        self.gripper_motion=True

    def _pulse_at(self,t):
        for start in self.pulse_starts_s:
            if start<=t<=start+self.pulse.duration:return float(t-start)
        return None

    def at(self,t):
        q,v=self.body.at(t);q=q.copy();v=v.copy();stamp=self._pulse_at(t)
        if stamp is not None:q[6],v[6]=self.pulse.at(stamp)[0][6],self.pulse.at(stamp)[1][6]
        return q,v

    def acceleration_at(self,t):
        a=self.body.acceleration_at(t).copy();stamp=self._pulse_at(t)
        if stamp is not None:a[6]=self.pulse.acceleration_at(stamp)[6]
        return a


class ShowcasePlan(base.DancePlan):
    def __init__(self,stages,*,bookend=False,gripper_motion=False):
        kind=stages[0].get('kind') if len(stages)==1 else None
        if kind in ('body_gripper_overlay','fixed_tcp_pivot','fixed_tcp_body','virtual_pivot','virtual_pivot_sway','virtual_pivot_joined'):
            if kind=='body_gripper_overlay':
                if bookend or not gripper_motion:raise ValueError('overlay requires gripper motion')
                curve=CompositeSegment(stages[0])
            else:
                if bookend or gripper_motion:raise ValueError('fixed TCP action cannot move gripper')
                if kind=='virtual_pivot_joined':
                    from rs_dance_pivot_join import JoinedSwaySegment
                    curve=JoinedSwaySegment(stages[0])
                    self.default_trial_speed=curve.default_trial_speed
                elif kind=='virtual_pivot_sway':
                    from rs_dance_pivot_sway import SwaySegment
                    curve=SwaySegment(stages[0])
                    self.default_trial_speed=curve.default_trial_speed
                elif kind=='virtual_pivot':
                    from rs_dance_virtual_pivot import VirtualPivotSegment
                    curve=VirtualPivotSegment(stages[0])
                    self.default_trial_speed=.3
                elif kind=='fixed_tcp_body':
                    from rs_dance_pivot_body import BodyPivotSegment
                    curve=BodyPivotSegment(stages[0])
                    self.default_trial_speed=1.
                else:
                    from rs_dance_pivot import PivotSegment
                    curve=PivotSegment(stages[0])
                    self.default_trial_speed=.3
            self.stages=[(stages[0]['label'],curve)]
            self.ends=np.array([curve.duration]);self.duration=curve.duration
            for attr in ('start','end','peak_velocity','peak_acceleration','velocity_limit','acceleration_limit',
                         'gripper_velocity_limit','gripper_acceleration_limit','gripper_following_limit','gripper_motion'):
                if hasattr(curve,attr):setattr(self,attr,getattr(curve,attr))
        else:super().__init__(stages,bookend=bookend,gripper_motion=gripper_motion)
        _c2(self)


def _orbit_points(gravity,idle):
    """Return 17 IK joint knots and evidence for one tilted 8 cm radius loop."""
    if not hasattr(gravity,'pin'):raise ValueError('spatial loop design requires the real URDF kinematics')
    pin,model,data=gravity.pin,gravity.model,gravity.data
    frame=model.getFrameId('gripper_end')
    def xyz(q):
        pin.forwardKinematics(model,data,q)
        pin.updateFramePlacements(model,data)
        return data.oMf[frame].translation.copy()
    anchor=xyz(idle[:6]);u=np.array([.2,.98,0.]);u/=np.linalg.norm(u)
    v=np.array([-.25,.05,.967]);v-=u*np.dot(u,v);v/=np.linalg.norm(v)
    radius=.08;points=[];errors=[];previous=idle[:6].copy()
    lower=np.maximum(np.asarray(gravity.lower)[:6],LOWER[:6])+1e-6
    upper=np.minimum(np.asarray(gravity.upper)[:6],UPPER[:6])-1e-6
    for u_time in np.linspace(0,1,17):
        theta=2*math.pi*(10*u_time**3-15*u_time**4+6*u_time**5)
        target=anchor+radius*(u*math.sin(theta)+v*(1-math.cos(theta)))
        preferred=idle[:6].copy();preferred[4]+=math.radians(10)*math.sin(theta)
        preferred[5]+=math.radians(22)*math.sin(theta)
        result=least_squares(lambda q:np.r_[100*(xyz(q)-target),.12*(q-preferred)],
                             previous,bounds=(lower,upper),ftol=1e-11,xtol=1e-11,gtol=1e-11,max_nfev=150)
        if not result.success:raise ValueError('orbit IK did not converge')
        previous=result.x
        errors.append(float(np.linalg.norm(xyz(previous)-target)))
        points.append(np.r_[previous,0.])
    points[0]=idle.copy();points[-1]=idle.copy()
    if max(errors)>.002:raise ValueError('orbit IK position residual exceeds 2 mm')
    return np.asarray(points),dict(method='URDF position IK; orientation follows authored joint preference',
        radius_m=radius,center_xyz_m=(anchor+radius*v).tolist(),plane_u=u.tolist(),plane_v=v.tolist(),
        knot_position_error_max_m=max(errors),precise_hover=False)


def _recipes(gravity,idle,bookends):
    p=lambda **kw:base._pose(idle,**kw)
    look=[idle,p(j1=25,j2=-9,j3=12,j4=4,j5=-10),p(j1=52,j2=-12,j3=20,j4=8,j5=-23,j6=12),
          p(j1=42,j2=-10,j3=15,j4=-8,j5=28,j6=-15),p(j1=-15,j2=8,j3=-5,j4=14,j5=-18,j6=8),idle]
    ripple=[idle,p(j2=20,j3=-5,j4=-8,j1=-8),p(j2=-10,j3=27,j4=-10,j5=12),
            p(j2=-5,j3=-10,j4=29,j1=12,j5=-14),p(j2=10,j3=16,j4=-20,j5=8),
            p(j2=-18,j3=-8,j4=14,j1=-8),idle]
    crouch=p(j1=-10,j2=16,j3=-17,j4=5,j6=-10)
    spring=p(j1=12,j2=-16,j3=26,j4=-8,j6=18)
    settle=[spring,p(j1=27,j2=-8,j3=14,j4=8,j5=-12),p(j1=-18,j2=8,j3=-12,j4=-16,j5=12),idle]
    orbit,orbit_evidence=_orbit_points(gravity,idle)
    body=[idle,p(j1=-18,j2=-8,j3=12,j4=8),p(j1=-22,j2=6,j3=-6,j4=-9),
          p(j1=16,j2=-10,j3=14,j4=7),p(j1=22,j2=7,j3=-7,j4=-10),idle]
    body=[idle+1.5*(q-idle) for q in body]
    body_stage=_spline(body,'body_accents',6.)
    if body_stage['duration_s']>6.000001:raise ValueError('body accents exceed six second budget')
    finale=[idle,p(j1=38,j2=-8,j3=16,j4=10,j5=-22,j6=12),
            p(j1=-30,j2=-14,j3=28,j4=8,j5=20,j6=-16),
            p(j1=10,j2=12,j3=-12,j4=-22,j5=-8),p(j1=-8,j2=-8,j3=10,j4=7),idle]
    recipes={0:([bookends['WAKE']],{},['从零位起身','进入互动姿态','等待亮相']),
      12:([_spline(look,'turn_then_counterlook')],{},['身体先转走','头部突然反向回望','划过另一侧回到正面']),
      13:([_spline(ripple,'wave_forward_and_back')],{},['肩部送出波峰','波峰传到腕部','腕部反送回肩肘']),
      14:([_transition(idle,crouch,'compress'),_transition(crouch,spring,'spring'),
           _transition(spring,spring,'freeze',.22),_spline(settle,'release_and_recover')],{},
          ['压低蓄力','快速舒展并定格 0.22 秒','斜向释放回到正面']),
      15:([_spline(orbit,'tilted_spatial_loop')],{'spatial_design':orbit_evidence},
          ['从近侧切入','末端绕过高点与远侧','闭合空间环路回原姿']),
      16:([dict(kind='body_gripper_overlay',label='body_and_claw_accents',duration_s=6.,
                body_stages=[body_stage],pulse_starts_s=[.5,3.5],pulse_stages=base._gripper_cycle(idle))],
          {'gripper_control':dict(profile='short_stroke',command_range_deg=[0.,45.],motor_range_deg=[0.,250.],
             full_open=False,following_limit_deg=10.8,velocity_limit_rad_s=1.6,acceleration_limit_rad_s2=5.1,
             torque_budget_nm=1.5,kp=5.,kd=.3,pulse_duration_s=2.,pulse_count=2)},
          ['侧身抬头打开短拍','闭合压拍并横移','另一侧再打一拍回正']),
      17:([_spline(finale,'look_stretch_bow_finale')],{},['左右回眸展开','舒展后深鞠躬','回到互动姿态结束']),
      11:([bookends['REST']],{},['从互动姿态收势','回到坐姿','结束']),}
    return recipes


def _validate(plan,gravity,number,idle,zero):
    expected_start=zero if number==0 else idle
    expected_end=zero if number==11 else idle
    if not np.allclose(plan.start,expected_start,rtol=0,atol=1e-9) or not np.allclose(plan.end,expected_end,rtol=0,atol=1e-9):
        raise ValueError('showcase endpoints differ from current Daily anchors')
    _c2(plan)
    check=base._validate_plan(plan,gravity)
    q=np.array([plan.at(t)[0] for t in np.linspace(0,plan.duration,1001)])
    if np.min(q[:,6]) < -1e-9 or np.max(q[:,6]) > base.GRIPPER_BEAT_OPEN+1e-9:
        raise ValueError('showcase J7 exceeds short-stroke range')
    v=np.array([plan.at(t)[1][:6] for t in np.linspace(0,plan.duration,1001)])
    check['mean_fastest_arm_axis_deg_s']=float(np.degrees(np.max(abs(v),axis=1)).mean())
    check['arm_axis_travel_deg']=np.degrees(np.sum(abs(np.diff(q[:,:6],axis=0)),axis=0)).tolist()
    if hasattr(gravity,'pin'):
        pin,model,data=gravity.pin,gravity.model,gravity.data
        frame=model.getFrameId('gripper_end');xyz=[];smallest=[]
        for sample in q[::5]:
            pin.forwardKinematics(model,data,sample[:6]);pin.updateFramePlacements(model,data)
            xyz.append(data.oMf[frame].translation.copy())
            jac=pin.computeFrameJacobian(model,data,sample[:6],frame,pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
            smallest.append(float(np.linalg.svd(jac[:3,:],compute_uv=False)[-1]))
        check['position_jacobian_min_singular_m_per_rad']=min(smallest)
        check['near_position_singularity']=min(smallest)<.01
        if number==15:
            xyz=np.asarray(xyz);center=xyz.mean(axis=0)
            # Best-fit circle evidence is independent of the loop's time warp.
            _,_,basis=np.linalg.svd(xyz-center,full_matrices=False)
            projected=(xyz-center)@basis[:2].T
            fitted=np.linalg.lstsq(np.c_[2*projected,np.ones(len(projected))],np.sum(projected**2,axis=1),rcond=None)[0]
            radius=math.sqrt(float(fitted[2]+np.sum(fitted[:2]**2)))
            radial=np.linalg.norm(projected-fitted[:2],axis=1)
            check['spatial_loop']=dict(fitted_radius_m=radius,
                radial_error_max_m=float(np.max(abs(radial-radius))),
                plane_error_max_m=float(np.max(abs((xyz-center)@basis[2]))),
                endpoint_gap_m=float(np.linalg.norm(xyz[-1]-xyz[0])),
                knot_step_max_deg=float(np.degrees(np.max(abs(np.diff(plan.stages[0][1].q_rad[:,:6],axis=0))))))
    return check


def _validate_extra(plan,gravity,number,idle,design):
    if number==21:
        if 'pivot_join_design' in design:
            from rs_dance_pivot_join import validate_join
            return validate_join(plan,gravity,idle,design)
        if 'pivot_sway_design' in design:
            from rs_dance_pivot_sway import validate_sway
            return validate_sway(plan,gravity,idle,design)
        if 'virtual_pivot_design' in design:
            from rs_dance_virtual_pivot import validate_virtual_pivot
            return validate_virtual_pivot(plan,gravity,idle,design)
        if 'body_design' in design:
            from rs_dance_pivot_body import validate_pivot_body
            return validate_pivot_body(plan,gravity,idle,design)
        from rs_dance_pivot import validate_pivot
        return validate_pivot(plan,gravity,idle,design)
    if number not in (18,19,20):return {}
    from rs_dance_showcase_extra import validate_extra
    return validate_extra(plan,gravity,number,idle,design=design)


def _story(number,plan,stages,intentions):
    narrative=[dict(order=i+1,intent=intent) for i,intent in enumerate(intentions)]
    if number==14:
        start=sum(c.duration for _,c in plan.stages[:2]);end=start+plan.stages[2][1].duration
        highlights=[dict(time_s=plan.stages[0][1].duration,label='蓄力最低姿态'),
                    dict(time_s=start,end_s=end,label='舒展后刻意定格 0.22 秒'),
                    dict(time_s=end+plan.stages[-1][1].t_s[2],label='斜向鞠躬释放')]
    elif number==16:
        highlights=[]
        for index,start in enumerate(stages[0]['pulse_starts_s']):
            highlights.extend([dict(time_s=start,end_s=start+2,label=f'第 {index+1} 次 45 度短拍'),
                dict(time_s=start+.95,end_s=start+1.05,label='短行程开口顶点'),
                dict(time_s=start+2,label='闭合压拍')])
    elif number in (12,13,17):
        indices={12:[2,3,4],13:[1,3,5],17:[1,2,3]}[number]
        labels={12:['身体转身 52 度','头部反向回望','掠过另一侧'],
                13:['肩部波峰','腕部波峰','反送回肩部'],
                17:['回眸亮相','伸展打开','深鞠躬']}[number]
        highlights=[dict(time_s=stages[0]['t_s'][i],label=label) for i,label in zip(indices,labels)]
    elif number==15:
        highlights=[dict(time_s=stages[0]['t_s'][8],label='空间环路远侧高点'),
                    dict(time_s=plan.duration,label='完整闭环回到互动姿态')]
    else:highlights=[dict(time_s=plan.duration,label=intentions[-1])]
    return narrative,highlights


def _bytes(data):return (json.dumps(data,ensure_ascii=False,indent=2)+'\n').encode()


def build_library(calibration,urdf,output=DEFAULT_OUTPUT,*,library_version='v10',pivot_config=None,pivot_scan=None):
    catalog=_catalog(library_version)
    if pivot_config is not None and library_version not in ('v4','v5','v6','v7','v8','v9','v10'):
        raise ValueError('pivot config requires showcase v4 through v10')
    if pivot_scan is not None and library_version!='v6':
        raise ValueError('pivot scan requires showcase v6')
    gravity,identity=preflight(Path(calibration),Path(urdf))
    if pivot_scan is not None:
        for key in ('calibration_sha256','urdf_sha256'):
            if pivot_scan.get('identity',{}).get(key)!=identity[key]:
                raise ValueError('pivot scan calibration or model differs from this robot')
        if pivot_config is None:pivot_config=pivot_scan['config']
    idle,zero,sources,bookends=base._anchors(identity)
    recipes=_recipes(gravity,idle,bookends);actions=[];payloads={}
    if library_version in ('v2','v3','v4','v5','v6','v7','v8','v9','v10'):
        from rs_dance_showcase_extra import extra_recipes, LARGE_EIGHT_SIZE, FAST_EIGHT_TIMING
        if library_version in ('v4','v5','v6','v7','v8','v9','v10'):
            recipes.update(extra_recipes(gravity,idle,eight_size=LARGE_EIGHT_SIZE,eight_timing=FAST_EIGHT_TIMING))
            if library_version=='v10':
                from rs_dance_pivot_join import build_recipe
            elif library_version in ('v7','v8','v9'):
                from rs_dance_pivot_sway import build_recipe,LOWERED_CONFIG,AMPLIFIED_CONFIG
                if library_version=='v8':pivot_config=dict(LOWERED_CONFIG,**(pivot_config or {}))
                if library_version=='v9':pivot_config=dict(AMPLIFIED_CONFIG,**(pivot_config or {}))
            elif library_version=='v6':
                from rs_dance_virtual_pivot import build_recipe
            elif library_version=='v5':
                from rs_dance_pivot_body import build_recipe
            else:
                from rs_dance_pivot import build_recipe
            recipes[21]=build_recipe(gravity,idle,config=pivot_config,**({'scan':pivot_scan} if library_version=='v6' else {}))
        elif library_version=='v3':recipes.update(extra_recipes(gravity,idle,eight_size=LARGE_EIGHT_SIZE))
        else:recipes.update(extra_recipes(gravity,idle))
    for number,key,name in catalog:
        stages,extra,intentions=recipes[number]
        plan=ShowcasePlan(stages,bookend=number in (0,11),gripper_motion=number==16)
        validation=_validate(plan,gravity,number,idle,zero)
        validation.update(_validate_extra(plan,gravity,number,idle,extra))
        duration=plan.duration
        beats=None if number in (0,11) or (number==21 and library_version in ('v7','v8','v9','v10')) else extra.get('virtual_pivot_design',extra.get('body_design',extra.get('pivot_design')))['config']['beats'] if number==21 else 8
        narrative,highlights=_story(number,plan,stages,intentions)
        spec=dict(schema='rebuddy.dance_showcase_action.v1',number=number,key=key,name=name,action_id=None,
            firmware_integrated=False,hardware_accepted=False,description='；'.join(intentions),
            beats=beats,bpm=None if beats is None else beats*60/duration,duration_s=duration,
            start_pose='ZERO' if number==0 else 'ATTENTIVE',end_pose='ZERO' if number==11 else 'ATTENTIVE',
            start_rad=plan.start.tolist(),end_rad=plan.end.tolist(),gripper_motion=number==16,
            design_difference={0:'复用 Daily 起身六轴轨迹',
                12:'相比单独底盘回转，身体先转、头部反向回望，再掠过另一侧',
                13:'相比重复同向波浪，波峰从肩传到腕，再反送回来',
                14:'相比等速蹲起，压低与快速舒展形成对比，仅在高点刻意定格 0.22 秒',
                15:'相比单轴摇摆，末端在真实 URDF 模型中完成倾斜的空间圆环',
                16:'相比单独夹爪开合，两次批准短拍与左右身体重拍同步',
                17:'回眸、展开、鞠躬连续衔接，完成后仍回互动姿态',
                18:'同频正弦沿关节依次错相，形成连续传播的蛇形波',
                19:'底盘摆动时腕部反向补偿，使夹爪世界朝向基本稳定；末端位置仍随身体移动',
                20:'末端在笛卡尔空间完成8字双环，沿中点连续穿越，适合加装轻小光源观察轨迹',
                21:'固定模型TCP空间位置，以点头、锥形朝向、姿态8字和定格完成按拍编舞',
                11:'复用 Daily 慢慢入睡六轴轨迹'}[number],
            calibration_sha256=identity['calibration_sha256'],urdf_sha256=identity['urdf_sha256'],dependencies=sources,
            narrative=narrative,highlights=highlights,
            stages=stages,validation=validation)
        spec.update(extra)
        if number==21 and library_version=='v5':
            spec['design_difference']='固定TCP位置，以J1明确左右摆动带动身体，其余关节补偿；末端朝向随动作变化'
            spec['default_trial_speed']=1.
        if number==21 and library_version=='v6':
            spec['design_difference']='工具轴外移的空间点P固定，工具与手臂围绕P运动；可达幅度见包络扫描'
            spec['default_trial_speed']=.3
            virtual=extra['virtual_pivot_design']
            spec['bpm']=virtual['actual_bpm']
            spec['bpm_kind']=virtual['actual_bpm_kind']
            spec['variable_bpm']=virtual['variable_bpm']
            axis=virtual['config']['cone_axis']
            spec['name']='空间定点绕动' if axis=='world_z' else '空间定点绕动（工具轴参考）'
        if number==21 and library_version in ('v7','v8','v9','v10'):
            spec['name']='定点左右上下'
            spec['design_difference']='同一虚拟P全程固定，左右一次、上下一次，回到互动姿态'
            if library_version=='v8':
                spec['name']='低位定点左右上下'
                spec['design_difference']='P下移后，左右和上下各大幅摆一次，平滑回到互动姿态'
            spec['default_trial_speed']=plan.default_trial_speed
            if library_version in ('v9','v10'):
                spec['name']='大幅定点左右上下'
                spec['design_difference']='降低P并调整准备朝向，放大左右上下摆幅，分程配速，默认正常速度'
        if number==21 and library_version=='v10':
            spec['design_difference']='保留v9摆幅、速度和总时长，左右回收时连续接入上下摆动'
        file=f'{number:02d}_{key}.json';content=_bytes(spec);payloads[file]=content
        actions.append({k:spec[k] for k in ('number','key','name','action_id','duration_s','beats','bpm','description','gripper_motion')}
                       |dict(file=file,sha256=hashlib.sha256(content).hexdigest()))
    manifest=dict(schema='rebuddy.dance_showcase.v1',hardware_accepted=False,firmware_integrated=False,
        screen_sync=False,actions=actions,idle_rad=idle.tolist(),zero_rad=zero.tolist(),dependencies=sources,
        calibration_sha256=identity['calibration_sha256'],urdf_sha256=identity['urdf_sha256'],
        json_action_bytes=sum(map(len,payloads.values())),
        compact_motion_json_bytes=sum(len(json.dumps(json.loads(v)['stages'],separators=(',',':')).encode()) for v in payloads.values()))
    if library_version!='v1':manifest['library_version']=library_version
    payloads['manifest.json']=_bytes(manifest);output=Path(output);output.mkdir(parents=True,exist_ok=True)
    for name,content in payloads.items():
        if (output/name).exists() and (output/name).read_bytes()!=content:raise ValueError(f'target already differs: {output/name}')
    for name,content in payloads.items():(output/name).write_bytes(content)
    return manifest


def load_library(calibration,urdf,folder=DEFAULT_OUTPUT):
    folder=Path(folder);gravity,identity=preflight(Path(calibration),Path(urdf))
    idle,zero,sources,bookends=base._anchors(identity);manifest_file=folder/'manifest.json'
    manifest=json.loads(manifest_file.read_text())
    if manifest.get('schema')!='rebuddy.dance_showcase.v1':raise ValueError('invalid showcase schema')
    catalog=_catalog(manifest.get('library_version','v1'))
    for key in ('calibration_sha256','urdf_sha256'):
        if manifest.get(key)!=identity[key]:raise ValueError('showcase calibration or model changed')
    if manifest.get('dependencies')!=sources:raise ValueError('showcase Daily source changed')
    if not np.array_equal(manifest.get('idle_rad'),idle) or not np.array_equal(manifest.get('zero_rad'),zero):
        raise ValueError('showcase anchors changed')
    if [a.get('number') for a in manifest['actions']]!=[n for n,_,_ in catalog]:raise ValueError('invalid showcase catalog')
    plans={};reports={}
    for entry,(number,key,name) in zip(manifest['actions'],catalog):
        path=folder/entry['file']
        if path.parent.resolve()!=folder.resolve() or digest(path)!=entry['sha256']:raise ValueError('showcase action checksum changed')
        spec=json.loads(path.read_text())
        if (spec.get('schema')!='rebuddy.dance_showcase_action.v1' or spec.get('number')!=number
            or spec.get('key')!=key or spec.get('action_id') is not None or spec.get('firmware_integrated') is not False
            or spec.get('gripper_motion')!=(number==16) or spec.get('dependencies')!=sources):
            raise ValueError('invalid showcase action identity')
        for field in ('calibration_sha256','urdf_sha256'):
            if spec.get(field)!=identity[field]:raise ValueError('showcase action model changed')
        if number in (0,11) and spec['stages']!=[bookends['WAKE' if number==0 else 'REST']]:
            raise ValueError('showcase Daily bookend changed')
        if number==21:
            expected_kind={'v10':'virtual_pivot_joined','v9':'virtual_pivot_sway','v8':'virtual_pivot_sway','v7':'virtual_pivot_sway','v6':'virtual_pivot','v5':'fixed_tcp_body'}.get(manifest.get('library_version'),'fixed_tcp_pivot')
            if len(spec['stages'])!=1 or spec['stages'][0].get('kind')!=expected_kind:
                raise ValueError('fixed-TCP action does not match showcase version')
            expected_speed=spec['stages'][0]['config'].get('default_trial_speed',.3) if manifest.get('library_version') in ('v9','v10') else .3
            if expected_kind in ('virtual_pivot','virtual_pivot_sway','virtual_pivot_joined') and spec.get('default_trial_speed')!=expected_speed:
                raise ValueError('virtual pivot default speed differs from its version/configuration')
            if expected_kind=='fixed_tcp_body' and spec.get('default_trial_speed')!=1.:
                raise ValueError('body pivot must declare its normal default playback speed')
        plan=ShowcasePlan(spec['stages'],bookend=number in (0,11),gripper_motion=number==16)
        checks=_validate(plan,gravity,number,idle,zero)
        checks.update(_validate_extra(plan,gravity,number,idle,spec))
        if abs(plan.duration-spec['duration_s'])>1e-9 or abs(plan.duration-entry['duration_s'])>1e-9:
            raise ValueError('showcase duration mismatch')
        if not np.allclose(spec['start_rad'],plan.start,rtol=0,atol=1e-9) or not np.allclose(spec['end_rad'],plan.end,rtol=0,atol=1e-9):
            raise ValueError('showcase declared endpoints mismatch')
        code=f'{number:02d}';plans[code]=plan
        reports[code]=dict(spec,validation=checks,source=str(path.resolve()),source_sha256=digest(path),
            library_manifest=str(manifest_file.resolve()),library_manifest_sha256=digest(manifest_file),playback_s=plan.duration)
    return manifest,plans,gravity,identity,reports


def main():
    import argparse
    from rs_gravity_record import DEFAULT_CALIBRATION, DEFAULT_URDF
    parser=argparse.ArgumentParser(description='离线生成舞台动作库，不连接CAN')
    parser.add_argument('--output',type=Path,default=DEFAULT_OUTPUT)
    parser.add_argument('--library-version',choices=('v1','v2','v3','v4','v5','v6','v7','v8','v9','v10'),default='v10')
    parser.add_argument('--pivot-config',type=Path,help='21定点转动JSON配置；覆盖默认配置中的指定项')
    parser.add_argument('--pivot-scan',type=Path,help='v6离线扫描结果；未指定配置时使用扫描中的配置')
    parser.add_argument('--calibration',type=Path,default=DEFAULT_CALIBRATION)
    parser.add_argument('--urdf',type=Path,default=DEFAULT_URDF)
    args=parser.parse_args()
    config=json.loads(args.pivot_config.read_text()) if args.pivot_config else None
    scan=json.loads(args.pivot_scan.read_text()) if args.pivot_scan else None
    try:
        manifest=build_library(args.calibration,args.urdf,args.output,
                               library_version=args.library_version,pivot_config=config,pivot_scan=scan)
    except ValueError as error:
        parser.exit(2,f'未生成动作库：{error}\n')
    print(f'已离线生成 {len(manifest["actions"])} 组动作：{args.output}')


if __name__=='__main__':main()
