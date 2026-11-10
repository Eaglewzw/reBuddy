#!/usr/bin/env python3
"""Animate the original approved Dance vector character from current motion plans.

Run with the robot Python environment; Cairo rendering uses /usr/bin/python3.
Offline only: no CAN connection, enable, or motor command is performed.
The original artwork is unchanged. Its six visual controls are a 2D motion
adaptation, not a geometric/dimensional simulation of the physical arm.
"""
from __future__ import annotations
import argparse
import importlib.util
import json
import math
from pathlib import Path
import subprocess
import sys
import hashlib
import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT=Path(__file__).resolve().parents[1]
ART=ROOT/'assets/images/source/dance'
BG=(255,253,245)
RESAMPLE=Image.Resampling
FONT='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
def digest(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def project(v):
    v=np.asarray(v)
    return np.stack((.82*v[...,0]-.57*v[...,1],-.25*v[...,0]-.36*v[...,1]-v[...,2]),axis=-1)
def rotation(a):
    a=math.radians(a)
    return np.array([[math.cos(a),-math.sin(a)],[math.sin(a),math.cos(a)]])
def angle(v):return math.degrees(math.atan2(v[1],v[0]))
def difference(a,b):return (a-b+180)%360-180

P0=np.array([114.,210.]);P1=np.array([75.,143.]);P2=np.array([139.,74.]);PC=np.array([227.,143.])
def visual_pose(target,head_angle,opening):
    wp=target-rotation(head_angle)@(PC-P2)
    d=wp-P0;l1=np.linalg.norm(P1-P0);l2=np.linalg.norm(P2-P1);r=np.linalg.norm(d)
    if not abs(l1-l2)+1<r<l1+l2-1:raise ValueError('unreachable 2D illustration pose')
    a=math.atan2(d[1],d[0])-math.acos(np.clip((r*r+l1*l1-l2*l2)/(2*r*l1),-1,1))
    ep=P0+l1*np.array([math.cos(a),math.sin(a)])
    s=difference(math.degrees(a),angle(P1-P0))
    e=difference(angle(wp-ep),angle(P2-P1))-s
    return [s,e,head_angle-s-e,0.,opening,0.]


def mapped_pose(value,gain,code,axis0):
    delta,head,opening,q,axis_angle,pivot_delta,yaw=value
    target=PC+delta*220*gain
    attitude=head*.7*gain
    if code=='10':
        # A constant virtual tool lever fixes the displayed pivot exactly in 2D.
        # Its entrance/exit follows the real virtual-point path; the middle
        # remains still. Scale attitude as a whole if the original rig needs it.
        lever=axis0/np.linalg.norm(axis0)*(44*gain)
        attitude=axis_angle*gain
        pivot=PC+lever+pivot_delta*220*gain
        target=pivot-rotation(attitude)@lever
    if code=='08':
        # Match the original 05_base_turn GIF: the whole rigid character turns
        # with its base. Keep the current J1 timing and two round trips, without
        # counter-moving the illustrated head or bending its shoulder/elbow.
        return [0.,0.,0.,0.,0.,yaw*.8]
    return visual_pose(target,attitude,opening)


def export_motion(args):
    from rs_dance_library import load_library,DEFAULT_OUTPUT
    from rs_gravity_record import DEFAULT_CALIBRATION,DEFAULT_URDF
    from render_dance_design import Skeleton
    manifest,plans,_,_,_=load_library(DEFAULT_CALIBRATION,DEFAULT_URDF)
    skeleton=Skeleton(DEFAULT_URDF)
    q0=np.asarray(manifest['idle_rad']);p0,_=skeleton.points(q0)
    r0=skeleton.data.oMf[skeleton.tip_id].rotation.copy()
    a0=angle(project(r0[:,0]));zero=np.asarray(manifest['zero_rad'])
    actions=[]
    for code in args.codes:
        plan=plans[code];entry=next(x for x in manifest['actions'] if int(x['number'])==int(code))
        eight_plane=None
        if code=='09':
            design=json.loads((DEFAULT_OUTPUT/entry['file']).read_text())['motion_design']
            # Look straight at the authored 8's plane: the general oblique view
            # collapses its lobes. Preserve the real path and phase, with its
            # long axis vertical so the drawn trace reads as the numeral 8.
            eight_plane=np.array([design['plane_v'],-np.array(design['plane_u'])])
        count=math.ceil(plan.duration/.08);ticks=np.rint(np.linspace(0,plan.duration*100,count+1)).astype(int)
        stamps=ticks[:-1]/100.;stamps[-1]=plan.duration
        values=[]
        for stamp in stamps:
            q=np.asarray(plan.at(float(stamp))[0]);points,_=skeleton.points(q);rot=skeleton.data.oMf[skeleton.tip_id].rotation.copy()
            # Orthographic TCP path drives the two original rigid links through IK.
            # Roll/yaw contribute head attitude while its original shape stays fixed.
            head=difference(angle(project(rot[:,0])),a0)+.40*math.degrees(q[5]-q0[5])
            tcp_delta=points[-1]-p0[-1]
            projected=eight_plane@tcp_delta if eight_plane is not None else project(tcp_delta)
            values.append((projected,head,float(np.clip(math.degrees(q[6])/90,0,1))*32,q.tolist(),difference(angle(project(rot[:,0])),a0),project(points[-1]+rot[:,0]*.20-p0[-1]-r0[:,0]*.20),math.degrees(q[0]-q0[0])))
        # One constant mapping gain per clip: never clamp individual frames.
        gain=1.
        while True:
            try:
                poses=[mapped_pose(value,gain,code,project(r0[:,0])) for value in values]
                break
            except ValueError:
                gain*=.95
                if gain<.15:raise RuntimeError('Original character cannot fit this projected path')
        for i in [0,len(poses)-1]:
            if np.allclose(values[i][3],q0,atol=1e-6):poses[i]=[0.]*6
        actions.append(dict(number=code,name=entry['name'],duration_s=plan.duration,duration_ms=int(ticks[-1]*10),
            plan_sha256=entry['sha256'],sample_times_s=stamps.tolist(),frame_start_times_ms=(ticks[:-1]*10).tolist(),
            frame_durations_ms=(np.diff(ticks)*10).tolist(),visual_poses=poses,joints_rad=[v[3] for v in values],
            projected_tcp_delta=[v[0].tolist() for v in values],mapping_gain=gain,visual_mapping=('original 05_base_turn whole-character perspective renderer; current J1 timing at 0.8 visual yaw scale; no head stabilization' if code=='08' else 'front view of actual Cartesian figure-eight plane, long axis vertical; 2D IK with visible TCP trace' if code=='09' else 'fixed virtual lever pivot with constant 2D tool lever; entrance/exit follows actual virtual pivot path' if code=='10' else 'projected real TCP with 2D IK')))
    result=dict(schema='rebuddy.dance_cartoon.v1',canvas=[640,640],fps=12.5,actions=actions,
        library_manifest_sha256=digest(DEFAULT_OUTPUT/'manifest.json'),model_sha256=digest(DEFAULT_URDF),
        renderer_sha256=digest(__file__),artwork_sha256=digest(ART/'artwork.py'),
        character='Original Dance 2D perspective_v4 source; matching legacy 01_nod.gif SHA recorded in provenance.json',
        render_method='Original vector layers, rigid two-link inverse kinematics driven by projected physical TCP samples',
        mapping='Original Cairo character, fixed link lengths/proportions; projected real TCP path drives continuous 2D two-link IK; attitude blends projected tool axis and J6. Per-clip constant gain fits the original rig. This is a 2D illustration, not exact 3D geometry or an aperture calibration.',
        alpha='Straight RGBA PNG; GIF binary transparency 255',
        endpoint_policy='Actual cumulative frame start times; final frame is exact endpoint held for at most 80ms.')
    args.output.mkdir(parents=True,exist_ok=True)
    dest=args.output/'motion.json';dest.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    cmd=['/usr/bin/python3','-B',str(Path(__file__)), '--render-json',str(dest)]
    if args.stills_only:cmd.append('--stills-only')
    subprocess.run(cmd,check=True)


class OriginalCharacter:
    def __init__(self):
        sys.path.insert(0,str(ART))
        import artwork
        self.art=artwork
        self.turn_draw=artwork.draw
    def render(self,pose,whole_turn=False):
        # The original perspective renderer is specifically for a rigid base
        # turn; other actions need its separate articulated drawing branch.
        original=self.art.draw
        self.art.draw=self.turn_draw if whole_turn else self.art.draw_previous
        try:
            image=self.art.render(pose,transparent=True)
        finally:
            self.art.draw=original
        # Suppress unstable RGB at near-zero alpha without touching visible art.
        a=np.array(image)
        # Restore the original shadow hue after low-alpha Cairo quantization.
        shadow=(np.indices(a.shape[:2])[0]>543)&(a[:,:,3]<=31)
        a[shadow,:3]=[153,142,115]
        a[a[:,:,3]<3]=0
        return Image.fromarray(a,'RGBA')

def write_gif(frames,path,durations,transparent=False):
    sample=Image.new('RGB',(640*4,640*4),BG)
    for i in range(16):
        rgba=frames[min(len(frames)-1,i*(len(frames)-1)//15)]
        base=Image.new('RGBA',rgba.size,BG+(255,));base.alpha_composite(rgba)
        sample.paste(base.convert('RGB'),(i%4*640,i//4*640))
    palette=sample.quantize(colors=255,method=Image.Quantize.MEDIANCUT,dither=Image.Dither.NONE)
    pal=palette.getpalette()[:765]+[0,0,0]
    encoded=[]
    for rgba in frames:
        base=Image.new('RGBA',rgba.size,BG+(255,));base.alpha_composite(rgba)
        target=Image.new('P',(1,1));target.putpalette(pal)
        frame=base.convert('RGB').quantize(palette=target,dither=Image.Dither.NONE)
        if transparent:
            data=np.asarray(frame).copy();data[np.asarray(rgba.getchannel('A'))<128]=255
            frame=Image.fromarray(data,'P');frame.putpalette(pal)
        encoded.append(frame)
    options=dict(save_all=True,append_images=encoded[1:],duration=durations,loop=0,disposal=2,optimize=False)
    if transparent:options.update(transparency=255)
    encoded[0].save(path,**options)


def trajectory_overlay(frame,points,index,stamp,duration):
    """Show the actual illustrated TCP path, with a moving tip and fading ends."""
    fade=min(1.,max(0.,stamp/.35),max(0.,(duration-stamp)/.4))
    fade=fade*fade*(3-2*fade)
    if fade<=0:return frame
    layer=Image.new('RGBA',frame.size)
    draw=ImageDraw.Draw(layer)
    def color(rgb,alpha):return (*rgb,round(alpha*fade))
    # Full faint guide makes both lobes visible before the tracing completes.
    draw.line(points,fill=color((17,86,111),130),width=9,joint='curve')
    draw.line(points,fill=color((83,218,244),150),width=5,joint='curve')
    if index:
        draw.line(points[:index+1],fill=color((15,91,125),245),width=9,joint='curve')
        draw.line(points[:index+1],fill=color((57,224,255),255),width=5,joint='curve')
    x,y=points[index]
    draw.ellipse((x-8,y-8,x+8,y+8),fill=color((16,111,150),255))
    draw.ellipse((x-5,y-5,x+5,y+5),fill=color((236,255,255),255))
    return Image.alpha_composite(frame,layer)


def render_frame(job):
    pose,dest,whole_turn,trail=job
    painter=OriginalCharacter()
    frame=painter.render(pose,whole_turn=whole_turn)
    if trail is not None:frame=trajectory_overlay(frame,*trail)
    bbox=frame.getbbox()
    if bbox is None or min(bbox[:2])<4 or max(bbox[2:])>636:
        raise ValueError(f'{dest} clips original character: {bbox}')
    frame.save(dest)
    return str(dest)


def write_gallery(out,index):
    from html import escape
    sections=[]
    for title,codes in [('10组基础舞蹈',{f'{n:02d}' for n in range(1,11)}),('开场与收尾',{'00','11'})]:
        cards=[]
        for a in index['actions']:
            if a['number'] not in codes:continue
            version=digest(out/'preview'/f'{a["number"]}.gif')[:12]
            cards.append(f'<article><img src="preview/{a["number"]}.gif?v={version}" alt="{escape(a["name"])}"><p><b>{a["number"]} {escape(a["name"])}</b><span>{a["duration_ms"]/1000:.2f} 秒</span></p><a href="{a["number"]}.gif?v={version}">640px 透明 GIF</a></article>')
        sections.append(f'<h2>{title}</h2><section>{"".join(cards)}</section>')
    (out/'index.html').write_text('<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>reBuddy · Dance 舞蹈预览</title><style>body{margin:0;background:#f6f4eb;color:#30362b;font:16px system-ui,sans-serif}main{max-width:1200px;margin:auto;padding:28px}h1{font-size:28px}h2{margin-top:32px;font-size:20px}section{display:grid;grid-template-columns:repeat(auto-fit,minmax(240px,1fr));gap:16px}article{background:#fffdf5;border:1px solid #dfdfce;border-radius:16px;padding:12px}img{display:block;width:100%;border-radius:10px}p{line-height:1.7}article p{display:flex;justify-content:space-between;gap:8px;margin:10px 0}span,a{font-size:13px;color:#647440}a{text-decoration:none}small{color:#64705d}</style><main><h1>reBuddy · Dance 舞蹈预览</h1><p>沿用原来的绿色机械臂角色，按当前动作的实际时间播放。</p><small>这是原画二维动画适配；末端路径和节奏来自当前实机动作，不代表真实机构的三维几何仿真。</small>'+''.join(sections)+'</main></html>')


def render_motion(path,stills_only):
    out=path.parent;index=json.loads(path.read_text());index['renderer_sha256']=digest(__file__);painter=OriginalCharacter()
    painter.render([0]*6).save(out/'neutral-original.png')
    for record in index['actions']:
        code=record['number'];poses=record.pop('visual_poses');durations=record['frame_durations_ms']
        points=None
        if code=='09':
            # Native art translates by (34,38) in its 400-unit view, then scales
            # to 640px. This is the same TCP target used by visual_pose(), not
            # a decorative unrelated figure eight.
            points=[tuple((PC+np.array(delta)*220*record['mapping_gain']+np.array([34,38]))*1.6)
                    for delta in record['projected_tcp_delta']]
            record['trajectory_overlay']='projected TCP figure-eight guide, progressive cyan trace and moving tip; fades at shared idle endpoints'
        def trail(i):
            return (points,i,record['sample_times_s'][i],record['duration_s']) if points is not None else None
        n=max(range(len(poses)),key=lambda n:np.linalg.norm(np.array(poses[n])[[0,1,2,5]]))
        poster=painter.render(poses[n],whole_turn=code=='08')
        if points is not None:poster=trajectory_overlay(poster,*trail(n))
        poster.save(out/f'{code}.png');record['poster']=f'{code}.png'
        if not stills_only:
            frames=[];paths=[];folder=out/'frames'/code;folder.mkdir(parents=True,exist_ok=True)
            from concurrent.futures import ProcessPoolExecutor
            jobs=[(pose,folder/f'{n:04d}.png',code=='08',trail(n)) for n,pose in enumerate(poses)]
            with ProcessPoolExecutor(max_workers=3) as pool:
                for saved in pool.map(render_frame,jobs,chunksize=4):
                    dest=Path(saved);paths.append(str(dest.relative_to(out)))
                    with Image.open(dest) as frame:frames.append(frame.copy())
            write_gif(frames,out/f'{code}.gif',durations,transparent=True)
            encoded=Image.open(out/f'{code}.gif');delays=[]
            for n in range(encoded.n_frames):encoded.seek(n);delays.append(encoded.info['duration'])
            if sum(delays)!=sum(durations):raise ValueError('GIF duration mismatch')
            previews=[]
            for frame in frames:
                bg=Image.new('RGBA',(640,640),BG+(255,));bg.alpha_composite(frame);previews.append(bg.resize((320,320),RESAMPLE.LANCZOS))
            (out/'preview').mkdir(exist_ok=True)
            previews[0].save(out/'preview'/f'{code}.gif',save_all=True,append_images=previews[1:],duration=durations,loop=0,disposal=2,optimize=False)
            record.update(png_frames=paths,gif=f'{code}.gif',preview_gif=f'preview/{code}.gif',encoded_frames=encoded.n_frames,encoded_delays_ms=delays)
        print(code,record['name'],record['duration_s'],flush=True)
    font=ImageFont.truetype(FONT,20,index=2);rows=math.ceil(len(index['actions'])/3)
    sheet=Image.new('RGB',(960,rows*360),BG);d=ImageDraw.Draw(sheet)
    for n,item in enumerate(index['actions']):
        thumb=Image.open(out/item['poster']).resize((320,320),RESAMPLE.LANCZOS);sheet.paste(thumb,(n%3*320,n//3*360),thumb)
        d.text((n%3*320+12,n//3*360+325),f"{item['number']} {item['name']} · {item['duration_s']:.2f}s",font=font,fill=(45,54,38))
    sheet.save(out/'contact-sheet.png');(out/'manifest.json').write_text(json.dumps(index,ensure_ascii=False,indent=2)+'\n')
    if not stills_only:write_gallery(out,index)

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=ROOT/'build/dance-original')
    parser.add_argument('--codes',nargs='+',default=[f'{x:02d}' for x in range(12)])
    parser.add_argument('--stills-only',action='store_true')
    parser.add_argument('--render-json',type=Path,help=argparse.SUPPRESS)
    args=parser.parse_args()
    if args.render_json:render_motion(args.render_json,args.stills_only)
    else:export_motion(args)

if __name__=='__main__':main()
