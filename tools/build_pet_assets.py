#!/usr/bin/env python3
"""Derive fixed-base 2D keyframes from the supplied reference, never regenerate a character.
These are modest mesh-deformed prototype frames, not completed 3D animation assets.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import struct
from collections import deque
from PIL import Image, ImageDraw, ImageFilter

ROOT = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser(description='Rebuild legacy firmware prototype resources; does not change approved artwork.')
parser.add_argument('--preview-dir', type=Path, help='Optional directory for diagnostic PNGs (use build/asset-review).')
args = parser.parse_args()
if args.preview_dir:
    args.preview_dir.mkdir(parents=True, exist_ok=True)
OUT = ROOT / 'assets/animation'
OUT.mkdir(exist_ok=True)
cfg = json.loads((ROOT/'config/behavior_defaults.json').read_text())
assert cfg['arm']['mode']=='mock' and not cfg['arm']['can_tx_enabled']
assert cfg['display']['width']==320 and cfg['display']['height']==240
assert cfg['display']['role_rect']=={'x':80,'y':40,'width':160,'height':160}
assert cfg['gesture']['poke_count']==3
assert 0<cfg['gesture']['speed_window_ms']<=140
assert cfg['display']['target_animation_fps']==15
assert cfg['queue']['max_pending']==1
assert all(isinstance(v,int) and v>0 for v in cfg['gesture'].values())
for entry in cfg['source_images']:
    source=ROOT/'assets/reference'/Path(entry['path']).name
    assert hashlib.sha256(source.read_bytes()).hexdigest()==entry['sha256']
source=Image.open(ROOT/'assets/reference/character_reference.png').convert('RGB')
crop=source.crop((760,165,1020,475))
# Trace only the chosen idle robot; exclude promotional text, rays and ground shadow.
outline=[(130,7),(174,15),(201,29),(223,43),(244,60),(252,84),(250,112),
 (242,132),(235,143),(251,166),(251,188),(242,204),(225,211),(209,207),
 (194,189),(181,181),(179,221),(197,237),(209,252),(210,282),(197,294),
 (171,302),(112,305),(58,301),(27,293),(16,279),(15,253),(23,239),
 (36,231),(35,216),(25,199),(23,176),(30,153),(39,138),(29,120),
 (28,98),(38,74),(57,49),(83,27),(107,13)]
mask=Image.new('L',crop.size,0); ImageDraw.Draw(mask).polygon(outline,fill=255)
# Remove only exterior pale background connected to crop borders, retaining enclosed silver highlights.
queue=deque([(x,y) for x in range(crop.width) for y in (0,crop.height-1)] +
            [(x,y) for y in range(crop.height) for x in (0,crop.width-1)])
visited=set()
while queue:
    x,y=queue.popleft()
    if (x,y) in visited or not (0<=x<crop.width and 0<=y<crop.height): continue
    visited.add((x,y)); r,g,b=crop.getpixel((x,y))
    if r>225 and g>225 and b>205 and max(r,g,b)-min(r,g,b)<40:
        mask.putpixel((x,y),0)
        queue.extend(((x-1,y),(x+1,y),(x,y-1),(x,y+1)))
# Replace the entire inner screen (including baked eyes/brows/blush) with a clean face.
face=Image.new('L',crop.size,0)
ImageDraw.Draw(face).polygon([(146,45),(180,40),(214,46),(235,60),(244,80),
 (247,103),(244,124),(214,131),(179,135),(141,131),(134,116),(132,89),(136,62)],fill=255)
face=face.filter(ImageFilter.GaussianBlur(1.0))
clean=Image.new('RGB',crop.size)
for y in range(crop.height):
    for x in range(crop.width):
        shade=int(31+9*(1-y/crop.height))
        clean.putpixel((x,y),(shade,shade+2,shade))
crop.paste(clean,(0,0),face)
crop.putalpha(mask)
crop=crop.resize((130,155),Image.Resampling.LANCZOS)
base=Image.new('RGBA',(160,160),(0,0,0,0)); base.alpha_composite(crop,(14,1))
if args.preview_dir:
    base.save(args.preview_dir/'clean_idle.png')
BG=(248,247,239)
stable=[(0,0,0,0),(-8,9,-3,0),(-6,14,5,0)]
def smooth(t): return t*t*(3-2*t)
def lerp(a,b,t): return tuple(x+(y-x)*smooth(t) for x,y in zip(a,b))
def curve(points,t):
    for (ta,a),(tb,b) in zip(points,points[1:]):
        if t<=tb: return lerp(a,b,max(0,min(1,(t-ta)/(tb-ta))))
    return points[-1][1]
def pose(action,origin,t):
    start=stable[origin]; idle=stable[0]
    paths={
      1:[(0,start),(.1,start),(.375,(5,-1,0,0)),(.625,(5,-1,0,0)),(1,idle)],
      2:[(0,start),(.25,(-3,0,-2,0)),(.65,(3,0,2,0)),(1,idle)],
      3:[(0,start),(.1,start),(.45,(0,0,5,0)),(.65,(0,0,5,0)),(1,idle)],
      4:[(0,start),(.13,start),(.72,stable[1]),(1,stable[1])],
      5:[(0,start),(.16,start),(.625,idle),(.8,(3,0,2,0)),(1,idle)],
      6:[(0,start),(.1,start),(.3,(0,-5,0,0)),(.45,(-3,-5,-2,2)),
         (.6,(3,-5,2,0)),(.7,(0,-5,0,0)),(1,idle)],
      7:[(0,start),(.267,start),(.733,stable[2]),(1,stable[2])],
      8:[(0,start),(.156,start),(.75,idle),(1,idle)]}
    return start if action==0 else curve(paths[action],t)
def weight(y): return min(1,max(0,(128-y)/70))
def map_point(x,y,p,inverse=False):
    dx,dy,angle,jaw=p; w=weight(y)
    angle=math.radians(angle)*w*(-1 if inverse else 1)
    xx,yy=x-108,y-60
    xx,yy=xx*math.cos(angle)-yy*math.sin(angle),xx*math.sin(angle)+yy*math.cos(angle)
    sign=-1 if inverse else 1
    jaw_weight=max(0,1-abs(y-98)/15)*max(0,1-abs(x-121)/20)
    return (108+xx+sign*(dx*w+jaw*jaw_weight*(1 if x>=121 else -1)),60+yy+sign*dy*w)
frames=[]; blobs=[]; seen={}
def add_frame(p):
    mesh=[]
    for y in range(0,160,8):
        for x in range(0,160,8):
            quad=[]
            for xx,yy in [(x,y),(x,y+8),(x+8,y+8),(x+8,y)]: quad.extend(map_point(xx,yy,p,True))
            mesh.append(((x,y,x+8,y+8),quad))
    im=base.transform((160,160),Image.Transform.MESH,mesh,Image.Resampling.BICUBIC)
    canvas=Image.new('RGB',(160,160),BG); canvas.paste(im,mask=im.getchannel('A'))
    data=bytearray()
    raw=canvas.tobytes()
    for i in range(0,len(raw),3):
        r,g,b=raw[i:i+3]; pixel=((r>>3)<<11)|((g>>2)<<5)|(b>>3)
        data.extend(struct.pack('<H',pixel))
    digest=hashlib.sha256(data).hexdigest()
    if digest in seen: return seen[digest]
    n=len(frames); seen[digest]=n
    anchors=[map_point(95,46,p),map_point(122,43,p)]
    frames.append({'offset':n*51212,'eyes':[[round(x),round(y)] for x,y in anchors],
                   'angle_tenths':round(p[2]*10),'sha256':digest})
    blobs.append(struct.pack('<BBHHHHH',0x19,0x12,0,160,160,320,0)+data)
    return n
clips=[]; mapping=[[0]*9 for _ in range(3)]
for origin in range(3):
    for action in range(9):
        if action and not (origin==0 or (origin==1 and action in (5,7,8)) or (origin==2 and action==8)):
            continue
        times=[0] if action==0 else [0,.1,.2,.3,.4,.5,.6,.7,.8,.9,1]
        entries=[add_frame(pose(action,origin,t)) for t in times]
        mapping[origin][action]=len(clips)
        clips.append({'origin':origin,'action':action,'frames':entries,'at_permille':[round(t*1000) for t in times],
                      'switch_points':[0,1000], 'loop':action==2,'end_pose':1 if action==4 else 2 if action==7 else origin if action==0 else 0})
reverse=[0]*9
for action in (2,3):
    times=[0,.1,.2,.3,.4,.5,.6,.7,.8,.9,1]
    entries=[]
    for t in times:
        dx,dy,angle,jaw=pose(action,0,t)
        entries.append(add_frame((-dx,dy,-angle,jaw)))
    reverse[action]=len(clips)
    clips.append({'origin':0,'action':action,'direction':-1,'frames':entries,
                  'at_permille':[round(t*1000) for t in times],
                  'switch_points':[0,1000],'loop':action==2,'end_pose':0})
(OUT/'frames.bin').write_bytes(b''.join(blobs))
manifest={'format':'LVGL9 RGB565 binary images, 12-byte header per frame','size':[160,160],
 'derivation':'Fixed-base 2D mesh prototype from the supplied idle figure; not finished 3D sequences',
 'source':'assets/reference/character_reference.png','unique_frames':len(frames),'bytes':sum(map(len,blobs)),
 'frames':frames,'clips':clips,'clip_map':mapping,'reverse_map':reverse}
(OUT/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
header='''/* Generated by tools/build_pet_assets.py. */
#pragma once
#include <stdint.h>
typedef struct { uint32_t offset; int16_t eyes[2][2], angle; } rb_frame_meta_t;
typedef struct { unsigned count; uint16_t frames[11], at[11]; } rb_clip_meta_t;
extern const rb_frame_meta_t rb_frames[];
extern const rb_clip_meta_t rb_clips[];
extern const uint8_t rb_clip_map[3][9];
extern const uint8_t rb_clip_reverse[9];
'''
(ROOT/'main/pet_animation_table.h').write_text(header+f'#define RB_FRAME_COUNT {len(frames)}\n#define RB_FRAME_BYTES 51212u\n')
lines=['/* Generated by tools/build_pet_assets.py. */','#include "pet_animation_table.h"','const rb_frame_meta_t rb_frames[]={']
for f in frames: lines.append('{%d,{{%d,%d},{%d,%d}},%d},'%(f['offset'],*f['eyes'][0],*f['eyes'][1],f['angle_tenths']))
lines+=['};','const rb_clip_meta_t rb_clips[]={']
for c in clips: lines.append('{%d,{%s},{%s}},'%(len(c['frames']),','.join(map(str,c['frames'])),','.join(map(str,c['at_permille']))))
lines+=['};','const uint8_t rb_clip_map[3][9]={']+[ '{'+','.join(map(str,row))+'},' for row in mapping]+['};']
lines+=['const uint8_t rb_clip_reverse[9]={'+','.join(map(str,reverse))+'};']
(ROOT/'main/pet_animation_table.c').write_text('\n'.join(lines)+'\n')
params=['/* Generated from config/behavior_defaults.json. */','#pragma once']
for name,value in cfg['gesture'].items(): params.append(f'#define RB_CFG_{name.upper()} {value}u')
for name,value in cfg['action_durations_ms'].items(): params.append(f'#define RB_MS_{name} {value}u')
params += [f'#define RB_STYLE_LIVELY {cfg["style_duration_scale"]["lively"]}f',
           f'#define RB_STYLE_LAZY {cfg["style_duration_scale"]["lazy"]}f']
rect=cfg['display']['role_rect']
for k,v in rect.items(): params.append(f'#define RB_ROLE_{k.upper()} {v}')
params.append(f'#define RB_BODY_FPS {cfg["display"]["target_animation_fps"]}u')
(ROOT/'components/rebuddy_core/include/rb_pet_config.h').write_text('\n'.join(params)+'\n')
# Diagnostic screenshots are optional and separate from the approved artwork.
if args.preview_dir:
    sheet=Image.new('RGB',(11*160,5*160),BG)
    for row,action in enumerate([1,2,4,6,7]):
        c=clips[mapping[0][action]]
        for col,idx in enumerate(c['frames']):
            body=blobs[idx][12:]; pixels=[]
            for (v,) in struct.iter_unpack('<H',body): pixels.append(((v>>11)*255//31,((v>>5)&63)*255//63,(v&31)*255//31))
            im=Image.new('RGB',(160,160)); im.putdata(pixels); sheet.paste(im,(col*160,row*160))
    sheet.save(args.preview_dir/'body-contact-sheet.png')
# A raw RGB565 calibration image for decoder/byte-order checks.
bar=bytearray()
colors=(0xf800,0x07e0,0x001f,0xffff,0x0000)
for y in range(160):
    for x in range(160): bar.extend(struct.pack('<H',colors[x//32]))
(OUT/'colorbars.bin').write_bytes(struct.pack('<BBHHHHH',0x19,0x12,0,160,160,320,0)+bar)
print(f'{len(frames)} unique body frames, {len(clips)} clips, {sum(map(len,blobs))} bytes')
