#!/usr/bin/env python3
"""Pack the imagegen 2D turnaround drawings and the original idle drawing.

This is atlas slicing/registration, not a 3D reconstruction. The original
40-degree drawing is retained verbatim as the idle key. The animation renderer supplies 2D landmark inbetweens between views.
"""
import hashlib
import json
import sys
from pathlib import Path

import numpy as np
from PIL import Image

ROOT=Path(__file__).resolve().parents[1]
SOURCE=ROOT/'assets/images/source/dance/multiview'
SIZE=512
TIP=np.array([280.,430.])
# Atlas-pixel landmarks, measured on the generated 1263 x 1246 sheet.
# Front view's attachment is behind the near ear cap, not the face center.
WRISTS=[(274,212),(714,214),(1156,205),(313,593),(526,594),
        (955,592),(137,987),(558,997),(977,1005)]


def prepare():
    import cairo
    sys.path.insert(0,str(ROOT/'assets/images/source/dance'))
    import artwork as art
    atlas=Image.open(SOURCE/'head_turnaround.png').convert('RGBA')
    folder=SOURCE/'heads';folder.mkdir(exist_ok=True)
    records=[]
    for i in range(9):
        x,y=i%3,i//3
        box=(round(x*atlas.width/3),round(y*atlas.height/3),
             round((x+1)*atlas.width/3),round((y+1)*atlas.height/3))
        cell=atlas.crop(box);alpha=np.array(cell.getchannel('A'))
        ys,xs=np.where(alpha>64);top,bottom=int(ys.min()),int(ys.max())
        # Register the bottom of the closed jaws, rather than the head bbox
        # center (which moves sideways during a turn).
        yy,xx=np.where((alpha>128)&(np.indices(alpha.shape)[0]>bottom-9))
        tip=np.array([(float(xx.min())+float(xx.max()))/2,bottom],float)
        scale=346/(bottom-top)
        affine=(1/scale,0,tip[0]-TIP[0]/scale,0,1/scale,tip[1]-TIP[1]/scale)
        packed=cell.transform((SIZE,SIZE),Image.Transform.AFFINE,affine,Image.Resampling.BICUBIC)
        wrist=TIP+(np.array(WRISTS[i])-np.array(box[:2])-tip)*scale
        if i==6:
            surface=cairo.ImageSurface(cairo.FORMAT_ARGB32,SIZE,SIZE);c=cairo.Context(surface)
            c.set_line_join(cairo.LINE_JOIN_ROUND);c.set_line_cap(cairo.LINE_CAP_ROUND)
            c.translate(*TIP);c.scale(2,2);c.translate(-227,-185)
            art.head(c);art.face(c);art.gripper(c,0);art.cap(c,*art.P2,33,-16)
            surface.flush()
            pixels=np.frombuffer(surface.get_data(),np.uint8).reshape(SIZE,SIZE,4)[:,:,[2,1,0,3]].copy()
            a=pixels[:,:,3:4].astype(float)
            pixels[:,:,:3]=np.minimum(255,pixels[:,:,:3].astype(float)*255/np.maximum(a,1)).astype(np.uint8)
            packed=Image.fromarray(pixels,'RGBA');wrist=TIP+2*(art.P2-np.array([227,185]))
        # Registered on the final, eye-corrected sheet; visible hinge center.
        if i in (4,5,7,8):wrist=np.array({4:(142,238),5:(106,240),7:(87,244),8:(95,265)}[i],float)
        packed.save(folder/f'{i:02d}.png')
        records.append(dict(index=i,view_deg=-80+i*20,tip=TIP.tolist(),wrist=wrist.tolist(),
                            source='original vector drawing' if i==6 else 'imagegen turnaround sheet',
                            sha256=hashlib.sha256((folder/f'{i:02d}.png').read_bytes()).hexdigest()))
    data=dict(schema='rebuddy.multiview_heads.v1',size=[SIZE,SIZE],pixels_per_character_unit=2,
              sheet_sha256=hashlib.sha256((SOURCE/'head_turnaround.png').read_bytes()).hexdigest(),
              generation='built-in image_gen; exact prompt in prompt.txt',views=records)
    (SOURCE/'heads.json').write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n')
    return data



if __name__=='__main__':prepare()
