#!/usr/bin/env python3
"""Register nine COMPLETE character drawings with one uniform scale per cell."""
import hashlib
import json
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

ROOT=Path(__file__).resolve().parents[1]
SOURCE=ROOT/'assets/images/source/dance/integrated_turn'


def prepare():
    atlas=Image.open(SOURCE/'turnaround.png').convert('RGBA')
    (SOURCE/'views').mkdir(exist_ok=True)
    sheet=Image.new('RGB',(1254,1254),'#fffdf5')
    records=[]
    for i in range(9):
        col,row=i%3,i//3
        box=(col*418,row*418,(col+1)*418,(row+1)*418)
        cell=atlas.crop(box);alpha=np.array(cell.getchannel('A'))
        ys,xs=np.where(alpha>128);bottom=int(ys.max())
        fy,fx=np.where((alpha>128)&(np.indices(alpha.shape)[0]>=bottom-35))
        scale=280/float(fx.max()-fx.min())
        offset=np.array([282,540])-np.array([(fx.max()+fx.min())/2,bottom])*scale
        packed=cell.transform((640,640),Image.Transform.AFFINE,
            (1/scale,0,-offset[0]/scale,0,1/scale,-offset[1]/scale),Image.Resampling.BICUBIC)
        path=SOURCE/'views'/f'{i:02d}.png';packed.save(path)
        records.append(dict(index=i,degree=[0,12,25,32,40,55,65,75,85][i],
            cell_box=box,scale=scale,offset=offset.tolist(),sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
        bg=Image.new('RGBA',cell.size,'#fffdf5');bg.alpha_composite(cell)
        sheet.paste(bg,(col*418,row*418))
        draw=ImageDraw.Draw(sheet)
        draw.text((col*418+4,row*418+4),str(i),fill='#333333')
    sheet.save(SOURCE/'registered_sheet.png')
    metadata=dict(schema='rebuddy.integrated_turn_views.v1',
        atlas_sha256=hashlib.sha256((SOURCE/'turnaround.png').read_bytes()).hexdigest(),
        generation='built-in image_gen, full characters only',views=records)
    (SOURCE/'views.json').write_text(json.dumps(metadata,indent=2)+'\n')


if __name__=='__main__':prepare()
