#!/usr/bin/env python3
"""Slice and register the nine complete imagegen character drawings."""
import hashlib
import json
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

ROOT=Path(__file__).resolve().parents[1]
SOURCE=ROOT/'assets/images/source/dance/whole_turn'


def prepare():
    atlas=Image.open(SOURCE/'turnaround.png').convert('RGBA')
    folder=SOURCE/'views';folder.mkdir(exist_ok=True)
    records=[];sheet=Image.new('RGB',(960,960),'#fffdf5')
    for i in range(9):
        x,y=i%3,i//3
        box=(round(x*atlas.width/3),round(y*atlas.height/3),round((x+1)*atlas.width/3),round((y+1)*atlas.height/3))
        cell=atlas.crop(box);alpha=np.array(cell.getchannel('A'))
        yy,xx=np.where(alpha>128);top,bottom=yy.min(),yy.max()
        yf,xf=np.where((alpha>128)&(np.indices(alpha.shape)[0]>bottom-40))
        base=(float(xf.min()+xf.max())/2,float(bottom))
        scale=np.array([280./(xf.max()-xf.min()),470./(bottom-top)])
        offset=np.array([282.,540.])-np.array(base)*scale
        packed=cell.transform((640,640),Image.Transform.AFFINE,
            (1/scale[0],0,-offset[0]/scale[0],0,1/scale[1],-offset[1]/scale[1]),Image.Resampling.BICUBIC)
        packed.save(folder/f'{i:02d}.png')
        bg=Image.new('RGBA',packed.size,'#fffdf5');bg.alpha_composite(packed)
        sheet.paste(bg.resize((320,320)),(x*320,y*320));ImageDraw.Draw(sheet).text((x*320+10,y*320+10),str(i),fill='#333333')
        records.append(dict(index=i,view_deg=-80+20*i,scale=scale.tolist(),offset=offset.tolist(),
            cell_box=list(box),base_center=[282,540],sha256=hashlib.sha256((folder/f'{i:02d}.png').read_bytes()).hexdigest()))
    sheet.save(SOURCE/'registered_views.png')
    result=dict(schema='rebuddy.whole_turn.v1',atlas_sha256=hashlib.sha256((SOURCE/'turnaround.png').read_bytes()).hexdigest(),
        size=[640,640],generation='built-in image_gen; complete character drawings',views=records)
    (SOURCE/'views.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')


def prepare_inbetweens():
    """Register the four complete drawings using their saved atlas transforms."""
    metadata=json.loads((SOURCE/'inbetweens.json').read_text())
    path=SOURCE/'inbetweens.png'
    if hashlib.sha256(path.read_bytes()).hexdigest()!=metadata['atlas_sha256']:
        raise ValueError('Inbetween atlas checksum mismatch')
    atlas=Image.open(path).convert('RGBA')
    folder=SOURCE/'inbetween_views';folder.mkdir(exist_ok=True)
    sheet=Image.new('RGB',(640,640),'#fffdf5')
    for row in metadata['views']:
        i=row['index'];sx,sy=row['scale'];dx,dy=row['offset']
        cell=atlas.crop(row['cell_box'])
        packed=cell.transform((640,640),Image.Transform.AFFINE,
            (1/sx,0,-dx/sx,0,1/sy,-dy/sy),Image.Resampling.BICUBIC)
        packed.save(folder/f'{i:02d}.png')
        row['sha256']=hashlib.sha256((folder/f'{i:02d}.png').read_bytes()).hexdigest()
        bg=Image.new('RGBA',packed.size,'#fffdf5');bg.alpha_composite(packed)
        sheet.paste(bg.resize((320,320)),((i%2)*320,(i//2)*320))
    sheet.save(SOURCE/'inbetweens_registered.png')
    (SOURCE/'inbetweens.json').write_text(json.dumps(metadata,indent=2)+'\n')


if __name__=='__main__':
    prepare()
    prepare_inbetweens()
