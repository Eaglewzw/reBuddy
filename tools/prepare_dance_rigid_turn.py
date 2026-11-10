#!/usr/bin/env python3
"""Pack complete drawn turn cels; apply only one uniform scale and translation."""
import hashlib
import json
from pathlib import Path
import numpy as np
from PIL import Image, ImageDraw
from scipy.ndimage import label

ROOT=Path(__file__).resolve().parents[1]
SOURCE=ROOT/'assets/images/source/dance/rigid_turn'
SHEETS=[('front.png',[5,8,11,14,17,20,23,26,29,33,36,40]),
        ('side.png',[40,43,46,49,52,56,59,62,65,69,72,75])]


def prepare():
    (SOURCE/'views').mkdir(exist_ok=True)
    records=[];sources={};overview=Image.new('RGB',(1280,1600),'#fffdf5')
    for filename,degrees in SHEETS:
        path=SOURCE/filename;sources[filename]=hashlib.sha256(path.read_bytes()).hexdigest()
        atlas=Image.open(path).convert('RGBA')
        for cell,degree in enumerate(degrees):
            if filename=='side.png' and cell==0:continue
            # The most oblique illustrations collapse the face into a sliver.
            # Keep the source sheet for provenance, but don't use those poses.
            if degree>62:continue
            col,row=cell%4,cell//4
            box=[round(col*atlas.width/4),round(row*atlas.height/3),
                 round((col+1)*atlas.width/4),round((row+1)*atlas.height/3)]
            image=atlas.crop(box);alpha=np.array(image.getchannel('A'))
            components,n=label(alpha>128)
            counts=np.bincount(components.ravel());counts[0]=0
            solid=components==counts.argmax()
            yy,xx=np.where(solid);bottom=int(yy.max())
            fy,fx=np.where(solid&(np.indices(solid.shape)[0]>bottom-30))
            center=(float(fx.min()+fx.max())/2,bottom)
            scale=280/float(fx.max()-fx.min())
            offset=np.array([282.,540.])-np.array(center)*scale
            packed=image.transform((640,640),Image.Transform.AFFINE,
                (1/scale,0,-offset[0]/scale,0,1/scale,-offset[1]/scale),Image.Resampling.BICUBIC)
            index=len(records);output=SOURCE/'views'/f'{index:02d}.png';packed.save(output)
            records.append(dict(index=index,degree=degree,sheet=filename,cell=cell,
                cell_box=box,uniform_scale=scale,offset=offset.tolist(),
                sha256=hashlib.sha256(output.read_bytes()).hexdigest()))
            bg=Image.new('RGBA',packed.size,'#fffdf5');bg.alpha_composite(packed)
            overview.paste(bg.resize((320,320)),((index%4)*320,(index//4)*320))
            ImageDraw.Draw(overview).text(((index%4)*320+8,(index//4)*320+8),str(degree),fill='#333333')
    overview.save(SOURCE/'registered_sheet.png')
    metadata=dict(schema='rebuddy.rigid_turn_cels.v1',generation='built-in image_gen, whole-character cels',
        source_sheets=sources,registration='whole-cell uniform scale and translation only',
        visual_angle_range=[5,62],neutral_angle=40,views=records)
    (SOURCE/'views.json').write_text(json.dumps(metadata,indent=2)+'\n')
    print(f'{len(records)} complete cels registered; no parts cropped or warped.')


if __name__=='__main__':prepare()
