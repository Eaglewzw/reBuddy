#!/usr/bin/env python3
"""Rejected 08/09 appearance experiment; retained only for offline comparison.

Retains the accepted projected TCP path and timing from the reference adapter.
08's illustrated helmet turns with its body, following the old GitHub turn
reference; it does not illustrate the physical wrist's orientation compensation.
"""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

from render_dance_cartoon import ROOT, BG, write_gif
from render_dance_reference import ReferenceCharacter
from dance_character_volume import VolumeCharacter


def trace(image, character, action, index):
    if action['number'] != '09': return image
    points=(np.array([p['tip'] for p in character.poses])+[34,38])*1.6
    t=action['frames'][index]['time_s']
    fade=min(1., max(0.,t/.35),max(0.,(action['duration_s']-t)/.4))
    if not fade:return image
    overlay=Image.new('RGBA',image.size);pen=ImageDraw.Draw(overlay)
    path=[tuple(p) for p in points]
    pen.line(path,fill=(37,155,181,round(85*fade)),width=2,joint='curve')
    if index:pen.line(path[:index+1],fill=(22,190,220,round(230*fade)),width=3,joint='curve')
    x,y=path[index]
    pen.ellipse((x-3,y-3,x+3,y+3),fill=(237,255,255,round(255*fade)),outline=(37,155,181,round(255*fade)))
    return Image.alpha_composite(image,overlay)


def write_audit(out, records):
    audit=out.parent.parent/'dance-reference-audit';audit.mkdir(exist_ok=True)
    reference=ROOT/'assets/images/source/dance/github_turn_reference.gif'
    (audit/'github-05-turn.gif').write_bytes(reference.read_bytes())
    cards=['<article><h2>GitHub 原版：完整转身</h2><img src="github-05-turn.gif"><p>旧编号05，原始9姿态逐格动画。</p></article>']
    for item in records:
        code=item['number'];version=item['preview_sha256'][:12]
        cards.append(f'<article><h2>已否定的3D实验 {code}</h2><img src="../dance-3d/volume/preview/{code}.gif?v={version}"><p>有厚度的卡通外壳；时长 {item["duration_ms"]/1000:.2f} 秒。</p></article>')
    (audit/'index.html').write_text('<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>转身效果对照</title><style>body{background:#fffdf5;color:#30362b;font:16px system-ui;max-width:1200px;margin:auto;padding:24px}.cards{display:flex;flex-wrap:wrap;gap:20px}article{flex:1;min-width:250px}img{width:100%}p{line-height:1.7}</style><h1>转身效果对照</h1><p>原版来自 Eaglewzw/reBuddy 提交 39097535。以下3D外壳已因偏离原角色画风而撤回，不是当前预览。主预览已恢复2D原画版。</p><div class="cards">'+''.join(cards)+'</div><p><a href="../dance-original/index.html">返回全部舞蹈</a> · <a href="../dance-3d/volume/angles.png">查看转动关键帧</a></p></html>')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input',type=Path,default=ROOT/'build/dance-3d/motion.json')
    args=parser.parse_args();manifest=json.loads(args.input.read_text())
    out=args.input.parent/'volume';out.mkdir(exist_ok=True)
    (out/'preview').mkdir(exist_ok=True)
    scene=VolumeCharacter();records=[]
    sheet=Image.new('RGB',(1600,640),BG)
    for row,action in enumerate(manifest['actions']):
        code=action['number'];character=ReferenceCharacter(manifest,action)
        folder=out/'rgba'/code;folder.mkdir(parents=True,exist_ok=True)
        ids=[0,int(np.argmin(character.yaws)),len(character.poses)//2,
             int(np.argmax(character.yaws)),len(character.poses)*3//4]
        frames=[];bounds=[]
        for i,p in enumerate(character.poses):
            im=trace(scene.render(p),character,action,i);frames.append(im)
            im.save(folder/f'{i:03d}.png')
            # Exclude only the low-alpha ground shadow, not solid geometry.
            alpha=np.array(im.getchannel('A'));ys,xs=np.where(alpha>64)
            bounds.append([int(xs.min()),int(ys.min()),int(xs.max()),int(ys.max())])
            if i in ids:
                bg=Image.new('RGBA',im.size,(*BG,255));bg.alpha_composite(im)
                bg.save(out/f'{code}-{i:03d}.png')
                for col,idx in enumerate(ids):
                    if idx==i:sheet.paste(bg.resize((320,320)),(col*320,row*320))
        write_gif(frames,out/f'{code}.gif',action['frame_durations_ms'],transparent=True)
        write_gif(frames,out/'preview'/f'{code}.gif',action['frame_durations_ms'])
        b=np.array(bounds)
        assert b[:,:2].min()>3 and b[:,2:].max()<637, f'{code}: clipped geometry'
        assert np.array_equal(np.array(frames[0]),np.array(frames[-1])), f'{code}: loop seam'
        records.append(dict(number=code,frames=len(frames),duration_ms=sum(action['frame_durations_ms']),
                            projection_scale=character.gain,bounds=b.tolist(),loop_seam_exact=True,
                            preview_sha256=hashlib.sha256((out/'preview'/f'{code}.gif').read_bytes()).hexdigest()))
        print(f'{code}: {len(frames)} frames, {sum(action["frame_durations_ms"])} ms',flush=True)
    sheet.save(out/'angles.png')
    data=dict(schema='rebuddy.dance_volume.v1',
              reference='https://github.com/Eaglewzw/reBuddy/blob/39097535d956281d3f7fca27d89527a52146327d/assets/gifs/dance/05_base_turn.gif',
              method='Closed cartoon surfaces, one depth buffer, original face and metal-cap artwork; screen anchors retargeted from current motion.',
              limitations='Decorative rig, not exact CAD geometry or a pixel-identical copy of the original character. 08 helmet follows visual body yaw.',
              motion_sha256=hashlib.sha256(args.input.read_bytes()).hexdigest(),
              renderer_sha256=hashlib.sha256(Path(__file__).with_name('dance_character_volume.py').read_bytes()).hexdigest(),
              actions=records)
    (out/'manifest.json').write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n')
    # Rejected experiment: never publish it to the active gallery.


if __name__=='__main__':main()
