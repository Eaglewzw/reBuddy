#!/usr/bin/env python3
"""08: play complete drawn cels with no deformation, optical flow or blending.

Only the screen illustration changes. The original motion clock and two-round
turn phase remain inputs; mechanical-arm trajectories are never written here.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import numpy as np
from PIL import Image
for _enum in ('Resampling','Quantize','Dither'):
    if not hasattr(Image,_enum):setattr(Image,_enum,Image)
from render_dance_cartoon import ROOT, BG, write_gif
SOURCE=ROOT/'assets/images/source/dance/rigid_turn'
MAX_SCREEN_TURN=52  # Exclude the rejected 56/59/62-degree side poses.


def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()


class WholeCharacterCels:
    """Select one COMPLETE cel per frame. Never alter its shape or opacity."""
    def __init__(self):
        self.meta=json.loads((SOURCE/'views.json').read_text())
        views=[v for v in self.meta['views'] if v['degree']<=MAX_SCREEN_TURN]
        self.degrees=np.array([v['degree'] for v in views],float)
        self.frames=[]
        for view in views:
            path=SOURCE/'views'/f'{view["index"]:02d}.png'
            if digest(path)!=view['sha256']:raise ValueError(f'Changed cel: {path}')
            self.frames.append(Image.open(path).convert('RGBA'))

    @staticmethod
    def visual_angle(motion_angle):
        # Keep the neutral drawing at 40 degrees. Compress only the side end
        # of the SCREEN turn, avoiding the rejected thin-face extreme profile.
        # This is an illustration mapping, not a mechanical joint command.
        return float(np.interp(motion_angle,[5,40,75],[5,40,MAX_SCREEN_TURN]))

    def select(self,motion_angle):
        angle=self.visual_angle(motion_angle)
        index=int(np.argmin(np.abs(self.degrees-angle)))
        return index,angle,self.frames[index].copy()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--stills-only',action='store_true');args=parser.parse_args()
    motion_path=ROOT/'build/dance-3d/motion.json'
    motion=json.loads(motion_path.read_text());action=next(a for a in motion['actions'] if a['number']=='08')
    initial=action['frames'][0]['q_rad'][0]
    turns=[40-math.degrees(frame['q_rad'][0]-initial) for frame in action['frames']]
    out=motion_path.parent/'whole_turn';(out/'preview').mkdir(parents=True,exist_ok=True)
    (out/'rgba').mkdir(exist_ok=True);drawing=WholeCharacterCels()
    indices=[0,9,14,18,23,27,32,41,50,68]
    sheet=Image.new('RGB',(1600,640),BG);frames=[];selected=[];visual=[]
    for i,angle in enumerate(turns):
        cel,view,frame=drawing.select(angle)
        frames.append(frame);selected.append(cel);visual.append(view)
        if not args.stills_only:frame.save(out/'rgba'/f'{i:03d}.png')
        if i in indices:
            k=indices.index(i);bg=Image.new('RGBA',frame.size,(*BG,255));bg.alpha_composite(frame)
            sheet.paste(bg.resize((320,320)),((k%5)*320,(k//5)*320))
    sheet.save(out/'sequence.png')
    if args.stills_only:return
    write_gif(frames,out/'08.gif',action['frame_durations_ms'],transparent=True)
    write_gif(frames,out/'preview/08.gif',action['frame_durations_ms'])
    data=dict(schema='rebuddy.whole_turn_animation.v3',
        method='Complete hand-drawn cels selected directly; zero runtime warping, optical flow, crossfade or part overlays.',
        motion_sha256=digest(motion_path),frames=len(frames),duration_ms=sum(action['frame_durations_ms']),
        view_degrees=turns,visual_degrees=visual,selected_cels=selected,
        source=str(SOURCE.relative_to(ROOT)),source_sheets=drawing.meta['source_sheets'],
        active_view_degrees=drawing.degrees.tolist(),complete_cels=len(drawing.frames),
        visual_turn_mapping={'input':[5,40,75],'screen':[5,40,MAX_SCREEN_TURN]},
        local_part_overlays=False,base_screenshot_reused=False,runtime_deformation=False,
        optical_flow=False,crossfade=False,frame_durations_ms=action['frame_durations_ms'])
    (out/'manifest.json').write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n')
    review=ROOT/'build/dance-whole-turn';review.mkdir(exist_ok=True)
    version=digest(out/'preview/08.gif')[:12]
    (review/'index.html').write_text(f'''<!doctype html><html lang="zh-CN"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>08 完整姿态逐格转身</title>
<style>body{{background:#fffdf5;color:#30362b;font:16px system-ui;max-width:1100px;margin:auto;padding:24px}}img{{max-width:100%}}.play{{width:480px}}</style>
<h1>08 · 完整姿态逐格转身</h1><p>16张完整姿态原画直接逐格播放，不拉伸、不叠图。肩肘与连杆保持原画结构，去掉56°、59°、62°三个偏侧面姿态。保留起始姿态、两轮摇摆与8.74秒时长；09未改。</p>
<img class="play" src="../dance-3d/whole_turn/preview/08.gif?v={version}">
<h2>原画存档（最后三个侧面姿态已停用）</h2><img src="../../assets/images/source/dance/rigid_turn/registered_sheet.png">
<h2>播放帧</h2><img src="../dance-3d/whole_turn/sequence.png?v={version}">
<p><a href="../dance-original/index.html">返回舞蹈库</a></p></html>''')
    from render_dance_3d import gallery
    gallery(motion_path.parent,motion)
    print(f'08 complete cels: {len(drawing.frames)} drawings / {len(frames)} timed samples / {data["duration_ms"]} ms')


if __name__=='__main__':main()
