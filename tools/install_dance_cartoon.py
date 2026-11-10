#!/usr/bin/env python3
"""Install reviewed original-character Dance renders and immutable RGBA inputs."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
from PIL import Image

ROOT=Path(__file__).resolve().parents[1]
NAMES=['START','NOD','HELLO','DIP','REACH','NUZZLE','HAPPY','WAVE','STEADY','EIGHT','PIVOT','END']
def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()

def reviewed_override(number, entry):
    """Resolve the two approved gallery overrides, keeping their full RGBA clocks."""
    folder=ROOT/'build/dance-3d'/('whole_turn' if number==8 else 'multiview')
    metadata_path=folder/'manifest.json'
    metadata=json.loads(metadata_path.read_text())
    motion_path=ROOT/'build/dance-3d/motion.json'
    if metadata['motion_sha256']!=sha(motion_path):raise ValueError('Stale reviewed motion render')
    motion=next(a for a in json.loads(motion_path.read_text())['actions'] if int(a['number'])==number)
    if motion['trajectory_sha256']!=entry['plan_sha256']:raise ValueError('Reviewed trajectory differs from library')
    timing=motion['frame_durations_ms']
    if motion['duration_ms']!=entry['duration_ms']:raise ValueError('Reviewed duration differs from library')
    if number==8:
        if metadata['frame_durations_ms']!=timing:raise ValueError('Reviewed cel timing mismatch')
        frames=[f'rgba/{i:03}.png' for i in range(len(timing))]
    else:
        frames=[f'rgba/09/{i:03}.png' for i in range(len(timing))]
    entry={**entry,'gif':f'{number:02}.gif','png_frames':frames,'frame_durations_ms':timing,
           'visual_mapping':metadata['method'],
           'render_provenance':{'metadata':str(metadata_path.relative_to(ROOT)),
                                'sha256':sha(metadata_path),'motion_sha256':sha(motion_path)}}
    if number==8:entry['endpoint_group']='whole_character_turn'
    return folder,entry

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input',type=Path,default=ROOT/'build/dance-original')
    parser.add_argument('--reviewed',action='store_true',help='Install the approved gallery including whole-character 08 and multiview 09')
    args=parser.parse_args();source=args.input.resolve()
    render=json.loads((source/'manifest.json').read_text())
    library_path=ROOT/'record/dance_library/manifest.json'
    library=json.loads(library_path.read_text())
    authored={int(a['number']):a for a in library['actions']}
    rendered={int(a['number']):a for a in render['actions']}
    if set(rendered)!=set(range(12)):raise ValueError('Render all ten actions and both automatic bookends first')
    target=ROOT/'assets/gifs/dance';stage=ROOT/'build/dance-install';stage.mkdir(parents=True,exist_ok=True)
    clips=[]
    for number in range(12):
        action=authored[number];entry=rendered[number]
        if entry['plan_sha256']!=sha(ROOT/'record/dance_library'/action['file']):raise ValueError(f'Stale trajectory {number:02}')
        duration=round(action['duration_s']*100)*10
        if entry['duration_ms']!=duration:raise ValueError(f'Timing mismatch {number:02}')
        frame_source=source
        if args.reviewed and number in (8,9):frame_source,entry=reviewed_override(number,entry)
        filename=Path(action['file']).with_suffix('.gif').name
        gif_source=frame_source/entry['gif']
        with Image.open(gif_source) as gif:
            delays=[]
            for tick in range(gif.n_frames):gif.seek(tick);delays.append(gif.info['duration'])
            if sum(delays)!=duration or gif.size!=(640,640):raise ValueError(f'Invalid GIF {number:02}')
        rows=[]
        if not 1<=len(entry['png_frames'])<=256:raise ValueError('Invalid frame count')
        delays_rgba=entry['frame_durations_ms']
        if sum(delays_rgba)!=duration or any(d<=0 or d%10 for d in delays_rgba):raise ValueError('Invalid RGBA timing')
        if len(entry['png_frames'])!=len(entry['frame_durations_ms']):raise ValueError('Missing timing rows')
        for i,(relative,delay) in enumerate(zip(entry['png_frames'],entry['frame_durations_ms'])):
            src=frame_source/relative
            if not src.resolve().is_relative_to(frame_source):raise ValueError('Frame escapes render directory')
            with Image.open(src) as image:
                if image.size!=(640,640) or image.mode!='RGBA':raise ValueError(f'Expected 640x640 RGBA: {src}')
            rel=Path('rgba')/f'{number:02}'/f'{i:04}.png';dst=stage/rel;dst.parent.mkdir(parents=True,exist_ok=True)
            shutil.copyfile(src,dst);rows.append({'file':rel.as_posix(),'sha256':sha(dst),'duration_ms':delay})
        shutil.copyfile(gif_source,stage/filename)
        clip={'id':'DANCE_'+NAMES[number],'number':number,'action_id':43 if number==0 else 44 if number==11 else 44+number,
              'name':action['name'],'file':filename,'sha256':sha(stage/filename),'screen_fit':'square240_center',
              'ticks':len(delays),'delays_ms':delays,'keys':[[i,1] for i in range(len(delays))],'duration_ms':duration,
              'loop':number not in (0,11),'claw_key':'idle:0','rgba_frames':rows,
              'trajectory':str((Path('record/dance_library')/action['file']).as_posix()),'trajectory_sha256':entry['plan_sha256']}
        clip['visual_mapping']=entry.get('visual_mapping',render['mapping'])
        clip['endpoint_group']=entry.get('endpoint_group','shared_neutral')
        if 'render_provenance' in entry:clip['render_provenance']=entry['render_provenance']
        if 'trajectory_overlay' in entry:clip['trajectory_overlay']=entry['trajectory_overlay']
        if number in (0,11):clip['role']='automatic_bookend'
        clips.append(clip)
    manifest={'schema':'rebuddy.dance_screen.v3','library_version':'reviewed-cartoon-v2' if args.reviewed else 'current-cartoon-v1','source':'Current Dance trajectories, original character layers and projected motion',
              'render_metadata':{k:v for k,v in render.items() if k!='actions'},
              'generator':'tools/install_dance_cartoon.py --reviewed' if args.reviewed else 'tools/render_dance_cartoon.py','nominal_fps':12.5,
              'alpha_source':'RGBA PNG coverage; GIF transparency is for desktop preview only',
              'conversion':'Full-quality RGBA source; firmware palette, alpha levels, resolution and sampling recorded in assets/animation/manifest.json',
              'clips':clips}
    (stage/'manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n')
    # All inputs are verified before the tracked source set changes.
    for clip in clips:shutil.copyfile(stage/clip['file'],target/clip['file'])
    shutil.copytree(stage/'rgba',target/'rgba',dirs_exist_ok=True)
    shutil.copyfile(stage/'manifest.json',target/'manifest.json')
    active={c['file'] for c in clips}
    for path in target.glob('*.gif'):
        if path.name not in active:path.unlink()
    active_png={r['file'] for c in clips for r in c['rgba_frames']}
    for path in (target/'rgba').rglob('*.png'):
        if path.relative_to(target).as_posix() not in active_png:path.unlink()
    print(f'Installed {len(clips)} clips / {sum(len(c["rgba_frames"]) for c in clips)} RGBA samples in {target}')
if __name__=='__main__':main()
