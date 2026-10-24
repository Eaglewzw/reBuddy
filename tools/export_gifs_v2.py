#!/usr/bin/env python3
"""Export current 8 FPS GIF sources; rebuild firmware resources separately."""
import argparse
import hashlib
import json
from pathlib import Path
from PIL import Image, ImageDraw, ImageFont, ImageFilter

ROOT=Path(__file__).resolve().parents[1]
DESIGN=ROOT/'assets/design/interaction-v3-8fps'
BG=(248,247,239)
RESAMPLE=getattr(Image,'Resampling',Image)

def sha(path): return hashlib.sha256(path.read_bytes()).hexdigest()
def pale(p): return min(p)>205 and max(p)-min(p)<35

def component(seed,allowed,seen):
    todo=[seed];result=[]
    while todo:
        p=todo.pop()
        if p in seen or p not in allowed: continue
        seen.add(p);result.append(p);x,y=p
        todo.extend(((x-1,y),(x+1,y),(x,y-1),(x,y+1)))
    return result

def idle_face_mask(pose):
    """Find the display interior, including its enclosed luminous eyes.

    A rectangular eye crop cut through the right eye and left old pixels below
    it. Replace the complete display interior instead, with the shell/body
    remaining the exact master pixels. No rescaling or generated new artwork.
    """
    region={(x,y) for y in range(27,83) for x in range(91,155)}
    dark={p for p in region if max(pose.getpixel(p))<140}
    seen=set();parts=[]
    for p in dark:
        if p not in seen:parts.append(component(p,dark,seen))
    panel=set(max(parts,key=len))
    assert len(panel)>500, 'Idle display interior not found'
    outside=set();remaining=region-panel
    for p in remaining:
        x,y=p
        if x in (91,154) or y in (27,82):component(p,remaining,outside)
    panel=region-outside
    mask=Image.new('L',pose.size)
    for p in panel:mask.putpixel(p,255)
    return mask

def compose_idle(poses):
    master=poses[0];mask=idle_face_mask(master)
    # Feather only the inner edge. The full eye region is opaque, so old open
    # eyes cannot survive beneath a closing eyelid.
    inner=mask.filter(ImageFilter.MinFilter(3))
    soft=inner.filter(ImageFilter.GaussianBlur(.45))
    # Constrain feathering to the black display; never touch the shell.
    soft.putdata([min(a,b) for a,b in zip(soft.getdata(),mask.getdata())])
    result=[]
    for pose in poses:
        frame=master.copy();frame.paste(pose,(0,0),soft);result.append(frame)
    return result,soft

def prepare(cell,clear_cavity,cavity_seeds=None):
    px=list(cell.getdata());edge=[(x,y) for x in range(160) for y in (0,159)]+[(x,y) for y in range(160) for x in (0,159)]
    allowed={(x,y) for y in range(160) for x in range(160) if pale(px[y*160+x])}
    seen=set();outside=[]
    for seed in edge: outside.extend(component(seed,allowed,seen))
    foreground={(x,y) for y in range(160) for x in range(160)}-seen
    # Ignore disconnected generated decoration; the character stays connected.
    visited=set();parts=[]
    for seed in foreground:
        if seed not in visited: parts.append(component(seed,foreground,visited))
    body=set(max(parts,key=len));holes=[]
    if cavity_seeds is not None:
        # Seeds are reviewed on the sampled source cell, before registration.
        # Only the connected arm gap is removed, not every bright material.
        cavity_colors={(x,y) for y in range(160) for x in range(160)
                       if (lambda p:min(p)>=160 and max(p)-min(p)<=65
                           and p[0]>=p[2]-6)(px[y*160+x])}
        visited=set()
        for xy in cavity_seeds:
            seed=tuple(xy)
            assert seed in cavity_colors, f'Arm-gap seed is not background: {seed}'
            part=component(seed,cavity_colors,visited)
            body.difference_update(part)
            if part:holes.append({'seed':list(seed),'removed_pixels':len(part)})
    elif clear_cavity:
        visited=set()
        for seed in allowed-seen:
            if seed in visited: continue
            part=component(seed,allowed-seen,visited)
            if not part: continue
            cx=sum(x for x,y in part)/len(part);cy=sum(y for x,y in part)/len(part)
            # Upright arm cavity, below wrist and above shoulder highlights.
            if len(part)>=12 and 84<cx<103 and 63<cy<96:
                body.difference_update(part);holes.append([round(cx,2),round(cy,2),len(part)])
    # Locate the wide base itself, not the lower claws in a sleeping pose.
    # The lime front panel is a stable base landmark even when a claw touches
    # the floor and joins the base silhouette. Do not anchor to those fingers.
    lime={(x,y) for y in range(118,160) for x in range(15,105)
          if (lambda p:p[1]>120 and p[0]>80 and p[2]<110 and p[1]>p[0]*.85)(cell.getpixel((x,y)))}
    visited=set();panels=[]
    for seed in lime:
        if seed not in visited:panels.append(component(seed,lime,visited))
    panel=max(panels,key=len)
    a=min(x for x,y in panel);b=max(x for x,y in panel);bottom=max(y for x,y in panel)
    dx=53-(a+b)//2;dy=149-bottom
    mask=Image.new('L',(160,160));mask.putdata([255 if (x,y) in body else 0 for y in range(160) for x in range(160)])
    clean=Image.new('RGB',(160,160),BG);clean.paste(cell,(0,0),mask)
    result=Image.new('RGB',(160,160),BG);result.paste(clean,(dx,dy))
    return result,{'translation_px':[dx,dy],'scale_after_atlas_sampling':1.0,'base_panel_width_px':b-a+1,'base_panel_anchor_after_registration':[53,149],'cavities':holes}

def cavity_seeds_for(name,index):
    path=ROOT/'assets/config/preview_cavity_seeds.json'
    if not path.exists():return None
    config=json.loads(path.read_text())
    return config.get('atlases',{}).get(name,{}).get(str(index))

def check_cavity_source(name,path):
    config=ROOT/'assets/config/preview_cavity_seeds.json'
    if config.exists():
        expected=json.loads(config.read_text()).get('source_sha256',{}).get(name)
        if expected:assert sha(path)==expected, f'Review arm-gap seeds after changing {name}'

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,default=ROOT/'assets/gifs-v2');args=parser.parse_args()
    out=args.output.resolve();old=ROOT/'assets/gifs'
    if out==old or old in out.parents: raise SystemExit('Refusing to overwrite current GIFs')
    protected=[ROOT/'assets/animation/frames.bin',ROOT/'assets/animation/manifest.json',ROOT/'main/pet_animation_table.c',ROOT/'main/pet_animation_table.h']
    before={str(p.relative_to(ROOT)):sha(p) for p in protected if p.is_file()}
    cfg=json.loads((DESIGN/'timing.json').read_text());out.mkdir(parents=True,exist_ok=True)
    room=Image.open(ROOT/'assets/design/room-v1/room.png').convert('RGB').resize((320,240),RESAMPLE.LANCZOS)
    crop=room.crop((80,40,240,200));atlases={};metadata={};sources={}
    qa=ROOT/'build/gifs-v2-qa';qa.mkdir(parents=True,exist_ok=True)
    for name in sorted({c['atlas'] for c in cfg['clips']}):
        source_path=cfg.get('atlas_sources',{}).get(name)
        path=ROOT/source_path if source_path else DESIGN/(name+'-atlas.png')
        check_cavity_source(name,path)
        source=Image.open(path).convert('RGB')
        assert source.width==source.height
        # ONE fixed resampling for the entire atlas, never fit individual poses.
        grid=cfg.get('atlas_grids',{}).get(name,4)
        assert grid in (2,4)
        sheet=source.resize((grid*160,grid*160),RESAMPLE.LANCZOS);poses=[];meta=[]
        for i in range(grid*grid):
            clear=name in ('probe','probe-look-up','nuzzle','nuzzle-clearer','nuzzle-storyboard','nuzzle-cheek','tilt','tilt-clear','happy','happy-open','happy-pose','happy-symmetric-clap','happy-rise-clap','idle') or (name=='withdraw' and i<7) or (name=='recover' and i>6) or (name=='rest' and i<8)
            im,record=prepare(sheet.crop((i%grid*160,i//grid*160,i%grid*160+160,i//grid*160+160)),clear,cavity_seeds_for(name,i))
            poses.append(im);meta.append(record)
        if name=='idle':
            poses,face_mask=compose_idle(poses)
            face_mask.save(qa/'idle-face-mask.png')
        atlases[name]=poses;metadata[name]=meta
        sources[name]={'path':str(path.relative_to(ROOT)),'sha256':sha(path),'size':list(source.size),'grid':[grid,grid],'uniform_atlas_sampling_scale':grid*160/source.width}
        contact=Image.new('RGB',(640,680),BG)
        for i,im in enumerate(poses):
            mask=Image.new('L',im.size);mask.putdata([0 if p==BG else 255 for p in im.getdata()]);stage=crop.copy();stage.paste(im,(0,0),mask)
            contact.paste(stage,(i%4*160,i//4*170+10));ImageDraw.Draw(contact).text((i%4*160+3,i//4*170),str(i),fill='black')
        contact.save(qa/(name+'-contact.png'))
    fonts={size:ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',size) for size in (14,16,18,20)}
    sleep_font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf',28)
    records=[];presentation={}
    for name in atlases:
        # Fixed asset layout calibration, shared by EVERY pose in this atlas.
        # Keep the front base panel at the same screen anchor (133, 189).
        size=round(160*cfg.get('presentation_scale_by_atlas',{}).get(name,1.0))
        scale=size/160
        presentation[name]={'size_px':size,'scale':scale,
                            'screen_origin':[133-round(53*scale),189-round(149*scale)]}
    for clip in cfg['clips']:
        indices=[i for i,ticks in clip['keys'] for _ in range(ticks)];count=len(indices)
        frames=[];durations=[((i+1)*125//10)*10-(i*125//10)*10 for i in range(count)]
        assert sum(durations)==clip['duration_ms'] and set(durations)<={120,130}
        for tick,index in enumerate(indices):
            body=atlases[clip['atlas']][index];mask=Image.new('L',body.size);mask.putdata([0 if p==BG else 255 for p in body.getdata()])
            layout=presentation[clip['atlas']]
            sprite=body.convert('RGBA');sprite.putalpha(mask)
            if layout['size_px']!=160:
                sprite=sprite.resize((layout['size_px'],layout['size_px']),RESAMPLE.LANCZOS)
            # Composite on the full room so the raised head is never cut by
            # the original 160px sprite box. The camera/background stay fixed.
            bounds=sprite.getbbox();sx,sy=layout['screen_origin']
            assert bounds and sx+bounds[0]>=0 and sy+bounds[1]>=0 and sx+bounds[2]<=320 and sy+bounds[3]<=240
            frame=room.copy();frame.paste(sprite,(sx,sy),sprite);phase=tick/count
            if clip['id']=='TILT':
                draw=ImageDraw.Draw(frame)
                for n,(x,y,size) in enumerate(((221,41,18),(241,31,20),(252,58,16))):
                    if .22+n*.08<=phase<.81:draw.text((x,y),'?',font=fonts[size],fill=(70,84,42))
            if clip['id']=='SLEEP':
                overlay=Image.new('RGBA',frame.size);draw=ImageDraw.Draw(overlay)
                for n in range(3):
                    age=(tick*125-n*2000)%6000
                    if age>=4000:continue
                    p=age/4000
                    # Stay clearly visible during the rise; fade only near
                    # the end rather than becoming faint immediately.
                    opacity=round(245*min(1,age/250)*min(1,(4000-age)/900))
                    draw.text((226+round(20*p),112-round(55*p)),'Z',font=sleep_font,fill=(74,101,26,opacity))
                frame=Image.alpha_composite(frame.convert('RGBA'),overlay).convert('RGB')
            frames.append(frame)
        palette_source=Image.new('RGB',(1280,960),BG)
        for i in range(16):palette_source.paste(frames[min(count-1,i*count//16)],(i%4*320,i//4*240))
        palette=palette_source.quantize(colors=256)
        encoded=[frame.quantize(palette=palette,dither=Image.NONE) for frame in frames]
        path=out/clip['file'];options={'save_all':True,'append_images':encoded[1:],'duration':durations,'disposal':2,'optimize':False}
        if clip['loop']:options['loop']=0
        encoded[0].save(path,**options)
        decoded=Image.open(path);actual=[]
        for i in range(decoded.n_frames):decoded.seek(i);actual.append(decoded.info['duration'])
        assert sum(actual)==clip['duration_ms'] and all(t>=120 and t%10==0 for t in actual)
        assert ('loop' in decoded.info)==clip['loop']
        frames[len(frames)//2].save(qa/(clip['id']+'.png'))
        records.append({**clip,'nominal_fps':8,'timeline_ticks':count,'encoded_frames':decoded.n_frames,'encoded_delays_ms':actual,'unique_authored_poses':len(set(indices)),'sha256':sha(path)})
    assert before=={str(p.relative_to(ROOT)):sha(p) for p in protected if p.is_file()},'Current assets changed'
    report={'status':'current_firmware_source_gifs_pending_device_review','fps':8,'canvas':[320,240],'scale_policy':'Fixed atlas sampling and constant per-atlas presentation calibration anchored at the base. No time-varying/per-pose scaling, tweening, warping or camera zoom.','sources':sources,'registration':metadata,'presentation':presentation,'clips':records}
    (out/'manifest.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(f'Exported {len(records)} current GIFs to {out}; rebuild runtime with tools/build_screen_preview.py')

if __name__=='__main__':main()
