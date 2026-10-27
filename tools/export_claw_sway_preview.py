#!/usr/bin/env python3
"""Original gripper sides plus in-place base yaw; separate from firmware assets."""
import hashlib
import argparse
import json
import math
from pathlib import Path
from PIL import Image, ImageChops

ROOT=Path(__file__).resolve().parents[1]
DESIGN=ROOT/'assets/images/source/cat-v6'
OUT=ROOT/'assets/gifs/cat-v6'
R=getattr(Image,'Resampling',Image)

def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--revision',choices=['claw-sway-v6'],default='claw-sway-v6')
    parser.parse_args()
    config=DESIGN/'rig.json';rig=json.loads(config.read_text())
    assert rig['fps']==8
    assert len(rig['jaw_crops']) == len(rig['jaw_pivots']) == 2
    assert all(len(sockets) == 2 for sockets in rig['socket_centers'])
    frame_count=len(rig['view_indices'])
    assert frame_count%2==0 and frame_count*125==rig['duration_ms']
    sheet=Image.open(ROOT/rig['body_source']).convert('RGBA')
    jaw_source=Image.open(ROOT/rig['jaw_source']).convert('RGBA')
    size=rig['body_cell_px'];target=rig['body_render_px'];scale=target/size
    assert sheet.size==(size*3,size)
    bodies=[];origins=[];bases=[]
    for i in range(3):
        body=sheet.crop((i*size,0,(i+1)*size,size))
        # Align the cylindrical floor contact, never the rotating front panel.
        x0,y0,x1,y1=body.getchannel('A').point(lambda v:255 if v>128 else 0).crop((0,600,size,size)).getbbox()
        origin=(rig['base_anchor'][0]-round((x0+x1)*scale/2),rig['base_anchor'][1]-round((600+y1)*scale))
        bodies.append(body);origins.append(origin)
        bases.append({'width_px':(x1-x0)*scale,'center_x':origin[0]+(x0+x1)*scale/2,'bottom_y':origin[1]+(600+y1)*scale})
    assert max(b['width_px'] for b in bases)-min(b['width_px'] for b in bases)<=2
    jaws=[];radius=128
    for box,pivot in zip(rig['jaw_crops'],rig['jaw_pivots']):
        # Each crop contains the entire green shell AND black pad. Rotate the
        # complete side once; never articulate the black tip inside its shell.
        part=jaw_source.crop(box)
        part=part.resize(tuple(round(v*rig['jaw_setup_scale']) for v in part.size),R.LANCZOS)
        px,py=[round((pivot[j]-box[j])*rig['jaw_setup_scale']) for j in (0,1)]
        layer=Image.new('RGBA',(radius*2,radius*2))
        layer.alpha_composite(part,(radius-px,radius-py));jaws.append(layer)
    room=Image.open(ROOT/'assets/images/source/room/room.png').convert('RGBA').resize((320,240),R.LANCZOS)
    frames=[];timeline=[]
    for i,view in enumerate(rig['view_indices']):
        angles=[];robot=bodies[view].copy()
        for j,layer in enumerate(jaws):
            if 'jaw_motion' in rig:
                motion=rig['jaw_motion'][j]
                angle=motion['rest_deg']
                if motion['kind']=='sine':
                    angle+=motion['amplitude_deg']*math.sin(2*math.pi*i*125/motion['period_ms']+motion.get('phase_rad',0))
                else:
                    assert motion['kind']=='fixed'
            else:
                phase=2*math.pi*(i*125)/rig['jaw_period_ms']+(rig['right_phase_rad'] if j else 0)
                angle=rig['jaw_rest_deg'][j]+rig['jaw_sway_deg']*math.sin(phase)
            if j not in rig.get('baked_jaws',[]):
                rotated=layer.rotate(angle,resample=R.BICUBIC,center=(radius,radius))
                x,y=rig['socket_centers'][view][j]
                robot.alpha_composite(rotated,(x-radius,y-radius))
            angles.append(angle)
        frame=room.copy();frame.alpha_composite(robot.resize((target,target),R.LANCZOS),origins[view])
        frames.append(frame.convert('RGB'));timeline.append({'at_ms':i*125,'view':view,'jaw_angles_deg':angles})
    contact=Image.new('RGB',(320*8,240*math.ceil(frame_count/8)))
    for i,frame in enumerate(frames):contact.paste(frame,(i%8*320,i//8*240))
    palette=contact.quantize(colors=256)
    indexed=[f.quantize(palette=palette,dither=getattr(Image,'Dither',Image).NONE) for f in frames]
    # Stabilize untouched background colours across Pillow palette-cache states.
    for i,frame in enumerate(frames[1:],1):
        r,g,b=ImageChops.difference(frame,frames[0]).split()
        mask=ImageChops.lighter(ImageChops.lighter(r,g),b).point(lambda v:255 if v else 0)
        indexed[i]=Image.composite(indexed[i],indexed[0],mask)
    OUT.mkdir(parents=True,exist_ok=True)
    output=OUT/rig.get('output_file','01_夹爪摇摆_底盘回转.gif')
    indexed[0].save(output,save_all=True,append_images=indexed[1:],duration=[120,130]*(frame_count//2),loop=0,disposal=2,optimize=False)
    with Image.open(output) as gif:
        assert gif.n_frames==frame_count
        duration=0;first=gif.convert('RGB');decoded=[]
        for i in range(gif.n_frames):
            gif.seek(i);frame=gif.convert('RGB');decoded.append(frame)
            assert gif.info['duration'] in (120,130);duration+=gif.info['duration']
            for box in [(0,0,65,240),(270,0,320,240),(0,205,320,240)]:
                assert frame.crop(box).tobytes()==first.crop(box).tobytes(), 'camera or room moves'
        assert duration==rig['duration_ms']
        # Check the configured moving jaw within each sampled yaw pose.
        for view in range(3):
            ids=[i for i,v in enumerate(rig['view_indices']) if v==view]
            ox,oy=origins[view]
            for j,(x,y) in enumerate(rig['socket_centers'][view]):
                if 'jaw_motion' in rig and rig['jaw_motion'][j]['kind']=='fixed':
                    assert len({t['jaw_angles_deg'][j] for t in timeline})==1
                    continue
                cx,cy=ox+round(x*scale),oy+round(y*scale)
                box=(cx-25,cy-25,cx+30,cy+30)
                expected=min(3,len({round(timeline[i]['jaw_angles_deg'][j],3) for i in ids}))
                assert expected>=2
                assert len({decoded[i].crop(box).tobytes() for i in ids})>=expected
    review=Image.new('RGB',(960,480))
    for n,i in enumerate(rig.get('review_frames',[0,4,8,16,20,24])):review.paste(decoded[i],(n%3*320,n//3*240))
    review.save(DESIGN/'poses.png')
    report={'status':rig.get('status','independent_preview_pending_user_review'),'fps':8,'frames':frame_count,'duration_ms':duration,
            'body_scale_constant':scale,'jaw_scale_constant':rig['jaw_setup_scale'],'base_registration':bases,
            'camera':'fixed','base_motion':'three sampled yaw views, fixed floor-contact center',
            'jaw_motion':rig.get('motion_description','two rigid original jaw tips rotating about their own attachment points'),
            'jaw_structure':rig['jaw_structure'],
            'sources':{p:digest(ROOT/p) for p in [rig['body_source'],rig['jaw_source'],str(config.relative_to(ROOT))]},
            'file':output.name,'sha256':digest(output),'timeline':timeline}
    (OUT/'manifest.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(f'PASS preview:{frame_count} frames/{duration}ms/8FPS; configured jaw motion; room stationary; base contact registered; constant scales')
    print(json.dumps(bases))

if __name__=='__main__':main()
