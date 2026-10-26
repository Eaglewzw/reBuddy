#!/usr/bin/env python3
"""Assemble reviewed imagegen sprite cells into fixed-camera 8 FPS dance GIFs."""
import hashlib
import json
from pathlib import Path
from PIL import Image, ImageChops
from dance_motion import build_dances

ROOT=Path(__file__).resolve().parents[1]
DESIGN=ROOT/'assets/images/source/dance/basic'
OUT=ROOT/'assets/gifs/dance'
R=getattr(Image,'Resampling',Image).LANCZOS

def sha(p): return hashlib.sha256(p.read_bytes()).hexdigest()
def panel(cell):
    w,h=cell.size
    allowed={(x,y) for y in range(int(h*.70),h) for x in range(int(w*.08),int(w*.70))
             if (lambda p:p[3]>230 and p[0]>90 and p[1]>120 and p[2]<105 and p[1]>.83*p[0])(cell.getpixel((x,y)))}
    groups=[]
    while allowed:
        todo=[allowed.pop()];group=set(todo)
        while todo:
            x,y=todo.pop()
            for q in ((x-1,y),(x+1,y),(x,y-1),(x,y+1)):
                if q in allowed:allowed.remove(q);group.add(q);todo.append(q)
        groups.append(group)
    group=max(groups,key=len)
    xs=[x for x,y in group];ys=[y for x,y in group]
    return (min(xs),min(ys),max(xs),max(ys))

specs=[
 dict(id='NOD',file='01_节拍点头.gif',atlas='nod-pose',grid=4,duration_ms=2000,
      keys=[[0,2],[1,2],[2,4],[1,2],[0,2],[3,2],[0,2]]),
 dict(id='SWAY',file='02_活泼摇摆.gif',atlas='sway',grid=2,duration_ms=4000,
      keys=[[1,4],[0,4],[1,2],[2,4],[1,4],[0,4],[1,2],[2,4],[1,4]]),
 dict(id='CLAW_BEAT',file='03_夹爪打拍.gif',atlas='claw',grid=2,duration_ms=2000,
      keys=[[0,2],[1,1],[2,2],[1,1],[0,2]]*2),
 dict(id='POSE',file='04_舒展亮相.gif',atlas='nod-pose',grid=4,duration_ms=4000,
      keys=[[0,4],[12,4],[14,16],[12,4],[0,4]]),
 dict(id='SHAKE',file='05_俏皮摇头.gif',atlas='extra',grid=[3,2],duration_ms=2000,
      keys=[[0,2],[1,4],[0,2],[2,4],[0,4]]),
 dict(id='BOW',file='06_礼貌鞠躬.gif',atlas='extra',grid=[3,2],duration_ms=4000,
      keys=[[3,4],[4,6],[5,10],[4,6],[3,6]]),
 dict(id='CAT_DEMO',file='cat-demo/01_猫步律动.gif',atlas='cat',grid=[2,1],duration_ms=2000,
      keys=[[0,2],[1,2],[2,2],[3,3],[4,2],[5,3],[0,2]])]

def cat_layers(room):
    """Rigid two-layer rig: the nose-covering finger never joins the motion."""
    config_path=DESIGN/'cat-hand-v3/rig.json'
    rig=json.loads(config_path.read_text())
    path=ROOT/rig['source']
    source=Image.open(path).convert('RGBA');size=source.height
    assert source.width==2*size
    robot=source.crop((0,0,size,size))
    finger=source.crop((size,0,2*size,size))
    # Extract the intended part, excluding near-transparent generator speckles
    # far outside its silhouette that would otherwise rotate across the body.
    isolated=Image.new('RGBA',finger.size)
    part_box=rig['finger_crop_px']
    isolated.paste(finger.crop(part_box),tuple(part_box[:2]))
    finger=isolated
    # One setup scale matches the isolated hinge to the vacant palm socket.
    # Every animation pose then uses this exact rigid part at the same scale.
    finger_size=round(size*rig['finger_rest_scale'])
    finger=finger.resize((finger_size,finger_size),R)
    finger_pivot=tuple(round(v*finger_size/size) for v in rig['finger_pivot_px'])
    rotating=Image.new('RGBA',(size*2,size*2))
    rotating.alpha_composite(finger,(size-finger_pivot[0],size-finger_pivot[1]))
    a,b,c,d=panel(robot)
    target=round(size*rig['base_panel_width_px']/(c-a+1))
    fixed=robot.resize((target,target),R);a,b,c,d=panel(fixed)
    anchor=rig['base_anchor_px'];origin=(anchor[0]-(a+c)//2,anchor[1]-d)
    frames={};registration={}
    for i,angle in enumerate(rig['angles_deg']):
        posed=robot.copy()
        moved=rotating.rotate(angle,resample=getattr(Image,'Resampling',Image).BICUBIC,
                              center=(size,size))
        posed.alpha_composite(moved,(rig['robot_pivot_px'][0]-size,rig['robot_pivot_px'][1]-size))
        sprite=posed.resize((target,target),R)
        frame=room.copy();frame.alpha_composite(sprite,origin)
        frames[('cat',i)]=frame.convert('RGB')
        registration[i]={'origin':origin,'base_panel_width':c-a+1,'base_panel_anchor':anchor}
    scale=target/size
    nose=rig['fixed_nose_box_px']
    nose_box=[round(origin[j%2]+v*scale) for j,v in enumerate(nose)]
    metadata={'path':str(path.relative_to(ROOT)),'sha256':sha(path),'grid':[2,1],
              'constant_sampling_scale':scale,'cell_size_px':target,
              'rig':str(config_path.relative_to(ROOT)),'rig_sha256':sha(config_path)}
    return frames,registration,metadata,nose_box

def main():
    OUT.mkdir(parents=True,exist_ok=True)
    room=Image.open(ROOT/'assets/images/source/room/room.png').convert('RGBA').resize((320,240),R)
    sprites={};sources={};registration={}
    for name,grid,reference in [('nod-pose',4,0),('sway',2,1),('claw',2,0),('extra',(3,2),0)]:
        p=DESIGN/(name+'-atlas.png')
        source=Image.open(p).convert('RGBA')
        # Sample each sheet uniformly once; never fit a pose to its own box.
        cols,rows=(grid,grid) if isinstance(grid,int) else grid
        size=source.width//cols
        sheet=source.resize((size*cols,size*rows),R)
        cells=[sheet.crop((i%cols*size,i//cols*size,(i%cols+1)*size,(i//cols+1)*size)) for i in range(cols*rows)]
        a,b,c,d=panel(cells[reference]);scale=36/(c-a+1)
        target=round(size*scale)
        sources[name]={'path':str(p.relative_to(ROOT)),'sha256':sha(p),'grid':[cols,rows],
                       'constant_sampling_scale':target/size,'cell_size_px':target}
        registration[name]={}
        used=sorted({i for spec in specs if spec['atlas']==name for i,n in spec['keys']})
        for i in used:
            sprite=cells[i].resize((target,target),R)
            a,b,c,d=panel(sprite)
            origin=(133-(a+c)//2,189-d)
            bbox=sprite.getbbox();assert bbox
            assert origin[0]+bbox[0]>=0 and origin[1]+bbox[1]>=0 and origin[0]+bbox[2]<=320 and origin[1]+bbox[3]<=240,(name,i,origin,bbox)
            frame=room.copy();frame.alpha_composite(sprite,origin)
            sprites[(name,i)]=frame.convert('RGB')
            registration[name][i]={'origin':origin,'base_panel_width':c-a+1,'base_panel_anchor':[133,189]}
    cat,registration['cat'],sources['cat'],nose_box=cat_layers(room)
    sprites.update(cat)
    new_sprites, new_specs, new_sources, new_registration = build_dances(room)
    sprites.update(new_sprites)
    sources.update(new_sources)
    registration.update(new_registration)
    # Both pedestal and covering-nose finger belong to the same fixed layer.
    base_box=(0,158,320,240)
    for frame in cat.values():
        assert frame.crop(base_box).tobytes()==cat[('cat',0)].crop(base_box).tobytes()
        assert frame.crop(nose_box).tobytes()==cat[('cat',0)].crop(nose_box).tobytes()
    sheet=Image.new('RGB',(320*4,240*((len(sprites)+3)//4)))
    for i,frame in enumerate(sprites.values()):sheet.paste(frame,(i%4*320,i//4*240))
    # Keep the approved shared palette stable when revising only one action.
    palette_path=DESIGN/'palette.png'
    palette=Image.open(palette_path) if palette_path.exists() else sheet.quantize(colors=256)
    if not palette_path.exists(): palette.save(palette_path)
    frames_by_id={};records=[]
    def write(path,frames,lock_static_pixels=False):
        durations=[((i+1)*125//10-i*125//10)*10 for i in range(len(frames))]
        encoded=[f.quantize(palette=palette,dither=getattr(Image,'Dither',Image).NONE) for f in frames]
        if lock_static_pixels:
            # Pillow's palette lookup cache can give the same static RGB pixel
            # a nearby index after moving colours change cache insertion order.
            # Preserve reference indices wherever the source RGB did not change.
            for i,frame in enumerate(frames[1:],1):
                r,g,b=ImageChops.difference(frame,frames[0]).split()
                changed=ImageChops.lighter(ImageChops.lighter(r,g),b).point(lambda v:255 if v else 0)
                encoded[i]=Image.composite(encoded[i],encoded[0],changed)
        encoded[0].save(path,save_all=True,append_images=encoded[1:],duration=durations,loop=0,disposal=2,optimize=False)
        with Image.open(path) as gif:
            assert gif.n_frames==len(frames),(path,gif.n_frames,len(frames))
            elapsed=0
            for n in range(gif.n_frames):gif.seek(n);assert gif.info['duration']==durations[n];elapsed+=gif.info['duration']
            assert elapsed==len(frames)*125
    for spec in specs + new_specs:
        frames=[sprites[(spec['atlas'],i)] for i,n in spec['keys'] for _ in range(n)]
        assert len(frames)*125==spec['duration_ms']
        path=OUT/spec['file'];path.parent.mkdir(parents=True,exist_ok=True)
        write(path,frames,lock_static_pixels=spec['id']=='CAT_DEMO' or spec in new_specs);frames_by_id[spec['id']]=frames
        records.append({**spec,'ticks':len(frames),'loop':True,'claw_key':'idle:0','sha256':sha(path)})
    demo=[]
    for name,repeats in [('BASE_TURN',2),('SIDE_BEAT',2),('BOUNCE',2),('ALTERNATE',2),('WAVE',1),('FINALE',1)]:demo.extend(frames_by_id[name]*repeats)
    demo_path=OUT/'00_活力律动_24秒.gif';write(demo_path,demo,lock_static_pixels=True)
    sheet.save(DESIGN/'registered-review.png')
    manifest={'fps':8,'bpm':120,'canvas':[320,240],'status':'screen_dance_pending_device_review',
              'sources':sources,'registration':registration,'clips':records,
              'demo':{'file':demo_path.name,'duration_ms':24000,'sha256':sha(demo_path)},
              'cat_hand_revision':3,'cat_fixed_base_box':list(base_box),'cat_fixed_nose_box':nose_box,
              'geometry_policy':'Constant source scale; no animated scaling, warping or camera movement. Cat base, body and nose-covering finger share one fixed layer; only the opposite finger rotates about its hinge.',
              'touch_policy':'Dance clips are preview-only; daily touch is suspended. Idle hitbox metadata is unused during preview.'}
    (OUT/'manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n')
    print('Exported twelve dance clips, separate cat-inspired demo and 24-second dance.')

if __name__=='__main__':main()
