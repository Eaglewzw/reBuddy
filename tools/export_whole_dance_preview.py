#!/usr/bin/env python3
"""Export complete authored character poses; never articulate body cutouts."""
import hashlib,json
from pathlib import Path
from PIL import Image, ImageDraw, ImageFilter, ImageChops
ROOT=Path(__file__).resolve().parents[1]
SOURCE=ROOT/'assets/images/source/dance/full-pose'
OUT=ROOT/'assets/gifs/dance-preview'
R=getattr(Image,'Resampling',Image)

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
    path=SOURCE/'groove-atlas.png'
    # Exactly one fixed sampling for the entire sheet: no pose-wise resizing.
    sheet=Image.open(path).convert('RGBA').resize((640,640),R.LANCZOS)
    room=Image.open(ROOT/'assets/images/source/room/room.png').convert('RGBA').resize((320,240),R.LANCZOS)
    frames=[];records=[]
    for index in range(16):
        sprite=sheet.crop((index%4*160,index//4*160,index%4*160+160,index//4*160+160))
        alpha=sprite.getchannel('A');solid=alpha.point(lambda n:255 if n>128 else 0)
        # Keep the full connected robot, excluding isolated generated alpha
        # specks. There are no head/arm/gripper masks or part transforms.
        pixels=solid.load();remaining={(x,y) for y in range(160) for x in range(160) if pixels[x,y]}
        groups=[]
        while remaining:
            seed=remaining.pop();todo=[seed];group={seed}
            while todo:
                x,y=todo.pop()
                for p in ((x-1,y),(x+1,y),(x,y-1),(x,y+1)):
                    if p in remaining:remaining.remove(p);group.add(p);todo.append(p)
            groups.append(group)
        body=max(groups,key=len);mask=Image.new('L',(160,160))
        for p in body:mask.putpixel(p,255)
        # Retain anti-aliased silhouette edges without the distant halo.
        sprite.putalpha(ImageChops.multiply(alpha,mask.filter(ImageFilter.MaxFilter(3))))
        base=mask.crop((0,140,160,160)).getbbox();assert base,(index,'no pedestal')
        floor=140+base[3]-1;center=(base[0]+base[2]-1)/2
        # Source redraws drift in scale. Normalize the COMPLETE authored pose
        # against the rigid pedestal, never its changing full-body bounding box.
        # This is asset calibration, not a time-varying playback transform.
        calibration=93/(base[2]-base[0])
        wanted=round(160*calibration)
        options=[]
        for dimension in range(wanted-2,wanted+3):
            candidate=sprite.resize((dimension,dimension),R.LANCZOS)
            calibrated_mask=candidate.getchannel('A').point(lambda n:255 if n>128 else 0)
            band=round(140*dimension/160)
            base=calibrated_mask.crop((0,band,dimension,dimension)).getbbox()
            options.append((abs(base[2]-base[0]-93),abs(dimension-wanted),dimension,candidate,base,band))
        _,_,dimension,sprite,base,band=min(options,key=lambda p:p[:3])
        calibration=dimension/160
        floor=band+base[3]-1;center=(base[0]+base[2]-1)/2
        origin=(round(155-center),190-floor)
        stage=room.copy();stage.alpha_composite(sprite,origin);frames.append(stage.convert('RGB'))
        records.append(dict(index=index,base_width=base[2]-base[0],source_calibration=dimension/160,origin=origin,base_floor=[155,190]))
    # Later redraws grow in several rigid parts. Exclude them instead of
    # blending or stretching body pieces. Three consistent complete poses
    # make two short downbeat / return phrases with clearly held accents.
    keys=[[1,3],[2,2],[3,4],[2,2],[1,1]]*2
    assert all(records[i]['base_width']==93 for i in (1,2,3)), 'Pedestal size drift'
    sequence=[frames[i] for i,n in keys for _ in range(n)];assert len(sequence)==24
    palette_source=Image.new('RGB',(1280,960))
    for i,f in enumerate(frames):palette_source.paste(f,(i%4*320,i//4*240))
    palette=palette_source.quantize(colors=256)
    encoded=[]
    neutral=sequence[0].quantize(palette=palette,dither=Image.NONE)
    for f in sequence:
        p=f.quantize(palette=palette,dither=Image.NONE)
        a,b,c=ImageChops.difference(f,sequence[0]).split()
        changed=ImageChops.lighter(ImageChops.lighter(a,b),c).point(lambda v:255 if v else 0)
        encoded.append(Image.composite(p,neutral,changed))
    OUT.mkdir(parents=True,exist_ok=True);out=OUT/'02_固定比例律动_预览.gif'
    encoded[0].save(out,save_all=True,append_images=encoded[1:],duration=[120,130]*12,loop=0,disposal=2,optimize=False)
    with Image.open(out) as gif:
        assert gif.n_frames==24
        total=0;first=gif.convert('RGB')
        for i in range(24):
            gif.seek(i);total+=gif.info['duration']
            for box in [(0,0,65,240),(270,0,320,240),(0,205,320,240)]:assert gif.convert('RGB').crop(box).tobytes()==first.crop(box).tobytes()
        assert total==3000 and gif.convert('RGB').tobytes()==first.tobytes()
    review=Image.new('RGB',(960,240))
    for i,pose in enumerate([1,2,3]):review.paste(frames[pose],(320*i,0))
    review.save(ROOT/'assets/images/previews/dance-whole-pose-contact.png')
    report=dict(status='single_whole_character_method_preview_not_integrated',fps=8,duration_ms=3000,keys=keys,
                file=out.name,sha256=sha(out),source=str(path.relative_to(ROOT)),source_sha256=sha(path),registration=records,
                geometry='Whole authored character per pose; offline complete-pose pedestal-size calibration to daily 93px; floor translation only during playback, no body-part transforms or interpolation.',
                used_poses=[1,2,3],rejected_size_drift_poses=[0,4,5,6,7,8,9,10,11,12,13,14,15],
                limitations=['Only three complete poses: consistency trial, not the finished dance library or a base-yaw demonstration.','Other nine actions have not been redrawn with this method.'])
    report['size_check']={'daily_pedestal_width_px':93,'delivered_pedestal_widths_px':[records[i]['base_width'] for i in (1,2,3)],'registered_floor':[155,190],'note':'Whole-pose calibration and pose selection reduce source scale drift; this does not prove identical local contours.'}
    (OUT/'manifest.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(records));print(out)
if __name__=='__main__':main()
