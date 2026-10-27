#!/usr/bin/env python3
"""Export ten independent complete-character dances, without a combined demo.

Artwork is authored as complete poses. A single constant scale is applied to
an entire atlas, then each complete pose is translated to the same floor point.
No body parts, optical-flow warps, crossfades, or animated scales are used.
"""
import argparse
import hashlib
import json
import statistics
import tempfile
from pathlib import Path
from PIL import Image, ImageChops, ImageFilter, ImageDraw

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT/'assets/images/source/dance/individual'
OUT = ROOT/'assets/gifs/dance'
R = getattr(Image, 'Resampling', Image)

def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

def complete_characters(sheet):
    """Extract twelve entire connected characters, including cell overflow."""
    alpha = sheet.getchannel('A')
    binary = alpha.point(lambda v:255 if v>128 else 0)
    width = sheet.width
    remaining = {(i%width,i//width) for i,v in enumerate(binary.getdata()) if v}
    groups = []
    while remaining:
        seed = remaining.pop(); group = {seed}; todo = [seed]
        while todo:
            x,y = todo.pop()
            for p in ((x-1,y),(x+1,y),(x,y-1),(x,y+1)):
                if p in remaining: remaining.remove(p);group.add(p);todo.append(p)
        if len(group)>100: groups.append(group)
    groups = sorted(groups,key=len,reverse=True)[:12]
    assert len(groups)==12, 'expected twelve complete connected robots'
    def bbox(group):
        xs,ys=zip(*group)
        return min(xs),min(ys),max(xs)+1,max(ys)+1
    # Floor rows identify temporal order even when an outstretched head crosses
    # an imaginary cell boundary. Never crop at those cell boundaries.
    groups.sort(key=lambda g:bbox(g)[3])
    groups = [g for row in range(0,12,4) for g in sorted(groups[row:row+4],key=lambda g:bbox(g)[0])]
    cells,edge_flags = [],[]
    for group in groups:
        bounds=bbox(group)
        edge_flags.append(bounds[0]==0 or bounds[1]==0 or bounds[2]==sheet.width or bounds[3]==sheet.height)
        box=(max(0,bounds[0]-2),max(0,bounds[1]-2),min(sheet.width,bounds[2]+2),min(sheet.height,bounds[3]+2))
        sprite=sheet.crop(box)
        mask=Image.new('L',sprite.size)
        ImageDraw.Draw(mask).point([(x-box[0],y-box[1]) for x,y in group],fill=255)
        sprite.putalpha(ImageChops.multiply(sprite.getchannel('A'),mask.filter(ImageFilter.MaxFilter(3))))
        cells.append(sprite)
    return cells,edge_flags

def pedestal(sprite, band_height):
    mask = sprite.getchannel('A').point(lambda v: 255 if v > 128 else 0)
    bounds = mask.getbbox()
    assert bounds
    floor = bounds[3]-1
    band = max(bounds[1], floor-band_height)
    b = mask.crop((0, band, sprite.width, floor+1)).getbbox()
    assert b
    return (b[0], band+b[1], b[2], band+b[3])

def authored_poses(path, room):
    sheet = Image.open(path).convert('RGBA')
    cells,edge_flags = complete_characters(sheet)
    source_band = round(sheet.height/3*.06)
    widths = [pedestal(c,source_band)[2]-pedestal(c,source_band)[0] for c in cells]
    # One sampling scale PER ATLAS. Do not normalize each moving body bbox,
    # which would create the very zoom effect this exporter must avoid.
    scale = 93/statistics.median(widths)
    frames, records = [], []
    for i, cell in enumerate(cells):
        sprite = cell.resize((round(cell.width*scale), round(cell.height*scale)), R.LANCZOS)
        b = pedestal(sprite,round(source_band*scale))
        origin = (round(155-(b[0]+b[2]-1)/2), 190-(b[3]-1))
        bounds = sprite.getchannel('A').point(lambda v:255 if v>128 else 0).getbbox()
        assert origin[0]+bounds[0] >= 60 and origin[0]+bounds[2] <= 270, (path, i, 'horizontal clipping', origin, bounds)
        assert origin[1]+bounds[1] >= 0, (path, i, 'top clipping')
        frame = room.copy(); frame.alpha_composite(sprite, origin)
        frames.append(frame.convert('RGB'))
        records.append(dict(source_index=i, floor=[155,190], base_bbox=[b[0]+origin[0],b[1]+origin[1],b[2]+origin[0],b[3]+origin[1]],
                            origin=list(origin), atlas_scale=scale, source_image_edge_clipped=edge_flags[i]))
    return frames, records

def encode(sequences):
    samples = [f for frames in sequences.values() for f in frames]
    sheet = Image.new('RGB', (320*6, 240*((len(samples)+5)//6)))
    for i,f in enumerate(samples): sheet.paste(f, (i%6*320,i//6*240))
    palette = sheet.quantize(colors=256)
    reference = next(iter(sequences.values()))[0]
    neutral = reference.quantize(palette=palette,dither=Image.NONE)
    result = {}
    for name, frames in sequences.items():
        encoded = []
        for frame in frames:
            p = frame.quantize(palette=palette,dither=Image.NONE)
            r,g,b = ImageChops.difference(frame,reference).split()
            changed = ImageChops.lighter(ImageChops.lighter(r,g),b).point(lambda v:255 if v else 0)
            encoded.append(Image.composite(p,neutral,changed))
        result[name] = encoded
    return result

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--only', type=int, help='render one action to build/ for visual inspection; never partially replace dance/')
    args = parser.parse_args()
    config = json.loads((SOURCE/'timing.json').read_text())
    clips = config['clips']
    if args.only: clips = [c for c in clips if c['number']==args.only]
    room = Image.open(ROOT/'assets/images/source/room/room.png').convert('RGBA').resize((320,240), R.LANCZOS)
    sequences, registration, records, sources = {}, {}, [], {}
    previews = []
    for c in clips:
        path = SOURCE/f"{c['number']:02d}-atlas.png"
        poses, reg = authored_poses(path,room)
        registration[c['id']] = reg
        assert not any(reg[index]['source_image_edge_clipped'] for index,count in c['timeline']), (c['id'], 'selected a pose clipped by the actual source image edge')
        sequence = [poses[index] for index,count in c['timeline'] for _ in range(count)]
        # Exact loop closure reuses the actual first pose, not a redraw that only
        # approximates it. The preceding pose is the authored return transition.
        sequence[-1] = sequence[0].copy()
        assert len(sequence)*125 == c['duration_ms']
        sequences[c['id']] = sequence
        previews.extend(poses[i] for i in c['review_poses'])
        records.append({**c, 'atlas':'individual', 'claw_key':'idle:0', 'ticks':len(sequence), 'loop':True,
                        'keys':[[i,1] for i in range(len(sequence))]})
        prompts = sorted(SOURCE.glob(f"{c['number']:02d}-*prompt.txt"))
        for p in [path, *prompts]:
            sources[str(p.relative_to(ROOT))] = dict(path=str(p.relative_to(ROOT)),sha256=sha(p))
    sources[str((SOURCE/'timing.json').relative_to(ROOT))] = dict(path=str((SOURCE/'timing.json').relative_to(ROOT)),sha256=sha(SOURCE/'timing.json'))
    encoded = encode(sequences)
    out = ROOT/'build/dance-individual-review' if args.only else OUT
    out.mkdir(parents=True,exist_ok=True)
    with tempfile.TemporaryDirectory(dir=out) as temp:
        staging = Path(temp)
        for clip in records:
            frames = encoded[clip['id']]
            path = staging/clip['file']
            frames[0].save(path,save_all=True,append_images=frames[1:],duration=[120,130]*(len(frames)//2),loop=0,disposal=2,optimize=False)
            with Image.open(path) as gif:
                assert gif.n_frames==len(frames),(clip['id'],gif.n_frames,len(frames))
            clip['sha256'] = sha(path)
        manifest = dict(fps=8,bpm=80,canvas=[320,240],status='individual_whole_character_revision_pending_visual_review',
                        sources=sources,clips=records,registration=registration,
                        geometry_policy='Complete authored character poses; one constant atlas sampling scale, floor translation only. No body-part compositing, crossfade, deformation, or animated scale. Local redraw dimensions still require visual review.',
                        physical_validation='Illustrated dance concepts, not measured B601-RS trajectories; real joint ranges/speeds/collision validation remain pending.',
                        timing_note='8 FPS playback clock, with complete key poses held for multiple ticks; not 24/48 unique drawings.',
                        touch_policy='Dance playback suspends daily touch; idle metadata unused.')
        (staging/'manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n')
        for p in staging.iterdir(): p.replace(out/p.name)
    if not args.only:
        (out/'00_基本舞蹈_45秒.gif').unlink(missing_ok=True)
    contact = Image.new('RGB',(1280,240*len(clips)))
    for i,frame in enumerate(previews): contact.paste(frame,(i%4*320,i//4*240))
    contact.save(out/'poses.png' if args.only else ROOT/'assets/images/previews/dance-primitives.png')
    print(f'Exported {len(clips)} independent dances to {out}; no combined choreography generated.')
    print(json.dumps({k:[p['base_bbox'][2]-p['base_bbox'][0] for p in r] for k,r in registration.items()}))

if __name__ == '__main__': main()
