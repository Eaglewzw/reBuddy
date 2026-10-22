#!/usr/bin/env python3
"""Reproduce the selected two-pose preview; never alter firmware resources."""
import argparse
from collections import deque
import hashlib
import json
from pathlib import Path
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--output', type=Path, default=ROOT/'build/approved-probe-preview')
args = parser.parse_args()
out = args.output
out.mkdir(parents=True, exist_ok=True)
manifest = json.loads((ROOT/'assets/approved/probe-v2/manifest.json').read_text())
source = ROOT/manifest['source']
if hashlib.sha256(source.read_bytes()).hexdigest() != manifest['source_sha256']:
    raise SystemExit('Selected source changed; update the approved baseline explicitly before rendering.')
source_image = Image.open(source).convert('RGB')
assert source_image.size == (1774, 887)
frames = []
for index in range(2):
    crop = source_image.crop((index*887, 0, (index+1)*887, 887))
    width, height = crop.size
    rgb = list(crop.get_flattened_data() if hasattr(crop, 'get_flattened_data') else crop.getdata())
    visited = bytearray(width*height)
    alpha = bytearray([255])*(width*height)
    pending = deque(list(range(width)) + [(height-1)*width+x for x in range(width)] +
                    [y*width for y in range(height)] + [y*width+width-1 for y in range(height)])
    # Clear only the exterior pale background; retain the original interior colors.
    while pending:
        p = pending.popleft()
        if visited[p]:
            continue
        visited[p] = 1
        r, g, b = rgb[p]
        if not (r>220 and g>215 and b>195 and max(r,g,b)-min(r,g,b)<55):
            continue
        alpha[p] = 0
        x, y = p%width, p//width
        if x: pending.append(p-1)
        if x+1<width: pending.append(p+1)
        if y: pending.append(p-width)
        if y+1<height: pending.append(p+width)
    crop.putalpha(Image.frombytes('L', (width, height), bytes(alpha)))
    base = [(x,y) for y in range(650,820) for x in range(width) if min(rgb[y*width+x])<130]
    center = (min(x for x,y in base)+max(x for x,y in base))/2
    bottom = max(y for x,y in base)
    bounds = crop.getbbox()
    sprite = crop.crop(bounds)
    frame = Image.new('RGB', (880,880), '#f8f7ef')
    position = (round(310-center+bounds[0]), 820-bottom+bounds[1])
    assert min(position)>=0 and position[0]+sprite.width<=880 and position[1]+sprite.height<=880
    frame.paste(sprite, position, sprite)
    frame = frame.resize((600,600), Image.Resampling.LANCZOS)
    frame.save(out/('idle.png' if index==0 else 'probe.png'))
    frames.append(frame)

comparison = Image.new('RGB', (1200,600))
comparison.paste(frames[0], (0,0))
comparison.paste(frames[1], (600,0))
palette = comparison.quantize(colors=256, method=Image.Quantize.MEDIANCUT)
for size, name in [(600,'probe-v2-poses.gif'), (160,'probe-v2-poses-160.gif')]:
    sequence = [f.resize((size,size), Image.Resampling.LANCZOS).quantize(
        palette=palette, dither=Image.Dither.NONE) for f in frames]
    sequence[0].save(out/name, save_all=True, append_images=sequence[1:],
                     duration=manifest['duration_ms'], loop=0, disposal=2, optimize=False)
comparison.save(out/'comparison.png')
print(out)
