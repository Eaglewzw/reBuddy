"""Check preview timing and the idle right-eye clipping regression."""
import hashlib
import json
from pathlib import Path
import sys
from PIL import Image, ImageChops

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tools'))
from export_gifs_v2 import RESAMPLE, compose_idle, prepare

directory = ROOT / 'assets/gifs-v2'
manifest = json.loads((directory / 'manifest.json').read_text())
for clip in manifest['clips']:
    path = directory / clip['file']
    assert hashlib.sha256(path.read_bytes()).hexdigest() == clip['sha256']
    with Image.open(path) as gif:
        delays = []
        for i in range(gif.n_frames):
            gif.seek(i)
            assert gif.size == (320, 240)
            delays.append(gif.info['duration'])
        assert set(delays) == {120, 130}
        assert sum(delays) == clip['duration_ms']
        assert sum(delays) == len(delays) * 125

# The old rectangle left a bright vertical strip of the open right eye here.
# Closed-eye frames must contain no luminous pixels above the lower eyelid.
idle = next(c for c in manifest['clips'] if c['id'] == 'IDLE')
indices = [pose for pose, ticks in idle['keys'] for _ in range(ticks)]
with Image.open(directory / idle['file']) as gif:
    checked = 0
    for tick, pose in enumerate(indices):
        if pose not in (5, 6):
            continue
        gif.seek(tick)
        rgb = gif.convert('RGB')
        assert all(min(rgb.getpixel((80+x, 40+y))) < 170
                   for x in range(142, 148) for y in range(52, 61)), tick
        checked += 1
    assert checked >= 2

# Compositing must not animate the shell/body or crop through the eye again.
sheet = Image.open(ROOT / 'assets/design/interaction-v3-8fps/idle-atlas.png').convert('RGB')
sheet = sheet.resize((640, 640), RESAMPLE.LANCZOS)
poses = [prepare(sheet.crop((i%4*160, i//4*160, i%4*160+160, i//4*160+160)), True)[0]
         for i in range(16)]
rendered, mask = compose_idle(poses)
outside = mask.point(lambda p: 255 if p == 0 else 0)
for frame in rendered:
    diff = ImageChops.difference(frame, poses[0]).convert('L')
    assert ImageChops.multiply(diff, outside).getbbox() is None

# Reviewed arm-gap locations must reveal the actual room after GIF decoding,
# including gaps that become enclosed during a gesture. Allow palette error.
room = Image.open(ROOT / 'assets/design/room-v1/room.png').convert('RGB').resize((320, 240), RESAMPLE.LANCZOS)
seeds = json.loads((ROOT / 'assets/config/preview_cavity_seeds.json').read_text())['atlases']
checks = 0
reports = [(directory, manifest)]
expression_dir = directory / 'expressions-preview'
reports.append((expression_dir, json.loads((expression_dir / 'manifest.json').read_text())))
for folder, report in reports:
    for clip in report['clips']:
        atlas = clip.get('atlas', 'three-joint-expressions')
        registration = report['registration'][atlas] if 'atlas' in clip else report['registration']
        sequence = [pose for pose, ticks in clip['keys'] for _ in range(ticks)]
        with Image.open(folder / clip['file']) as gif:
            for tick, pose in enumerate(sequence):
                assert str(pose) in seeds[atlas], (atlas, pose, 'unreviewed arm gap')
                gif.seek(tick)
                rgb = gif.convert('RGB')
                dx, dy = registration[pose]['translation_px']
                for x, y in seeds[atlas][str(pose)]:
                    layout=report.get('presentation',{}).get(atlas,{'scale':1.0,'screen_origin':[80,40]})
                    sx,sy=layout['screen_origin'];scale=layout['scale']
                    point = (sx+round((x+dx)*scale),sy+round((y+dy)*scale))
                    error = max(abs(a-b) for a, b in zip(rgb.getpixel(point), room.getpixel(point)))
                    assert error <= 16, (clip['id'], tick, point, 'room replaced by source background', error)
                    checks += 1
assert checks > 100
print(f'PASS 8 FPS previews, idle eyes/body, and {checks} decoded arm-gap background checks')

# The happy clip used to shrink at entry/exit (32px vs 36px base panel).
# Compare actual decoded lime-panel bounds, rather than trusting scale metadata.
def front_panel_bounds(image):
    rgb=image.convert('RGB')
    points=[(x,y) for y in range(174,191) for x in range(108,157)
            if (lambda c:c[1]>120 and c[0]>80 and c[2]<110 and c[1]>c[0]*.85)(rgb.getpixel((x,y)))]
    assert points
    return min(x for x,y in points), max(x for x,y in points), max(y for x,y in points)

with Image.open(directory / idle['file']) as gif:
    reference=front_panel_bounds(gif)
happy=next(c for c in manifest['clips'] if c['id']=='HAPPY')
with Image.open(directory / happy['file']) as gif:
    for tick in (0,gif.n_frames-1):
        gif.seek(tick)
        bounds=front_panel_bounds(gif)
        assert all(abs(a-b)<=2 for a,b in zip(reference,bounds)), ('happy entry/exit scale',tick,reference,bounds)
print('PASS happy entry/exit base size and anchor match idle within 2 pixels')
