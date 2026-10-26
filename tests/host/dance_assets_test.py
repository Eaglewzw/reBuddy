"""Check delivered dance timing, montage, stationary room and source provenance."""
import hashlib
import json
from pathlib import Path
from PIL import Image
root = Path(__file__).resolve().parents[2]
folder = root/'assets/gifs/dance'
manifest = json.loads((folder/'manifest.json').read_text())
assert manifest['fps'] == 8
clips = {}
for item in manifest['clips']:
    frames = []
    with Image.open(folder/item['file']) as gif:
        elapsed = 0
        for i in range(gif.n_frames):
            gif.seek(i)
            assert gif.size == (320, 240) and gif.info['duration'] in (120, 130)
            elapsed += gif.info['duration']
            frames.append(gif.convert('RGB').copy())
        assert elapsed == item['duration_ms'] == len(frames)*125
    assert len({f.tobytes() for f in frames}) >= 3, item['id']
    clips[item['id']] = frames
assert len(clips) == 13
# New dance phrases return to their entry pose, including the final held pose.
for action in ('BASE_TURN', 'SIDE_BEAT', 'BOUNCE', 'ALTERNATE', 'WAVE', 'FINALE'):
    assert clips[action][0].tobytes() == clips[action][-1].tobytes(), action + ' does not return to center'
alternate = clips['ALTERNATE']
assert len({frame.crop((0,158,320,240)).tobytes() for frame in alternate}) == 1, 'alternating jaws move the base'
# One hand covers the nose continuously; only the opposite finger may move.
cat = clips['CAT_DEMO']
assert manifest['cat_hand_revision'] == 3
base_box = tuple(manifest['cat_fixed_base_box'])
assert base_box == (0,158,320,240)
assert len({frame.crop(base_box).tobytes() for frame in cat}) == 1
assert cat[0].tobytes() == cat[-1].tobytes(), 'cat loop jumps at restart'
nose_box=(170,105,187,120)
assert tuple(manifest['cat_fixed_nose_box']) == nose_box
assert len({frame.crop(nose_box).tobytes() for frame in cat}) == 1, 'covering hand moves'
hand_box = (187,97,224,140)
assert len({cat[t].crop(hand_box).tobytes() for t in (0,2,4,6,9,11)}) == 6
for frame in cat:
    for box in [(0,0,187,240),(224,0,320,240),(0,0,320,97),(0,140,320,240)]:
        assert frame.crop(box).tobytes() == cat[0].crop(box).tobytes(), 'motion escaped the single moving finger'
# Screen edges/floor are fixed across every pose and clip; no camera movement.
reference = clips['NOD'][0]
for frames in clips.values():
    for frame in frames:
        for box in [(0,0,60,240),(270,0,320,240),(0,210,320,240)]:
            assert frame.crop(box).tobytes() == reference.crop(box).tobytes()
expected = []
for name, count in [('BASE_TURN',2),('SIDE_BEAT',2),('BOUNCE',2),('ALTERNATE',2),('WAVE',1),('FINALE',1)]:
    expected.extend(clips[name]*count)
path = folder/manifest['demo']['file']
assert hashlib.sha256(path.read_bytes()).hexdigest() == manifest['demo']['sha256']
with Image.open(path) as gif:
    assert gif.n_frames == len(expected) == 192
    for i, frame in enumerate(expected):
        gif.seek(i)
        assert gif.convert('RGB').tobytes() == frame.tobytes(), i
for source in manifest['sources'].values():
    path = root/source['path']
    assert hashlib.sha256(path.read_bytes()).hexdigest() == source['sha256']
    if 'rig' in source:
        assert hashlib.sha256((root/source['rig']).read_bytes()).hexdigest() == source['rig_sha256']
    with Image.open(path) as artwork:
        alpha = artwork.getchannel('A')
        assert alpha.getextrema()[0] == 0, 'opaque atlas background'
for poses in manifest['registration'].values():
    if all('fixed_scale' in p for p in poses.values()):
        assert len({p['fixed_scale'] for p in poses.values()}) == 1, 'per-pose scaling'
        assert all(p['fixed_scale'] > 0 and p['floor_center'] == 160 and p['floor_y'] == 190 for p in poses.values())
        continue
    widths = [p['base_panel_width'] for p in poses.values()]
    assert max(widths)-min(widths) <= 2
    assert all(p['base_panel_anchor'] == [133,189] for p in poses.values())
print('PASS thirteen dance/demo clips; cat nose-covering hand/body/base fixed, only opposite finger moves; 8 FPS, exact 24-second montage and source hashes')
