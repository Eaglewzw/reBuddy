"""Check ten independent whole-character dance GIFs and gross size drift.

These delivery checks do not establish mechanical accuracy or identical local
head/gripper geometry. Complete redraws still need visual and device review.
"""
import hashlib
import json
from pathlib import Path
from PIL import Image

root = Path(__file__).resolve().parents[2]
folder = root/'assets/gifs/dance'
manifest = json.loads((folder/'manifest.json').read_text())
expected = [
    ('GROOVE_BOUNCE', 3000), ('GROOVE_ROCK', 3000), ('GROOVE_SIDE', 3000),
    ('DANCE_POINT', 6000), ('ARM_WAVE', 6000), ('DANCE_ROLL', 6000),
    ('WRIST_ISOLATION', 3000), ('DANCE_TUT', 6000), ('DANCE_HIT', 3000),
    ('DIME_STOP', 6000),
]
assert manifest['fps'] == 8 and manifest['bpm'] == 80
assert manifest['canvas'] == [320, 240]
assert len(manifest['clips']) == 10
assert [(c['id'], c['duration_ms']) for c in manifest['clips']] == expected
assert [c['action_id'] for c in manifest['clips']] == list(range(23, 33))
assert 'demo' not in manifest and 'trace' not in manifest
assert manifest.get('geometry_policy'), 'document the complete-character rendering policy'
assert {p.name for p in folder.glob('*.gif')} == {c['file'] for c in manifest['clips']}
assert not (folder/'cat-demo').exists()
assert not (folder/'10_左右交替.gif').exists()
assert not (folder/'00_基本舞蹈_45秒.gif').exists()
assert manifest['sources']
for source in manifest['sources'].values():
    path = root/source['path']
    assert hashlib.sha256(path.read_bytes()).hexdigest() == source['sha256'], path

resampling = getattr(Image, 'Resampling', Image)
room = Image.open(root/'assets/images/source/room/room.png').convert('RGB').resize((320, 240), resampling.LANCZOS)


def pedestal_width(frame):
    """Measure the fixed floor-level silhouette, not the moving body bounds.

Require foreground in five rows of a narrow pedestal band to suppress palette
noise and isolated edges. This is a coarse zoom alarm, not a rigid-body proof.
    """
    xs = []
    for x in range(65, 270):
        count = sum(max(abs(a-b) for a, b in zip(frame.getpixel((x, y)), room.getpixel((x, y)))) > 40
                    for y in range(177, 189))
        if count >= 5:
            xs.append(x)
    assert xs, 'no pedestal detected at the registered floor'
    return max(xs)-min(xs)+1


with Image.open(root/'assets/gifs/daily/01_待机眨眼.gif') as daily:
    daily_width = pedestal_width(daily.convert('RGB'))

clips = {}
width_report = {}
for item in manifest['clips']:
    path = folder/item['file']
    assert hashlib.sha256(path.read_bytes()).hexdigest() == item['sha256'], path
    assert item['atlas'] == 'individual' and item['claw_key'] == 'idle:0'
    assert item['loop'] is True
    assert item['ticks'] == item['duration_ms']//125
    assert len(item['keys']) == item['ticks']
    assert all(len(key) == 2 and key[1] == 1 for key in item['keys'])
    registration = manifest['registration'][item['id']]
    assert len(registration) == 12, (item['id'], 'missing complete source poses')
    assert all({'floor', 'base_bbox', 'source_index'} <= set(pose) for pose in registration)
    assert not any(registration[index]['source_image_edge_clipped'] for index, count in item['timeline']), (item['id'], 'source-edge-clipped pose used')
    frames = []
    with Image.open(path) as gif:
        assert gif.n_frames == item['ticks'] in (24, 48)
        elapsed = 0
        for tick in range(gif.n_frames):
            gif.seek(tick)
            assert gif.size == (320, 240)
            assert gif.info['duration'] == (120 if tick % 2 == 0 else 130)
            elapsed += gif.info['duration']
            frames.append(gif.convert('RGB').copy())
    assert elapsed == item['duration_ms'] == len(frames)*125
    assert len({frame.tobytes() for frame in frames}) >= 4, (item['id'], 'too few distinct poses')
    reference = frames[0]
    assert reference.tobytes() == frames[-1].tobytes(), (item['id'], 'loop jumps')
    for frame in frames:
        for box in [(0, 0, 60, 240), (270, 0, 320, 240), (0, 195, 320, 240)]:
            assert frame.crop(box).tobytes() == reference.crop(box).tobytes(), (item['id'], 'room moves')
    widths = [pedestal_width(frame) for frame in frames]
    assert min(widths) >= daily_width*.8 and max(widths) <= daily_width*1.2, (item['id'], 'pedestal differs substantially from daily', widths, daily_width)
    assert max(widths)/min(widths) <= 1.15, (item['id'], 'gross scale drift in pedestal silhouette', widths)
    width_report[item['id']] = [min(widths), max(widths)]
    clips[item['id']] = frames

assert len({hashlib.sha256(b''.join(frame.tobytes() for frame in frames)).hexdigest()
            for frames in clips.values()}) == 10, 'duplicate action sequences'
print('PASS ten independent whole-character GIFs; 8 FPS; fixed room; loop continuity; source hashes; no combined demo')
print(f'Coarse pedestal-size check against daily {daily_width}px: {width_report}')
print('Visual review still required: this check does not prove identical local geometry or physical joint accuracy.')
