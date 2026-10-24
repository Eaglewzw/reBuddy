"""Independently decode shipped records and compare every tick with source GIFs."""
import hashlib
import json
from pathlib import Path
import re
import struct
import zlib
from PIL import Image

root = Path(__file__).resolve().parents[2]
cfg = json.loads((root/'assets/config/behavior_defaults.json').read_text())
manifest = json.loads((root/'assets/animation/manifest.json').read_text())
previews = json.loads((root/'assets/gifs-v2/manifest.json').read_text())
claw_regions = json.loads((root/'assets/config/claw_touch_regions.json').read_text())['frames']
blob = (root/'assets/animation/frames.bin').read_bytes()
assert manifest['asset_version'] == 'gif-v2-rgb565'
assert manifest['source'] == 'assets/gifs-v2/manifest.json'
assert hashlib.sha256((root/manifest['source']).read_bytes()).hexdigest() == manifest['source_sha256']
assert len(blob) == manifest['bytes']
assert len(manifest['frames']) == manifest['unique_frames']
room = (root/'assets/animation/room.bin').read_bytes()
assert len(room) == manifest['room_bytes'] == 153612
assert struct.unpack_from('<BBHHHHH', room) == (0x19, 0x12, 0, 320, 240, 640, 0)
assert hashlib.sha256(room).hexdigest() == manifest['room_sha256']
assert hashlib.sha256((root/manifest['room_source']).read_bytes()).hexdigest() == manifest['room_source_sha256']

def rgb565(image):
    # Explicit conversion, independent of the resource builder.
    result = bytearray()
    for red, green, blue in image.convert('RGB').getdata():
        result += struct.pack('<H', ((red >> 3) << 11) | ((green >> 2) << 5) | (blue >> 3))
    return bytes(result)

records = []
end = 0
for frame in manifest['frames']:
    assert frame['offset'] == end, 'overlapping, gapped or reordered records'
    end += frame['bytes']
    assert 16 < frame['bytes'] and end <= len(blob)
    magic, width, height, raw_length, codec = struct.unpack_from('<4sHHII', blob, frame['offset'])
    assert (magic, width, height, raw_length) == (b'RBG2', 320, 240, 153600)
    assert codec in (0, 1, 2)
    records.append((codec, blob[frame['offset']+16:end]))
assert end == len(blob)
assert any(codec == 2 for codec, _ in records), 'lossless delta encoding is not present'

def inflate(data):
    decoder = zlib.decompressobj()
    pixels = decoder.decompress(data)
    assert decoder.eof and not decoder.unused_data and not decoder.unconsumed_tail
    assert len(pixels) == 153600
    return pixels

def independent_pixels(index):
    codec, data = records[index]
    assert codec in (0, 1), 'delta references another delta (chains are forbidden)'
    pixels = data if codec == 0 else inflate(data)
    assert len(pixels) == 153600
    return pixels

decoded = []
for index, frame in enumerate(manifest['frames']):
    codec, data = records[index]
    if codec == 2:
        assert len(data) > 4
        base = struct.unpack_from('<I', data)[0]
        assert 0 <= base < len(records) and base != index, 'invalid/self-referencing delta base'
        original = independent_pixels(base)
        delta = inflate(data[4:])
        pixels = bytes(a ^ b for a, b in zip(original, delta))
    else:
        pixels = independent_pixels(index)
    assert len(pixels) == 153600
    assert hashlib.sha256(pixels).hexdigest() == frame['sha256']
    if index == 0:
        assert codec == 0, 'allocation-free fallback must be a raw record'
    polygon = frame['claw_region']
    assert polygon == claw_regions[frame['claw_key']], 'generated hitbox differs from reviewed source config'
    assert len(polygon) == 4 and all(0 <= x < 160 and 0 <= y < 160 for x, y in polygon)
    turns = []
    for i in range(4):
        a, b, c = polygon[i], polygon[(i+1)%4], polygon[(i+2)%4]
        turns.append((b[0]-a[0])*(c[1]-b[1])-(b[1]-a[1])*(c[0]-b[0]))
    assert all(t > 0 for t in turns) or all(t < 0 for t in turns), 'nonconvex claw hitbox'
    decoded.append(pixels)

source_by_id = {clip['id']: clip for clip in previews['clips']}
assert len(manifest['clips']) == len(source_by_id) + 1
assert len([c for c in manifest['clips'] if c.get('source_gif')]) == len(source_by_id)
checked = 0
for clip in manifest['clips']:
    assert len(clip['frames']) == len(clip['at_ms']) <= 64
    assert clip['at_ms'][0] == 0
    assert clip['at_ms'] == sorted(set(clip['at_ms']))
    assert clip['at_ms'][-1] < clip['duration_ms']
    assert all(0 <= i < len(decoded) for i in clip['frames'])
    if not clip.get('source_gif'):
        assert len(clip['frames']) == 1  # Deliberate folded hold, no separate GIF.
        continue
    source = source_by_id[clip['id']]
    path = root/'assets/gifs-v2'/source['file']
    assert root/clip['source_gif'] == path
    assert hashlib.sha256(path.read_bytes()).hexdigest() == clip['source_sha256'] == source['sha256']
    with Image.open(path) as gif:
        assert gif.size == (320, 240) and gif.n_frames == len(clip['frames'])
        elapsed = 0
        for tick, frame_id in enumerate(clip['frames']):
            gif.seek(tick)
            assert clip['at_ms'][tick] == elapsed
            assert gif.info['duration'] in (120, 130)
            assert decoded[frame_id] == rgb565(gif), (clip['id'], tick, 'firmware changes source GIF pixels')
            elapsed += gif.info['duration']
            checked += 1
        assert elapsed == clip['duration_ms'] == source['duration_ms']
        assert elapsed == gif.n_frames * 125
assert {c['id'] for c in manifest['clips']} >= set(source_by_id)
assert checked == sum(c['ticks'] if 'ticks' in c else sum(n for _, n in c['keys']) for c in previews['clips'])
assert {i for c in manifest['clips'] for i in c['frames']} == set(range(len(decoded))), 'obsolete unused frames remain in runtime'
assert set(manifest['source_gifs']) == {str(Path('assets/gifs-v2')/c['file']) for c in previews['clips']}

mapping = manifest['clip_map']
assert len(mapping) == 3 and all(len(row) == 9 for row in mapping)
ids = ['IDLE', 'PROBE', 'NUZZLE', 'TILT', 'WITHDRAW', 'RECOVER', 'HAPPY', 'REST', 'WAKE']
for row in mapping:
    for action in range(1, 9):
        assert manifest['clips'][row[action]]['id'] == ids[action]
assert manifest['clips'][mapping[0][0]]['id'] == 'IDLE'
assert manifest['clips'][mapping[2][0]]['id'] == 'SLEEP'
folded = manifest['clips'][mapping[1][0]]
withdraw = manifest['clips'][mapping[0][4]]
assert set(folded['frames']) == {withdraw['frames'][-1]}, 'folded idle unexpectedly animates or unfolds'

header = (root/'components/rebuddy_core/include/rb_pet_config.h').read_text()
for key, value in cfg['gesture'].items():
    assert re.search(r'^#define RB_CFG_'+key.upper()+r' '+str(value)+r'u$', header, re.M)
for key, value in cfg['action_durations_ms'].items():
    assert re.search(r'^#define RB_MS_'+key+r' '+str(value)+r'u$', header, re.M)
    source_id = 'NUZZLE' if key == 'NUZZLE_CYCLE' else key
    assert value == source_by_id[source_id]['duration_ms']
print(f'PASS {len(decoded)} independently decoded RGB565 records (raw/zlib/XOR-delta), {checked} exact GIF frame/timing comparisons, clip maps, hitboxes and behavior durations')
