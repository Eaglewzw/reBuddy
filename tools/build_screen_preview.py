#!/usr/bin/env python3
"""Encode the current, reviewed GIFs for firmware without redrawing their pixels.

Full 320x240 decoded GIF frames become RGB565 and are deduplicated. Each action
has an independent anchor; other frames use lossless XOR deltas when smaller.
The first frame stays raw for allocation-free fallback. Source GIF timing is
preserved exactly; no rescaling, extra effects or old atlas inputs are used.
"""
import hashlib
import json
from pathlib import Path
import struct
import zlib
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / 'assets/gifs/daily'
OUT = ROOT / 'assets/animation'
WIDTH, HEIGHT = 320, 240
RAW_BYTES = WIDTH * HEIGHT * 2
ACTIONS = ['IDLE', 'PROBE', 'NUZZLE', 'TILT', 'WITHDRAW', 'RECOVER', 'HAPPY', 'REST', 'WAKE']


def sha(data):
    return hashlib.sha256(data).hexdigest()


def rgb565(image):
    return b''.join(struct.pack('<H', ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3))
                    for r, g, b in image.convert('RGB').getdata())


def main():
    source = json.loads((SOURCE / 'manifest.json').read_text())
    claws = json.loads((ROOT / 'assets/config/claw_touch_regions.json').read_text())['frames']
    frames, clips, blobs, seen, decoded = [], [], [], {}, []
    offset = 0
    dance_folder = ROOT / 'assets/gifs/dance'
    dance_path = dance_folder / 'manifest.json'
    dance = json.loads(dance_path.read_text())
    records = [(SOURCE, record) for record in source['clips']] + [(dance_folder, record) for record in dance['clips']]
    for folder, record in records:
        path = folder / record['file']
        assert sha(path.read_bytes()) == record['sha256'], f'Stale source hash: {path}'
        poses = [pose for pose, count in record['keys'] for _ in range(count)]
        indices, times, elapsed = [], [], 0
        with Image.open(path) as gif:
            assert gif.size == (WIDTH, HEIGHT) and gif.n_frames == len(poses)
            for tick, pose in enumerate(poses):
                gif.seek(tick)
                pixels = rgb565(gif)
                claw_key = record.get('claw_key', f"{record['atlas']}:{pose}")
                polygon = claws[claw_key]
                key = (pixels, tuple(tuple(p) for p in polygon))
                if key not in seen:
                    index = len(frames)
                    codec = 0 if index == 0 else 1
                    data = pixels if codec == 0 else zlib.compress(pixels, 9)
                    blob = struct.pack('<4sHHII', b'RBG2', WIDTH, HEIGHT, RAW_BYTES, codec) + data
                    seen[key] = index
                    decoded.append(pixels)
                    frames.append(dict(name=f"{record['id']}_{tick:03d}", offset=offset,
                                       bytes=len(blob), sha256=sha(pixels), claw_key=claw_key,
                                       claw_region=polygon))
                    blobs.append(blob)
                    offset += len(blob)
                indices.append(seen[key])
                times.append(elapsed)
                duration = gif.info['duration']
                assert duration in (120, 130)
                elapsed += duration
        assert elapsed == record['duration_ms'] and len(indices) <= 64
        clips.append(dict(id=record['id'], source_gif=str(path.relative_to(ROOT)),
                          source_sha256=record['sha256'], frames=indices, at_ms=times,
                          duration_ms=elapsed, loop=record['loop']))
    # Independent anchors allow arbitrary seeking, reverse playback and STOP
    # recovery without replaying earlier animation frames. Never chain deltas.
    anchors = {c['frames'][0] for c in clips}
    candidates = {i: set() for i in range(len(frames))}
    for clip in clips:
        for i in clip['frames']:
            candidates[i].add(clip['frames'][0])
    # Turning the pedestal changes the whole projected body. Keep one
    # independently decodable anchor per sampled yaw view, so those poses do
    # not XOR against a different view. Never introduce delta chains.
    yaw_anchors = {}
    for clip in clips:
        trace = dance.get('trace', {}).get(clip['id'])
        if not trace:
            continue
        for tick, state in enumerate(trace):
            view = state.get('view')
            if view is None:
                continue
            score = sum(abs(v) for v in state['angles_deg']) + abs(state['wrist_roll_deg']) + state['gripper_open_deg']
            candidate = (score, clip['frames'][tick])
            if view not in yaw_anchors or candidate < yaw_anchors[view]:
                yaw_anchors[view] = candidate
    anchors.update(frame for _, frame in yaw_anchors.values())
    for clip in clips:
        trace = dance.get('trace', {}).get(clip['id'])
        if trace:
            for tick, state in enumerate(trace):
                if state.get('view') in yaw_anchors:
                    candidates[clip['frames'][tick]].add(yaw_anchors[state['view']][1])
    original_bytes = offset
    offset, blobs, delta_count = 0, [], 0
    for i, (frame, pixels) in enumerate(zip(frames, decoded)):
        codec = 0 if i == 0 else 1
        data = pixels if codec == 0 else zlib.compress(pixels, 9)
        if i not in anchors and i != 0:
            for base in sorted(candidates[i]):
                delta = bytes(a ^ b for a, b in zip(pixels, decoded[base]))
                packed = struct.pack('<I', base) + zlib.compress(delta, 9)
                if len(packed) < len(data):
                    codec, data = 2, packed
        if codec == 2:
            delta_count += 1
        blob = struct.pack('<4sHHII', b'RBG2', WIDTH, HEIGHT, RAW_BYTES, codec) + data
        frame.update(offset=offset, bytes=len(blob))
        blobs.append(blob)
        offset += len(blob)
    by_id = {clip['id']: i for i, clip in enumerate(clips)}
    dance_ids = {c['action_id']: c['id'] for c in dance['clips']}
    assert set(dance_ids) == set(range(23,33))
    choreography_mapping = [by_id[ACTIONS[i]] if i < 9 else by_id[dance_ids[i]] if i in dance_ids else 255 for i in range(33)]
    folded_index = len(clips)
    clips.append(dict(id='FOLDED_HOLD', frames=[clips[by_id['WITHDRAW']]['frames'][-1]],
                      at_ms=[0], duration_ms=1000, loop=True))
    mapping = [[by_id[a] for a in ACTIONS] for _ in range(3)]
    mapping[1][0] = folded_index
    mapping[2][0] = by_id['SLEEP']
    # Full GIF frames carry the room. This static resource is only a fallback.
    room_source = ROOT / 'assets/images/source/room/room.png'
    room = Image.open(room_source).convert('RGB').resize((WIDTH, HEIGHT), getattr(Image, 'Resampling', Image).LANCZOS)
    room_blob = struct.pack('<BBHHHHH', 0x19, 0x12, 0, WIDTH, HEIGHT, WIDTH * 2, 0) + rgb565(room)
    OUT.mkdir(exist_ok=True)
    (OUT / 'frames.bin').write_bytes(b''.join(blobs))
    (OUT / 'room.bin').write_bytes(room_blob)
    manifest = dict(asset_version='gif-v2-rgb565', interaction_revision='gif-v2-8fps',
                    visual_status='current_gifs_pending_device_review', format='RBG2 RGB565 LE; codec0 raw, codec1 zlib, codec2 uint32 base + zlib XOR; no delta chains',
                    size=[WIDTH, HEIGHT], nominal_fps=8, unique_frames=len(frames), bytes=offset,
                    face_mode='baked', effects_mode='baked', source='assets/gifs/daily/manifest.json',
                    source_sha256=sha((SOURCE / 'manifest.json').read_bytes()),
                    dance_source=str(dance_path.relative_to(ROOT)), dance_source_sha256=sha(dance_path.read_bytes()),
                    choreography_clip_map=choreography_mapping,
                    room_source=str(room_source.relative_to(ROOT)), room_source_sha256=sha(room_source.read_bytes()),
                    room_bytes=len(room_blob), room_sha256=sha(room_blob),
                    source_gifs=[c['source_gif'] for c in clips if c.get('source_gif')],
                    frames=frames, clips=clips, clip_map=mapping,
                    compression=dict(scheme='independent-anchor-xor-zlib',
                                     full_frame_zlib_bytes=original_bytes, delta_frames=delta_count,
                                     extra_psram_bytes=RAW_BYTES),
                    limitations=['RGB565 quantization only; GIF geometry and timing preserved.',
                                 '20ms UI timer and device load may affect actual presentation times.',
                                 'Three separate expression previews remain supplementary GIFs.'])
    (OUT / 'manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + '\n')
    header = '''/* Generated by tools/build_screen_preview.py from assets/gifs/daily. */
#pragma once
#include <stdint.h>
typedef struct { uint32_t offset, bytes; int16_t claw_region[4][2]; } rb_frame_meta_t;
typedef struct { unsigned count; uint16_t duration_ms; uint16_t frames[64], at[64]; } rb_clip_meta_t;
extern const rb_frame_meta_t rb_frames[];
extern const rb_clip_meta_t rb_clips[];
extern const uint8_t rb_clip_map[3][9];
extern const uint8_t rb_choreo_clip_map[33];
#define RB_FRAME_WIDTH 320u
#define RB_FRAME_HEIGHT 240u
#define RB_FRAME_X 0
#define RB_FRAME_Y 0
#define RB_FRAME_BYTES 153600u
#define RB_FRAME_COUNT @COUNT@u
'''
    (ROOT / 'main/pet_animation_table.h').write_text(header.replace('@COUNT@', str(len(frames))))
    lines = ['/* Generated by tools/build_screen_preview.py. */', '#include "pet_animation_table.h"',
             'const rb_frame_meta_t rb_frames[]={']
    for frame in frames:
        polygon = ','.join('{%d,%d}' % tuple(p) for p in frame['claw_region'])
        lines.append('{%du,%du,{%s}},' % (frame['offset'], frame['bytes'], polygon))
    lines += ['};', 'const rb_clip_meta_t rb_clips[]={']
    for clip in clips:
        lines.append('{%d,%d,{%s},{%s}},' % (len(clip['frames']), clip['duration_ms'],
                     ','.join(map(str, clip['frames'])), ','.join(map(str, clip['at_ms']))))
    lines += ['};', 'const uint8_t rb_clip_map[3][9]={']
    lines += ['{' + ','.join(map(str, row)) + '},' for row in mapping]
    lines += ['};', 'const uint8_t rb_choreo_clip_map[33]={'+','.join(map(str,choreography_mapping))+'};']
    (ROOT / 'main/pet_animation_table.c').write_text('\n'.join(lines) + '\n')
    print(f'Encoded {len(records)} current GIFs, {len(frames)} unique frames, {offset} bytes; original pixels/timings preserved through RGB565 conversion')


if __name__ == '__main__':
    main()
