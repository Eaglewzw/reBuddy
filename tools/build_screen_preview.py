#!/usr/bin/env python3
"""Encode the current, reviewed GIFs for firmware without redrawing their pixels.

Full 320x240 decoded GIF frames become RGB565 and are deduplicated. Each action
has an independent anchor; other frames use lossless XOR deltas when smaller.
The first frame stays raw for allocation-free fallback. Source GIF timing is
preserved for Daily. Dance uses fixed whole-frame fitting and optional temporal
sampling, preserving total duration. No old atlas inputs are used.
"""
import hashlib
import json
from pathlib import Path
import struct
import zlib
import bisect
from array import array
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
    records = [(SOURCE, record) for record in source['clips']]
    dance_dir = ROOT / 'assets/gifs/dance'
    dance = json.loads((dance_dir / 'manifest.json').read_text())
    # One palette across the dance collection avoids frame-to-frame color
    # flicker and allows matching pixels/poses to deduplicate more effectively.
    palette_samples=[]
    for record in dance['clips']:
        with Image.open(dance_dir/record['file']) as gif:
            for tick in (0,gif.n_frames//2,gif.n_frames-1):
                gif.seek(tick)
                palette_samples.append(gif.convert('RGB').resize((240,240),getattr(Image,'Resampling',Image).LANCZOS))
    montage=Image.new('RGB',(240*len(palette_samples),240))
    for i,sample in enumerate(palette_samples):montage.paste(sample,(i*240,0))
    dance_palette=montage.quantize(colors=dance['runtime_colors'],method=0,dither=0)
    palette_rgb=dance_palette.getpalette()[:768]
    records += [(dance_dir, record) for record in dance['clips']]
    for folder, record in records:
        path = folder / record['file']
        assert sha(path.read_bytes()) == record['sha256'], f'Stale source hash: {path}'
        poses = [pose for pose, count in record['keys'] for _ in range(count)]
        indices, times, elapsed = [], [], 0
        with Image.open(path) as gif:
            assert gif.n_frames == len(poses)
            assert gif.size == ((640, 640) if record.get('screen_fit') else (WIDTH, HEIGHT))
            ticks = list(range(len(poses)))
            sampling = record.get('runtime_sampling_ms')
            if sampling:
                source_times, source_elapsed = [], 0
                for tick in ticks:
                    gif.seek(tick)
                    source_times.append(source_elapsed)
                    source_elapsed += gif.info['duration']
                assert source_elapsed == record['duration_ms'] and source_elapsed % sampling == 0
                ticks = [bisect.bisect_right(source_times, t)-1 for t in range(0, source_elapsed, sampling)]
            for tick in ticks:
                pose = poses[tick]
                gif.seek(tick)
                screen = gif.convert('RGB')
                if record.get('screen_fit'):
                    fitted = screen.resize((240, 240), getattr(Image, 'Resampling', Image).LANCZOS)
                    screen = Image.new('RGB', (WIDTH, HEIGHT), gif.convert('RGB').getpixel((0, 0)))
                    screen.paste(fitted, (40, 0))
                    screen=screen.quantize(palette=dance_palette,dither=0).convert('RGB')
                pixels = rgb565(screen)
                claw_key = record['claw_key'] if 'claw_key' in record else f"{record['atlas']}:{pose}"
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
                duration = sampling or gif.info['duration']
                assert duration > 0 and duration % 10 == 0
                elapsed += duration
        assert elapsed == record['duration_ms'] and len(indices) <= 128
        clips.append(dict(id=record['id'], source_gif=str(path.relative_to(ROOT)),
                          source_sha256=record['sha256'], frames=indices, at_ms=times,
                          duration_ms=elapsed, loop=record['loop']))
    # Independent anchors allow arbitrary seeking, reverse playback and STOP
    # recovery without replaying earlier animation frames. Never chain deltas.
    anchors = {c['frames'][0] for c in clips}
    candidates = {i:set() for i in range(len(frames))}
    for clip in clips:
        for i in clip['frames']:candidates[i].add(clip['frames'][0])
    original_bytes = offset
    offset, blobs, delta_count = 0, [], 0
    for i, (frame, pixels) in enumerate(zip(frames, decoded)):
        codec = 0 if i == 0 else 1
        data = pixels if codec == 0 else zlib.compress(pixels, 9)
        if i:
            values=array('H');values.frombytes(pixels)
            if __import__('sys').byteorder!='little':values.byteswap()
            palette=list(dict.fromkeys(values))
            if len(palette)<=256:
                lookup={value:index for index,value in enumerate(palette)}
                indexed=bytes(lookup[v] for v in values)
                packed=struct.pack('<H',len(palette))+struct.pack('<'+'H'*len(palette),*palette)+zlib.compress(indexed,9)
                if len(packed)<len(data):codec,data=3,packed
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
    choreography_mapping = [by_id[ACTIONS[i]] if i < 9 else 255 for i in range(45)]
    for record in dance['clips']:
        choreography_mapping[record['action_id']] = by_id[record['id']]
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
                    visual_status='current_gifs_pending_device_review', format='RBG2 RGB565 LE; codec0 raw, codec1 zlib, codec2 uint32 base + zlib XOR; codec3 uint16 palette count + RGB565 palette + zlib indices; no delta chains',
                    size=[WIDTH, HEIGHT], nominal_fps=8, unique_frames=len(frames), bytes=offset,
                    face_mode='baked', effects_mode='baked', source='assets/gifs/daily/manifest.json',
                    source_sha256=sha((SOURCE / 'manifest.json').read_bytes()),
                    dance_source='assets/gifs/dance/manifest.json',
                    dance_source_sha256=sha((dance_dir / 'manifest.json').read_bytes()),
                    dance_conversion=dance['conversion'], dance_palette_rgb=palette_rgb,
                    dance_palette_method='shared 256-color median-cut palette from first/middle/last source frames; no dithering',
                    choreography_clip_map=choreography_mapping,
                    room_source=str(room_source.relative_to(ROOT)), room_source_sha256=sha(room_source.read_bytes()),
                    room_bytes=len(room_blob), room_sha256=sha(room_blob),
                    source_gifs=[c['source_gif'] for c in clips if c.get('source_gif')],
                    frames=frames, clips=clips, clip_map=mapping,
                    compression=dict(scheme='independent-anchor-xor-palette-zlib',
                                     full_frame_zlib_bytes=original_bytes, delta_frames=delta_count,
                                     extra_psram_bytes=RAW_BYTES),
                    limitations=['Daily pixels/timing preserved through RGB565; imported dances use fixed screen fit and documented time sampling.',
                                 '20ms UI timer and device load may affect actual presentation times.',
                                 'Three separate expression previews remain supplementary GIFs.'])
    (OUT / 'manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + '\n')
    header = '''/* Generated by tools/build_screen_preview.py from assets/gifs/daily. */
#pragma once
#include <stdint.h>
typedef struct { uint32_t offset, bytes; int16_t claw_region[4][2]; } rb_frame_meta_t;
typedef struct { unsigned count; uint16_t duration_ms; uint16_t frames[128], at[128]; } rb_clip_meta_t;
extern const rb_frame_meta_t rb_frames[];
extern const rb_clip_meta_t rb_clips[];
extern const uint8_t rb_clip_map[3][9];
extern const uint8_t rb_choreo_clip_map[45];
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
    lines += ['};', 'const uint8_t rb_choreo_clip_map[45]={'+','.join(map(str,choreography_mapping))+'};']
    (ROOT / 'main/pet_animation_table.c').write_text('\n'.join(lines) + '\n')
    print(f'Encoded {len(records)} current GIFs, {len(frames)} unique frames, {offset} bytes; Daily preserved; dance fitted/sampled per source manifest')


if __name__ == '__main__':
    main()
