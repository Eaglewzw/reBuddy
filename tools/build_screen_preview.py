#!/usr/bin/env python3
"""Encode daily GIFs and derive compact, connected runtime dance frames.

Full 320x240 decoded GIF frames become RGB565 and are deduplicated. Each action
has an independent anchor; other frames use lossless XOR deltas when smaller.
The first frame stays raw for allocation-free fallback. Source GIF timing is
preserved for Daily. Dance uses fixed whole-frame fitting and optional temporal
sampling and shared neutral endpoints. Sparse turns get offline neutral bridges.
Source GIF files are never modified.
"""
import hashlib
import json
from pathlib import Path
import struct
import zlib
import subprocess
import sys
import tempfile
from array import array
from PIL import Image
import numpy as np
from dance_assets import indexed_frame, rgb565_array, composite

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
    planar = {}
    matte_cache = {}
    room_source = ROOT / "assets/images/source/room/room.png"
    room = Image.open(room_source).convert("RGB").resize((WIDTH, HEIGHT), getattr(Image, "Resampling", Image).LANCZOS)
    room_pixels = rgb565_array(room)
    offset = 0
    records = [(SOURCE, record) for record in source['clips']]
    dance_dir = ROOT / 'assets/gifs/dance'
    dance = json.loads((dance_dir / 'manifest.json').read_text())
    cavity_path=ROOT/'assets/config/dance_cavity_seeds.json'
    cavities=json.loads(cavity_path.read_text())
    assert cavities['runtime_sampling_ms']==50
    for record in dance['clips']:
        assert cavities['source_sha256'][record['id']]==record['sha256'], 'Review dance cavities after source changes'
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
    dance_palette=montage.quantize(colors=64,method=0,dither=0)
    palette_rgb=dance_palette.getpalette()[:768]
    assert sha(bytes(palette_rgb))==cavities['palette_sha256'], 'Review cavity coverage after palette changes'
    palette565 = rgb565_array(Image.fromarray(np.array(palette_rgb, dtype=np.uint8).reshape(1,-1,3))).ravel().tolist()
    records += [(dance_dir, record) for record in dance['clips']]
    with tempfile.TemporaryDirectory() as temp:
        turn_path=Path(temp)/'turn.npz'
        command=[sys.executable,str(ROOT/'tools/interpolate_turn.py'),'--output',str(turn_path)]
        result=subprocess.run(command,capture_output=True)
        if result.returncode:
            subprocess.run([sys.executable,'-s',*command[1:]],check=True)
        turn_frames=np.load(turn_path)['frames']
    neutral=Image.open(dance_dir/'01_nod.gif').convert('RGB').resize((240,240),getattr(Image,'Resampling',Image).LANCZOS)
    for folder, record in records:
        print('Encoding',record['id'],flush=True)
        path = folder / record['file']
        assert sha(path.read_bytes()) == record['sha256'], f'Stale source hash: {path}'
        poses = [pose for pose, count in record['keys'] for _ in range(count)]
        indices, times, elapsed = [], [], 0
        with Image.open(path) as gif:
            assert gif.n_frames == len(poses)
            assert gif.size == ((640, 640) if record.get('screen_fit') else (WIDTH, HEIGHT))
            ticks = list(range(len(poses)))
            sampling = 50 if record.get('screen_fit') else None
            if sampling:
                source_times, source_elapsed = [], 0
                for sample_index, tick in enumerate(ticks):
                    gif.seek(tick)
                    source_times.append(source_elapsed)
                    source_elapsed += gif.info['duration']
                assert source_elapsed == record['duration_ms'] and source_elapsed % sampling == 0
                ticks = [round((gif.n_frames-1)*i/(source_elapsed//sampling-1)) for i in range(source_elapsed//sampling)]
            for sample_index, tick in enumerate(ticks):
                pose = poses[tick]
                gif.seek(tick)
                screen = gif.convert('RGB')
                plane = None
                if record.get('screen_fit'):
                    fitted = Image.fromarray(turn_frames[sample_index]) if record['id']=='DANCE_BASE_TURN' else screen.resize((240, 240), getattr(Image, 'Resampling', Image).LANCZOS)
                    at_neutral=(record.get('role')!='automatic_bookend' and sample_index in (0,len(ticks)-1)) or (record['id']=='DANCE_START' and sample_index==len(ticks)-1) or (record['id']=='DANCE_END' and sample_index==0)
                    if at_neutral: fitted=neutral
                    screen = Image.new('RGB', (WIDTH, HEIGHT), fitted.getpixel((0,0)))
                    screen.paste(fitted, (40, 0))
                if record.get('screen_fit'):
                    regions=cavities['clips'][record['id']].get(str(sample_index),[])
                    image_key=(screen.tobytes(),json.dumps(regions,sort_keys=True))
                    if image_key not in matte_cache: matte_cache[image_key]=indexed_frame(screen,palette_rgb,regions)
                    plane, original = matte_cache[image_key]
                    matte_key = int(original[0,0])
                    pixels = composite(plane, palette565, matte_key, room_pixels)
                else:
                    pixels = rgb565(screen)
                claw_key = record['claw_key'] if 'claw_key' in record else f"{record['atlas']}:{pose}"
                polygon = claws[claw_key]
                key = (plane if plane is not None else pixels, tuple(tuple(p) for p in polygon), record.get('screen_fit'), matte_key if plane is not None else 0)
                if key not in seen:
                    index = len(frames)
                    codec = 0 if index == 0 else 1
                    data = pixels if codec == 0 else zlib.compress(pixels, 9)
                    blob = struct.pack('<4sHHII', b'RBG2', WIDTH, HEIGHT, RAW_BYTES, codec) + data
                    seen[key] = index
                    decoded.append(pixels)
                    if plane is not None: planar[index] = (plane, matte_key)
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
                          duration_ms=elapsed, loop=record['loop'], source_ticks=ticks,
                          runtime_sampling_ms=sampling, composited=bool(record.get('screen_fit'))))
    # Independent anchors allow arbitrary seeking, reverse playback and STOP
    # recovery. Daily deltas stay independent; dance deltas have bounded depth.
    anchors = {c['frames'][0] for c in clips}
    dance_anchors = {i for c in clips if c['composited'] for i in c['frames'][::8]}
    anchors |= dance_anchors
    depths={}
    candidates = {i:set() for i in range(len(frames))}
    for clip in clips:
        for n,i in enumerate(clip['frames']):
            candidates[i].add(clip['frames'][0])
            if clip['composited'] and n and clip['frames'][n-1]<i: candidates[i].add(clip['frames'][n-1])
            if clip['composited']:
                candidates[i].update(j for j in clip['frames'] if j in dance_anchors)
    original_bytes = offset
    offset, blobs, delta_count = 0, [], 0
    for i, (frame, pixels) in enumerate(zip(frames, decoded)):
        if i%100==0: print('Compressing',i,'/',len(frames),flush=True)
        codec = 0 if i == 0 else 1
        data = pixels if codec == 0 else zlib.compress(pixels, 9)
        if i and i not in planar:
            values=array('H');values.frombytes(pixels)
            if __import__('sys').byteorder!='little':values.byteswap()
            palette=list(dict.fromkeys(values))
            if len(palette)<=256:
                lookup={value:index for index,value in enumerate(palette)}
                indexed=bytes(lookup[v] for v in values)
                packed=struct.pack('<H',len(palette))+struct.pack('<'+'H'*len(palette),*palette)+zlib.compress(indexed,9)
                if len(packed)<len(data):codec,data=3,packed
        if i not in anchors and i != 0 and i not in planar:
            for base in sorted(candidates[i]):
                delta = bytes(a ^ b for a, b in zip(pixels, decoded[base]))
                packed = struct.pack('<I', base) + zlib.compress(delta, 9)
                if len(packed) < len(data):
                    codec, data = 2, packed
        if codec == 2:
            delta_count += 1
        if i in planar:
            plane, matte = planar[i]
            codec = 4
            data = struct.pack('<HH',len(palette565),matte) + struct.pack('<'+'H'*len(palette565),*palette565) + zlib.compress(plane,9)
            if i not in dance_anchors:
                for base in sorted(candidates[i]):
                    if base>=i or base not in planar or depths.get(base,8)>=7: continue
                    delta = np.bitwise_xor(np.frombuffer(plane,dtype=np.uint8),np.frombuffer(planar[base][0],dtype=np.uint8)).tobytes()
                    packed = struct.pack('<I',base) + zlib.compress(delta,9)
                    if planar[base][1] == matte and len(packed)<len(data): codec,data = 5,packed
            depths[i]=0 if codec==4 else depths[struct.unpack_from('<I',data)[0]]+1
            frame['composited'] = True
        blob = struct.pack('<4sHHII', b'RBG2', WIDTH, HEIGHT, RAW_BYTES, codec) + data
        frame.update(offset=offset, bytes=len(blob), crc32=zlib.crc32(pixels))
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
    budget=json.loads((ROOT/'assets/config/flash_budget.json').read_text())
    if offset>budget['animation_limit_bytes']:
        raise SystemExit(f'Animation budget exceeded: {offset} > {budget["animation_limit_bytes"]}; no assets written')
    (OUT / 'frames.bin').write_bytes(b''.join(blobs))
    (OUT / 'room.bin').write_bytes(room_blob)
    manifest = dict(asset_version='gif-v2-rgb565', interaction_revision='gif-v2-8fps',
                    visual_status='current_gifs_pending_device_review', format='RBG2 LE; 0 raw RGB565; 1 zlib RGB565; 2 base + zlib XOR RGB565; 3 palette + zlib indices; 4 count/key/palette + zlib index/coverage planes; 5 earlier dance frame + zlib XOR planes; maximum 7 deltas',
                    size=[WIDTH, HEIGHT], nominal_fps=8, unique_frames=len(frames), bytes=offset,
                    face_mode='baked', effects_mode='baked', source='assets/gifs/daily/manifest.json',
                    source_sha256=sha((SOURCE / 'manifest.json').read_bytes()),
                    dance_source='assets/gifs/dance/manifest.json',
                    dance_source_sha256=sha((dance_dir / 'manifest.json').read_bytes()),
                    dance_cavity_source=str(cavity_path.relative_to(ROOT)),
                    dance_cavity_sha256=sha(cavity_path.read_bytes()),
                    dance_conversion=dance['conversion'], dance_palette_rgb=palette_rgb,
                    dance_palette_method='shared 64-color median-cut palette; fresh lookup per frame; no dithering',
                    dance_palette565=palette565, dance_runtime_fps=20, dance_runtime_colors=64,
                    base_turn_interpolation='offline neutral bridges only; original sparse turn poses; screen only, no joint inference',
                    dance_max_delta_depth=7, dance_compositing='offline exterior coverage; runtime indexed alpha against exact room RGB565',
                    dance_sampling='endpoint-inclusive 50ms source-frame selection; shared neutral endpoints; sparse base turn with neutral bridges',
                    choreography_clip_map=choreography_mapping,
                    room_source=str(room_source.relative_to(ROOT)), room_source_sha256=sha(room_source.read_bytes()),
                    room_bytes=len(room_blob), room_sha256=sha(room_blob),
                    source_gifs=[c['source_gif'] for c in clips if c.get('source_gif')],
                    frames=frames, clips=clips, clip_map=mapping,
                    compression=dict(scheme='daily-independent-rgb565; dance-index-coverage-xor-max7',
                                     full_frame_zlib_bytes=original_bytes, delta_frames=delta_count,
                                     extra_psram_bytes=RAW_BYTES),
                    limitations=['Daily pixels/timing preserved through RGB565; endpoint-inclusive dance samples target 20 FPS; base turn retains sparse source poses with neutral bridges, pending device visual acceptance.',
                                 '20ms UI timer and device load may affect actual presentation times.',
                                 'Three separate expression previews remain supplementary GIFs.'])
    (OUT / 'manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + '\n')
    header = '''/* Generated by tools/build_screen_preview.py from assets/gifs/daily. */
#pragma once
#include <stdint.h>
typedef struct { uint32_t offset, bytes, crc32; int16_t claw_region[4][2]; } rb_frame_meta_t;
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
        lines.append('{%du,%du,%du,{%s}},' % (frame['offset'], frame['bytes'], frame['crc32'], polygon))
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
