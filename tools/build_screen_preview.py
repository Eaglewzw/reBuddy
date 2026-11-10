#!/usr/bin/env python3
"""Encode Daily GIFs unchanged and compact trajectory-driven Dance RGBA frames.

Dance preserves authored frame durations and true alpha over the room, with a
shared palette and bounded XOR deltas. No temporal interpolation, endpoint
replacement, color-key flood fill or legacy cavity seed file is required.
"""
import argparse
from bisect import bisect_right
import hashlib
import json
from pathlib import Path
import struct
import zlib
from array import array
from PIL import Image
import numpy as np
from dance_assets import indexed_rgba_frame, rgb565_array, composite_native

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / 'assets/gifs/daily'
OUT = ROOT / 'assets/animation'
WIDTH, HEIGHT = 320, 240
RAW_BYTES = WIDTH * HEIGHT * 2
ACTIONS = ['IDLE', 'PROBE', 'NUZZLE', 'TILT', 'RESERVED_4', 'RESERVED_5', 'HAPPY', 'REST', 'WAKE']


def sha(data):
    return hashlib.sha256(data).hexdigest()


def rgb565(image):
    return b''.join(struct.pack('<H', ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3))
                    for r, g, b in image.convert('RGB').getdata())


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dance-render-size',type=int,choices=(120,128,160,240),default=160,
                        help='Screen detail resolution; full-quality source GIF/PNG remains unchanged')
    parser.add_argument('--dance-colors',type=int,choices=(16,32,64),default=32)
    parser.add_argument('--dance-alpha-levels',type=int,choices=(4,8,16),default=8)
    parser.add_argument('--dance-max-fps',type=float,choices=(10,12.5),default=12.5)
    args=parser.parse_args()
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
    # Renderer RGBA is the source of truth for Dance coverage, including holes.
    # Daily GIF conversion below deliberately retains its existing path.
    def rgba_frame(record, index):
        row=record['rgba_frames'][index]
        path=dance_dir/row['file']
        if path.resolve().is_relative_to(dance_dir.resolve()) is False:
            raise ValueError('dance RGBA frame escapes source directory')
        data=path.read_bytes()
        if sha(data)!=row['sha256']:raise ValueError(f'Stale RGBA frame: {path}')
        with Image.open(path) as image:
            fitted=image.convert('RGBA').resize((args.dance_render_size,args.dance_render_size),Image.Resampling.LANCZOS)
            return fitted

    palette_samples=[]
    for record in dance['clips']:
        count=len(record['rgba_frames'])
        for tick in (0,count//2,count-1):
            rgba=rgba_frame(record,tick)
            palette_samples.append(Image.alpha_composite(
                Image.new('RGBA',rgba.size,(255,255,255,255)),rgba).convert('RGB'))
    montage=Image.new('RGB',(args.dance_render_size*len(palette_samples),args.dance_render_size))
    for i,sample in enumerate(palette_samples):montage.paste(sample,(i*args.dance_render_size,0))
    # Thin trajectory lines occupy too few pixels for median-cut to retain
    # their hue. Reserve their ink colors instead of letting cyan become grey.
    trace_colors=[(17,86,111),(57,224,255),(155,236,247)] if any(c.get('trajectory_overlay') for c in dance['clips']) else []
    artwork_colors=args.dance_colors-len(trace_colors)
    dance_palette=montage.quantize(colors=artwork_colors,method=0,dither=0)
    palette_rgb=dance_palette.getpalette()[:artwork_colors*3]+[v for color in trace_colors for v in color]
    palette565=rgb565_array(Image.fromarray(np.array(palette_rgb,dtype=np.uint8).reshape(1,-1,3))).ravel().tolist()
    records += [(dance_dir, record) for record in dance['clips']]
    neutrals={}
    for record in dance['clips']:
        if record.get('role')!='automatic_bookend':
            neutrals.setdefault(record.get('endpoint_group','shared_neutral'),rgba_frame(record,0).tobytes())
    for folder, record in records:
        print('Encoding',record['id'],flush=True)
        path=folder/record['file']
        assert sha(path.read_bytes())==record['sha256'],f'Stale source hash: {path}'
        poses=[pose for pose,count in record['keys'] for _ in range(count)]
        indices,times,elapsed=[],[],0
        is_dance=folder==dance_dir
        sampling=None
        with Image.open(path) as gif:
            assert gif.n_frames==len(poses)
            if is_dance:
                rows=record['rgba_frames'];starts=[];source_ms=0
                for row in rows:starts.append(source_ms);source_ms+=row['duration_ms']
                assert source_ms==record['duration_ms']
                # Seek on the authored timeline; retain both exact endpoints.
                targets=range(0,source_ms,round(1000/args.dance_max_fps))
                ticks=sorted({max(0,bisect_right(starts,t)-1) for t in targets}|{0,len(rows)-1})
                assert 1<=len(ticks)<=256
            else:
                assert gif.size==(WIDTH,HEIGHT)
                ticks=list(range(len(poses)))
            for sample_index,tick in enumerate(ticks):
                plane=None
                if is_dance:
                    fitted=rgba_frame(record,tick)
                    at_neutral=(record.get('role')!='automatic_bookend' and sample_index in (0,len(ticks)-1)) or (record['id']=='DANCE_START' and sample_index==len(ticks)-1) or (record['id']=='DANCE_END' and sample_index==0)
                    # The renderer must supply identical endpoints; never hide a
                    # pose mismatch by replacing its first or last frame.
                    if at_neutral:assert fitted.tobytes()==neutrals[record.get('endpoint_group','shared_neutral')],f'Non-neutral endpoint: {record["id"]}'
                    screen=fitted
                    image_key=screen.tobytes()
                    if image_key not in matte_cache:matte_cache[image_key]=indexed_rgba_frame(screen,palette_rgb,args.dance_alpha_levels)
                    plane,original=matte_cache[image_key]
                    matte_key=65535
                    pixels=composite_native(plane,palette565,matte_key,room_pixels,args.dance_render_size)
                    next_ms=starts[ticks[sample_index+1]] if sample_index+1<len(ticks) else source_ms
                    duration=next_ms-starts[tick]
                    pose=0
                else:
                    pose=poses[tick];gif.seek(tick)
                    pixels=rgb565(gif.convert('RGB'));duration=gif.info['duration']
                claw_key=record['claw_key'] if 'claw_key' in record else f"{record['atlas']}:{pose}"
                polygon=claws[claw_key]
                key=(plane if plane is not None else pixels,tuple(tuple(p) for p in polygon),is_dance,matte_key if plane is not None else 0)
                if key not in seen:
                    index=len(frames)
                    codec=0 if index==0 else 1
                    data=pixels if codec==0 else zlib.compress(pixels,9)
                    blob=struct.pack('<4sHHII',b'RBG2',WIDTH,HEIGHT,RAW_BYTES,codec)+data
                    seen[key]=index;decoded.append(pixels)
                    if plane is not None:planar[index]=(plane,matte_key)
                    frames.append(dict(name=f"{record['id']}_{tick:03d}",offset=offset,
                                       bytes=len(blob),sha256=sha(pixels),claw_key=claw_key,claw_region=polygon))
                    blobs.append(blob);offset+=len(blob)
                indices.append(seen[key]);times.append(elapsed)
                assert duration>0 and duration%10==0
                elapsed+=duration
        assert elapsed==record['duration_ms'] and len(indices)<=256
        clips.append(dict(id=record['id'],source_gif=str(path.relative_to(ROOT)),
                          source_sha256=record['sha256'],frames=indices,at_ms=times,
                          duration_ms=elapsed,loop=record['loop'],source_ticks=ticks,
                          runtime_sampling_ms=sampling,composited=is_dance,
                          alpha_source='renderer_rgba' if is_dance else None))
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
            codec = 6
            data = struct.pack('<HHHH',len(palette565),matte,args.dance_render_size,args.dance_render_size) + struct.pack('<'+'H'*len(palette565),*palette565) + zlib.compress(plane,9)
            if i not in dance_anchors:
                for base in sorted(candidates[i]):
                    if base>=i or base not in planar or depths.get(base,8)>=7: continue
                    delta = np.bitwise_xor(np.frombuffer(plane,dtype=np.uint8),np.frombuffer(planar[base][0],dtype=np.uint8)).tobytes()
                    packed = struct.pack('<I',base) + zlib.compress(delta,9)
                    if planar[base][1] == matte and len(packed)<len(data): codec,data = 7,packed
            depths[i]=0 if codec==6 else depths[struct.unpack_from('<I',data)[0]]+1
            frame['composited'] = True
        blob = struct.pack('<4sHHII', b'RBG2', WIDTH, HEIGHT, RAW_BYTES, codec) + data
        frame.update(offset=offset, bytes=len(blob), crc32=zlib.crc32(pixels))
        blobs.append(blob)
        offset += len(blob)
    by_id = {clip['id']: i for i, clip in enumerate(clips)}
    choreography_mapping = [by_id.get(ACTIONS[i], 255) if i < 9 else 255 for i in range(55)]
    for record in dance['clips']:
        choreography_mapping[record['action_id']] = by_id[record['id']]
    # Preserve numeric slots; retired daily events fall back to idle, while
    # choreography rejects the same retired IDs with the 255 sentinel.
    mapping = [[by_id.get(a, by_id['IDLE']) for a in ACTIONS] for _ in range(3)]
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
                    visual_status='current_gifs_pending_device_review', format='RBG2 LE; 0 raw RGB565; 1 zlib RGB565; 2 base + zlib XOR RGB565; 3 palette + zlib indices; 4 count/key/palette + zlib index/coverage planes; 5 earlier dance frame + zlib XOR planes; 6 native width/height palette-alpha with premultiplied bilinear 240px reconstruction; 7 native XOR; maximum 7 deltas',
                    size=[WIDTH, HEIGHT], nominal_fps=8, unique_frames=len(frames), bytes=offset,
                    face_mode='baked', effects_mode='baked', source='assets/gifs/daily/manifest.json',
                    source_sha256=sha((SOURCE / 'manifest.json').read_bytes()),
                    dance_source='assets/gifs/dance/manifest.json',
                    dance_source_sha256=sha((dance_dir / 'manifest.json').read_bytes()),
                    dance_alpha_source='renderer_rgba; no color-key cavity inference',
                    dance_conversion=dance['conversion'], dance_palette_rgb=palette_rgb,
                    dance_palette_method=f'shared {args.dance_colors}-color palette ({len(trace_colors)} reserved trajectory colors); fresh lookup per frame; no dithering',
                    dance_reserved_colors_rgb=trace_colors,
                    dance_palette565=palette565, dance_runtime_fps=args.dance_max_fps, dance_runtime_colors=args.dance_colors,
                    dance_render_size=args.dance_render_size,dance_alpha_levels=args.dance_alpha_levels,
                    base_turn_interpolation=None,
                    dance_max_delta_depth=7, dance_compositing='native indexed RGBA, fixed-point premultiplied bilinear reconstruction to 240 square; exact room elsewhere',
                    dance_sampling=f'authored frame timestamps, target {args.dance_max_fps:g} FPS with preserved endpoints and duration',
                    choreography_clip_map=choreography_mapping,
                    room_source=str(room_source.relative_to(ROOT)), room_source_sha256=sha(room_source.read_bytes()),
                    room_bytes=len(room_blob), room_sha256=sha(room_blob),
                    source_gifs=[c['source_gif'] for c in clips if c.get('source_gif')],
                    frames=frames, clips=clips, clip_map=mapping,
                    compression=dict(scheme='daily-independent-rgb565; dance-index-coverage-xor-max7',
                                     full_frame_zlib_bytes=original_bytes, delta_frames=delta_count,
                                     extra_psram_bytes=RAW_BYTES),
                    limitations=['Daily pixels/timing preserved; Dance preserves total trajectory duration with screen-only sampling/detail reduction; pending device visual acceptance.',
                                 '20ms UI timer and device load may affect actual presentation times.',
                                 'Three separate expression previews remain supplementary GIFs.'])
    (OUT / 'manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + '\n')
    header = '''/* Generated by tools/build_screen_preview.py from assets/gifs/daily. */
#pragma once
#include <stdint.h>
typedef struct { uint32_t offset, bytes, crc32; int16_t claw_region[4][2]; } rb_frame_meta_t;
typedef struct { unsigned count; uint16_t duration_ms; uint16_t frames[256], at[256]; } rb_clip_meta_t;
extern const rb_frame_meta_t rb_frames[];
extern const rb_clip_meta_t rb_clips[];
extern const uint8_t rb_clip_map[3][9];
extern const uint8_t rb_choreo_clip_map[55];
#define RB_FRAME_WIDTH 320u
#define RB_FRAME_HEIGHT 240u
#define RB_FRAME_X 0
#define RB_FRAME_Y 0
#define RB_FRAME_BYTES 153600u
#define RB_FRAME_COUNT @COUNT@u
'''
    (ROOT / 'main/generated/pet_animation_table.h').write_text(header.replace('@COUNT@', str(len(frames))))
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
    lines += ['};', 'const uint8_t rb_choreo_clip_map[55]={'+','.join(map(str,choreography_mapping))+'};']
    (ROOT / 'main/generated/pet_animation_table.c').write_text('\n'.join(lines) + '\n')
    print(f'Encoded {len(records)} current GIFs, {len(frames)} unique frames, {offset} bytes; Daily preserved; Dance follows trajectory-derived RGBA and variable frame delays')


if __name__ == '__main__':
    main()
