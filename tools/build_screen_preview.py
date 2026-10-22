#!/usr/bin/env python3
"""Slice the authored atlas, register its ground line and encode RGB565.

Actions use authored atlas cells. PROBE uses the GitHub poses and timing.
"""
import hashlib
import json
from pathlib import Path
import struct
from collections import deque
from PIL import Image
from sleep_assets import load_sleep_poses
from cavity_background import clear_cavity

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / 'assets/design/screen-preview-v1/pose-atlas.png'
OUT = ROOT / 'assets/animation'
BG = (248, 247, 239)
SIZE = 160
GROUND_STRIP = 148
atlas = Image.open(SOURCE).convert('RGB')
assert atlas.size == (1536, 1024), 'Expected exactly 6 × 4 square cells'
names = ['idle', 'attention', 'probe_25', 'probe_50', 'probe_75', 'probe_full',
         'nuzzle_left_50', 'nuzzle_left', 'smile_center', 'nuzzle_right_50',
         'nuzzle_right', 'tilt_left', 'tilt_right', 'startle', 'fold_half',
         'folded', 'drowsy', 'sleep', 'wake_eyes', 'happy_up', 'happy_open',
         'happy_close', 'recover_half', 'blink']
def register_pose(cell):
    cell = cell.resize((SIZE, SIZE), getattr(Image, 'Resampling', Image).LANCZOS)
    # Background-only normalization; no painting or deformation of the character.
    pixels = list(cell.getdata())
    queue = deque([(xx, yy) for xx in range(SIZE) for yy in (0, SIZE - 1)] +
                  [(xx, yy) for yy in range(SIZE) for xx in (0, SIZE - 1)])
    visited = bytearray(SIZE * SIZE)
    while queue:
        xx, yy = queue.popleft()
        if not (0 <= xx < SIZE and 0 <= yy < SIZE):
            continue
        index = yy * SIZE + xx
        if visited[index]:
            continue
        visited[index] = 1
        pixel = pixels[index]
        if min(pixel) > 205 and max(pixel) - min(pixel) < 35:
            pixels[index] = BG
            queue.extend(((xx - 1, yy), (xx + 1, yy), (xx, yy - 1), (xx, yy + 1)))
    cell.putdata(pixels)
    occupied = [(xx, yy) for yy in range(120, SIZE) for xx in range(SIZE)
                if min(cell.getpixel((xx, yy))) < 150]
    bottom = max(yy for _, yy in occupied)
    foot = [xx for xx, yy in occupied if yy >= bottom - 8]
    center = (min(foot) + max(foot)) // 2
    dx, dy = 80 - center, 151 - bottom
    # Only integer registration and one common scale, never per-pose scaling.
    canvas = Image.new('RGB', (SIZE, SIZE), BG)
    canvas.paste(cell, (dx, dy))
    return canvas, [dx, dy]

images, registration = [], []
for n, name in enumerate(names):
    x, y = n % 6 * 256, n // 6 * 256
    canvas, offset = register_pose(atlas.crop((x, y, x + 256, y + 256)))
    images.append(canvas)
    registration.append({'name': name, 'cell': n, 'offset_px': offset})

# Append the four approved-for-preview sleep poses without altering other actions.
sleep_source = ROOT / 'assets/design/sleep-v2/breath-atlas.png'
sleep_images, sleep_registration = load_sleep_poses(sleep_source)
images.extend(sleep_images)
names.extend(['sleep_exhale', 'sleep_inhale_1', 'sleep_inhale_2', 'sleep_inhale_peak'])
registration.extend({'name': names[24+i], 'cell': i, 'offset_px': offset,
                     'source': str(sleep_source.relative_to(ROOT))}
                    for i, offset in enumerate(sleep_registration))

# Enclosed gaps are backdrop too. Explicit per-pose seeds avoid removing white
# eye/metal highlights; normalize before rotating so backdrop never moves.
cavity_recipe = ROOT / 'assets/design/screen-preview-v1/cavity-background.json'
cavity_config = json.loads(cavity_recipe.read_text())
for name, region in cavity_config['poses'].items():
    clear_cavity(images[names.index(name)], region,
                 cavity_config['min_channel'], cavity_config['max_channel_spread'])

# The ground-contact rim is a common static plate. Upper base areas are allowed
# to be occluded by the folded head/claw, so they are not blindly overwritten.
plate = images[0].crop((0, GROUND_STRIP, SIZE, SIZE))
for im in images:
    im.paste(plate, (0, GROUND_STRIP))

# Composite the same room crop behind every pose; no runtime alpha buffer or
# extra frame-cache allocation is needed. Background edges match exactly.
room_source=ROOT/'assets/design/room-v1/room.png'
room=Image.open(room_source).convert('RGB').resize((320,240),getattr(Image,'Resampling',Image).LANCZOS)
room_crop=room.crop((80,40,240,200))
for n, im in enumerate(images):
    mask=Image.new('L',im.size)
    mask.putdata([0 if p==BG else 255 for p in im.getdata()])
    composed=room_crop.copy(); composed.paste(im,(0,0),mask)
    images[n]=composed
room_pixels=b''.join(struct.pack('<H',((r>>3)<<11)|((g>>2)<<5)|(b>>3)) for r,g,b in room.getdata())
(OUT/'room.bin').write_bytes(struct.pack('<BBHHHHH',0x19,0x12,0,320,240,640,0)+room_pixels)

claw_regions=json.loads((ROOT/'assets/config/claw_touch_regions.json').read_text())['frames']
frames, blobs = [], []
for n, im in enumerate(images):
    data = b''.join(struct.pack('<H', ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3))
                    for r, g, b in im.getdata())
    frames.append({'offset': n * 51212,
                   'claw_region': claw_regions[names[n]], 'face_mode': 'baked', 'name': names[n],
                   'sha256': hashlib.sha256(data).hexdigest()})
    blobs.append(struct.pack('<BBHHHHH', 0x19, 0x12, 0, SIZE, SIZE, SIZE * 2, 0) + data)

clips, mapping, reverse = [], [[0] * 9 for _ in range(3)], [0] * 9
def clip(origin, action, points, direction=1):
    index = len(clips)
    record = {'origin': origin, 'action': action, 'frames': [f for _, f in points],
              'at_permille': [t for t, _ in points], 'direction': direction,
              'loop': action == 2, 'switch_points': [0, 1000],
              'end_pose': 1 if action == 4 else 2 if action == 7 else origin if action == 0 else 0}
    clips.append(record)
    if direction < 0:
        reverse[action] = index
    else:
        mapping[origin][action] = index

for origin, frame in enumerate((0, 15, 24)):
    clip(origin, 0, [(0, frame)])
# PROBE restored from GitHub commit b4a6a88.
clip(0, 1, [(0, 0), (100, 1), (250, 2), (720, 2), (850, 1), (1000, 0)])
# A single directional nuzzle per cycle avoids a mechanical left-right shake.
clip(0, 2, [(0, 0), (100, 8), (220, 9), (380, 10), (650, 9), (820, 8), (1000, 0)])
clip(0, 2, [(0, 0), (100, 8), (220, 6), (380, 7), (650, 6), (820, 8), (1000, 0)], -1)
clip(0, 3, [(0, 0), (100, 1), (250, 12), (650, 12), (850, 1), (1000, 0)])
clip(0, 3, [(0, 0), (100, 1), (250, 11), (650, 11), (850, 1), (1000, 0)], -1)
clip(0, 4, [(0, 0), (60, 13), (135, 13), (360, 14), (720, 15), (1000, 15)])
clip(1, 5, [(0, 15), (156, 15), (300, 22), (470, 14), (625, 0),
            (720, 6), (812, 8), (910, 8), (1000, 0)])
# Reuse the earlier artwork: lift → open/close twice like clapping → lower.
# Contact returns to the raised closed-claw pose (19), not the lowered pose (21).
clip(0, 6, [(0, 0), (80, 8), (180, 19), (300, 20), (430, 19),
            (550, 20), (680, 19), (790, 21), (890, 8), (1000, 0)])
clip(0, 7, [(0, 0), (160, 23), (267, 14), (440, 15), (580, 16), (733, 24), (1000, 24)])
clip(1, 7, [(0, 15), (267, 16), (550, 16), (733, 24), (1000, 24)])
clip(2, 8, [(0, 24), (156, 18), (300, 16), (440, 15), (570, 22), (750, 0), (1000, 0)])
# Existing engine retraces interrupted REST to its origin before WAKE.
clip(1, 8, [(0, 15), (156, 22), (440, 14), (750, 0), (1000, 0)])
clip(0, 8, [(0, 0), (156, 1), (750, 0), (1000, 0)])
# Defensive normal-origin RECOVER mapping; normal state does not request it.
mapping[0][5] = mapping[0][0]
assert all(len(c['frames']) <= 16 for c in clips)
OUT.mkdir(exist_ok=True)
(OUT / 'frames.bin').write_bytes(b''.join(blobs))
questions = [dict(x=151, y=36, size=18, start=280, end=850),
             dict(x=183, y=23, size=24, start=360, end=900),
             dict(x=216, y=37, size=18, start=440, end=950)]
manifest = {'interaction_revision': 'interaction-v2', 'probe_revision': 'github-original', 'probe_source_commit': 'b4a6a885cbf1fa7630145321f0afac9c74c5f2b3',
            'cavity_recipe':str(cavity_recipe.relative_to(ROOT)),
            'cavity_recipe_sha256':hashlib.sha256(cavity_recipe.read_bytes()).hexdigest(),
            'happy_revision': 'original-art-two-claps',
            'question_effect': questions, 'format': 'LVGL9 RGB565 binary images, 12-byte header per frame', 'size': [160, 160],
            'asset_version': 'room-preview-v1', 'visual_status': 'key_pose_preview_not_final_animation',
            'room_source': str(room_source.relative_to(ROOT)), 'room_source_sha256':hashlib.sha256(room_source.read_bytes()).hexdigest(),
            'room_bytes':len(room_pixels)+12,'room_sha256':hashlib.sha256((OUT/'room.bin').read_bytes()).hexdigest(),
            'face_mode': 'baked', 'derivation': 'Authored cells with fixed registration. PROBE restores GitHub source poses and timing; enclosed backdrop is composited with the room.',
            'source': str(SOURCE.relative_to(ROOT)), 'source_sha256': hashlib.sha256(SOURCE.read_bytes()).hexdigest(),
            'unique_frames': len(frames), 'bytes': sum(map(len, blobs)), 'ground_lock_y': GROUND_STRIP,
            'sleep_source': str(sleep_source.relative_to(ROOT)), 'sleep_source_sha256': hashlib.sha256(sleep_source.read_bytes()).hexdigest(),
            'sleep_loop': {'frames': [24,25,26,27,26,25,24], 'at_ms': [0,500,1000,1600,2100,2800,3500], 'period_ms': 4200, 'ground_lock_y':139},
            'registration': registration, 'frames': frames, 'clips': clips, 'clip_map': mapping, 'reverse_map': reverse,
            'limitations': ['Sparse key poses, not 8–12 unique poses per action',
                           'Upper base paint and limb geometry can drift between generated cells',
                           'Baked gaze uses one attention pose; not continuous eye tracking',
                           'Interruptions use the existing reverse-to-origin behavior, not new bespoke bridges']}
(OUT / 'manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + '\n')
header = '''/* Generated by tools/build_screen_preview.py. */
#pragma once
#include <stdint.h>
typedef struct { uint32_t offset; int16_t claw_region[4][2]; } rb_frame_meta_t;
typedef struct { unsigned count; uint16_t frames[16], at[16]; } rb_clip_meta_t;
typedef struct { int16_t x, y; uint16_t size, start, end; } rb_question_meta_t;
#define RB_QUESTION_COUNT 3u
extern const rb_question_meta_t rb_questions[RB_QUESTION_COUNT];
extern const rb_frame_meta_t rb_frames[];
extern const rb_clip_meta_t rb_clips[];
extern const uint8_t rb_clip_map[3][9];
extern const uint8_t rb_clip_reverse[9];
#define RB_FRAME_IDLE_BLINK 23u
#define RB_FRAME_IDLE_ATTENTION 1u
#define RB_FRAME_COUNT @FRAME_COUNT@
#define RB_SLEEP_FRAME_FIRST 24u
#define RB_SLEEP_PERIOD_MS 4200u
#define RB_FRAME_BYTES 51212u
'''
(ROOT / 'main/pet_animation_table.h').write_text(header.replace('@FRAME_COUNT@', str(len(frames))))
lines = ['/* Generated by tools/build_screen_preview.py. */', '#include "pet_animation_table.h"',
         'const rb_frame_meta_t rb_frames[]={']
for f in frames:
    polygon=','.join('{%d,%d}' % tuple(point) for point in f['claw_region'])
    lines.append('{%d,{%s}},' % (f['offset'],polygon))
lines += ['};', 'const rb_question_meta_t rb_questions[RB_QUESTION_COUNT]={']
for q in questions:
    lines.append('{%d,%d,%d,%d,%d},' % tuple(q[k] for k in ('x','y','size','start','end')))
lines += ['};', 'const rb_clip_meta_t rb_clips[]={']
for c in clips:
    lines.append('{%d,{%s},{%s}},' % (len(c['frames']), ','.join(map(str, c['frames'])), ','.join(map(str, c['at_permille']))))
lines += ['};', 'const uint8_t rb_clip_map[3][9]={']
lines += ['{' + ','.join(map(str, row)) + '},' for row in mapping]
lines += ['};', 'const uint8_t rb_clip_reverse[9]={' + ','.join(map(str, reverse)) + '};']
(ROOT / 'main/pet_animation_table.c').write_text('\n'.join(lines) + '\n')
preview = ROOT / 'build/screen-preview-assets'
preview.mkdir(parents=True, exist_ok=True)
for name, im in zip(names, images):
    im.save(preview / (name + '.png'))
print(f'Exported {len(frames)} authored poses, {len(clips)} clips, {manifest["bytes"]} bytes')

# RGB565 calibration bars, also checked by the host resource test.
colors=(0xf800,0x07e0,0x001f,0xffff,0)
bar=b''.join(struct.pack('<H',colors[x//32]) for y in range(160) for x in range(160))
(OUT/'colorbars.bin').write_bytes(struct.pack('<BBHHHHH',0x19,0x12,0,160,160,320,0)+bar)
