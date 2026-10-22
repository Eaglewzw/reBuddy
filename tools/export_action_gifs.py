#!/usr/bin/env python3
"""Export one review GIF per action, without changing firmware resources.

Decode current room-composited RGB565 poses, including the sleep loop. Code-native question marks and floating Zs follow the runtime timing.
"""
import argparse
import hashlib
import json
from pathlib import Path
import struct
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[1]
BG = (248, 247, 239)
parser = argparse.ArgumentParser()
parser.add_argument('--output', type=Path, default=ROOT/'build/action-gifs')
parser.add_argument('--font', type=Path, default=Path('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'))
parser.add_argument('--style', choices=('lively', 'lazy'), default='lively')
args = parser.parse_args()
config = json.loads((ROOT/'config/behavior_defaults.json').read_text())
playback_scale = config['animation_playback']['duration_scale']
action_scale = playback_scale * config['style_duration_scale'][args.style]
sleep_scale = playback_scale * (1.3 if args.style == 'lazy' else 1.0)
if args.output.exists() and any(args.output.iterdir()):
    raise SystemExit('Output must be empty; use a new directory to preserve reviewed GIFs.')
args.output.mkdir(parents=True, exist_ok=True)
manifest = json.loads((ROOT / 'assets/animation/manifest.json').read_text())
blob = (ROOT / 'assets/animation/frames.bin').read_bytes()
poses = []
for frame in manifest['frames']:
    raw = blob[frame['offset'] + 12:frame['offset'] + 51212]
    rgb = bytearray()
    for (p,) in struct.iter_unpack('<H', raw):
        rgb.extend((((p >> 11) & 31) * 255 // 31, ((p >> 5) & 63) * 255 // 63, (p & 31) * 255 // 31))
    poses.append(Image.frombytes('RGB', (160, 160), bytes(rgb)))

room_raw = (ROOT/'assets/animation/room.bin').read_bytes()[12:]
room_rgb = bytearray()
for (p,) in struct.iter_unpack('<H', room_raw):
    room_rgb.extend((((p >> 11) & 31) * 255 // 31, ((p >> 5) & 63) * 255 // 63, (p & 31) * 255 // 31))
room = Image.frombytes('RGB', (320, 240), bytes(room_rgb))

def stage(body):
    im = room.copy()
    im.paste(body, (80, 40))
    return im

def clip_pose(action, phase, origin=0):
    clip = manifest['clips'][manifest['clip_map'][origin][action]]
    i = 0
    while i + 1 < len(clip['frames']) and clip['at_permille'][i+1] <= phase:
        i += 1
    return poses[clip['frames'][i]]

def breathe_pose(ms):
    # 1.6 seconds inhale, 0.5 seconds peak, 2.1 seconds exhale/settle.
    keys = [(0, 0), (500, 1), (1000, 2), (1600, 3), (2100, 2), (2800, 1), (3500, 0)]
    return poses[24 + next(i for t, i in reversed(keys) if ms % 4200 >= t)]

fonts = {size: ImageFont.truetype(str(args.font), size) for size in range(8, 25)}
def sleep_frame(ms):
    ms /= sleep_scale  # Authored sleep timeline stays in its original 4200 ms units.
    im = stage(breathe_pose(ms)).convert('RGBA')
    overlay = Image.new('RGBA', im.size)
    draw = ImageDraw.Draw(overlay)
    for i in range(3):
        age = (ms - i * 1400) % 4200
        if age >= 2800: continue
        p = age / 2800
        size = round(24 - 16 * p)
        opacity = round(230 * min(1, age / 120) * (1 - p) ** .8)
        draw.text((226 + round(33 * p), 110 - round(59 * p)), 'Z',
                  font=fonts[size], fill=(105, 133, 56, opacity), anchor='lt')
    return Image.alpha_composite(im, overlay).convert('RGB')

def action_frame(action, phase, origin=0):
    im = stage(clip_pose(action, phase, origin))
    if action == 3:
        draw = ImageDraw.Draw(im)
        for q in manifest['question_effect']:
            if q['start'] <= phase < q['end']:
                y = q['y'] - 4 * (phase-q['start']) // (q['end']-q['start'])
                draw.text((q['x'], y), '?', font=fonts[q['size']],
                          fill=(105, 133, 56), anchor='lt')
    return im

records = []
qa = ROOT / 'build/gif-preview'
qa.mkdir(parents=True, exist_ok=True)
def save(name, frames, durations, loop, description):
    # One palette across a clip prevents frame-to-frame palette flicker.
    palette_source = Image.new('RGB', (320 * 4, 240 * 4), BG)
    for i in range(16):
        frame = frames[min(len(frames)-1, i * len(frames) // 16)]
        palette_source.paste(frame, (i % 4 * 320, i // 4 * 240))
    palette = palette_source.quantize(colors=256)
    encoded = [f.quantize(palette=palette, dither=Image.NONE) for f in frames]
    filename = name + '.gif'
    destination = args.output / filename
    options = {'save_all': True, 'append_images': encoded[1:], 'duration': durations,
               'disposal': 2, 'optimize': False}
    if loop: options['loop'] = 0
    encoded[0].save(destination, **options)
    with Image.open(destination) as gif:
        actual_ms = 0
        for i in range(gif.n_frames):
            gif.seek(i); actual_ms += gif.info['duration']
        assert actual_ms == sum(durations), (name, actual_ms, sum(durations))
        assert gif.size == (320, 240)
        assert ('loop' in gif.info) == loop
        count = gif.n_frames
    records.append({'file': filename, 'duration_ms': actual_ms, 'encoded_frames': count,
                    'loop': loop, 'description': description,
                    'sha256': hashlib.sha256(destination.read_bytes()).hexdigest()})
    frames[len(frames)//2].save(qa / (name + '.png'))

idle = [stage(poses[23 if 2400 <= t < 2550 else 0]) for t in range(0, 4000, 50)]
save('01_待机眨眼', idle, [50] * len(idle), True, '待机与短眨眼；循环。')
specs = [(1, '02_抬头看你', 0, True), (2, '03_轻蹭', 0, True),
         (3, '04_疑惑歪头', 0, True), (4, '05_害羞缩回', 0, False),
         (5, '06_安抚展开', 1, False), (6, '07_开心回应', 0, True),
         (7, '08_慢慢入睡', 0, False), (8, '10_迷糊唤醒', 2, False)]
action_names = ['', 'PROBE', 'NUZZLE_CYCLE', 'TILT', 'WITHDRAW', 'RECOVER', 'HAPPY', 'REST', 'WAKE']
for action, name, origin, loop in specs:
    duration = round(config['action_durations_ms'][action_names[action]] * action_scale)
    frames = [action_frame(action, t * 1000 // duration, origin) for t in range(0, duration, 50)]
    # Hold the actual endpoint before replaying. For one-way state changes,
    # the return to the first frame is review playback, not an authored action.
    frames.append(action_frame(action, 1000, origin))
    save(name, frames, [min(50, duration-t) for t in range(0, duration, 50)] + [1000 if loop else 1800], True,
         f'{args.style} 实机动作 {duration} ms；结束停留后' + ('循环。' if loop else '自动重播，跳回起始姿态仅用于预览。'))

sleep_duration = round(4200 * sleep_scale)
sleep_frames = [sleep_frame(t) for t in range(0, sleep_duration, 50)]
save('09_熟睡呼吸与Z', sleep_frames, [min(50, sleep_duration-t) for t in range(0, sleep_duration, 50)], True,
     f'{sleep_duration} ms 呼吸循环；Z 出现后从大到小上浮淡出。')
for i, t in enumerate((0, 700, 1400, 1750, 2450, 3500)):
    sleep_frame(t*sleep_scale).save(qa / f'sleep-phase-{i}.png')
sheet_out = Image.new('RGB', (960, 480), BG)
for i, t in enumerate((0, 700, 1400, 1750, 2450, 3500)):
    sheet_out.paste(sleep_frame(t*sleep_scale), (i % 3 * 320, i // 3 * 240))
sheet_out.save(qa / 'sleep-phases.png')
report = {'style': args.style, 'duration_scale': playback_scale,
          'behavior_config_sha256': hashlib.sha256((ROOT/'config/behavior_defaults.json').read_bytes()).hexdigest(),
          'status': 'current_room_action_preview', 'canvas': [320, 240],
          'role_size': [160, 160], 'source_runtime_sha256': hashlib.sha256(blob).hexdigest(),
          'room_sha256': hashlib.sha256((ROOT/'assets/animation/room.bin').read_bytes()).hexdigest(),
          'limitations': ['Sparse authored poses; GIFs do not include touch controls.',
                          'One-way state transitions restart only for review playback.',
                          'Question mark and Z typography approximate LVGL with a desktop font.'],
          'gifs': sorted(records, key=lambda r: r['file'])}
(args.output/'manifest.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
print(f'Exported {len(records)} GIFs to {args.output}')
