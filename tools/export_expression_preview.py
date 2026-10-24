#!/usr/bin/env python3
"""Export three separate looping expression GIFs from selected atlas poses."""
import json
from PIL import Image, ImageDraw, ImageFont
from export_gifs_v2 import BG, ROOT, RESAMPLE, prepare, sha, cavity_seeds_for, check_cavity_source

DESIGN = ROOT / 'assets/design/three-joint-expressions'
OUTPUT = ROOT / 'assets/gifs-v2/expressions-preview'


def main():
    cfg = json.loads((DESIGN / 'timing.json').read_text())
    protected = [ROOT / p for p in ('assets/animation/frames.bin', 'assets/animation/manifest.json',
                                     'main/pet_animation_table.c', 'main/pet_animation_table.h')]
    before = {str(p.relative_to(ROOT)): sha(p) for p in protected}
    source = DESIGN / 'atlas.png'
    check_cavity_source('three-joint-expressions', source)
    atlas = Image.open(source).convert('RGB')
    assert atlas.width == atlas.height
    # All cells receive this ONE fixed sampling ratio. No per-pose sizing.
    sheet = atlas.resize((640, 640), RESAMPLE.LANCZOS)
    poses, registration = [], []
    for i in range(16):
        cell = sheet.crop((i%4*160, i//4*160, i%4*160+160, i//4*160+160))
        pose, record = prepare(cell, True, cavity_seeds_for('three-joint-expressions', i))
        poses.append(pose)
        registration.append(record)
    room = Image.open(ROOT / 'assets/design/room-v1/room.png').convert('RGB').resize((320, 240), RESAMPLE.LANCZOS)
    font = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf', 18)
    OUTPUT.mkdir(parents=True, exist_ok=True)
    legacy = OUTPUT / '三关节_待机_开心_好奇.gif'
    if legacy.exists():
        archive = ROOT / 'build/expressions-combined-previous'
        archive.mkdir(parents=True, exist_ok=True)
        legacy.replace(archive / legacy.name)
    contact = Image.new('RGB', (960, 260), 'white')
    records = []
    for col, clip in enumerate(cfg['clips']):
        timeline = [i for i, ticks in clip['keys'] for _ in range(ticks)]
        # Wider separated-finger poses 5 and 9–11 are intentionally excluded.
        assert set(timeline) <= {0, 1, 3, 4, 8, 12}
        frames = []
        for i in timeline:
            pose = poses[i]
            mask = Image.new('L', pose.size)
            mask.putdata([0 if pixel == BG else 255 for pixel in pose.getdata()])
            frame = room.copy()
            frame.paste(pose, (80, 40), mask)
            if clip['id'] == 'CURIOUS' and i != 0:
                ImageDraw.Draw(frame).text((240, 42), '?', font=font, fill=(115, 145, 35))
            frames.append(frame)
        durations = [((i+1)*125//10)*10 - (i*125//10)*10 for i in range(len(frames))]
        palette_source = Image.new('RGB', (1280, 960))
        for i in range(16):
            palette_source.paste(frames[i*len(frames)//16], (i%4*320, i//4*240))
        palette = palette_source.quantize(colors=256)
        encoded = [frame.quantize(palette=palette, dither=Image.NONE) for frame in frames]
        output = OUTPUT / clip['file']
        encoded[0].save(output, save_all=True, append_images=encoded[1:], duration=durations,
                        loop=0, disposal=2, optimize=False)
        actual = []
        with Image.open(output) as gif:
            assert gif.size == (320, 240) and gif.info['loop'] == 0
            first = gif.convert('RGB').tobytes()
            for i in range(gif.n_frames):
                gif.seek(i)
                actual.append(gif.info['duration'])
            assert first == gif.convert('RGB').tobytes()
        assert len(actual) == len(timeline) and sum(actual) == clip['duration_ms']
        assert set(actual) == {120, 130}
        records.append({**clip, 'encoded_frames': len(actual), 'delays_ms': actual,
                        'sha256': sha(output), 'loop_endpoints_identical': True})
        contact.paste(frames[timeline.index(clip['key_pose'])], (col*320, 20))
        ImageDraw.Draw(contact).text((col*320+8, 4), clip['id'], fill='black')
        print(f"PASS {clip['file']}: {len(actual)} frames, {sum(actual)} ms, 8 FPS, identical loop ends")
    assert len(records) == 3
    assert {p.name for p in OUTPUT.glob('*.gif')} == {c['file'] for c in records}
    assert before == {str(p.relative_to(ROOT)): sha(p) for p in protected}
    report = {'fps': cfg['fps'], 'canvas': [320, 240], 'clips': records,
              'source': str(source.relative_to(ROOT)), 'source_sha256': sha(source),
              'source_dimensions': list(atlas.size), 'fixed_sampling_ratio': 640/atlas.width,
              'registration': registration,
              'joint_review': 'Selected poses visually reviewed: three silver joint caps and one closed two-finger gripper. Wider separated-finger poses excluded.',
              'scale_policy': 'One fixed atlas sample; integer base registration; no dynamic zoom, warping or tweening.',
              'status': 'three_separate_previews_not_integrated_or_hardware_validated'}
    (OUTPUT / 'manifest.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
    contact.save(DESIGN / 'keyframes.png')


if __name__ == '__main__':
    main()
