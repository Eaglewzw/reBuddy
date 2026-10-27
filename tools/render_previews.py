#!/usr/bin/env python3
"""Convert the actual host LVGL RGB565 screenshots to reviewable PNGs."""
from pathlib import Path
import argparse
from PIL import Image, ImageDraw, ImageFont

parser = argparse.ArgumentParser()
parser.add_argument('input', type=Path)
parser.add_argument('output', type=Path)
args = parser.parse_args()
args.output.mkdir(parents=True, exist_ok=True)
names = ['idle', 'curious', 'folded', 'relaxed', 'happy', 'settings', 'lazy', 'sleep', 'stopped']
sheet = Image.new('RGB', (960, 810), '#111b19')
draw = ImageDraw.Draw(sheet)
for n, name in enumerate(names):
    with Image.open(args.input / f'{name}.ppm') as source:
        image = source.convert('RGB')
    image.save(args.output / f'{name}.png')
    x, y = (n % 3) * 320, (n // 3) * 270
    sheet.paste(image, (x, y + 22))
    draw.text((x + 12, y + 5), name.upper(), fill='#d8f56c')
sheet.save(args.output / 'contact-sheet.png')
print(args.output / 'contact-sheet.png')
choreography = [('library', '作品列表'), ('editor', '动作编排'), ('params', '步骤参数'),
                ('performance', '动作库'), ('tempo', '屏幕节奏'), ('preview', '屏幕预览'),
                ('dance_demo', '基础律动'), ('slot10', '第十个作品')]
if all((args.input / f'choreo_{name}.ppm').exists() for name, _ in choreography):
    sheet = Image.new('RGB', (960, 840), '#efeee2')
    draw = ImageDraw.Draw(sheet)
    font = ImageFont.truetype('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc', 16, index=2)
    for n, (name, title) in enumerate(choreography):
        with Image.open(args.input / f'choreo_{name}.ppm') as source:
            frame = source.convert('RGB')
        frame.save(args.output / f'choreo_{name}.png')
        x, y = (n % 3) * 320, (n // 3) * 280
        sheet.paste(frame, (x, y + 32))
        draw.text((x + 12, y + 3), title, font=font, fill='#333b2d')
    sheet.save(args.output / 'choreography.png')
    print(args.output / 'choreography.png')
