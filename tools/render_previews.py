#!/usr/bin/env python3
"""Convert the actual host LVGL RGB565 screenshots to reviewable PNGs."""
from pathlib import Path
import argparse
from PIL import Image, ImageDraw

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
