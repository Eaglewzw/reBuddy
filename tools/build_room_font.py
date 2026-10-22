#!/usr/bin/env python3
"""Export the handful of Chinese menu glyphs; Noto Sans CJK is OFL-1.1."""
from pathlib import Path
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[1]
chars = sorted(set('活泼慵懒休息唤醒收起设置长按左上恢复点击继续请释放急停下角'))
font = ImageFont.truetype('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc', 16, index=2)
bitmap, glyphs = [], ['{0}']
for char in chars:
    im = Image.new('L', (18, 20))
    ImageDraw.Draw(im).text((1, -2), char, font=font, fill=255)
    data = list(im.getdata())
    offset = len(bitmap)
    bitmap.extend(((data[i] >> 4) << 4) | (data[i+1] >> 4) for i in range(0, len(data), 2))
    glyphs.append('{.bitmap_index=%d,.adv_w=288,.box_w=18,.box_h=20,.ofs_x=0,.ofs_y=0}' % offset)
start = ord(chars[0])
lines = ['/* Generated Noto Sans CJK subset. See assets/fonts/OFL.txt. */', '#include "lvgl.h"',
         'static const uint8_t bitmap[]={']
lines += [','.join(map(str, bitmap[i:i+24]))+',' for i in range(0,len(bitmap),24)]
lines += ['};', 'static const lv_font_fmt_txt_glyph_dsc_t glyphs[]={'+','.join(glyphs)+'};',
          'static const uint16_t unicode_list[]={'+','.join(str(ord(c)-start) for c in chars)+'};',
          'static const lv_font_fmt_txt_cmap_t cmaps[]={{.range_start=%d,.range_length=%d,.glyph_id_start=1,.unicode_list=unicode_list,.list_length=%d,.type=LV_FONT_FMT_TXT_CMAP_SPARSE_TINY}};' % (start,ord(chars[-1])-start+1,len(chars)),
          'static const lv_font_fmt_txt_dsc_t dsc={.glyph_bitmap=bitmap,.glyph_dsc=glyphs,.cmaps=cmaps,.cmap_num=1,.bpp=4,.bitmap_format=0};',
          'const lv_font_t rb_room_font={.get_glyph_dsc=lv_font_get_glyph_dsc_fmt_txt,.get_glyph_bitmap=lv_font_get_bitmap_fmt_txt,.line_height=20,.base_line=0,.dsc=&dsc};']
(ROOT/'main/room_font.c').write_text('\n'.join(lines)+'\n')
