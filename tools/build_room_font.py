#!/usr/bin/env python3
"""Generate Chinese/ASCII UI font subsets from the application's UI sources.
Noto Sans CJK is OFL-1.1; generated bitmaps stay local to the firmware.
"""
from pathlib import Path
import re
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[1]
source = '\n'.join(p.read_text() for p in sorted((ROOT/'main').glob('*.c')) if p.name != 'room_font.c')
source += '\n' + (ROOT/'components/rebuddy_core/src/rb_choreo.c').read_text()
chars = sorted(set(chr(n) for n in range(32, 127)) | set(re.findall(r'[\u3000-\u303f\u4e00-\u9fff\uff00-\uffef·倍]', source)))
lines = ['/* Generated Noto Sans CJK subset. See assets/fonts/OFL.txt. */', '#include "lvgl.h"']
for name, size in [('rb_room_font',16), ('rb_choreo_font',14), ('rb_choreo_small_font',12)]:
    font = ImageFont.truetype('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc', size, index=2)
    bitmap, glyphs = [], ['{0}']
    for char in chars:
        width = size+2 if ord(char)>127 else max(4, int(font.getlength(char)+1))
        width += width % 2  # packed 4-bit pixels
        height = size+4
        im = Image.new('L', (width,height))
        ImageDraw.Draw(im).text((1,-2),char,font=font,fill=255)
        data = list(im.getdata())
        offset = len(bitmap)
        bitmap.extend(((data[i]>>4)<<4)|(data[i+1]>>4) for i in range(0,len(data),2))
        glyphs.append('{.bitmap_index=%d,.adv_w=%d,.box_w=%d,.box_h=%d,.ofs_x=0,.ofs_y=0}' % (offset,width*16,width,height))
    start = ord(chars[0])
    lines += ['static const uint8_t '+name+'_bitmap[]={']
    lines += [','.join(map(str,bitmap[i:i+24]))+',' for i in range(0,len(bitmap),24)]
    lines += ['};', 'static const lv_font_fmt_txt_glyph_dsc_t '+name+'_glyphs[]={'+','.join(glyphs)+'};',
        'static const uint16_t '+name+'_unicode[]={'+','.join(str(ord(c)-start) for c in chars)+'};',
        'static const lv_font_fmt_txt_cmap_t '+name+'_cmaps[]={{.range_start=%d,.range_length=%d,.glyph_id_start=1,.unicode_list=%s_unicode,.list_length=%d,.type=LV_FONT_FMT_TXT_CMAP_SPARSE_TINY}};' % (start,ord(chars[-1])-start+1,name,len(chars)),
        'static const lv_font_fmt_txt_dsc_t '+name+'_dsc={.glyph_bitmap='+name+'_bitmap,.glyph_dsc='+name+'_glyphs,.cmaps='+name+'_cmaps,.cmap_num=1,.bpp=4,.bitmap_format=0};',
        'const lv_font_t '+name+'={.get_glyph_dsc=lv_font_get_glyph_dsc_fmt_txt,.get_glyph_bitmap=lv_font_get_bitmap_fmt_txt,.line_height=%d,.base_line=0,.dsc=&%s_dsc};' % (size+4,name)]
(ROOT/'main/room_font.c').write_text('\n'.join(lines)+'\n')
print('Generated %d glyphs in three UI sizes' % len(chars))
