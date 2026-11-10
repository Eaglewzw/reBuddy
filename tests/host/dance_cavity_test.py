"""True alpha clears enclosed holes while opaque white highlights stay painted."""
import importlib.util
import json
import struct
import zlib
from functools import lru_cache
from pathlib import Path
import numpy as np
from PIL import Image
root=Path(__file__).resolve().parents[2]
spec=importlib.util.spec_from_file_location('dance_assets',root/'tools/dance_assets.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)

# Two identical white RGB patches: renderer alpha distinguishes the enclosed
# opening from an opaque metal highlight, independent of the palette or color.
pixels=np.full((32,48,4),(32,40,32,255),dtype=np.uint8)
pixels[10:18,10:18]=(255,255,255,0)
pixels[10:18,30:38]=(255,255,255,255)
palette=[255,255,255,32,40,32]+[0]*762
planes,_=module.indexed_rgba_frame(Image.fromarray(pixels,'RGBA'),palette)
alpha=np.frombuffer(planes[len(planes)//2:],np.uint8).reshape(32,48)
assert np.all(alpha[10:18,10:18]==0)
assert np.all(alpha[10:18,30:38]==255)

source=json.loads((root/'assets/gifs/dance/manifest.json').read_text())
manifest=json.loads((root/'assets/animation/manifest.json').read_text())
blob=(root/'assets/animation/frames.bin').read_bytes()
assert 'dance_cavity_sha256' not in manifest
assert manifest['dance_alpha_source'].startswith('renderer_rgba')
@lru_cache(maxsize=32)
def planes(frame):
    meta=manifest['frames'][frame];record=blob[meta['offset']:meta['offset']+meta['bytes']]
    codec=struct.unpack_from('<I',record,12)[0]
    if codec==7:
        base=struct.unpack_from('<I',record,16)[0];assert base<frame
        return np.frombuffer(zlib.decompress(record[20:]),np.uint8)^planes(base)
    assert codec==6
    count=struct.unpack_from('<H',record,16)[0]
    return np.frombuffer(zlib.decompress(record[24+2*count:]),np.uint8)
by_id={c['id']:c for c in manifest['clips']};checked=0
for clip in source['clips']:
    runtime=by_id[clip['id']]
    for tick in {0,len(runtime['frames'])//2,len(runtime['frames'])-1}:
        row=clip['rgba_frames'][runtime['source_ticks'][tick]]
        with Image.open(root/'assets/gifs/dance'/row['file']) as im:
            size=manifest['dance_render_size']
            expected=np.asarray(im.convert('RGBA').resize((size,size),Image.Resampling.LANCZOS).getchannel('A'),dtype=float)
        step=255/(manifest['dance_alpha_levels']-1)
        expected=np.rint(np.rint(expected/step)*step).astype(np.uint8)
        actual=planes(runtime['frames'][tick])[size*size:].reshape(size,size)
        assert np.array_equal(actual,expected),(clip['id'],tick)
        checked+=1
print(f'PASS transparent cavities, opaque white highlights and {checked} authored alpha samples')
