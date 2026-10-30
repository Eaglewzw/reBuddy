"""Regression: reviewed linkage holes show the room; white highlights survive."""
import hashlib
import importlib.util
import json
import struct
import zlib
from functools import lru_cache
from pathlib import Path
import numpy as np

root=Path(__file__).resolve().parents[2]
spec=importlib.util.spec_from_file_location('dance_assets',root/'tools/dance_assets.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)

# Two identical cream patches enclosed in dark material: color alone cannot
# distinguish a linkage opening from a painted highlight. Only one is reviewed.
pixels=np.full((32,48),0xffde,dtype=np.uint16)
pixels[4:28,4:44]=0x1082
pixels[10:18,10:18]=0xffde
pixels[10:18,30:38]=0xffde
cavity={'seed':[13,13],'bounds':[10,10,18,18],'area':64}
before=module.coverage(pixels)
after=module.coverage(pixels,[cavity])
assert np.all(before[10:18,10:18]==255)
assert np.all(after[10:18,10:18]==0)
assert np.all(after[10:18,30:38]==255)
assert np.all(after[4:9,4:44]==255)
for stale in ({**cavity,'area':63},{**cavity,'seed':[0,0]},
              {**cavity,'bounds':[9,10,18,18]}):
    try:module.coverage(pixels,[stale])
    except ValueError:pass
    else:raise AssertionError('Stale cavity silently accepted')

config_path=root/'assets/config/dance_cavity_seeds.json'
config=json.loads(config_path.read_text())
manifest=json.loads((root/'assets/animation/manifest.json').read_text())
blob=(root/'assets/animation/frames.bin').read_bytes()
assert manifest['dance_cavity_sha256']==hashlib.sha256(config_path.read_bytes()).hexdigest()
assert config['palette_sha256']==hashlib.sha256(bytes(manifest['dance_palette_rgb'])).hexdigest()

@lru_cache(None)
def planes(frame):
    meta=manifest['frames'][frame]
    record=blob[meta['offset']:meta['offset']+meta['bytes']]
    codec=struct.unpack_from('<I',record,12)[0]
    if codec==5:
        base=struct.unpack_from('<I',record,16)[0]
        assert base<frame
        return np.frombuffer(zlib.decompress(record[20:]),np.uint8)^planes(base)
    assert codec==4
    count=struct.unpack_from('<H',record,16)[0]
    return np.frombuffer(zlib.decompress(record[20+2*count:]),np.uint8)

checked=0
for clip in manifest['clips']:
    if clip['id'] not in config['clips']:continue
    assert clip['source_sha256']==config['source_sha256'][clip['id']]
    for tick,regions in config['clips'][clip['id']].items():
        alpha=planes(clip['frames'][int(tick)])[76800:].reshape(240,320)
        for region in regions:
            x,y=region['seed'];assert alpha[y,x]==0,(clip['id'],tick,'uncleared hole')
            x0,y0,x1,y1=region['bounds']
            assert np.count_nonzero(alpha[y0:y1,x0:x1]==0)>=region['area']
        for x,y in config['protected_highlight_seeds'][clip['id']][tick]:
            assert alpha[y,x]==255,(clip['id'],tick,'highlight erased')
        checked+=1
assert checked==25
print(f'PASS {checked} affected timeline poses, enclosed cavity coverage, protected highlights and stale-seed rejection')
