"""Validate shipped animation data independently of Pillow and the generator."""
import hashlib
import json
from pathlib import Path
import re
import struct

root=Path(__file__).resolve().parents[2]
cfg=json.loads((root/'config/behavior_defaults.json').read_text())
manifest=json.loads((root/'assets/animation/manifest.json').read_text())
blob=(root/'assets/animation/frames.bin').read_bytes()
assert len(blob)==manifest['bytes']==manifest['unique_frames']*51212
for source in cfg['source_images']:
    data=(root/'assets/reference'/Path(source['path']).name).read_bytes()
    assert hashlib.sha256(data).hexdigest()==source['sha256']
base=None
for frame in manifest['frames']:
    pos=frame['offset']
    assert pos%51212==0 and 0<=pos<=len(blob)-51212
    assert struct.unpack_from('<BBHHHHH',blob,pos)==(0x19,0x12,0,160,160,320,0)
    pixels=blob[pos+12:pos+51212]
    assert hashlib.sha256(pixels).hexdigest()==frame['sha256']
    fixed=pixels[128*320:]
    if base is None: base=fixed
    assert fixed==base, 'base moves between body frames'
    assert len(frame['eyes'])==2
    for x,y in frame['eyes']: assert 10<=x<150 and 14<=y<146
for clip in manifest['clips']:
    assert len(clip['frames'])==len(clip['at_permille'])
    assert clip['at_permille'][0]==0
    assert clip['at_permille']==sorted(set(clip['at_permille']))
    assert all(0<=i<manifest['unique_frames'] for i in clip['frames'])
    if clip['action']: assert clip['at_permille'][-1]==1000
for action in (2,3):
    alternate=manifest['clips'][manifest['reverse_map'][action]]
    assert alternate['direction']==-1 and alternate['action']==action
    assert alternate['frames']!=manifest['clips'][manifest['clip_map'][0][action]]['frames']
bar=(root/'assets/animation/colorbars.bin').read_bytes()
assert len(bar)==51212
for index,expected in enumerate((0xf800,0x07e0,0x001f,0xffff,0)):
    for row in (0,159):
        assert struct.unpack_from('<H',bar,12+row*320+index*64)==(expected,)
header=(root/'components/rebuddy_core/include/rb_pet_config.h').read_text()
for k,v in cfg['gesture'].items():
    assert re.search(r'^#define RB_CFG_'+k.upper()+r' '+str(v)+r'u$',header,re.M)
for k,v in cfg['action_durations_ms'].items():
    assert re.search(r'^#define RB_MS_'+k+r' '+str(v)+r'u$',header,re.M)
print(f'PASS source hashes, {manifest["unique_frames"]} frame headers/hashes/fixed bases, clips/anchors, colorbar order and behavior config')
