"""Validate shipped animation data independently of Pillow and the generator."""
import hashlib
import json
from pathlib import Path
import re
import struct

root=Path(__file__).resolve().parents[2]
cfg=json.loads((root/'assets/config/behavior_defaults.json').read_text())
manifest=json.loads((root/'assets/animation/manifest.json').read_text())
claw_regions=json.loads((root/'assets/config/claw_touch_regions.json').read_text())['frames']
blob=(root/'assets/animation/frames.bin').read_bytes()
neutral_sources={int(target):int(source) for source,target in manifest.get('rest_neutral_map',{}).items()}
assert len(blob)==manifest['bytes']==manifest['unique_frames']*51212
for source in cfg['source_images']:
    data=(root/source['path']).read_bytes()
    assert hashlib.sha256(data).hexdigest()==source['sha256']
base=None
ground_y=128
if manifest.get('asset_version') in ('screen-preview-v1','screen-preview-v2','room-preview-v1'):
    # Folded claws can occlude the upper base. Only the grounded rim is a
    # shared plate; this does not certify identical generated limb geometry.
    assert manifest['ground_lock_y']==148
    ground_y=148
    assert manifest['face_mode']=='baked'
    assert manifest['visual_status']=='key_pose_preview_not_final_animation'
    atlas=(root/manifest['source']).read_bytes()
    assert hashlib.sha256(atlas).hexdigest()==manifest['source_sha256']
    expected_count=(28 if manifest['asset_version'] in ('screen-preview-v2','room-preview-v1') else 24)+len(neutral_sources)
    assert manifest['unique_frames']==expected_count
    assert len({f['sha256'] for f in manifest['frames']})==expected_count
for frame in manifest['frames']:
    polygon=frame['claw_region']
    assert polygon==claw_regions[frame['name']] and len(polygon)==4
    assert all(0<=x<160 and 0<=y<160 for x,y in polygon)
    turns=[]
    for i in range(4):
        a,b,c=polygon[i],polygon[(i+1)%4],polygon[(i+2)%4]
        turns.append((b[0]-a[0])*(c[1]-b[1])-(b[1]-a[1])*(c[0]-b[0]))
    assert all(t>0 for t in turns) or all(t<0 for t in turns), 'claw region must be convex'
    pos=frame['offset']
    assert pos%51212==0 and 0<=pos<=len(blob)-51212
    assert struct.unpack_from('<BBHHHHH',blob,pos)==(0x19,0x12,0,160,160,320,0)
    pixels=blob[pos+12:pos+51212]
    assert hashlib.sha256(pixels).hexdigest()==frame['sha256']
    fixed=pixels[ground_y*320:]
    if base is None: base=fixed
    assert fixed==base, 'ground-contact plate moves between body frames'
for clip in manifest['clips']:
    assert len(clip['frames'])==len(clip['at_permille'])
    assert clip['at_permille'][0]==0
    assert clip['at_permille']==sorted(set(clip['at_permille']))
    assert all(0<=i<manifest['unique_frames'] for i in clip['frames'])
    if clip['action']: assert clip['at_permille'][-1]==1000
    if manifest.get('asset_version') in ('screen-preview-v1','screen-preview-v2','room-preview-v1'):
        assert len(clip['frames'])<=16
        first=neutral_sources.get(clip['frames'][0],clip['frames'][0])
        assert first==(0,15,24 if manifest['asset_version'] in ('screen-preview-v2','room-preview-v1') else 17)[clip['origin']]
        assert clip['frames'][-1]==(0,15,24 if manifest['asset_version'] in ('screen-preview-v2','room-preview-v1') else 17)[clip['end_pose']]
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
print(f'PASS source hashes, {manifest["unique_frames"]} frame headers/hashes/ground plate, clips/anchors, colorbar order and behavior config')

if manifest.get('asset_version') in ('screen-preview-v2','room-preview-v1'):
    assert hashlib.sha256((root/manifest['sleep_source']).read_bytes()).hexdigest()==manifest['sleep_source_sha256']
    loop=manifest['sleep_loop']
    assert loop['frames'][0]==loop['frames'][-1]==24 and loop['period_ms']==4200
    assert loop['at_ms']==sorted(set(loop['at_ms'])) and loop['at_ms'][-1]<4200
    bases=[blob[f*51212+12+139*320:(f+1)*51212] for f in range(24,28)]
    assert all(b==bases[0] for b in bases), 'sleeping base must remain fixed'

if manifest.get('asset_version')=='room-preview-v1':
    room=(root/'assets/animation/room.bin').read_bytes()
    assert len(room)==153612 and hashlib.sha256(room).hexdigest()==manifest['room_sha256']
    assert struct.unpack_from('<BBHHHHH',room)==(0x19,0x12,0,320,240,640,0)
    assert hashlib.sha256((root/manifest['room_source']).read_bytes()).hexdigest()==manifest['room_source_sha256']
    for frame in manifest['frames']:
        # Some authored probe silhouettes reach the right edge; verify the
        # clear top/bottom margins and left edge, not opaque character pixels.
        border=[(x,y) for x in range(160) for y in (0,159)]+[(0,y) for y in range(160)]
        for x,y in border:
            offset=frame['offset']+12+y*320+x*2
            bg=12+(y+40)*640+(x+80)*2
            assert blob[offset:offset+2]==room[bg:bg+2], 'room/pose seam'

# Restored GitHub PROBE: original source cells and original timing.
assert manifest['probe_revision']=='github-original'
clip=manifest['clips'][manifest['clip_map'][0][1]]
assert clip['frames']==[0,1,2,2,1,0]
assert clip['at_permille']==[0,100,250,720,850,1000]
assert [manifest['registration'][i]['cell'] for i in (0,1,2)]==[0,1,2]
assert manifest['unique_frames']==33
happy=manifest['clips'][manifest['clip_map'][0][6]]
assert happy['frames']==[0,8,19,20,19,20,19,21,8,0]

# Sleep must convey fatigue before folding, without the wide-eyed shy poses.
rest=manifest['clips'][manifest['clip_map'][0][7]]
rest_sources=[neutral_sources.get(f,f) for f in rest['frames']]
assert not {13,14,15}.intersection(rest_sources), 'REST reuses startled/shy poses'
assert rest_sources.count(23)==2, 'REST should start with two slow blinks'
assert rest_sources.index(23)<rest_sources.index(16)<rest_sources.index(17)
assert rest['frames'][-1]==manifest['sleep_loop']['frames'][0]
drowsy_start=rest_sources.index(16)
assert set(rest_sources[drowsy_start:])<={16,17,24}, 'Sleepy pet becomes wide-eyed again'
folded_rest=manifest['clips'][manifest['clip_map'][1][7]]
folded_sources=[neutral_sources.get(f,f) for f in folded_rest['frames']]
assert folded_sources[0]==15 and set(folded_sources[1:])<={16,17,24}

# No rosy shared frame may slip into REST. Only cheek regions may differ from
# original interaction poses; protect face/eye geometry and the whole body.
recipe=root/manifest['neutral_sleep_recipe']
assert hashlib.sha256(recipe.read_bytes()).hexdigest()==manifest['neutral_sleep_recipe_sha256']
retouch=json.loads(recipe.read_text())
for name,sha in retouch['edit_sha256'].items():
    assert hashlib.sha256((recipe.parent/name).read_bytes()).hexdigest()==sha
for clip in (rest,folded_rest):
    assert set(clip['frames'])<=set(neutral_sources)|{24,25,26,27}
for source_key,boxes in retouch['cheek_bounds'].items():
    source=int(source_key)
    target=manifest['rest_neutral_map'].get(source_key,source)
    for y in range(160):
        for x in range(160):
            pos=12+y*320+x*2
            in_cheek=any(x0-1<=x<x1+1 and y0-1<=y<y1+1 for x0,y0,x1,y1 in boxes)
            if target!=source and not in_cheek:
                assert blob[target*51212+pos:target*51212+pos+2]==blob[source*51212+pos:source*51212+pos+2], 'Neutral variant changes more than cheeks'
    # The centers of both formerly luminous cheek strokes are dark screen now.
    for x,y in retouch['cheek_samples'][source_key]:
        pixel=struct.unpack_from('<H',blob,target*51212+12+y*320+x*2)[0]
        r=((pixel>>11)&31)*255//31;g=((pixel>>5)&63)*255//63;b=(pixel&31)*255//31
        assert max(r,g,b)<90, 'Sleep cheek still has luminous blush'

# Enclosed negative space must show the room at the SAME screen position.
cavity_recipe=root/manifest['cavity_recipe']
assert hashlib.sha256(cavity_recipe.read_bytes()).hexdigest()==manifest['cavity_recipe_sha256']
for name,region in json.loads(cavity_recipe.read_text())['poses'].items():
    frame=next(f for f in manifest['frames'] if f['name']==name)
    x,y=region['seed']
    pos=frame['offset']+12+y*320+x*2
    bg=12+(y+40)*640+(x+80)*2
    assert blob[pos:pos+2]==room[bg:bg+2], f'{name}: enclosed backdrop is opaque'
