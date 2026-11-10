"""Independently decode shipped records and compare every tick with source GIFs."""
import hashlib
import json
from pathlib import Path
import re
import struct
import zlib
from PIL import Image
import numpy as np

root = Path(__file__).resolve().parents[2]
cfg = json.loads((root/'assets/config/behavior_defaults.json').read_text())
manifest = json.loads((root/'assets/animation/manifest.json').read_text())
previews = json.loads((root/'assets/gifs/daily/manifest.json').read_text())
dance = json.loads((root/'assets/gifs/dance/manifest.json').read_text())
all_sources = previews['clips'] + dance['clips']
source_paths = {c['id']: root/'assets/gifs/daily'/c['file'] for c in previews['clips']}
source_paths.update({c['id']: root/'assets/gifs/dance'/c['file'] for c in dance['clips']})
claw_regions = json.loads((root/'assets/config/claw_touch_regions.json').read_text())['frames']
blob = (root/'assets/animation/frames.bin').read_bytes()
assert manifest['asset_version'] == 'gif-v2-rgb565'
assert manifest['source'] == 'assets/gifs/daily/manifest.json'
assert hashlib.sha256((root/manifest['source']).read_bytes()).hexdigest() == manifest['source_sha256']
assert len(blob) == manifest['bytes']
assert len(manifest['frames']) == manifest['unique_frames']
room = (root/'assets/animation/room.bin').read_bytes()
assert len(room) == manifest['room_bytes'] == 153612
assert struct.unpack_from('<BBHHHHH', room) == (0x19, 0x12, 0, 320, 240, 640, 0)
assert hashlib.sha256(room).hexdigest() == manifest['room_sha256']
assert hashlib.sha256((root/manifest['room_source']).read_bytes()).hexdigest() == manifest['room_source_sha256']

def rgb565(image):
    # Explicit conversion, independent of the resource builder.
    result = bytearray()
    for red, green, blue in image.convert('RGB').getdata():
        result += struct.pack('<H', ((red >> 3) << 11) | ((green >> 2) << 5) | (blue >> 3))
    return bytes(result)

records = []
end = 0
for frame in manifest['frames']:
    assert frame['offset'] == end, 'overlapping, gapped or reordered records'
    end += frame['bytes']
    assert 16 < frame['bytes'] and end <= len(blob)
    magic, width, height, raw_length, codec = struct.unpack_from('<4sHHII', blob, frame['offset'])
    assert (magic, width, height, raw_length) == (b'RBG2', 320, 240, 153600)
    assert codec in (0, 1, 2, 3, 4, 5, 6, 7)
    records.append((codec, blob[frame['offset']+16:end]))
assert end == len(blob)
assert any(codec == 2 for codec, _ in records), 'lossless delta encoding is not present'

def inflate(data, expected=153600):
    decoder = zlib.decompressobj()
    pixels = decoder.decompress(data)
    assert decoder.eof and not decoder.unused_data and not decoder.unconsumed_tail
    assert len(pixels) == expected
    return pixels

def independent_pixels(index):
    codec, data = records[index]
    assert codec in (0, 1, 3), 'delta references another delta (chains are forbidden)'
    if codec==3:
        count=struct.unpack_from('<H',data)[0];assert 1<=count<=256
        palette=data[2:2+count*2];indices=inflate(data[2+count*2:],76800)
        assert max(indices)<count
        pixels=b''.join(palette[2*i:2*i+2] for i in indices)
    else: pixels = data if codec == 0 else inflate(data)
    assert len(pixels) == 153600
    return pixels

room_pixels=np.frombuffer(room[12:],dtype='<u2').reshape(240,320)
planes_by_frame={}
coverage_by_frame={}
def dance_planes(index, depth=0):
    assert depth<=7
    codec,data=records[index]
    if codec==7:
        base=struct.unpack_from('<I',data)[0]
        assert base<index and records[base][0] in (6,7)
        plane,palette,key,size=dance_planes(base,depth+1)
        delta=inflate(data[4:],2*size*size)
        return bytes(a^b for a,b in zip(plane,delta)),palette,key,size
    assert codec==6
    count,key,width,height=struct.unpack_from('<HHHH',data)
    assert 1<=count<=256 and width==height and 2<=width<=240
    palette=np.frombuffer(data[8:8+2*count],dtype='<u2')
    return inflate(data[8+2*count:],2*width*height),palette,key,width

def composite_dance(index):
    plane,palette,key,size=dance_planes(index)
    n=size*size
    indices=np.frombuffer(plane[:n],dtype=np.uint8).reshape(size,size)
    alpha=np.frombuffer(plane[n:],dtype=np.uint8).reshape(size,size).astype(np.int32)
    assert int(indices.max())<len(palette)
    source=palette[indices]
    def unpack(p):
        p=p.astype(np.int32)
        return np.stack([(p>>11)*255//31,((p>>5)&63)*255//63,(p&31)*255//31],axis=-1)
    foreground=np.clip(unpack(source)-(255-alpha[...,None])*unpack(np.array(key))//255,0,alpha[...,None])
    rgba=np.concatenate((foreground,alpha[...,None]),axis=2)
    coordinates=[max(0,min((size-1)*256,((2*x+1)*size*128)//240-128)) for x in range(240)]
    low=np.array([x//256 for x in coordinates]);high=np.minimum(low+1,size-1)
    weight=np.array([x%256 for x in coordinates])
    total=np.zeros((240,240,4),dtype=np.int64)
    for yy,wy in ((low,256-weight),(high,weight)):
        for xx,wx in ((low,256-weight),(high,weight)):
            total+=rgba[yy[:,None],xx[None,:]]*wy[:,None,None]*wx[None,:,None]
    interpolated=(total+32768)//65536
    a=interpolated[:,:,3]
    color=np.clip(interpolated[:,:,:3]+(255-a[...,None])*unpack(room_pixels[:,40:280])//255,0,255)
    out=room_pixels.copy()
    center=((color[:,:,0]>>3)<<11)|((color[:,:,1]>>2)<<5)|(color[:,:,2]>>3)
    center[a==0]=room_pixels[:,40:280][a==0]
    out[:,40:280]=center
    coverage=np.zeros((240,320),dtype=np.uint8);coverage[:,40:280]=a
    planes_by_frame[index]=(source,alpha)
    coverage_by_frame[index]=coverage
    return out.astype('<u2').tobytes()

decoded = []
for index, frame in enumerate(manifest['frames']):
    codec, data = records[index]
    if codec>=4:
        pixels=composite_dance(index)
    elif codec == 2:
        assert len(data) > 4
        base = struct.unpack_from('<I', data)[0]
        assert 0 <= base < len(records) and base != index, 'invalid/self-referencing delta base'
        original = independent_pixels(base)
        delta = inflate(data[4:])
        pixels = bytes(a ^ b for a, b in zip(original, delta))
    else:
        pixels = independent_pixels(index)
    assert len(pixels) == 153600
    assert hashlib.sha256(pixels).hexdigest() == frame['sha256']
    if index == 0:
        assert codec == 0, 'allocation-free fallback must be a raw record'
    polygon = frame['claw_region']
    assert polygon == claw_regions[frame['claw_key']], 'generated hitbox differs from reviewed source config'
    assert len(polygon) == 4 and all(0 <= x < 160 and 0 <= y < 160 for x, y in polygon)
    turns = []
    for i in range(4):
        a, b, c = polygon[i], polygon[(i+1)%4], polygon[(i+2)%4]
        turns.append((b[0]-a[0])*(c[1]-b[1])-(b[1]-a[1])*(c[0]-b[0]))
    assert all(t > 0 for t in turns) or all(t < 0 for t in turns), 'nonconvex claw hitbox'
    decoded.append(pixels)

source_by_id = {clip['id']: clip for clip in all_sources}
assert len(manifest['clips']) == len(source_by_id)
assert len([c for c in manifest['clips'] if c.get('source_gif')]) == len(source_by_id)
checked = 0
for clip in manifest['clips']:
    assert len(clip['frames']) == len(clip['at_ms']) <= 256
    assert clip['at_ms'][0] == 0
    assert clip['at_ms'] == sorted(set(clip['at_ms']))
    assert clip['at_ms'][-1] < clip['duration_ms']
    assert all(0 <= i < len(decoded) for i in clip['frames'])
    assert clip.get('source_gif'), 'runtime clip must have an active source GIF'
    source = source_by_id[clip['id']]
    path = source_paths[clip['id']]
    assert root/clip['source_gif'] == path
    assert hashlib.sha256(path.read_bytes()).hexdigest() == clip['source_sha256'] == source['sha256']
    with Image.open(path) as gif:
        assert gif.size == ((640,640) if source.get('screen_fit') else (320,240)) and gif.n_frames == source.get('ticks', sum(n for _, n in source['keys']))
        source_times = []
        source_elapsed = 0
        for i in range(gif.n_frames):
            gif.seek(i)
            source_times.append(source_elapsed)
            source_elapsed += gif.info['duration']
        assert source_elapsed == source['duration_ms'] == clip['duration_ms']
        elapsed = 0
        if source.get('screen_fit'):
            assert clip['alpha_source']=='renderer_rgba' and clip['runtime_sampling_ms'] is None
            assert len(clip['frames'])==len(clip['source_ticks'])
            assert clip['source_ticks'][0]==0 and clip['source_ticks'][-1]==len(source['rgba_frames'])-1
            starts=np.cumsum([0]+[row['duration_ms'] for row in source['rgba_frames']]).tolist()
            for tick,frame_id in enumerate(clip['frames']):
                source_tick=clip['source_ticks'][tick];row=source['rgba_frames'][source_tick]
                frame_path=root/'assets/gifs/dance'/row['file']
                assert hashlib.sha256(frame_path.read_bytes()).hexdigest()==row['sha256']
                with Image.open(frame_path) as rgba:
                    size=manifest['dance_render_size']
                    fitted=rgba.convert('RGBA').resize((size,size),Image.Resampling.LANCZOS)
                screen=fitted
                step=255/(manifest['dance_alpha_levels']-1)
                expected_alpha=np.rint(np.rint(np.asarray(screen.getchannel('A'),dtype=float)/step)*step).astype(np.uint8)
                screen.putalpha(Image.fromarray(expected_alpha))
                matte=Image.alpha_composite(Image.new('RGBA',screen.size,(255,255,255,255)),screen).convert('RGB')
                palette=Image.new('P',(1,1));palette.putpalette(manifest['dance_palette_rgb'])
                quantized=matte.quantize(palette=palette,dither=0).convert('RGB')
                expected=np.frombuffer(rgb565(quantized),dtype='<u2').reshape(size,size)
                actual,alpha=planes_by_frame[frame_id]
                assert np.array_equal(alpha,expected_alpha),(clip['id'],tick,'alpha changed')
                assert np.array_equal(actual[alpha>0],expected[alpha>0]),(clip['id'],tick,'palette color changed')
                scene=np.frombuffer(decoded[frame_id],dtype='<u2').reshape(240,320)
                coverage=coverage_by_frame[frame_id]
                assert np.array_equal(scene[coverage==0],room_pixels[coverage==0]),(clip['id'],tick,'room hole filled')
                assert clip['at_ms'][tick]==elapsed
                assert row['duration_ms']>0 and row['duration_ms']%10==0
                next_tick=clip['source_ticks'][tick+1] if tick+1<len(clip['source_ticks']) else len(source['rgba_frames'])
                elapsed+=starts[next_tick]-starts[source_tick];checked+=1
            trajectory=root/source['trajectory']
            assert hashlib.sha256(trajectory.read_bytes()).hexdigest()==source['trajectory_sha256']
        else:
            assert len(clip['frames'])==gif.n_frames
            for tick,frame_id in enumerate(clip['frames']):
                gif.seek(tick)
                assert clip['at_ms'][tick]==elapsed
                assert decoded[frame_id]==rgb565(gif.convert('RGB')),(clip['id'],tick,'Daily source pixels changed')
                elapsed+=gif.info['duration'];checked+=1
            assert elapsed==gif.n_frames*125
        assert elapsed==clip['duration_ms']
        if clip['id']=='DANCE_EIGHT' and source.get('trajectory_overlay'):
            # The thin path must survive the device palette, not turn grey or
            # disappear into the room after downsampling and alpha compositing.
            visible=0
            for frame_id in clip['frames']:
                pixels=np.frombuffer(decoded[frame_id],dtype='<u2').reshape(240,320)
                red=(pixels>>11)*255//31
                green=((pixels>>5)&63)*255//63
                blue=(pixels&31)*255//31
                cyan=(red<120)&(green>150)&(blue>170)&(pixels!=room_pixels)
                visible=max(visible,int(cyan.sum()))
            assert visible>=10,'figure-eight trail lost its cyan ink in the firmware palette'
assert {c['id'] for c in manifest['clips']} >= set(source_by_id)
assert checked == sum(len(c['frames']) for c in manifest['clips'] if c.get('source_gif'))
assert {i for c in manifest['clips'] for i in c['frames']} == set(range(len(decoded))), 'obsolete unused frames remain in runtime'
assert set(manifest['source_gifs']) == {str(p.relative_to(root)) for p in source_paths.values()}

choreo = manifest['choreography_clip_map']
assert len(choreo) == 55 and choreo[15] == 255
retired_ids = {4, 5} | set(range(9, 15)) | set(range(16, 23))
assert all(choreo[action] == 255 for action in retired_ids), 'retired work IDs alias new animation'
assert all(choreo[action] == 255 for action in range(23, 43)), 'removed dances still mapped'
for action, name in {0:'IDLE', 1:'PROBE', 2:'NUZZLE', 3:'TILT', 6:'HAPPY'}.items():
    assert manifest['clips'][choreo[action]]['id'] == name, 'daily mapping changed'
mapping = manifest['clip_map']
assert len(mapping) == 3 and all(len(row) == 9 for row in mapping)
ids = ['IDLE', 'PROBE', 'NUZZLE', 'TILT', 'IDLE', 'IDLE', 'HAPPY', 'REST', 'WAKE']
for row in mapping:
    for action in range(1, 9):
        assert manifest['clips'][row[action]]['id'] == ids[action]
assert manifest['clips'][mapping[0][0]]['id'] == 'IDLE'
assert manifest['clips'][mapping[2][0]]['id'] == 'SLEEP'
assert manifest['clips'][mapping[1][0]]['id'] == 'IDLE'

header = (root/'components/rebuddy_core/include/rb_pet_config.h').read_text()
for key, value in cfg['gesture'].items():
    assert re.search(r'^#define RB_CFG_'+key.upper()+r' '+str(value)+r'u$', header, re.M)
for key, value in cfg['action_durations_ms'].items():
    assert re.search(r'^#define RB_MS_'+key+r' '+str(value)+r'u$', header, re.M)
    source_id = 'NUZZLE' if key == 'NUZZLE_CYCLE' else key
    assert value == source_by_id[source_id]['duration_ms']
print(f'PASS {len(decoded)} independently decoded RGB565 records (raw/zlib/XOR-delta), {checked} exact GIF frame/timing comparisons, clip maps, hitboxes and behavior durations')

for c in dance['clips']:
    assert manifest['clips'][choreo[c['action_id']]]['id'] == c['id']
assert hashlib.sha256((root/manifest['dance_source']).read_bytes()).hexdigest() == manifest['dance_source_sha256']

# Every selectable action loops exactly; shared-artwork groups also match
# across actions. The approved whole-character 08 has its own drawn neutral.
dances=[c for c in manifest['clips'] if c['id'].startswith('DANCE_') and c['id'] not in ('DANCE_START','DANCE_END')]
groups={c['id']:c.get('endpoint_group','shared_neutral') for c in dance['clips']}
assert {k:v for k,v in groups.items() if v!='shared_neutral'} in ({},{'DANCE_STEADY':'whole_character_turn'})
for a in dances:
    assert decoded[a['frames'][-1]]==decoded[a['frames'][0]],(a['id'],'loop seam')
    for b in dances:
        if groups[a['id']]==groups[b['id']]:
            assert decoded[a['frames'][-1]]==decoded[b['frames'][0]],(a['id'],b['id'],'seam')
assert decoded[manifest['clips'][choreo[43]]['frames'][-1]]==decoded[dances[0]['frames'][0]]
assert decoded[manifest['clips'][choreo[44]]['frames'][0]]==decoded[dances[0]['frames'][0]]
for i,frame in enumerate(manifest['frames']):assert zlib.crc32(decoded[i])==frame['crc32']
