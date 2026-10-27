"""Daily-proportion layered screen rig; does not generate motor commands."""
import hashlib
import json
import math
from pathlib import Path
from PIL import Image, ImageChops, ImageDraw

ROOT=Path(__file__).resolve().parents[1]
SOURCE=ROOT/'assets/images/source/dance/primitives'
R=getattr(Image,'Resampling',Image)

def rotate(v,a):
    a=math.radians(a);x,y=v
    return math.cos(a)*x+math.sin(a)*y,-math.sin(a)*x+math.cos(a)*y

def ease(t):return t*t*t*(10+t*(-15+6*t))

def key_sample(keys,t):
    for left,right in zip(keys,keys[1:]):
        if t<=right[0]:
            u=ease(max(0,min(1,(t-left[0])/(right[0]-left[0]))))
            return tuple(a+(b-a)*u for a,b in zip(left[1:],right[1:]))
    return tuple(keys[-1][1:])

def angle(v):return math.degrees(math.atan2(v[1],v[0]))

class DanceRig:
    def __init__(self):
        self.config=json.loads((SOURCE/'rig.json').read_text());r=self.config
        self.scale=r['render_px']/r['cell_px'];self.radius=180
        self.rest=[0,0,0]
        source=Image.open(ROOT/r['source']).convert('RGBA')
        self.views=[]
        for view,(shoulder,elbow,wrist) in enumerate(r['view_axes']):
            size=r['cell_px'];body=source.crop((view*size,0,(view+1)*size,size))
            def layer(mask,pivot):
                sprite=body.copy();sprite.putalpha(ImageChops.multiply(sprite.getchannel('A'),mask))
                # A skin region may intersect a detached sliver of another part.
                # Bind only the component containing this joint, never floating scraps.
                component=sprite.getchannel('A').point(lambda n:255 if n>10 else 0)
                candidates=[(x,y) for y in range(size) for x in range(size) if component.getpixel((x,y))]
                seed=min(candidates,key=lambda p:(p[0]-pivot[0])**2+(p[1]-pivot[1])**2)
                ImageDraw.floodfill(component,seed,128,thresh=0)
                sprite.putalpha(ImageChops.multiply(sprite.getchannel('A'),component.point(lambda n:255 if n==128 else 0)))
                sprite=sprite.resize((r['render_px'],r['render_px']),R.LANCZOS)
                return self.center(sprite,[round(v*self.scale) for v in pivot])
            def mask():return Image.new('L',(size,size))
            # Masks are the editable rig's skin bindings. Original artwork is
            # retained; downstream complete round caps cover the overlapping seams.
            base=mask();d=ImageDraw.Draw(base);d.rectangle((0,540,size,size),fill=255)
            d.ellipse((shoulder[0]-70,shoulder[1]-90,shoulder[0]+110,shoulder[1]+87),fill=0)
            x,y=shoulder;ex,ey=elbow;wx,wy=wrist
            lower=mask();d=ImageDraw.Draw(lower)
            d.polygon([(ex-65,ey-15),(ex+105,ey-15),(x+205,y-88),(x+205,y+48),(x+100,y+84),(x-65,y+84),(ex-65,ey+30)],fill=255)
            d.ellipse((x-65,y-90,x+110,y+87),fill=255)
            upper=mask();d=ImageDraw.Draw(upper)
            d.line((ex+20,ey,wx+20,wy),fill=255,width=155)
            d.ellipse((ex-65,ey-84,ex+100,ey+86),fill=255)
            head=mask();d=ImageDraw.Draw(head)
            d.ellipse((wx-77,wy-100,wx+112,wy+103),fill=255)
            d.polygon([(wx,0),(size,0),(size,wy+195),(wx+75,wy+195),(wx+70,wy+85),(wx,wy+65)],fill=255)
            # Keep only the link skin, excluding face pixels caught by its
            # capsule. Preserve overlap inside the round wrist joint.
            cap=mask();ImageDraw.Draw(cap).ellipse((wx-80,wy-105,wx+115,wy+108),fill=255)
            face_only=ImageChops.subtract(head,cap)
            upper=ImageChops.subtract(upper,face_only)
            upper=ImageChops.subtract(upper,head)
            lower=ImageChops.subtract(lower,head)
            elbow_cap=mask();ImageDraw.Draw(elbow_cap).ellipse((ex-65,ey-84,ex+100,ey+86),fill=255)
            lower=ImageChops.subtract(lower,elbow_cap)
            alpha=body.getchannel('A').point(lambda v:255 if v>128 else 0)
            b=alpha.crop((0,600,size,size)).getbbox();floor=((b[0]+b[2])/2,600+b[3])
            root=(r['root'][0]+(shoulder[0]-floor[0]-(235-331.5))*self.scale,r['root'][1])
            # Fixed per-view rest rotations align the short, broad source links
            # with the daily reference joint axes. No per-frame resizing.
            vectors=[[(elbow[i]-shoulder[i])*self.scale for i in (0,1)],
                     [(wrist[i]-elbow[i])*self.scale for i in (0,1)]]
            # True fixed source link lengths; solve only the neutral pose.
            l1,l2=[math.hypot(*v) for v in vectors];tx,ty=r['neutral_wrist_offset_px']
            joint=math.acos((tx*tx+ty*ty-l1*l1-l2*l2)/(2*l1*l2))
            theta=math.atan2(ty,tx)-math.atan2(l2*math.sin(joint),l1+l2*math.cos(joint))
            rests=[angle(vectors[0])-math.degrees(theta),angle(vectors[1])-math.degrees(theta+joint)]
            self.views.append(dict(layers=[layer(base,floor),layer(lower,shoulder),layer(upper,elbow),layer(head,wrist)],root=root,rests=rests,vectors=vectors))
        jaws=Image.open(ROOT/r['jaw_source']).convert('RGBA');self.jaws=[]
        for box,pivot in zip(r['jaw_crops'],r['jaw_pivots']):
            jaw=jaws.crop(box);scale=r['jaw_screen_scale']
            jaw=jaw.resize(tuple(round(v*scale) for v in jaw.size),R.LANCZOS)
            self.jaws.append(self.center(jaw,[round((pivot[i]-box[i])*scale) for i in (0,1)]))

    def center(self,sprite,pivot):
        layer=Image.new('RGBA',(2*self.radius,2*self.radius))
        layer.alpha_composite(sprite,(self.radius-pivot[0],self.radius-pivot[1]))
        return layer

    def pose(self,clip,tick):
        return key_sample(clip['keyframes'],tick/(clip['duration_ms']//125-1))

    def render(self,room,channels):
        yaw,s,e,w,roll,opening=channels
        view=0 if yaw<-8 else 2 if yaw>8 else 1
        v=self.views[view];positions=[v['root']];absolute=0
        for i,(vec,a) in enumerate(zip(v['vectors'],[s,e])):
            absolute+=a;delta=rotate(vec,absolute+v['rests'][i])
            positions.append(tuple(positions[-1][j]+delta[j] for j in (0,1)))
        frame=room.copy()
        def put(layer,pos,rotation):
            image=layer.rotate(rotation,R.BICUBIC,center=(self.radius,self.radius)) if abs(rotation)>1e-8 else layer
            box=image.getchannel('A').point(lambda v:255 if v>128 else 0).getbbox()
            origin=(round(pos[0])-self.radius,round(pos[1])-self.radius)
            assert box and 0<=origin[0]+box[0]<origin[0]+box[2]<=320,(channels,box,origin)
            assert 0<=origin[1]+box[1]<origin[1]+box[3]<=240,(channels,box,origin)
            frame.alpha_composite(image,origin)
        put(v['layers'][0],self.config['base_anchor'],0)
        for i,a in enumerate([s,s+e]):put(v['layers'][i+1],positions[i],a+v['rests'][i])
        # Pitch tilts the head housing about its wrist axis; the complete paired
        # gripper can roll about the single shared mounting axis independently.
        heading=s+e+w
        put(v['layers'][3],positions[2],heading)
        jaw_angles=[]
        # Coupled opening: exactly two whole jaw units, equal/opposite opening.
        for j in self.config['draw_order']:
            local=self.config['jaw_mounts_px'][j]
            middle=[sum(p[i] for p in self.config['jaw_mounts_px'])/2 for i in (0,1)]
            offset=rotate([local[i]-middle[i] for i in (0,1)],roll)
            rotated=rotate([middle[i]+offset[i] for i in (0,1)],heading)
            mount=tuple(positions[2][i]+rotated[i] for i in (0,1))
            ja=self.config['jaw_rest_deg'][j]+self.config['jaw_outward_sign'][j]*opening
            put(self.jaws[j],mount,heading+roll+ja)
            jaw_angles.append(ja)
        trace=dict(angles_deg=[round(a,5) for a in (s,e,w)],base_yaw_deg=round(yaw,5),view=view,
                   wrist_roll_deg=round(roll,5),gripper_open_deg=round(opening,5),jaw_angles_deg=jaw_angles,
                   joints_px=[[round(n,5) for n in p] for p in positions],link_lengths_px=[math.hypot(*n) for n in v['vectors']],floor=list(self.config['base_anchor']))
        return frame.convert('RGB'),trace

def build_dances(room):
    config=json.loads((SOURCE/'motion.json').read_text());rig=DanceRig()
    frames,records,traces={},[],{}
    for clip in config['clips']:
        seq=[];trace=[]
        for tick in range(clip['duration_ms']//125):
            frame,state=rig.render(room,rig.pose(clip,tick));seq.append(frame);trace.append(state)
        seq[-1]=seq[0].copy();frames[clip['id']]=seq;traces[clip['id']]=trace
        records.append({**clip,'atlas':'primitives','keys':[[i,1] for i in range(len(seq))],'claw_key':'idle:0','ticks':len(seq),'loop':True})
    paths=[ROOT/rig.config['source'],ROOT/rig.config['jaw_source'],SOURCE/'rig.json',SOURCE/'motion.json']
    sources={str(p.relative_to(ROOT)):{'path':str(p.relative_to(ROOT)),'sha256':hashlib.sha256(p.read_bytes()).hexdigest()} for p in paths}
    return frames,records,sources,traces,config
