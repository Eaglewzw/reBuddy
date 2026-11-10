#!/usr/bin/env python3
"""08/09 as multi-view 2D drawings. No meshes, lighting, or 3D rasterizer.

The nine drawn head views supply actual changing silhouettes and occlusion.
The original vector idle is one of the keys. Corresponding 2D landmarks connect
the head drawings; separately authored body turn keys control its contours,
green panels and occlusion, using the original character's ink and palette.
The previous accepted screen-space TCP samples and durations remain unchanged.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw
from scipy.ndimage import map_coordinates
from scipy.spatial import Delaunay
from scipy.interpolate import PchipInterpolator

from render_dance_cartoon import ROOT, OriginalCharacter, rotation, write_gif, BG
from render_dance_reference import ReferenceCharacter, solve_arm, TIP, art
from dance_body_views import BodyViews

SOURCE=ROOT/'assets/images/source/dance/multiview'


def review_page(out,records):
    folder=out.parent.parent/'dance-multiview';folder.mkdir(exist_ok=True)
    cards=['<article><h2>GitHub 旧版转身参考</h2><img src="../../assets/images/source/dance/github_turn_reference.gif"><p>参照正侧面绘图方式。</p></article>']
    for entry in records:
        code=entry['number'];version=hashlib.sha256((out/'preview'/f'{code}.gif').read_bytes()).hexdigest()[:12]
        cards.append(f'<article><h2>{code} · {entry["duration_ms"]/1000:.2f}秒</h2><img src="../dance-3d/multiview/preview/{code}.gif?v={version}"><p><a href="../dance-3d/multiview/{code}.gif?v={version}">透明GIF</a></p></article>')
    (folder/'index.html').write_text('<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>08 / 09 多视角二维绘图</title><style>body{background:#fffdf5;color:#26342a;font:16px system-ui;max-width:1200px;margin:auto;padding:24px}.cards{display:flex;flex-wrap:wrap;gap:20px}article{flex:1;min-width:260px}img{width:100%}p{line-height:1.7}</style><h1>08 / 09 · 多视角二维绘图</h1><p>以原绿色卡通角色补绘转身视角，保留原待机首尾、动作节奏和09横向∞路径。保留头部角色原画；08统一腕部连接与视角过渡，身体参考旧GIF分别绘制正面、斜侧面和侧面，底座保留原画。</p><div class="cards">'+''.join(cards)+'</div><h2>关键角度</h2><img src="../dance-3d/multiview/angles.png"><p><a href="../dance-original/index.html">返回全部舞蹈</a></p></html>')


class HeadViews:
    def __init__(self, smooth_joint=False):
        self.smooth_joint=smooth_joint
        self.metadata=json.loads((SOURCE/'heads.json').read_text())
        self.keys=[];self.grid=np.indices((512,512),dtype=np.float32)
        for record in self.metadata['views']:
            path=SOURCE/'heads'/f'{record["index"]:02d}.png'
            assert hashlib.sha256(path.read_bytes()).hexdigest()==record['sha256']
            pixels=np.array(Image.open(path)).astype(np.float32)/255
            pixels[:,:,:3]*=pixels[:,:,3:4]
            self.keys.append(pixels)
        self.landmarks={i:self.features(i) for i in range(4,9)}
        if smooth_joint:
            self.feature_curve=PchipInterpolator([0,20,40,60,80],
                np.array([self.landmarks[i] for i in range(4,9)]),axis=0)
            self.wrist_curve=PchipInterpolator([0,20,40,60,80],
                np.array([self.metadata['views'][i]['wrist'] for i in range(4,9)]),axis=0)

    def features(self,i):
        """Corresponding 2D drawing features, not inferred 3D surfaces."""
        image=self.keys[i];a=image[:,:,3]
        yy,xx=self.grid
        points=[[0,0],[255,0],[511,0],[511,255],[511,511],[255,511],[0,511],[0,255]]
        # Register the drawn silhouette at corresponding directions.
        mask=(a>.5)&(yy<336)&(yy>60);ys,xs=np.where(mask)
        center=np.array([(xs.min()+xs.max())/2,(ys.min()+ys.max())/2])
        for theta in np.linspace(0,math.tau,16,endpoint=False):
            ray=np.arange(1,300)[:,None]*[math.cos(theta),math.sin(theta)]+center
            coords=np.rint(ray).astype(int);valid=(coords>=0).all(axis=1)&(coords<512).all(axis=1)
            ray,coords=ray[valid],coords[valid]
            hits=mask[coords[:,1],coords[:,0]]
            points.append(ray[np.where(hits)[0][-1]].tolist())
        eyes={4:((225,253,28,35,0),(338,253,27,35,0)),
              5:((244,252,28,35,0),(346,251,20,34,0)),
              6:((248,244,24,36,22),(348,280,19,31,22)),
              7:((246,254,29,36,0),(336,266,17,30,0)),
              8:((289,254,24,38,0),(323,255,1,28,0))}[i]
        for cx,cy,rx,ry,theta in eyes:
            points.append([cx,cy])
            for off in [(-rx,0),(0,-ry),(rx,0),(0,ry)]:
                points.append((np.array([cx,cy])+rotation(theta)@off).tolist())
        cx,cy=self.metadata['views'][i]['wrist']
        rx,ry={4:(12,47),5:(49,65),6:(69,67),7:(71,69),8:(78,73)}[i]
        points.append([cx,cy])
        for theta in np.linspace(0,math.tau,8,endpoint=False):
            points.append([cx+rx*math.cos(theta),cy+ry*math.sin(theta)])
        jaws={4:((242,387,40,42),(315,387,39,42)),5:((243,387,41,43),(310,387,37,42)),
              6:((285,387,42,44),(345,388,26,35)),7:((275,390,40,40),(327,382,29,37)),
              8:((280,391,37,38),(306,383,19,34))}[i]
        for cx,cy,rx,ry in jaws:
            points.extend([[cx,cy],[cx-rx,cy],[cx,cy-ry],[cx+rx,cy],[cx,cy+ry]])
        return np.array(points,float)

    def parameters(self, degrees):
        if not 0<=degrees<=80: raise ValueError('View outside authored 0..80 degree range')
        value=(degrees+80)/20; i=min(7,math.floor(value));return i,value-i

    def wrist(self,degrees):
        if self.smooth_joint:
            return (self.wrist_curve(degrees)-[280,430])/2
        i,t=self.parameters(degrees);keys=self.metadata['views']
        return ((1-t)*np.array(keys[i]['wrist'])+t*np.array(keys[i+1]['wrist'])-keys[i]['tip'])/2

    def image(self,degrees):
        i,t=self.parameters(degrees)
        if t<1e-7:return self.keys[i]
        if 1-t<1e-7:return self.keys[i+1]
        a,b=self.landmarks[i],self.landmarks[i+1]
        target=self.feature_curve(degrees) if self.smooth_joint else (1-t)*a+t*b
        triangulation=Delaunay(target)
        query=np.column_stack([self.grid[1].ravel(),self.grid[0].ravel()])
        simplex=triangulation.find_simplex(query)
        transform=triangulation.transform[simplex]
        uv=np.einsum('nij,nj->ni',transform[:,:2],query-transform[:,2])
        weights=np.column_stack([uv,1-uv.sum(axis=1)])
        vertices=triangulation.simplices[simplex]
        def warp(pixels,source):
            xy=np.einsum('ni,nij->nj',weights,source[vertices])
            coordinates=xy[:,::-1].T.reshape(2,512,512)
            return np.stack([map_coordinates(pixels[:,:,k],coordinates,order=1,mode='constant',cval=0)
                             for k in range(4)],axis=-1)
        # Preserve a single clean ink drawing. Cross-dissolving two independently
        # drawn outlines caused double eyes/rings; geometry still interpolates.
        return warp(self.keys[i],a) if t<.5 else warp(self.keys[i+1],b)

    def draw(self,c,pose):
        pixels=np.rint(np.clip(self.image(pose['view_deg']),0,1)*255).astype(np.uint8)
        bgra=np.ascontiguousarray(pixels[:,:,[2,1,0,3]])
        surface=art.cairo.ImageSurface.create_for_data(bgra,art.cairo.FORMAT_ARGB32,512,512)
        c.save();c.translate(*pose['tip']);c.rotate(math.radians(pose['attitude']))
        c.scale(.5,.5);c.translate(-280,-430)
        c.set_source_surface(surface,0,0);c.paint();c.restore()
        if self.smooth_joint:
            # One native ink hinge is shared by every head view. Its center is
            # also the body's attachment; changing atlas drawings cannot swap
            # this joint's metal pattern or create a cross-faded double ring.
            features=self.feature_curve(pose['view_deg'])
            radii=np.max(np.abs(features[35:43]-features[34]),axis=0)
            c.save();c.translate(*pose['wrist']);c.rotate(math.radians(pose['attitude']))
            c.scale(radii[0]/69.,radii[1]/67.)
            art.cap(c,0,0,33,-16);c.restore()


class Character:
    def __init__(self,manifest,action,heads):
        accepted=ReferenceCharacter(manifest,action)
        self.original=OriginalCharacter();self.heads=HeadViews(smooth_joint=True) if action['number']=='08' else heads
        self.action=action
        self.body=BodyViews(self.heads,action['number'])
        self.gain=accepted.gain;self.poses=[]
        for previous in accepted.poses:
            p=dict(previous);p['view_deg']=40-p['yaw']
            p['wrist']=p['tip']+rotation(p['attitude'])@self.heads.wrist(p['view_deg'])
            p['elbow'],p['local_wrist']=solve_arm(p['wrist'],p['yaw'])
            self.poses.append(p)

    def draw(self,c,p):
        c.set_line_join(art.cairo.LINE_JOIN_ROUND);c.set_line_cap(art.cairo.LINE_CAP_ROUND)
        c.translate(34,38)
        self.body.draw(c,p)
        self.heads.draw(c,p)

    def render(self,i):
        p=self.poses[i]
        if np.max(np.abs(p['tip']-TIP))<1e-8 and abs(p['yaw'])<1e-8 and abs(p['attitude'])<1e-8:
            image=self.original.render([0.]*6)
        else:
            old=art.draw
            art.draw=lambda c,q,bg:self.draw(c,p)
            try:image=art.render([0.]*6,transparent=True)
            finally:art.draw=old
            pixels=np.array(image)
            shadow=(np.indices(pixels.shape[:2])[0]>543)&(pixels[:,:,3]<=31)
            pixels[shadow,:3]=[153,142,115];pixels[pixels[:,:,3]<3]=0
            image=Image.fromarray(pixels,'RGBA')
        if self.action['number']=='09':
            points=(np.array([p['tip'] for p in self.poses])+[34,38])*1.6
            stamp=self.action['frames'][i]['time_s'];duration=self.action['duration_s']
            fade=min(1.,max(0.,stamp/.35),max(0.,(duration-stamp)/.4))
            if fade:
                overlay=Image.new('RGBA',image.size);pen=ImageDraw.Draw(overlay);path=[tuple(v) for v in points]
                pen.line(path,fill=(37,155,181,round(85*fade)),width=2,joint='curve')
                if i:pen.line(path[:i+1],fill=(22,190,220,round(230*fade)),width=3,joint='curve')
                x,y=path[i];pen.ellipse((x-3,y-3,x+3,y+3),fill=(237,255,255,round(255*fade)),outline=(37,155,181,round(255*fade)))
                image=Image.alpha_composite(image,overlay)
        return image


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input',type=Path,default=ROOT/'build/dance-3d/motion.json')
    parser.add_argument('--stills-only',action='store_true')
    args=parser.parse_args();manifest=json.loads(args.input.read_text());heads=HeadViews()
    out=args.input.parent/'multiview';out.mkdir(exist_ok=True);records=[]
    sheet=Image.new('RGB',(1600,640),BG)
    for row,action in enumerate(manifest['actions']):
        character=Character(manifest,action,heads);code=action['number']
        yaws=np.array([p['yaw'] for p in character.poses]);ids=[0,int(yaws.argmin()),len(yaws)//2,int(yaws.argmax()),len(yaws)*3//4]
        frames=[];rgba=out/'rgba'/code
        if not args.stills_only:rgba.mkdir(parents=True,exist_ok=True)
        for i in sorted(set(ids)) if args.stills_only else range(len(yaws)):
            im=character.render(i);frames.append(im)
            if not args.stills_only:im.save(rgba/f'{i:03d}.png')
            if i in ids:
                bg=Image.new('RGBA',im.size,(*BG,255));bg.alpha_composite(im)
                bg.save(out/f'{code}-{i:03d}.png')
                for col,k in enumerate(ids):
                    if k==i:sheet.paste(bg.resize((320,320)),(col*320,row*320))
        if not args.stills_only:
            (out/'preview').mkdir(exist_ok=True)
            write_gif(frames,out/f'{code}.gif',action['frame_durations_ms'],transparent=True)
            write_gif(frames,out/'preview'/f'{code}.gif',action['frame_durations_ms'])
        records.append(dict(number=code,frames=len(yaws),duration_ms=sum(action['frame_durations_ms']),
                            projection_scale=character.gain,
                            body_poses=[{k:np.asarray(v).tolist() for k,v in character.body.landmarks(p).items()} for p in character.poses],
                            poses=[{k:(v.tolist() if isinstance(v,np.ndarray) else float(v)) for k,v in p.items()} for p in character.poses]))
        print(f'{code}: {len(frames)} rendered',flush=True)
    sheet.save(out/'angles.png')
    if not args.stills_only:
        data=dict(schema='rebuddy.dance_multiview.v2',method='Original head atlases; 08 uses C1 feature/hinge interpolation and a shared native ink wrist cover, with a fixed-length connected body rig. 09 unchanged. Stationary original base; no 3D rendering.',
                  motion_sha256=hashlib.sha256(args.input.read_bytes()).hexdigest(),
                  artwork_sha256=hashlib.sha256(Path(art.__file__).read_bytes()).hexdigest(),
                  body_artwork_sha256=hashlib.sha256((ROOT/'tools/dance_body_views.py').read_bytes()).hexdigest(),
                  atlas_sha256=heads.metadata['sheet_sha256'],actions=records)
        (out/'manifest.json').write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n')
        review_page(out,records)
        from render_dance_3d import gallery
        gallery(args.input.parent,manifest)


if __name__=='__main__':main()
