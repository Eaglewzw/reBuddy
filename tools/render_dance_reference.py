#!/usr/bin/env python3
"""Animate the complete approved drawing, using the spatial motion as a guide.

The original two rigid arm lengths, head, eyes, jaws and pedestal are retained.
One per-clip scale fits the projected TCP curve to this illustration; this is
character retargeting, not a CAD image. No frame-wise limb stretching or CAN.
"""
import argparse
import hashlib
import json
import math
import sys
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

from render_dance_cartoon import ROOT, OriginalCharacter, rotation, angle, difference, write_gif, BG

sys.path.insert(0, str(ROOT/'assets/images/source/dance'))
import artwork as art

TIP = np.array([227., 185.])


def turn_matrix(yaw):
    """Exactly the perspective transform in the original artwork.project_turn."""
    a0 = math.radians(20); a = a0+math.radians(yaw)
    f = math.cos(a)/math.cos(a0)
    h = .32*(math.sin(a)-math.sin(a0))/math.cos(a0)
    return np.array([[f, 0.], [h, 1.]]), np.array([142*(1-f), -142*h])


class BodyVolume:
    """Original ink surfaces on thick beams/cylinders, in one turntable view."""
    def __init__(self, yaw):
        self.yaw = yaw
        self.transform, self.offset = turn_matrix(yaw)
        a0 = math.radians(20); a = a0+math.radians(yaw)
        # Physical depth projection, not a difference from the reference yaw.
        # At rest a 28-unit barrel has the original ~8-unit rear offset.
        self.depth_axis = np.array([math.sin(a), .25-.32*math.cos(a)])
        self.width = math.sqrt(math.cos(a)**2+(.85*math.sin(a))**2)/math.sqrt(math.cos(a0)**2+(.85*math.sin(a0))**2)
        u = min(abs(yaw)/6., 1.)
        self.reveal = u*u*(3-2*u)

    def point(self, p, depth=0):
        return self.transform@np.array(p)+self.offset+self.depth_axis*depth

    def beam(self, c, p, end, radius=17):
        # Two long side walls and closed ends; depths are in the original
        # drawing's units, fixed over the entire clip.
        v = end-p; n = np.array([-v[1], v[0]])/np.linalg.norm(v)
        corners = [p-n*radius, end-n*radius, end+n*(radius+15), p+n*(radius+15)]
        faces = []
        for i in range(4):
            a,b = corners[i], corners[(i+1)%4]
            polygon = np.array([self.point(a,28),self.point(b,28),self.point(b),self.point(a)])
            faces.append(polygon)
        bounds=np.concatenate(faces);lo=bounds.min(axis=0)-2;hi=bounds.max(axis=0)+2
        c.save();c.rectangle(*lo,*(hi-lo));c.clip();c.push_group()
        for polygon in faces:
            art.path(c,[('M',*polygon[0])]+[('L',*p) for p in polygon[1:]]+[('Z',)],
                     art.linear(*polygon.min(axis=0),*polygon.max(axis=0),[(0,'#747064'),(.4,'#46473d'),(1,'#272c25')]),width=1.5)
        c.pop_group_to_source(); c.paint_with_alpha(self.reveal);c.restore()
        c.save(); art.project_turn(c,self.yaw); art.arm(c,p,end,radius); c.restore()

    def barrel(self, c, p, radius, angle):
        from scipy.spatial import ConvexHull
        points = []
        for depth in (0, 28):
            center = self.point(p,depth)
            for theta in np.linspace(0,math.tau,72,endpoint=False):
                off = rotation(angle)@np.array([(radius+2)*math.cos(theta),(radius+1)*math.sin(theta)])
                points.append(center+self.transform@off)
        points=np.array(points);hull=points[ConvexHull(points).vertices]
        lo=hull.min(axis=0)-2;hi=hull.max(axis=0)+2
        c.save();c.rectangle(*lo,*(hi-lo));c.clip();c.push_group()
        art.path(c,[('M',*hull[0])]+[('L',*x) for x in hull[1:]]+[('Z',)],
                 art.linear(*hull.min(axis=0),*hull.max(axis=0),[(0,'#747569'),(.3,'#41463b'),(.7,'#282d27'),(1,'#5b6051')]),width=1.6)
        c.pop_group_to_source(); c.paint_with_alpha(self.reveal);c.restore()

    def cap(self, c, p, radius, angle):
        front=self.point(p)
        c.save();c.translate(*front)
        # Face foreshortening and the cylinder shell are separate: only the
        # silver circular end becomes an ellipse, not the whole joint body.
        c.transform(art.cairo.Matrix(self.transform[0,0],self.transform[1,0],0,1,0,0))
        original_back=(8+24*math.sin(math.radians(self.yaw)))*(1-self.reveal)
        art.cap(c,0,0,radius,angle,back_offset=original_back)
        c.restore()

    def shoulder(self, c):
        center=np.array([151.,208.]);new=self.point(center)
        c.save();c.translate(*new);c.scale(self.width,1);c.translate(*(-center))
        art.shoulder_shell(c);c.restore()
        self.cap(c,art.P0,31,-14)


def solve_arm(wrist, yaw):
    transform, offset = turn_matrix(yaw)
    local = np.linalg.solve(transform, wrist-offset)
    d = local-art.P0
    l1 = np.linalg.norm(art.P1-art.P0); l2 = np.linalg.norm(art.P2-art.P1)
    r = np.linalg.norm(d)
    if not abs(l1-l2)+1 < r < l1+l2-1:
        raise ValueError('Projected wrist outside original rigid arm reach')
    a = math.atan2(d[1], d[0])-math.acos(np.clip((r*r+l1*l1-l2*l2)/(2*r*l1), -1, 1))
    elbow = art.P0+l1*np.array([math.cos(a), math.sin(a)])
    return elbow, local


def projected_targets(manifest, action):
    cfg = action.get('camera', manifest['camera'])
    front = np.array(cfg['direction']); front /= np.linalg.norm(front)
    right = np.cross(cfg['up'], front); right /= np.linalg.norm(right)
    up = np.cross(front, right); camera_target = np.array(cfg['target'])
    focal = 200/math.tan(math.radians(30.642073454187955)/2)
    def project(p):
        p = p-camera_target
        return np.array([p@right, -p@up])*focal/(2-p@front)
    points = []; attitudes = []
    for row in action['frames']:
        tool = np.array(row['links']['gripper_end'])
        p = tool[:3, 3]
        points.append(project(p))
        attitudes.append(angle(project(p+tool[:3, 2]*.08)-project(p)))
    points = np.array(points); attitudes = np.unwrap(np.radians(attitudes))
    # World tool rotation, rather than the perspective direction to a translated
    # point, controls the head: compensation in 08 must not introduce head roll.
    if action['number']=='08': attitudes = np.zeros_like(attitudes)
    return points-points[0], np.degrees(attitudes-attitudes[0])


class ReferenceCharacter:
    def __init__(self, manifest, action):
        self.action = action
        self.original = OriginalCharacter()
        self.delta, self.angles = projected_targets(manifest, action)
        yaw0 = action['frames'][0]['q_rad'][0]
        self.yaws = np.array([math.degrees(f['q_rad'][0]-yaw0) for f in action['frames']])
        self.gain = 1.
        while True:
            try:
                self.poses = [self.pose(i) for i in range(len(action['frames']))]
                # A single scale for the ENTIRE path, never per-frame clamps.
                for index in range(len(self.poses)):
                    surface = art.cairo.RecordingSurface(art.cairo.CONTENT_COLOR_ALPHA, None)
                    self.draw(art.cairo.Context(surface), self.poses[index])
                    x, y, w, h = surface.ink_extents()
                    if x < 10 or y < 10 or x+w > 390 or y+h > 390:
                        raise ValueError('Original character would be clipped')
                break
            except ValueError:
                self.gain *= .96
                if self.gain < .2: raise

    def pose(self, index):
        tip = TIP+self.delta[index]*self.gain
        attitude = self.angles[index]*self.gain
        wrist = tip-rotation(attitude)@(TIP-art.P2)
        elbow, local_wrist = solve_arm(wrist, self.yaws[index])
        return dict(tip=tip, attitude=attitude, wrist=wrist, elbow=elbow,
                    local_wrist=local_wrist, yaw=self.yaws[index])

    def draw(self, c, p):
        c.set_line_join(art.cairo.LINE_JOIN_ROUND); c.set_line_cap(art.cairo.LINE_CAP_ROUND)
        c.translate(34, 38)
        yaw = p['yaw']; theta = p['attitude']; elbow = p['elbow']; wrist = p['local_wrist']
        art.base(c); art.turntable(c, yaw)
        volume = BodyVolume(yaw)
        forearm = difference(angle(wrist-elbow), angle(art.P2-art.P1))
        for point,radius,orientation in [(art.P0,31,-14),(elbow,29,forearm-14),(wrist,33,theta-16)]:
            volume.barrel(c,point,radius,orientation)
        volume.beam(c,art.P0,elbow);volume.beam(c,elbow,wrist)
        # Entire ORIGINAL head and face, without remeshing or texture wrapping.
        c.save(); art.at(c, art.P2, p['wrist'], theta)
        art.head(c); art.face(c); c.restore()
        palm = p['tip']-rotation(theta)@(TIP-art.PC)
        c.save(); art.at(c, art.PC, palm, theta); art.gripper(c, 0); c.restore()
        volume.cap(c,elbow,29,forearm-14)
        c.save(); art.at(c, art.P2, p['wrist'], theta)
        art.cap(c, *art.P2, 33, -16); c.restore()
        volume.shoulder(c)

    def render(self, index):
        pose = self.poses[index]
        if np.max(abs(pose['tip']-TIP)) < 1e-8 and abs(pose['attitude']) < 1e-8 and abs(pose['yaw']) < 1e-8:
            image = self.original.render([0.]*6)
        else:
            original_draw = art.draw
            def draw(c, q, bg):
                self.draw(c, pose)
            art.draw = draw
            try: image = art.render([0.]*6, transparent=True)
            finally: art.draw = original_draw
            pixels = np.array(image)
            shadow = (np.indices(pixels.shape[:2])[0]>543)&(pixels[:,:,3]<=31)
            pixels[shadow,:3] = [153,142,115]; pixels[pixels[:,:,3]<3] = 0
            image = Image.fromarray(pixels, 'RGBA')
        if self.action['number']=='09':
            points = (np.array([p['tip'] for p in self.poses])+[34,38])*1.6
            row = self.action['frames'][index]
            fade = min(1., max(0., row['time_s']/.35), max(0., (self.action['duration_s']-row['time_s'])/.4))
            if fade>0:
                overlay = Image.new('RGBA', image.size); pen = ImageDraw.Draw(overlay)
                pts = [tuple(p) for p in points]
                pen.line(pts, fill=(37,155,181,round(85*fade)), width=2, joint='curve')
                if index: pen.line(pts[:index+1], fill=(22,190,220,round(230*fade)), width=3, joint='curve')
                x,y=pts[index]
                pen.ellipse((x-3,y-3,x+3,y+3),fill=(237,255,255,round(255*fade)),outline=(37,155,181,round(255*fade)))
                image = Image.alpha_composite(image, overlay)
        return image


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', type=Path, default=ROOT/'build/dance-3d/motion.json')
    parser.add_argument('--stills-only', action='store_true')
    args = parser.parse_args(); manifest = json.loads(args.input.read_text())
    out = args.input.parent/'reference'; out.mkdir(exist_ok=True)
    records = []
    for action in manifest['actions']:
        character = ReferenceCharacter(manifest, action); code = action['number']
        samples = sorted(set([0, len(action['frames'])//4, len(action['frames'])*3//4,
                              int(np.argmin(character.yaws)), int(np.argmax(character.yaws))]))
        frames = []; rgba = out/'rgba'/code
        if not args.stills_only: rgba.mkdir(parents=True, exist_ok=True)
        for i in samples if args.stills_only else range(len(action['frames'])):
            image = character.render(i); frames.append(image)
            if not args.stills_only: image.save(rgba/f'{i:03d}.png')
            if i in samples:
                bg = Image.new('RGBA', image.size, BG+(255,)); bg.alpha_composite(image)
                bg.save(out/f'{code}-{i:03d}.png')
        if not args.stills_only:
            (out/'preview').mkdir(exist_ok=True)
            write_gif(frames, out/f'{code}.gif', action['frame_durations_ms'], transparent=True)
            write_gif(frames, out/'preview'/f'{code}.gif', action['frame_durations_ms'])
        records.append(dict(number=code, frames=len(action['frames']), duration_ms=sum(action['frame_durations_ms']),
                            projection_scale=character.gain,
                            poses=[{k:(v.tolist() if isinstance(v,np.ndarray) else float(v)) for k,v in p.items()} for p in character.poses]))
        print(code, 'scale', character.gain, 'frames', len(frames), flush=True)
    if not args.stills_only:
        data = dict(schema='rebuddy.dance_reference.v1', motion_sha256=hashlib.sha256(args.input.read_bytes()).hexdigest(),
                    artwork_sha256=hashlib.sha256(Path(art.__file__).read_bytes()).hexdigest(),
                    renderer_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                    source='Complete unchanged original vector character, identical rest image to other Dance GIFs',
                    method='Original rigid two-link character IK, closed beam side walls and joint-cylinder shells with original ink faces. Current perspective-projected TCP curve fitted with one constant per-clip scale; original body yaw and stabilized head. Not a CAD geometric render.',
                    actions=records)
        (out/'manifest.json').write_text(json.dumps(data, ensure_ascii=False, indent=2)+'\n')
        from render_dance_3d import gallery
        gallery(args.input.parent, manifest)


if __name__ == '__main__': main()
