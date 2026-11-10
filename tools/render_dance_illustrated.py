#!/usr/bin/env python3
"""Bind the approved vector character to the verified 08/09 spatial motion.

This is a stylized skin, not CAD geometry: the head encloses the wrist joints.
Joint positions, tool rotation, timing and TCP trace come from motion.json.
No 2D IK or motor commands. The original artwork is imported unchanged.
"""
import argparse
import hashlib
import json
import math
import sys
from pathlib import Path

import cairo
import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'assets/images/source/dance'))
import artwork as art
from render_dance_cartoon import write_gif, BG

TIP = np.array([227., 180.])
SIZE = 640


def unit(v):
    return v/np.linalg.norm(v)


def affine(c, columns, position, anchor):
    offset = position-columns@anchor
    c.transform(cairo.Matrix(columns[0, 0], columns[1, 0],
                             columns[0, 1], columns[1, 1], *offset))


class Character:
    def __init__(self, manifest, action):
        self.action = action
        cfg = action.get('camera', manifest['camera'])
        front = unit(np.array(cfg['direction'], float))
        right = unit(np.cross(cfg['up'], front))
        up = np.cross(front, right)
        self.projection = np.array([right, -up])
        self.plane = self.projection.T
        self.rest = {k: np.array(v) for k, v in action['frames'][0]['links'].items()}
        self.scale = 360.
        self.origin = art.P0+np.array([34., 38.])
        self.shoulder0 = self.rest['link2'][:3, 3]
        # Keep the approved proportions in the shared rest pose. Retarget real
        # projected displacements to those drawing anchors; do not replace the
        # original large head with a CAD-sized sphere or solve a new 2D path.
        self.tip0 = TIP+np.array([34., 38.])
        self.elbow0 = art.P1+np.array([34., 38.])
        self.head_bind = np.eye(2)
        self.fit_scale = 1.
        self.fit_offset = np.zeros(2)

    def screen(self, p):
        return self.origin+self.projection@(p-self.shoulder0)*self.scale

    def pose(self, row):
        links = {k: np.array(v) for k, v in row['links'].items()}
        tool = links['gripper_end']
        relative = tool[:3, :3]@self.rest['gripper_end'][:3, :3].T
        head = self.projection@relative@self.plane@self.head_bind
        tip = self.tip0+self.projection@(tool[:3, 3]-self.rest['gripper_end'][:3, 3])*self.scale
        wrist = tip+head@(art.P2-TIP)
        return links, head, tip, wrist

    def draw(self, c, row):
        links, head, tip, wrist = self.pose(row)
        shoulder = self.screen(links['link2'][:3, 3])
        elbow = self.elbow0+self.projection@(links['link3'][:3, 3]-self.rest['link3'][:3, 3])*self.scale
        c.set_line_join(cairo.LINE_JOIN_ROUND)
        c.set_line_cap(cairo.LINE_CAP_ROUND)
        # Original pedestal, including green front panel and brushed top ring.
        c.save(); c.translate(*(self.origin-art.P0)); art.base(c)
        art.turntable(c, math.degrees(row['q_rad'][0])); c.restore()
        art.arm(c, shoulder, elbow, 17.)
        art.arm(c, elbow, wrist, 17.)
        c.save(); affine(c, head, tip, TIP)
        art.head(c); art.face(c); art.gripper(c, 0.)
        art.cap(c, *art.P2, 33, -16)
        c.restore()
        # Disks remain rigid on their real link frames: yaw foreshortening
        # therefore follows the mechanism instead of turning the whole image.
        for link, p, radius in [('link3', elbow, 29.), ('link2', shoulder, 31.)]:
            relative = links[link][:3, :3]@self.rest[link][:3, :3].T
            plane = self.projection@relative@self.plane
            c.save(); affine(c, plane, p, np.zeros(2))
            if link == 'link2':
                c.save(); c.translate(*(-art.P0)); art.shoulder_shell(c); c.restore()
            art.cap(c, 0, 0, radius, -14); c.restore()

    def trace(self, c, index):
        if self.action['number'] != '09':
            return
        row = self.action['frames'][index]
        t = row['time_s']; duration = self.action['duration_s']
        fade = min(1., max(0., t/.3), max(0., (duration-t)/.3))
        if not fade:
            return
        points = [self.pose(f)[2] for f in self.action['frames']]
        for values, alpha, width in [(points, .35, 1.2), (points[:index+1], .95, 1.8)]:
            if len(values) < 2:
                continue
            c.new_path(); c.move_to(*values[0])
            for p in values[1:]: c.line_to(*p)
            c.set_source_rgba(.05, .65, .84, alpha*fade); c.set_line_width(width); c.stroke()
        p = points[index]
        art.ellipse(c, *p, 2.4, 2.4, '#f1ffff', '#27bddd', .8)

    def render(self, index, trace=True, transparent=False):
        surface = cairo.ImageSurface(cairo.FORMAT_ARGB32, SIZE*2, SIZE*2)
        c = cairo.Context(surface); c.scale(SIZE*2/400, SIZE*2/400)
        if not transparent:
            art.color(c, '#fffdf5'); c.paint()
        c.translate(*self.fit_offset); c.scale(self.fit_scale, self.fit_scale)
        self.draw(c, self.action['frames'][index])
        if trace: self.trace(c, index)
        surface.flush()
        raw = np.frombuffer(surface.get_data(), np.uint8).reshape(SIZE*2, SIZE*2, 4)
        rgba = raw[:, :, [2, 1, 0, 3]].copy()
        if transparent:
            alpha = rgba[:, :, 3:4].astype(float)
            rgba[:, :, :3] = np.minimum(255, rgba[:, :, :3].astype(float)*255/np.maximum(alpha, 1)).astype(np.uint8)
        return Image.fromarray(rgba, 'RGBA').resize((SIZE, SIZE), Image.Resampling.LANCZOS)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', type=Path, default=ROOT/'build/dance-3d/motion.json')
    parser.add_argument('--stills-only', action='store_true')
    args = parser.parse_args()
    manifest = json.loads(args.input.read_text())
    folder = args.input.parent/'illustrated'; folder.mkdir(exist_ok=True)
    skins = [Character(manifest, action) for action in manifest['actions']]
    bounds = []
    for skin in skins:
        for row in skin.action['frames']:
            surface = cairo.RecordingSurface(cairo.CONTENT_COLOR_ALPHA, None)
            skin.draw(cairo.Context(surface), row)
            x, y, w, h = surface.ink_extents(); bounds.append([x, y, x+w, y+h])
    bounds = np.array(bounds); lo = bounds[:, :2].min(axis=0); hi = bounds[:, 2:].max(axis=0)
    scale = min(1., float(np.min(360/(hi-lo))))
    offset = np.array([200., 200.])-(lo+hi)*scale/2
    records = []
    for skin in skins:
        skin.fit_scale = scale; skin.fit_offset = offset
        action = skin.action; code = action['number']
        representative = sorted(set([0, len(action['frames'])//4, len(action['frames'])*3//4,
                                    int(np.argmin([f['q_rad'][0] for f in action['frames']])),
                                    int(np.argmax([f['q_rad'][0] for f in action['frames']]))]))
        indices = representative if args.stills_only else range(len(action['frames']))
        frames = []
        png_dir = folder/'rgba'/code
        if not args.stills_only: png_dir.mkdir(parents=True, exist_ok=True)
        for index in indices:
            frame = skin.render(index, transparent=True); frames.append(frame)
            if not args.stills_only: frame.save(png_dir/f'{index:03d}.png')
            if index in representative:
                base = Image.new('RGBA', frame.size, BG+(255,)); base.alpha_composite(frame)
                base.save(folder/f'{code}-{index:03d}.png')
        if not args.stills_only:
            write_gif(frames, folder/f'{code}.gif', action['frame_durations_ms'], transparent=True)
            (folder/'preview').mkdir(exist_ok=True)
            write_gif(frames, folder/'preview'/f'{code}.gif', action['frame_durations_ms'])
            records.append(dict(number=code, frames=len(frames), duration_ms=sum(action['frame_durations_ms']),
                                gif_sha256=hashlib.sha256((folder/f'{code}.gif').read_bytes()).hexdigest()))
        print(code, len(frames), flush=True)
    if not args.stills_only:
        result = dict(schema='rebuddy.dance_illustrated.v1',
                      motion_sha256=hashlib.sha256(args.input.read_bytes()).hexdigest(),
                      artwork_sha256=hashlib.sha256(Path(art.__file__).read_bytes()).hexdigest(),
                      renderer_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                      physical_motion_unchanged=True, frame_size=[SIZE, SIZE],
                      pixels_per_metre=skin.scale*scale*SIZE/400,
                      fit_scale=scale, fit_offset=offset.tolist(),
                      method='Original vector artwork; shoulder/elbow/TCP projected displacements retargeted to original rest anchors. Head and joint-cover planes use actual relative link rotations. Arm shells join those anchors; cartoon proportions are not CAD geometry.',
                      actions=records)
        (folder/'manifest.json').write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
        from render_dance_3d import gallery
        gallery(args.input.parent, manifest)


if __name__ == '__main__':
    main()
