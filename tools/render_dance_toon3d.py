#!/usr/bin/env python3
"""Rigid cartoon shells on the real RS rig, projected by one VTK camera.

Every visible piece is a closed volume or a decal on that volume. No per-part
screen offsets, 2D IK, billboarding, animated scale or stretching. Offline only.
Run under Xvfb with the system Python (VTK, Cairo, NumPy, Pillow).
"""
import argparse
import hashlib
import json
import math
from pathlib import Path

import numpy as np
import vtk
from vtk.util.numpy_support import numpy_to_vtk, vtk_to_numpy
from PIL import Image

from render_dance_3d import Scene, ROOT, matrix
from render_dance_cartoon import BG, write_gif

GREEN = (.84, 1., .09)
DARK = (.19, .20, .18)
SILVER = (.86, .83, .75)
INK = (.065, .07, .06)
STYLE = {
    'render_style': 'original_artwork_unlit_illustration',
    'outline_width_m': .0032,
    # Approved projection stays fixed while only the decorative geometry changes.
    'vertical_fov_deg': 30.642073454187955,
    'head_center': [-.155, 0., .105],
    'head_radii': [.166, .139, .112],
    'face_source': 'artwork.face',
    'joint_source': 'artwork.cap',
    'base_radius': .180,
    'major_joint_radii': [.086, .080, .084],
}


def poly_mesh(points, triangles, uv=None):
    pts = vtk.vtkPoints()
    pts.SetData(numpy_to_vtk(np.asarray(points, dtype=np.float32), deep=True))
    cells = vtk.vtkCellArray()
    for triangle in triangles:
        cells.InsertNextCell(3)
        for index in triangle: cells.InsertCellPoint(int(index))
    poly = vtk.vtkPolyData(); poly.SetPoints(pts); poly.SetPolys(cells)
    if uv is not None:
        coords = numpy_to_vtk(np.asarray(uv, dtype=np.float32), deep=True)
        coords.SetNumberOfComponents(2); poly.GetPointData().SetTCoords(coords)
    normals = vtk.vtkPolyDataNormals(); normals.SetInputData(poly)
    normals.ConsistencyOn(); normals.AutoOrientNormalsOn(); normals.SplittingOff(); normals.Update()
    return normals.GetOutput()


def grid_mesh(fn, rows=32, columns=64, uv=False):
    points = []; coords = []; faces = []
    for j in range(rows+1):
        for i in range(columns+1):
            u, v = i/columns, j/rows
            points.append(fn(u, v)); coords.append([u, v])
    for j in range(rows):
        for i in range(columns):
            a = j*(columns+1)+i; b = a+columns+1
            faces.extend([(a, a+1, b+1), (a, b+1, b)])
    return poly_mesh(points, faces, coords if uv else None)


def ellipsoid(center, radii):
    sphere = vtk.vtkSphereSource(); sphere.SetThetaResolution(80); sphere.SetPhiResolution(48)
    sphere.SetRadius(1); sphere.Update()
    transform = vtk.vtkTransform(); transform.Translate(*center); transform.Scale(*radii)
    result = vtk.vtkTransformPolyDataFilter(); result.SetInputData(sphere.GetOutput())
    result.SetTransform(transform); result.Update()
    return result.GetOutput()


def brushed_texture():
    """Use the actual original joint artwork, including its nested rings."""
    import cairo
    size = 256
    surface = cairo.ImageSurface(cairo.FORMAT_ARGB32, size, size); c = cairo.Context(surface)
    c.translate(size/2, size/2); c.scale(size/62, size/62)
    import sys
    sys.path.insert(0, str(ROOT/'assets/images/source/dance'))
    import artwork as art
    art.color(c, '#e8dfcf'); c.paint()
    art.cap(c, 0, 0, 31, -14, back_offset=0)
    surface.flush()
    rgb = np.frombuffer(surface.get_data(), np.uint8).reshape(size, size, 4)[:, :, [2, 1, 0]].copy()
    data = vtk.vtkImageData(); data.SetDimensions(size, size, 1)
    data.GetPointData().SetScalars(numpy_to_vtk(np.flipud(rgb).reshape(-1, 3), deep=True))
    texture = vtk.vtkTexture(); texture.SetInputData(data); texture.InterpolateOn()
    return texture


def original_face_texture():
    """The SAME face paths as the other GIFs, not a newly arranged eye pair."""
    import cairo
    import sys
    sys.path.insert(0, str(ROOT/'assets/images/source/dance'))
    import artwork as art
    width, height = 684, 588
    surface = cairo.ImageSurface(cairo.FORMAT_ARGB32, width, height)
    c = cairo.Context(surface)
    c.scale(6, 6); c.translate(-178, -47)
    art.face(c)
    surface.flush()
    rgba = np.frombuffer(surface.get_data(), np.uint8).reshape(height, width, 4)[:, :, [2, 1, 0, 3]].copy()
    alpha = rgba[:, :, 3:4].astype(float)
    rgba[:, :, :3] = np.minimum(255, rgba[:, :, :3].astype(float)*255/np.maximum(1, alpha)).astype(np.uint8)
    # Paint the surrounding helmet with the same fixed pigments as its mesh,
    # avoiding both a rectangular decal seam and transparent-pass depth holes.
    u, v = np.meshgrid((np.arange(width)+.5)/width, 1-(np.arange(height)+.5)/height)
    ry, rz = STYLE['head_radii'][1:]
    y = (u-.5)*.248; z = (v-.5)*.166-.006
    x = np.sqrt(np.maximum(.001, 1-(y/ry)**2-(z/rz)**2))
    t = .72*(1-(z/rz+1)/2)+.28*(x+1)/2
    palette = np.array([[237,255,136], [223,255,35], [215,252,19], [178,211,12]])
    background = np.stack([np.interp(t, [0,.2,.7,1], palette[:,i]) for i in range(3)], axis=-1)
    rgb = np.rint(rgba[:,:,:3]*(alpha/255)+background*(1-alpha/255)).astype(np.uint8)
    data = vtk.vtkImageData(); data.SetDimensions(width, height, 1)
    data.GetPointData().SetScalars(numpy_to_vtk(np.flipud(rgb).reshape(-1, 3).copy(), deep=True))
    texture = vtk.vtkTexture(); texture.SetInputData(data); texture.InterpolateOn()
    return texture, Image.fromarray(rgba, 'RGBA')


def painted_fill(poly, family):
    """Fixed original illustration pigments, never computed from lights/normals."""
    result = vtk.vtkPolyData(); result.DeepCopy(poly)
    points = vtk_to_numpy(result.GetPoints().GetData())
    lo, hi = points.min(axis=0), points.max(axis=0)
    uv = (points-lo)/np.maximum(hi-lo, 1e-7)
    t = .72*(1-uv[:, 2])+.28*uv[:, 0]
    stops = ([0, .2, .7, 1], ['edff88', 'dfff23', 'd7fc13', 'b2d30c']) if family=='green' else ([0, .3, .75, 1], ['67615b', '363534', '272827', '494642'])
    palette = np.array([[int(s[i:i+2], 16) for i in (0, 2, 4)] for s in stops[1]])
    colors = np.column_stack([np.interp(t, stops[0], palette[:, i]) for i in range(3)]).astype(np.uint8)
    values = numpy_to_vtk(colors, deep=True); values.SetName('original_illustration_pigments')
    result.GetPointData().SetScalars(values)
    return result


class ToonScene(Scene):
    def __init__(self, manifest):
        self.outlines = []
        self.metal_texture = brushed_texture()
        super().__init__(manifest, 'cartoon', size=960)
        self.window.SetMultiSamples(0)
        self.renderer.SetUseFXAA(True)
        # Measure conservative bounds for diagnostics, but retain the approved
        # focal length while changing proportions. No automatic zoom or scaling.
        import itertools
        corners = []
        for actor, link, mount in self.bindings:
            b = actor.GetMapper().GetInput().GetBounds()
            vertices = np.array([(*p, 1.) for p in itertools.product(b[:2], b[2:4], b[4:6])])
            corners.append((link, vertices@mount.T))
        tangent = .46/2
        for action in manifest['actions']:
            cfg = action.get('camera', manifest['camera'])
            front = np.array(cfg['direction']); front = front/np.linalg.norm(front)
            right = np.cross(cfg['up'], front); right /= np.linalg.norm(right)
            up = np.cross(front, right); target = np.array(cfg['target'])
            for row in action['frames']:
                for link, vertices in corners:
                    world = (vertices@np.array(row['links'][link]).T)[:, :3]-target
                    depth = 2-world@front
                    tangent = max(tangent, float(np.max(np.abs(world@np.array([right, up]).T)/depth[:, None]))/.90)
        self.required_tangent = tangent
        self.vertical_tangent = math.tan(math.radians(STYLE['vertical_fov_deg'])/2)

    def add_poly(self, poly, color, renderer=None):
        actor = super().add_poly(poly, color, renderer)
        prop = actor.GetProperty()
        # Geometry provides motion and occlusion only. The displayed frame is
        # an inked, flat-colour illustration, without lit 3D surface shading.
        prop.LightingOff(); prop.SetAmbient(1.); prop.SetDiffuse(0.)
        prop.SetSpecular(0.)
        return actor

    def bind(self, poly, color, link, outline=True, texture=None):
        if texture is None and color in (GREEN, DARK):
            poly = painted_fill(poly, 'green' if color==GREEN else 'dark')
        actor = self.add_poly(poly, color)
        self.bindings.append((actor, link, np.eye(4)))
        if texture is not None:
            actor.SetTexture(texture); actor.GetProperty().SetAmbient(.8)
            actor.GetProperty().SetDiffuse(.2); actor.GetProperty().SetSpecular(0)
        if outline:
            # Inverted hull, expanded along mesh normals. It is a rigid mesh
            # in the SAME link coordinates, rendered by the SAME depth buffer.
            normals = vtk.vtkPolyDataNormals(); normals.SetInputData(poly)
            normals.SplittingOff(); normals.ConsistencyOn(); normals.AutoOrientNormalsOn(); normals.Update()
            hull = vtk.vtkPolyData(); hull.DeepCopy(normals.GetOutput())
            hull.GetPointData().SetScalars(None)
            points = hull.GetPoints(); n = hull.GetPointData().GetNormals()
            for i in range(points.GetNumberOfPoints()):
                points.SetPoint(i, np.array(points.GetPoint(i))+STYLE['outline_width_m']*np.array(n.GetTuple(i)))
            points.Modified()
            reverse = vtk.vtkReverseSense(); reverse.SetInputData(hull)
            reverse.ReverseCellsOn(); reverse.ReverseNormalsOn(); reverse.Update()
            ink = self.add_poly(reverse.GetOutput(), INK); ink.GetProperty().LightingOff()
            ink.GetProperty().BackfaceCullingOn()
            self.bindings.append((ink, link, np.eye(4)))
            self.outlines.append(ink)
        return actor

    def stroke_loop(self, points, link):
        pts = vtk.vtkPoints()
        for p in points: pts.InsertNextPoint(*p)
        lines = vtk.vtkCellArray(); lines.InsertNextCell(len(points)+1)
        for i in range(len(points)): lines.InsertCellPoint(i)
        lines.InsertCellPoint(0)
        poly = vtk.vtkPolyData(); poly.SetPoints(pts); poly.SetLines(lines)
        tube = vtk.vtkTubeFilter(); tube.SetInputData(poly)
        tube.SetRadius(.0016); tube.SetNumberOfSides(8); tube.Update()
        self.bind(tube.GetOutput(), INK, link, False)

    def solid_cylinder(self, start, end, radius, color, link='base_link', outline=True):
        start = np.array(start); end = np.array(end); axis = end-start
        direction = axis/np.linalg.norm(axis)
        seed = np.array([1., 0, 0]) if abs(direction[0])<.9 else np.array([0., 0, 1.])
        x = np.cross(direction, seed); x /= np.linalg.norm(x); z = np.cross(x, direction)
        transform = np.eye(4); transform[:3, :3] = np.column_stack([x, direction, z])
        transform[:3, 3] = (start+end)/2
        source = vtk.vtkCylinderSource(); source.SetResolution(80); source.SetRadius(radius)
        source.SetHeight(np.linalg.norm(axis)); source.CappingOn(); source.Update()
        local = vtk.vtkTransform(); local.SetMatrix(matrix(transform))
        filt = vtk.vtkTransformPolyDataFilter(); filt.SetInputData(source.GetOutput()); filt.SetTransform(local); filt.Update()
        return self.bind(filt.GetOutput(), color, link, outline)

    def disk(self, center, radius, sign, link):
        # Closed silver ring and concentric brushed-metal center.
        c = np.array(center)
        self.solid_cylinder(c, c+[0, 0, sign*.005], radius, SILVER, link)
        c += [0, 0, sign*.007]
        points = [c.tolist()]; uv = [[.5, .5]]; faces = []
        for i in range(97):
            a = i/96*math.tau
            points.append((c+radius*.96*np.array([math.cos(a), math.sin(a), 0])).tolist())
            uv.append([.5+.5*math.cos(a), .5+.5*math.sin(a)])
            if i: faces.append((0, i, i+1) if sign>0 else (0, i+1, i))
        self.bind(poly_mesh(points, faces, uv), (1, 1, 1), link, False, self.metal_texture)

    def add_shells(self):
        # Wide bevelled drum with vertical sides, not the previous thin saucer.
        radius = STYLE['base_radius']
        profile = [(0., -.012), (.155, -.012), (.169, -.008), (.178, .002),
                   (radius, .016), (radius, .061), (.177, .073),
                   (.168, .082), (.156, .085), (0., .085)]
        points = []; faces = []; segments = 96
        for r, z in profile:
            for i in range(segments+1):
                a = i/segments*math.tau
                points.append([r*math.cos(a), r*math.sin(a), z])
        for j in range(len(profile)-1):
            for i in range(segments):
                a = j*(segments+1)+i; b = a+segments+1
                faces.extend([(a, a+1, b+1), (a, b+1, b)])
        self.bind(poly_mesh(points, faces), DARK, 'base_link')
        self.solid_cylinder([0, 0, .080], [0, 0, .088], .165, SILVER)
        self.solid_cylinder([0, 0, .089], [0, 0, .097], .146, DARK)
        def panel(u, v):
            phi = -1.075+(u-.5)*1.15*(1-.08*abs(2*v-1)**10); z = -.005+v*.079
            r = float(np.interp(z, [p[1] for p in profile[1:-1]], [p[0] for p in profile[1:-1]]))+.0015
            return [r*math.cos(phi), r*math.sin(phi), z]
        self.bind(grid_mesh(panel), GREEN, 'base_link', False)
        def base_glint(u, v):
            phi = -1.03+u*.025; z = .018+v*.034
            return [(radius+.002)*math.cos(phi), (radius+.002)*math.sin(phi), z]
        self.bind(grid_mesh(base_glint, 24, 4), (1, 1, .86), 'base_link', False)
        self.solid_cylinder([0, 0, -.002], [.020343, .027237, .045], .032, DARK, 'link1')
        for link, end, radius in [('link2', [-.236, 0, 0], .046),
                                  ('link3', [.228, -.072746, .0045], .046),
                                  ('link4', [.087, -.048, -.03075], .028),
                                  ('link5', [.0365, 0, .048], .023)]:
            end = np.array(end); direction = end/np.linalg.norm(end)
            self.solid_cylinder(np.zeros(3), end, radius, DARK, link)
            # Fixed green curved face strip on the near (negative local Z) side.
            side = np.cross(direction, [0, 0, -1.]); side /= np.linalg.norm(side)
            near = np.cross(side, direction)
            def panel(u, v, end=end, side=side, near=near, r=radius):
                a = -.65+u*1.3
                return end*(.08+.84*v)+(r+.0015)*(near*math.cos(a)+side*math.sin(a))
            self.bind(grid_mesh(panel, 20, 16), GREEN, link, False)
            def glint(u, v, end=end, side=side, near=near, r=radius):
                a = -.52+u*.085
                return end*(.14+.72*v)+(r+.002)*(near*math.cos(a)+side*math.sin(a))
            self.bind(grid_mesh(glint, 16, 4), (.98, 1., .76), link, False)
        r2, r3, r4 = STYLE['major_joint_radii']
        for link, radius, depth in [('link2', r2, .047), ('link3', r3, .044),
                                    ('link4', r4, .105)]:
            self.solid_cylinder([0, 0, -depth], [0, 0, depth], radius, DARK, link)
            for sign in (-1, 1): self.disk([0., 0., sign*depth], radius*.94, sign, link)
        # Green shoulder shell leaves the front silver disk visible.
        def shoulder_panel(u, v):
            a = -1.9+u*2.; z = -.025+v*.070
            return [(r2+.0015)*math.cos(a), (r2+.0015)*math.sin(a), z]
        self.bind(grid_mesh(shoulder_panel), GREEN, 'link2', False)
        # Rounded mittens, green housings and dark closed tips; TCP lies at x=0.
        for side in (-1, 1):
            center = np.array([-.045, side*.026, -.002])
            self.bind(ellipsoid(center, [.052, .028, .039]), DARK, 'gripper_end')
            self.bind(ellipsoid(center+[-.023, 0, .018], [.036, .029, .031]), GREEN, 'gripper_end')
            self.bind(ellipsoid(center+[.020, 0, -.004], [.030, .026, .034]), DARK, 'gripper_end')
            self.bind(ellipsoid(center+[-.013, -.009, .045], [.008, .004, .003]), (1, 1, .88), 'gripper_end', False)

    def add_head(self):
        # Large helmet instead of the previous small ball. Physical wrist
        # joints are enclosed by this rigid decorative tool-frame shell.
        center = np.array(STYLE['head_center']); radii = np.array(STYLE['head_radii'])
        self.bind(ellipsoid(center, radii), GREEN, 'gripper_end')
        def crown(u, v):
            x = -.91+u*1.41; y = -.16+v*.32
            return center+(radii+.0012)*np.array([x, y, math.sqrt(max(0, 1-x*x-y*y))])
        self.bind(grid_mesh(crown), DARK, 'gripper_end', False)
        # Original head's curved cream paint mark (same Bezier control points).
        edges = []; start = np.array([201., 33.])
        for controls in [([211,29], [219,34], [223,44]),
                         ([222,46], [216,47], [211,48]),
                         ([207,46], [201,38], [201,33])]:
            a, b, end = map(np.array, controls)
            for t in np.linspace(0, 1, 18, endpoint=False):
                edges.append((1-t)**3*start+3*(1-t)**2*t*a+3*(1-t)*t*t*b+t**3*end)
            start = end
        def glint_point(p):
            x = .25+(p[0]-201)/22*.25; y = -.55+(p[1]-29)/19*.095
            return center+(radii+.002)*np.array([x, y, math.sqrt(1-x*x-y*y)])
        vertices = [glint_point(np.mean(edges, axis=0))]+[glint_point(p) for p in edges]
        triangles = [(0, i+1, (i+1)%len(edges)+1) for i in range(len(edges))]
        self.bind(poly_mesh(vertices, triangles), (1., 1., .86), 'gripper_end', False)
        # Front visor is curved in 3D and therefore gets foreshortened and
        # occluded naturally. UV features never turn independently to camera.
        points = []; uv = []; faces = []
        rings = 32; segments = 96
        for j in range(rings+1):
            r = j/rings
            for i in range(segments+1):
                a = math.tau*i/segments
                y = .124*r*math.copysign(abs(math.cos(a))**.82, math.cos(a))
                z = .083*r*math.copysign(abs(math.sin(a))**.82, math.sin(a))-.006
                x = radii[0]*math.sqrt(max(.001, 1-(y/radii[1])**2-(z/radii[2])**2))+.002
                points.append(center+[x, y, z]); uv.append([.5+y/.248, .5+(z+.006)/.166])
        for j in range(rings):
            for i in range(segments):
                a = j*(segments+1)+i; b = a+segments+1
                faces.extend([(a, b, b+1), (a, b+1, a+1)])
        texture, self.face_image = original_face_texture()
        self.bind(poly_mesh(points, faces, uv), (1, 1, 1), 'gripper_end', False, texture)
        # The actual enlarged link4 disk is the helmet hinge. No extra floating
        # ear disks: the original character has three prominent silver joints.

    def set_camera(self, cfg):
        super().set_camera(cfg)
        camera = self.renderer.GetActiveCamera()
        # A single mild perspective projection for all meshes and the trace.
        # No action-specific or per-piece camera compensation.
        camera.ParallelProjectionOff()
        camera.SetViewAngle(math.degrees(2*math.atan(self.vertical_tangent)))
        camera.SetClippingRange(.1, 5.)

    def set_trace(self, action, index):
        super().set_trace(action, index)
        # Annotation sits in the same world geometry and obeys occlusion.
        self.overlay.SetPreserveDepthBuffer(True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', type=Path, default=ROOT/'build/dance-3d/motion.json')
    parser.add_argument('--stills-only', action='store_true')
    args = parser.parse_args(); manifest = json.loads(args.input.read_text())
    folder = args.input.parent/'toon3d'; folder.mkdir(exist_ok=True)
    scene = ToonScene(manifest); records = []
    for action in manifest['actions']:
        scene.set_camera(action.get('camera', manifest['camera']))
        code = action['number']; frames = []
        samples = sorted(set([0, len(action['frames'])//4, len(action['frames'])*3//4,
                             int(np.argmin([f['q_rad'][0] for f in action['frames']])),
                             int(np.argmax([f['q_rad'][0] for f in action['frames']]))]))
        png_dir = folder/'rgba'/code
        if not args.stills_only: png_dir.mkdir(parents=True, exist_ok=True)
        for index in samples if args.stills_only else range(len(action['frames'])):
            scene.set_pose(action['frames'][index]); scene.set_trace(action, index)
            frame = scene.image().resize((640, 640), Image.Resampling.LANCZOS); frames.append(frame)
            if not args.stills_only: frame.save(png_dir/f'{index:03d}.png')
            if index in samples:
                bg = Image.new('RGBA', frame.size, BG+(255,)); bg.alpha_composite(frame)
                bg.save(folder/f'{code}-{index:03d}.png')
        if not args.stills_only:
            write_gif(frames, folder/f'{code}.gif', action['frame_durations_ms'], transparent=True)
            (folder/'preview').mkdir(exist_ok=True)
            write_gif(frames, folder/'preview'/f'{code}.gif', action['frame_durations_ms'])
            records.append(dict(number=code, frames=len(frames), duration_ms=sum(action['frame_durations_ms'])))
        print(code, len(frames), flush=True)
    scene.set_pose(manifest['actions'][0]['frames'][0]); scene.set_trace(manifest['actions'][0], 0)
    scene.face_image.save(folder/'face-original.png')
    scene.export_model(folder/'model.gltf')
    result = dict(schema='rebuddy.dance_toon3d.v1',
                  motion_sha256=hashlib.sha256(args.input.read_bytes()).hexdigest(),
                  renderer_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                  render_helpers_sha256=hashlib.sha256((ROOT/'tools/render_dance_3d.py').read_bytes()).hexdigest(),
                  artwork_sha256=hashlib.sha256((ROOT/'assets/images/source/dance/artwork.py').read_bytes()).hexdigest(),
                  method='Rigid meshes in URDF link coordinates; actor transform = exact link FK; one perspective camera and shared depth; fixed geometry; no per-part 2D anchor retargeting.',
                  display_style='Original artwork face and joint-cap paths; fixed illustrated pigments and ink contours; no material lighting or specular shading.',
                  artwork_functions=['artwork.face', 'artwork.cap'],
                  perspective=True, physical_motion_unchanged=True, bindings=len(scene.bindings),
                  appearance=STYLE,
                  fixed_vertical_fov_deg=math.degrees(2*math.atan(scene.vertical_tangent)),
                  conservative_bounds_fov_deg=math.degrees(2*math.atan(scene.required_tangent)),
                  outline_meshes=len(scene.outlines), actions=records)
    (folder/'manifest.json').write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
    scene.window.Finalize()
    if not args.stills_only:
        from render_dance_3d import gallery
        gallery(args.input.parent, manifest)


if __name__ == '__main__':
    main()
