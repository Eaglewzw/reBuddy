"""Closed cartoon surfaces, drawn with the original ink/paint conventions.

The reference turning GIF changes visible faces, not just the width of a
drawing. This renderer uses a depth buffer, cylindrical joints and a rounded
helmet/visor. It is a visual character rig, not the robot collision model.
Screen anchors are supplied by render_dance_reference; no robot I/O occurs.
"""
import math
import sys
from pathlib import Path

import numpy as np
from PIL import Image, ImageFilter

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'assets/images/source/dance'))
import artwork as art
import turn_renderer as gl
from OpenGL import GL as g

INK = '#171916'


def unit(v):
    v = np.asarray(v, dtype=float)
    return v/np.linalg.norm(v)


def yaw_matrix(degrees):
    a = math.radians(degrees)
    return np.array([[math.cos(a), 0, math.sin(a)], [0, 1, 0],
                     [-math.sin(a), 0, math.cos(a)]])


def roll_matrix(degrees):
    a = math.radians(degrees)
    return np.array([[math.cos(a), -math.sin(a), 0],
                     [math.sin(a), math.cos(a), 0], [0, 0, 1]])


class Mesh:
    def __init__(self, points, triangles, colors, uv=None, texture=None):
        indices = np.asarray(triangles).reshape(-1)
        self.points = np.asarray(points, np.float32)[indices]
        self.colors = np.asarray(colors, np.float32)[indices]
        self.uv = None if uv is None else np.asarray(uv, np.float32)[indices]
        self.texture = texture

    def draw(self, matrix=None, offset=None):
        vertices = self.points
        if matrix is not None: vertices = vertices@matrix.T
        if offset is not None: vertices = vertices+offset
        vertices = np.asarray(vertices, np.float32)
        g.glEnableClientState(g.GL_VERTEX_ARRAY)
        g.glVertexPointer(3, g.GL_FLOAT, 0, vertices)
        if self.texture is not None:
            g.glEnable(g.GL_TEXTURE_2D); g.glBindTexture(g.GL_TEXTURE_2D, self.texture)
            g.glEnableClientState(g.GL_TEXTURE_COORD_ARRAY)
            g.glTexCoordPointer(2, g.GL_FLOAT, 0, self.uv); g.glColor4f(1, 1, 1, 1)
        else:
            g.glEnableClientState(g.GL_COLOR_ARRAY)
            g.glColorPointer(3, g.GL_FLOAT, 0, self.colors)
        g.glDrawArrays(g.GL_TRIANGLES, 0, len(vertices))
        g.glDisableClientState(g.GL_COLOR_ARRAY)
        g.glDisableClientState(g.GL_TEXTURE_COORD_ARRAY)
        g.glDisableClientState(g.GL_VERTEX_ARRAY); g.glDisable(g.GL_TEXTURE_2D)


def surface(fn, paint, rows=32, cols=96, texture=None):
    points, colors, uv, faces = [], [], [], []
    for j in range(rows+1):
        for i in range(cols+1):
            u, v = i/cols, j/rows
            points.append(fn(u, v)); colors.append(paint(u, v)); uv.append([u, v])
    for j in range(rows):
        for i in range(cols):
            a = j*(cols+1)+i; b = a+cols+1
            faces.extend([(a, a+1, b+1), (a, b+1, b)])
    return Mesh(points, faces, colors, uv, texture)


def color(hex_color):
    return np.array(art.rgb(hex_color))


def pigment(v, green=True):
    stops = ['f1ff98', 'dfff23', 'b4d318'] if green else ['77716a', '353632', '252825']
    palette = np.array([art.rgb('#'+s) for s in stops])
    return np.array([np.interp(v, [0, .45, 1], palette[:, k]) for k in range(3)])


def ellipsoid(center, radii, green=True, crown=False, fill=None):
    center, radii = np.array(center), np.array(radii)
    def point(u, v):
        t, p = u*math.tau, v*math.pi
        result=center+radii*np.array([math.sin(p)*math.cos(t), -math.cos(p), math.sin(p)*math.sin(t)])
        if crown: result[2]=min(result[2],24.)
        return result
    def paint(u, v):
        p = point(u, v)-center
        # Paint is fixed to the shell: it turns away with the shell.
        return color(fill) if fill else pigment(.15+.75*v, green)
    return surface(point, paint)


def stroke(points, matrix, offset, width=1.5, ink=INK, closed=True):
    vertices = np.asarray(np.asarray(points)@matrix.T+offset, np.float32)
    g.glColor3f(*art.rgb(ink)); g.glLineWidth(width*gl.W/400)
    g.glEnableClientState(g.GL_VERTEX_ARRAY); g.glVertexPointer(3, g.GL_FLOAT, 0, vertices)
    g.glDrawArrays(g.GL_LINE_LOOP if closed else g.GL_LINE_STRIP, 0, len(vertices))
    g.glDisableClientState(g.GL_VERTEX_ARRAY)


class VolumeCharacter:
    def __init__(self):
        self.context = gl.init()
        self.joint_texture = gl.upload(gl.texture(self.joint_art))
        self.face_texture = gl.upload(gl.texture(self.face_art))
        # Local X is face left/right, Y is down, Z is toward the face.
        self.head = ellipsoid([0, -94, -29], [66, 64, 79], crown=True)
        def crown_point(u,v):
            x=-15+30*u; t=-1.5+2.65*v
            s=math.sqrt(1-(x/66)**2)
            return np.array([x,-94-64.5*s*math.cos(t),min(24.5,-29+79.5*s*math.sin(t))])
        self.crown=surface(crown_point,lambda u,v:pigment(.3+.5*v,False))
        self.crown_edges=[[crown_point(u,v) for v in np.linspace(0,1,100)] for u in (0,1)]
        self.glint=surface(lambda u,v:crown_point(.63+.30*u,.58+.14*v)+[0,-.3,.3],
                           lambda u,v:color('#fffddc'),8,12)
        points, uv, triangles = [], [], []
        for j in range(33):
            for i in range(97):
                r=j/32; t=i/96*math.tau
                u=.5+.5*r*math.copysign(abs(math.cos(t))**.7,math.cos(t))
                v=.5+.5*r*math.copysign(abs(math.sin(t))**.7,math.sin(t))
                points.append(self.visor_point(u,v));uv.append([u,v])
        for j in range(32):
            for i in range(96):
                a=j*97+i;b=a+97
                triangles.extend([(a,a+1,b+1),(a,b+1,b)])
        self.visor=Mesh(points,triangles,[(1,1,1)]*len(points),uv,self.face_texture)
        self.visor_edge = [self.visor_point(.5+.5*math.copysign(abs(math.cos(t))**.7,math.cos(t)),
                                               .5+.5*math.copysign(abs(math.sin(t))**.7,math.sin(t)))
                           for t in np.linspace(0, math.tau, 180, endpoint=False)]
        # Two separate, solid jaws. The far jaw can disappear behind the near.
        self.jaws = []
        for x in (-19,19):
            self.jaws.extend([ellipsoid([x,-27,5],[20,27,23],False),
                              ellipsoid([x,-38,9],[21,19,26]),
                              ellipsoid([x,-16,22],[18,17,16],False),
                              ellipsoid([x-4,-47,32],[3,4,1],fill='#fffddc')])
        self.normal = unit([.45,.13,.883])
        horizontal = unit([self.normal[2],0,-self.normal[0]])
        vertical = np.cross(self.normal, horizontal)
        self.head_basis = np.column_stack([horizontal,vertical,self.normal])
        self.joint_axis = unit([-.60, -.08, .80])

    @staticmethod
    def joint_art(c):
        c.translate(200,200); c.scale(5.8,5.8)
        art.cap(c,0,0,31,-14,back_offset=0)

    @staticmethod
    def face_art(c):
        art.color(c,'#1d231f'); c.paint()
        # Undo the reference drawing's eye-line shear before projecting it on
        # the visor. Thus the original face is not perspective-skewed twice.
        c.translate(200,200); c.scale(3.8,3.8)
        c.transform(art.cairo.Matrix(1,-.36,0,1,0,0));c.translate(-236,-101)
        art.face(c)

    @staticmethod
    def visor_point(u,v):
        x=(u-.5)*116; y=(v-.5)*94
        # Slightly bowed visor; the closed helmet behind it provides the side
        # silhouette. Alpha masking supplies rounded corners on the texture.
        z=43-7*(x/58)**2-6*(y/47)**2
        return np.array([x,-86+y,z])

    def cylinder(self, start, end, radius, green=False, cap=False):
        start,end=np.array(start),np.array(end); axis=unit(end-start)
        u=unit(np.cross(axis,[0,1,0] if abs(axis[1])<.9 else [1,0,0]));v=np.cross(axis,u)
        basis=np.column_stack([u,v,axis]); length=np.linalg.norm(end-start)
        ring=lambda t: np.array([radius*math.cos(t),radius*math.sin(t),0])
        mesh=surface(lambda a,b: ring(a*math.tau)+[0,0,length*b],
                     lambda a,b: pigment(.25+.65*(.5+.5*math.sin(a*math.tau)),green and (a>.72 or a<.12)),4,64)
        mesh.draw(basis,start)
        for z in (0,length):
            points=[np.array([0,0,z])];uv=[[.5,.5]];triangles=[]
            for i,t in enumerate(np.linspace(0,math.tau,65)):
                points.append(ring(t)+[0,0,z]);uv.append([.5+.5*math.cos(t),.5+.5*math.sin(t)])
                if i:triangles.append([0,i,i+1])
            disk=Mesh(points,triangles,[color('#373a32')]*len(points),uv,self.joint_texture if cap else None)
            disk.draw(basis,start)
            stroke([ring(t)+[0,0,z] for t in np.linspace(0,math.tau,96)],basis,start,1.4)

    def render(self, pose):
        g.glClearColor(0,0,0,0);g.glClear(g.GL_COLOR_BUFFER_BIT|g.GL_DEPTH_BUFFER_BIT)
        g.glMatrixMode(g.GL_PROJECTION);g.glLoadIdentity()
        g.glOrtho(-34,366,362,-38,-1000,1000)
        g.glMatrixMode(g.GL_MODELVIEW);g.glLoadIdentity()
        # Camera-space anchors retain the accepted projected motion. Their
        # depth comes from a yawing arm plane, so occlusion changes continuously.
        from render_dance_reference import turn_matrix
        yaw=pose['yaw'];t,o=turn_matrix(yaw);ry=yaw_matrix(yaw)
        def anchor(p):
            xy=t@p+o
            depth=-(p[0]-142)*math.sin(math.radians(yaw))
            return np.array([*xy,depth])
        shoulder=anchor(art.P0); elbow=anchor(pose['elbow']); wrist=np.array([*pose['wrist'],anchor(pose['local_wrist'])[2]])
        axis=ry@self.joint_axis
        for a,b in [(shoulder,elbow),(elbow,wrist)]:
            self.cylinder(a-axis*38,b-axis*38,22,True)
        for p,r in [(shoulder,31),(elbow,29),(wrist,33)]:
            self.cylinder(p-axis*34,p+axis*5,r,cap=True)
        # Shoulder housing remains round as its silver end turns edge-on.
        self.cylinder(shoulder-axis*38,shoulder-axis*4,33,True)
        basis=roll_matrix(pose['attitude'])@ry@self.head_basis
        tip=np.array([*pose['tip'],wrist[2]+12])-basis@np.array([0,1,22])
        # The original character wears its helmet at a slight jaunty tilt.
        head_basis=basis@roll_matrix(16)
        center=np.array([0,-94,-29]);head_origin=tip+basis@center-head_basis@center
        self.head.draw(head_basis,head_origin);self.crown.draw(head_basis,head_origin);self.glint.draw(head_basis,head_origin)
        for edge in self.crown_edges:stroke(edge,head_basis,head_origin,1.4,closed=False)
        self.visor.draw(head_basis,head_origin)
        stroke(self.visor_edge,head_basis,head_origin,1.8)
        for jaw in self.jaws:jaw.draw(basis,tip)
        g.glFinish()
        raw=g.glReadPixels(0,0,gl.W,gl.H,g.GL_RGBA,g.GL_UNSIGNED_BYTE)
        pixels=np.frombuffer(raw,np.uint8).reshape(gl.H,gl.W,4)[::-1].copy()
        alpha=pixels[:,:,3:4].astype(float)
        pixels[:,:,:3]=np.minimum(255,pixels[:,:,:3]*255./np.maximum(alpha,1)).astype(np.uint8)
        fg=Image.fromarray(pixels,'RGBA')
        outline=Image.new('RGBA',fg.size,(*[round(x*255) for x in art.rgb(INK)],255))
        outline.putalpha(fg.getchannel('A').filter(ImageFilter.MaxFilter(7)))
        # Original pedestal is the stationary part, as in the reference turn.
        surf=art.cairo.ImageSurface(art.cairo.FORMAT_ARGB32,gl.W,gl.H)
        c=art.cairo.Context(surf);c.scale(gl.W/400,gl.H/400);c.translate(34,38)
        art.base(c);art.turntable(c,yaw);surf.flush()
        bg=np.frombuffer(surf.get_data(),np.uint8).reshape(gl.H,gl.W,4)[:,:,[2,1,0,3]].copy()
        a=bg[:,:,3:4].astype(float);bg[:,:,:3]=np.minimum(255,bg[:,:,:3]*255./np.maximum(a,1)).astype(np.uint8)
        out=Image.fromarray(bg,'RGBA');out.alpha_composite(outline);out.alpha_composite(fg)
        return out.resize((640,640),Image.Resampling.LANCZOS)
