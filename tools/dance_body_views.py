"""Hand-drawn body turn keys, matched to the old GitHub 2D turnaround.

All coordinates are illustration coordinates, not a mesh or a camera model.
The front drawing has two green shoulder cheeks and overlapping links. The
side drawing opens the zigzag and reveals the round silver joint covers.
Only these body drawings are interpolated; the approved head is untouched.
"""
import math

import numpy as np
from scipy.interpolate import PchipInterpolator

from render_dance_reference import art, TIP
from render_dance_cartoon import rotation, angle, difference


VIEWS = np.array([0., 20., 40., 60., 80.])
# Drawn landmarks for front, near-front, original, three-quarter, side views.
KEYS = {
    'shoulder_link': [[142, 210], [126, 210], [114, 210], [111, 210], [109, 210]],
    'elbow_link': [[185, 150], [125, 145], [75, 143], [69, 146], [68, 150]],
    'wrist_inset': [[44, 0], [18, 0], [0, 0], [-2, 0], [-3, 0]],
    'elbow_inset': [26, 13, 0, 0, 0],
    'cap_width': [.14, .58, 1., 1., 1.],
    'shoulder_cap': [[103, 210], [106, 210], [114, 210], [111, 210], [109, 210]],
    'frontness': [1., .48, 0., 0., 0.],
    'link_width': [20., 19., 17., 17., 17.],
}
CURVES = {key: PchipInterpolator(VIEWS, values, axis=0) for key, values in KEYS.items()}
# Drawing-plane foreshortening shared by both rigid links. The neutral view
# is exactly the original illustration; the frontal view retains hinge depth.
ARM_PLANE = PchipInterpolator(VIEWS, [.30, .55, 1., 1.08, 1.10])


def connected_arm(wrist, view):
    """Solve one fixed-length chain from the centered shoulder to the head.

    Both links use the same drawing-plane transform and the same elbow. This
    is a 2D character rig, not physical robot IK. There are no per-joint nudges.
    """
    scale = np.array([float(ARM_PLANE(view)), 1.])
    shoulder = np.array([142., 210.]) + (art.P0-[142.,210.])*scale
    target = (wrist-shoulder)/scale
    lower = float(np.linalg.norm(art.P1-art.P0))
    upper = float(np.linalg.norm(art.P2-art.P1))
    distance = float(np.linalg.norm(target))
    if not abs(lower-upper) < distance < lower+upper:
        raise ValueError('Accepted head is outside the fixed-length drawing rig')
    theta = math.atan2(target[1], target[0])-math.acos(
        np.clip((distance*distance+lower*lower-upper*upper)/(2*distance*lower),-1.,1.))
    elbow = shoulder + lower*np.array([math.cos(theta),math.sin(theta)])*scale
    return shoulder, elbow, scale


def mix_paths(a, b, t):
    """Inbetween corresponding ink-path control points, never image dissolve."""
    return [(x[0], * ((1-t)*np.array(x[1:])+t*np.array(y[1:]))) for x, y in zip(a, b)]


class BodyViews:
    def __init__(self, heads, action_number=None):
        self.heads = heads
        self.action_number = action_number

    def landmarks(self, pose):
        view = pose['view_deg']
        key = {name: curve(view) for name, curve in CURVES.items()}
        # The accepted head supplies the exact wrist attachment. Retarget the
        # two drawn links to that attachment, without flattening the whole arm.
        key['wrist'] = pose['wrist'] + rotation(pose['attitude']) @ key['wrist_inset']
        key['shoulder_offset'] = np.zeros(2)
        if self.action_number == '08':
            key['shoulder_link'],key['elbow'],key['arm_plane_scale'] = connected_arm(key['wrist'],view)
        else:
            nominal_wrist = TIP + self.heads.wrist(view)
            key['elbow'] = key['elbow_link'] + .55*(pose['wrist']-nominal_wrist)
        return key

    @staticmethod
    def link(c, start, end, key):
        front = float(key['frontness'])
        if front < 1e-8:
            art.arm(c, start, end, 17)
            return
        # A separately drawn frontal strip: dark middle with narrow green
        # cheeks. Its width survives the turn; it is not a squeezed side strip.
        vector = end-start
        length = float(np.linalg.norm(vector))
        w = float(key['link_width'])
        c.save(); c.translate(*start); c.rotate(math.atan2(vector[1],vector[0])+math.pi/2)
        right = w+15*(1-front)
        green_right = 7*(1-front)-9*front
        art.path(c,[('M',-w,0),('C',-w-2,-length*.3,-w-2,-length*.7,-w,-length),
                    ('C',-w,-length-12,right-3,-length-12,right-1,-length),
                    ('L',right,-5),('C',right,14,-w,14,-w,0),('Z',)],
                 art.linear(-w,-length,right,0,[(0,'#67615b'),(.3,'#363534'),(1,'#272827')]),width=1.8)
        art.path(c,[('M',-w+2,0),('L',-w+2,-length),
                    ('C',-w+1,-length-7,green_right-6,-length-11,green_right-1,-length+1),
                    ('L',green_right,-3),('C',green_right-2,10,-w+1,9,-w+2,0),('Z',)],
                 art.linear(-w,-length,8,0,[(0,'#f0ff7b'),(.2,'#d9ff1e'),(.8,'#d1f512'),(1,'#a6cd0a')]),width=1.1)
        art.path(c,[('M',-w+3,-length+6),('L',-w+3,-6),
                    ('L',-w+7-2*front,-4),('L',-w+7-2*front,-length+5),('Z',)],'#f6ffc6',None)
        art.path(c,[('M',green_right-5,-length+4),('L',green_right,-length+1),
                    ('L',green_right,-4),('L',green_right-5,-7),('Z',)],'#a5cb0b',None)
        art.path(c,[('M',12-14*front,-length+5),('L',right-5-4*front,-length+4),
                    ('L',right-4-5*front,-5),('L',16-16*front,-8),('Z',)],'#494642',None)
        # The opposite cheek becomes visible only in the front drawing.
        x = right-2
        art.path(c,[('M',x-8*front,-length+2),('L',x,-length+2),('L',x,-3),
                    ('L',x-8*front,-3),('Z',)],'#b8d91a',None)
        art.line(c,[('M',right-6,-length+4),('L',right-5,-7)],'#817b70',1.2)
        c.restore()

    @staticmethod
    def shoulder(c, key):
        f = float(key['frontness'])
        # Exact original shoulder at the approved three-quarter idle view.
        if f < 1e-8:
            delta = key['shoulder_cap']-art.P0
            c.save();c.translate(*delta);art.shoulder_shell(c)
            art.cap(c,*art.P0,31,-14,back_offset=8);c.restore()
            return
        # Rear edge appears on the opposite side in the front key.
        c.save();c.translate(180,210);c.scale(.14*f,1)
        art.cap(c,0,0,27,0,back_offset=0);c.restore()
        original=[('M',113,180),('C',131,173,159,174,177,179),
                  ('C',193,187,198,213,188,230),('C',176,236,137,240,116,234),('Z',)]
        front=[('M',108,180),('C',124,174,161,174,175,179),
               ('C',184,186,187,216,179,233),('C',160,237,126,237,107,233),('Z',)]
        art.path(c,mix_paths(original,front,f),
                 art.linear(130,179,171,235,[(0,'#53504a'),(.5,'#282a27'),(1,'#42413c')]),width=1.9)
        right=[('M',153,177),('C',164,177,174,177,180,185),
               ('C',193,199,195,213,187,228),('L',165,232),('C',175,211,173,192,153,177),('Z',)]
        front_right=[('M',158,177),('C',165,177,173,178,176,185),
                     ('C',182,199,183,217,178,231),('L',160,233),('C',164,213,162,191,158,177),('Z',)]
        art.path(c,mix_paths(right,front_right,f),
                 art.linear(153,180,189,229,[(0,'#f1ff7e'),(.35,'#dfff22'),(.75,'#d6f81b'),(1,'#adc90b')]),width=1.5)
        # Front-left green cheek is uncovered as the silver cover moves aside.
        left=[('M',113,180),('C',118,178,123,177,129,178),
              ('C',124,195,125,214,128,234),('L',109,233),
              ('C',104,215,104,191,113,180),('Z',)]
        narrow=[('M',113,180),('C',114,180,115,180,116,180),
                ('C',116,198,116,217,116,234),('L',113,234),
                ('C',112,216,112,192,113,180),('Z',)]
        art.path(c,mix_paths(narrow,left,f),
                 art.linear(108,180,130,235,[(0,'#f1ff7e'),(.35,'#dfff22'),(1,'#adc90b')]),width=1.1)
        art.path(c,[('M',171-7*f,183),('C',177-7*f,181,183-9*f,184,185-9*f,190),
                    ('L',177-9*f,190),('Z',)],art.CREAM,None)
        c.save();c.translate(*key['shoulder_cap']);c.scale(float(key['cap_width']),1)
        art.cap(c,0,0,31,-14*(1-f),back_offset=8*(1-f));c.restore()

    @staticmethod
    def elbow(c, key):
        front = float(key['frontness'])
        center = key['elbow']-np.array([float(key['elbow_inset']),0])
        if front > 1e-8:
            # Broad frontal hinge housing, not an infinitely thin silver disk.
            x,y=center; width=52*front
            art.path(c,[('M',x,y-26),('C',x+width*.3,y-30,x+width*.8,y-28,x+width,y-23),
                        ('C',x+width+5,y-8,x+width+5,y+12,x+width,y+24),
                        ('C',x+width*.7,y+29,x+width*.3,y+29,x,y+26),('Z',)],
                     art.linear(x,y-26,x+width,y+26,[(0,'#57544d'),(.4,'#30312d'),(1,'#4a4941')]),width=1.6)
            art.path(c,[('M',x+width*.65,y-27),('L',x+width*.9,y-25),
                        ('C',x+width+3,y-13,x+width+3,y+12,x+width*.9,y+25),
                        ('L',x+width*.65,y+27),('C',x+width*.8,y+10,x+width*.8,y-12,x+width*.65,y-27),('Z',)],
                     '#d7f523',None)
        forearm=difference(angle(key['wrist']-key['elbow']),angle(art.P2-art.P1))
        c.save();c.translate(*center);c.scale(float(key['cap_width']),1)
        art.cap(c,0,0,29,(forearm-14)*(1-front),back_offset=8*(1-front));c.restore()

    def draw(self, c, pose):
        key = self.landmarks(pose)
        art.base(c);art.turntable(c,pose['yaw'])
        self.link(c,key['shoulder_link'],key['elbow'],key)
        self.link(c,key['elbow'],key['wrist'],key)
        self.elbow(c,key)
        self.shoulder(c,key)
