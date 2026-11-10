"""Complete hand-authored 2D vector character and rigid articulation.
Every frame is drawn from paths. No reference image pixels, textures, crops or image rotations.
"""
import cairo,math,pathlib,json,sys
import numpy as np
from scipy.interpolate import make_interp_spline
from PIL import Image
R=pathlib.Path(__file__).resolve().parent.parent
SIZE=640;VIEW=400
OUTLINE='#171916';GREEN='#d7ff20';CREAM='#fffdf0'
def rgb(s):return tuple(int(s[i:i+2],16)/255 for i in (1,3,5))
def color(c,s,a=1):c.set_source_rgba(*rgb(s),a)
def linear(x1,y1,x2,y2,stops):
 g=cairo.LinearGradient(x1,y1,x2,y2)
 for t,s in stops:g.add_color_stop_rgb(t,*rgb(s))
 return g
def radial(x,y,r,stops):
 g=cairo.RadialGradient(x,y,0,x,y,r)
 for t,s,a in stops:g.add_color_stop_rgba(t,*rgb(s),a)
 return g
def paint(c,fill,stroke=OUTLINE,width=1.7):
 if isinstance(fill,str):color(c,fill)
 else:c.set_source(fill)
 if stroke:
  c.fill_preserve();color(c,stroke);c.set_line_width(width);c.stroke()
 else:c.fill()
def path(c,cmds,fill,stroke=OUTLINE,width=1.7):
 c.new_path()
 for cmd in cmds:
  op,*v=cmd
  if op=='M':c.move_to(*v)
  elif op=='L':c.line_to(*v)
  elif op=='C':c.curve_to(*v)
  elif op=='Q':raise ValueError('Use cubic curves')
  elif op=='Z':c.close_path()
 paint(c,fill,stroke,width)
def ellipse(c,x,y,rx,ry,fill,stroke=None,width=1):
 c.save();c.translate(x,y);c.scale(rx,ry);c.arc(0,0,1,0,math.tau);c.restore();paint(c,fill,stroke,width)
def line(c,cmds,s,width):
 c.new_path()
 for op,*v in cmds:
  if op=='M':c.move_to(*v)
  elif op=='L':c.line_to(*v)
  elif op=='C':c.curve_to(*v)
 color(c,s);c.set_line_width(width);c.stroke()
def rotation(a):
 a=math.radians(a);return np.array([[math.cos(a),-math.sin(a)],[math.sin(a),math.cos(a)]])
def at(c,pivot,target,ang):
 c.translate(*target);c.rotate(math.radians(ang));c.translate(-pivot[0],-pivot[1])

P0=np.array([114.,210.]);P1=np.array([75.,143.]);P2=np.array([139.,74.]);PC=np.array([227.,143.])

def base(c):
 ellipse(c,143,298,88,8,radial(143,296,93,[(0,'#998e73',.12),(1,'#998e73',0)]))
 path(c,[('M',55,251),('C',56,237,73,228,101,225),('C',134,221,181,221,207,233),('C',223,240,229,252,229,277),('C',231,291,204,298,164,300),('C',123,302,72,298,57,287),('C',52,283,52,265,55,251),('Z',)],linear(62,237,215,298,[(0,'#6b6662'),(.35,'#42413e'),(.75,'#353534'),(1,'#55514c')]),width=2.1)
 path(c,[('M',61,251),('C',61,271,59,280,65,286),('C',76,294,102,294,114,296),('L',102,244),('Z',)],'#343432',None)
 path(c,[('M',205,245),('C',216,251,219,269,218,281),('C',218,285,214,289,207,291),('C',222,288,225,283,224,277),('C',225,259,221,248,213,244),('Z',)],'#716b64',None)
 ellipse(c,142,242,80,17,linear(143,227,143,259,[(0,'#d2c6b7'),(.4,'#aaa093'),(.55,'#6c655f'),(1,'#57534e')]),OUTLINE,1.8)
 ellipse(c,142,239,65,12,'#383936',OUTLINE,1.4)
 path(c,[('M',90,226),('C',106,219,176,218,192,226),('L',200,239),('C',190,249,101,248,85,240),('Z',)],linear(100,224,175,246,[(0,'#746e65'),(.4,'#403e3b'),(1,'#494641')]),width=1.7)
 line(c,[('M',94,229),('C',114,224,169,224,187,229)],'#b5ab9e',1.2)
 path(c,[('M',76,230),('C',65,235,59,240,62,245),('C',67,251,106,252,123,257),('C',133,261,132,277,131,289),('C',131,295,129,298,124,298),('C',104,297,78,296,73,290),('C',69,285,69,266,72,255),('C',62,253,59,251,58,247),('C',59,240,65,235,76,230),('Z',)],linear(80,236,125,297,[(0,'#efff82'),(.28,'#dfff30'),(.75,'#cefa0d'),(1,'#a8cd10')]),width=1.6)
 path(c,[('M',64,243),('C',74,250,107,253,126,258),('L',127,261),('C',107,258,75,255,64,249),('Z',)],'#faffb1',None)
 path(c,[('M',77,289),('C',91,292,116,294,129,292),('L',127,296),('C',105,295,86,295,77,291),('Z',)],'#a5c80b',None)
 ellipse(c,111,274,2.5,10,CREAM)
 path(c,[('M',118,254),('C',118,248,125,251,127,257),('L',119,256),('Z',)],CREAM,None)

def arm(c,p,endpoint,w):
 v=endpoint-p;length=float(np.linalg.norm(v));angle=math.atan2(v[1],v[0])+math.pi/2
 c.save();c.translate(*p);c.rotate(angle)
 path(c,[('M',-w,0),('C',-w-2,-length*.3,-w-2,-length*.7,-w,-length),('C',-w,-length-12,w+12,-length-12,w+14,-length),('L',w+15,-5),('C',w+15,14,-w,14,-w,0),('Z',)],linear(-w,-length,w+15,0,[(0,'#67615b'),(.3,'#363534'),(1,'#272827')]),width=1.8)
 path(c,[('M',-w+2,0),('L',-w+2,-length),('C',-w+1,-length-7,1,-length-11,6,-length+1),('L',7,-3),('C',5,10,-w+1,9,-w+2,0),('Z',)],linear(-w,-length,8,0,[(0,'#f0ff7b'),(.2,'#d9ff1e'),(.8,'#d1f512'),(1,'#a6cd0a')]),width=1.1)
 path(c,[('M',-w+3,-length+6),('L',-w+3,-6),('L',-w+7,-4),('L',-w+7,-length+5),('Z',)],'#f6ffc6',None)
 path(c,[('M',2,-length+4),('L',7,-length+1),('L',7,-4),('L',2,-7),('Z',)],'#a5cb0b',None)
 path(c,[('M',12,-length+5),('L',w+10,-length+4),('L',w+11,-5),('L',16,-8),('Z',)],'#494642',None)
 line(c,[('M',w+9,-length+4),('L',w+10,-7)],'#817b70',1.2)
 c.restore()

def cap(c,x,y,r,angle=-15,turn=0,back_offset=None):
 c.save();c.translate(x,y);c.rotate(math.radians(angle))
 ellipse(c,(8+24*math.sin(math.radians(turn))) if back_offset is None else back_offset,0,r+2,r+1,'#272825',OUTLINE,1.6)
 ellipse(c,4,0,r,r+1,linear(-r,-r,r,r,[(0,'#fcf8eb'),(.4,'#e3d9c7'),(.7,'#a99e8e'),(1,'#dbcfba')]),OUTLINE,1.2)
 ellipse(c,0,0,r*.89,r*.96,linear(-r,-r,r,r,[(0,'#fffdf0'),(.25,'#e1d6c5'),(.75,'#bdb09d'),(1,'#f5eddf')]),OUTLINE,1.15)
 ellipse(c,-1,-1,r*.68,r*.75,'#e8dfcf',OUTLINE,1)
 c.save();c.translate(-1,-1);c.scale(r*.64,r*.71);c.arc(0,0,1,0,math.tau);c.clip()
 # Hand-drawn brushed metal sectors; geometry is redrawn at each pose.
 for a,b,co in [(-1.2,.3,'#faf7ed'),(.3,1.8,'#c8bca9'),(1.8,3.1,'#fffaf0'),(3.1,5.08,'#bcb09e')]:
  c.move_to(0,0);c.arc(0,0,1.03,a,b);c.close_path();paint(c,linear(-.7,-.8,.8,.9,[(0,'#fffaf0'),(.45,co),(1,'#b1a491')]),None)
 c.restore()
 line(c,[('M',-r*.67,-r*.20),('C',-r*.65,-r*.78,-r*.1,-r*.97,r*.38,-r*.66)],'#fffdf7',1.3)
 path(c,[('M',r*.39,-r*.86),('C',r*.74,-r*.69,r*.91,-r*.35,r*.90,-r*.10),('L',r*.65,-r*.15),('C',r*.61,-r*.43,r*.5,-r*.65,r*.28,-r*.75),('Z',)],'#fffdf5',None)
 c.restore()

def shoulder_shell(c):
 path(c,[('M',113,180),('C',131,173,159,174,177,179),('C',193,187,198,213,188,230),('C',176,236,137,240,116,234),('Z',)],linear(130,179,171,235,[(0,'#53504a'),(.5,'#282a27'),(1,'#42413c')]),width=1.9)
 path(c,[('M',153,177),('C',164,177,174,177,180,185),('C',193,199,195,213,187,228),('L',165,232),('C',175,211,173,192,153,177),('Z',)],linear(153,180,189,229,[(0,'#f1ff7e'),(.35,'#dfff22'),(.75,'#d6f81b'),(1,'#adc90b')]),width=1.5)
 path(c,[('M',171,183),('C',177,181,183,184,185,190),('L',177,190),('Z',)],CREAM,None)

def shoulder(c,turn=0):
 shoulder_shell(c)
 cap(c,*P0,31,-14,turn)

def eye(c,x,y,rx,ry,angle):
 c.save();c.translate(x,y);c.rotate(math.radians(angle))
 ellipse(c,0,0,rx*1.7,ry*1.4,radial(0,0,ry*1.4,[(0,'#7fffbc',.28),(.7,'#5cffb1',.05),(1,'#5cffb1',0)]))
 ellipse(c,0,0,rx,ry,'#edfff4')
 ellipse(c,-rx*.62,-ry*.29,rx*.83,ry*.79,'#20362c')
 ellipse(c,-rx*.55,-ry*.53,rx*.22,ry*.15,'#f5fff9')
 ellipse(c,-rx*.14,-ry*.68,rx*.085,ry*.065,'#f5fff9')
 c.restore()

def head_rear(c):
 path(c,[('M',125,70),('C',134,44,168,20,190,16),('C',218,7,242,22,253,42),('C',262,56,263,59,271,66),('C',290,80,293,105,283,127),('C',275,145,260,146,244,140),('C',228,137,206,139,185,128),('C',160,119,145,99,125,70),('Z',)],linear(151,25,259,142,[(0,'#e1f94a'),(.2,'#bedc20'),(.7,'#b6d312'),(1,'#8fa70c')]),width=2)

def head(c):
 path(c,[('M',125,70),('C',134,44,168,20,190,16),('C',218,7,242,22,253,42),('C',262,56,263,59,271,66),('C',290,80,293,105,283,127),('C',275,145,260,146,244,140),('C',228,137,206,139,185,128),('C',160,119,145,99,125,70),('Z',)],linear(151,25,259,142,[(0,'#edff88'),(.2,'#dfff23'),(.7,'#d7fc13'),(1,'#b2d30c')]),width=2)
 path(c,[('M',155,39),('C',178,20,190,14,207,17),('C',224,18,245,39,253,56),('C',233,46,219,42,204,50),('C',198,29,183,23,171,30),('Z',)],linear(178,18,224,55,[(0,'#77736b'),(.35,'#393936'),(1,'#252725')]),width=1.4)
 path(c,[('M',190,17),('C',210,10,234,19,247,35),('C',251,40,254,48,255,53),('C',240,42,223,32,210,26),('Z',)],'#deff36',OUTLINE,1.1)
 line(c,[('M',191,16),('C',209,13,233,22,246,36)],'#fbffcf',1.8)
 path(c,[('M',149,53),('C',153,39,165,28,178,26),('C',187,26,194,32,198,38),('L',175,60),('Z',)],'#e2ff30',OUTLINE,1.1)
 line(c,[('M',151,46),('C',161,34,173,28,181,28)],'#fbffcb',1.4)
 path(c,[('M',167,91),('C',158,106,164,117,187,127),('C',203,137,220,136,233,138),('C',210,131,185,121,175,104),('Z',)],'#c1e61a',None)
 path(c,[('M',201,33),('C',211,29,219,34,223,44),('C',222,46,216,47,211,48),('C',207,46,201,38,201,33),('Z',)],CREAM,None)

def head_details(c):
 path(c,[('M',155,39),('C',178,20,190,14,207,17),('C',224,18,245,39,253,56),('C',233,46,219,42,204,50),('C',198,29,183,23,171,30),('Z',)],linear(178,18,224,55,[(0,'#77736b'),(.35,'#393936'),(1,'#252725')]),width=1.4)
 path(c,[('M',190,17),('C',210,10,234,19,247,35),('C',251,40,254,48,255,53),('C',240,42,223,32,210,26),('Z',)],'#deff36',OUTLINE,1.1)
 line(c,[('M',191,16),('C',209,13,233,22,246,36)],'#fbffcf',1.8)
 path(c,[('M',149,53),('C',153,39,165,28,178,26),('C',187,26,194,32,198,38),('L',175,60),('Z',)],'#e2ff30',OUTLINE,1.1)
 line(c,[('M',151,46),('C',161,34,173,28,181,28)],'#fbffcb',1.4)
 path(c,[('M',167,91),('C',158,106,164,117,187,127),('C',203,137,220,136,233,138),('C',210,131,185,121,175,104),('Z',)],'#c1e61a',None)
 path(c,[('M',201,33),('C',211,29,219,34,223,44),('C',222,46,216,47,211,48),('C',207,46,201,38,201,33),('Z',)],CREAM,None)

def face(c):
 path(c,[('M',216,52),('C',239,47,266,63,277,79),('C',290,97,288,119,276,132),('C',267,144,252,139,237,135),('L',203,126),('C',184,122,180,114,183,98),('L',190,75),('C',194,61,204,54,216,52),('Z',)],linear(199,61,270,134,[(0,'#353937'),(.25,'#1c211e'),(.7,'#171b18'),(1,'#343631')]),width=1.8)
 line(c,[('M',201,61),('C',218,48,245,58,261,68)],'#5e6159',1.3)
 eye(c,211,92,12,18,22);eye(c,261,110,9.5,15.5,22)
 line(c,[('M',217,63),('C',220,59,226,60,229,64)],'#edfff3',2)
 line(c,[('M',267,80),('C',270,77,275,80,277,84)],'#edfff3',1.6)
 for x,y,rx,ry in [(195,110,10,6),(267,127,8,5)]:
  c.save();c.translate(x,y);c.scale(rx,ry);ellipse(c,0,0,1,1,radial(0,0,1,[(0,'#ff9c94',.8),(.5,'#ff817e',.55),(1,'#ff817e',0)]));c.restore()
 for x,y in [(191,107),(195,109),(199,110),(263,128),(267,127),(270,125)]:
  line(c,[('M',x,y),('L',x+1.4,y+4)],'#ff8677',1.8)

def finger(c,p,scale,angle):
 c.save();c.translate(*p);c.rotate(math.radians(angle));c.scale(scale,scale)
 path(c,[('M',-16,2),('C',-19,-9,-6,-18,7,-15),('C',20,-12,25,2,22,16),('C',21,30,13,43,2,42),('C',-9,39,-19,23,-16,2),('Z',)],linear(-16,-15,23,40,[(0,'#56514a'),(.5,'#292a28'),(1,'#47433e')]),width=1.6)
 path(c,[('M',-16,13),('C',-23,1,-18,-10,-8,-14),('C',7,-20,19,-10,22,4),('C',8,0,-4,6,-10,20),('C',-14,21,-16,17,-16,13),('Z',)],linear(-18,-13,17,16,[(0,'#ecff6e'),(.4,'#ddff25'),(1,'#bad811')]),width=1.4)
 path(c,[('M',-10,23),('C',-5,11,6,3,17,7),('C',24,16,22,33,11,40),('C',4,44,-6,34,-10,23),('Z',)],linear(-7,11,22,35,[(0,'#55524d'),(.4,'#383936'),(1,'#252725')]),width=1.3)
 path(c,[('M',4,9),('C',11,6,15,13,17,22),('L',17,31),('C',12,25,8,18,4,9),('Z',)],'#5b5750',None)
 ellipse(c,7,10,1.8,3.4,'#eee8dc')
 path(c,[('M',-4,-12),('C',2,-14,8,-9,9,-4),('C',7,-1,3,-2,1,-6),('Z',)],CREAM,None)
 ellipse(c,-13,1,2.0,2.5,'#82920c')
 c.restore()

def gripper(c,opening=0):
 # Rigid jaws move only left/right; a fixed palm covers both slide roots.
 gap=opening*.60
 path(c,[('M',218,132),('C',224,130,239,133,245,138),('L',247,145),('L',216,145),('Z',)],'#343633',OUTLINE,1.5)
 if gap>1e-9:
  # Short telescoping ends are mostly occluded by the palm and finger roots.
  for x0,x1 in [(218-gap,231),(238,253+gap)]:
   path(c,[('M',x0,142),('L',x1,142),('L',x1,148),('L',x0,148),('Z',)],linear(0,142,0,148,[(0,'#77796e'),(.4,'#484b42'),(1,'#292d27')]),width=1)
 path(c,[('M',217,135),('C',226,131,241,133,249,139),('L',249,149),('C',239,152,227,151,215,147),('Z',)],linear(219,134,243,151,[(0,'#e9ff69'),(.45,'#d2f326'),(1,'#94b117')]),width=1.5)
 line(c,[('M',220,137),('C',228,135,240,137,245,140)],'#f5ffb5',1.1)
 path(c,[('M',218,146),('C',229,149,241,150,248,147),('L',247,151),('C',235,153,225,151,218,149),('Z',)],'#373b2e',None)
 finger(c,(251+gap,151),.76,-8)
 finger(c,(220-gap,148),1,-22)

def project_turn(c,yaw,depth=0):
 # Orthographic projection of complete vector layers on rigid vertical planes.
 # Baseline view is 20 degrees from the arm plane. A yaw changes horizontal
 # foreshortening, depth displacement, and vertical depth projection.
 a0=math.radians(20);a=a0+math.radians(yaw);ox=142.
 f=math.cos(a)/math.cos(a0);h=.32*(math.sin(a)-math.sin(a0))/math.cos(a0)
 dx=(math.sin(a)-math.sin(a0))*depth
 dy=-.32*(math.cos(a)-math.cos(a0))*depth
 c.transform(cairo.Matrix(f,h,0,1,ox*(1-f)+dx,-h*ox+dy))

def turntable(c,yaw):
 # Marks belong to the rotating upper ring; the grounded foot stays fixed.
 for offset in [-66,0,66]:
  a=math.radians(yaw+offset);x=142+55*math.sin(a);y=239+10*math.cos(a)
  path(c,[('M',x-2.2,y-1),('L',x+2.2,y-1),('L',x+2,y+1.8),('L',x-2,y+1.8),('Z',)],'#d9fa32',None)

def draw_previous(c,q,bg=True):
 if bg:color(c,'#fffdf5');c.paint()
 c.set_line_join(cairo.LINE_JOIN_ROUND);c.set_line_cap(cairo.LINE_CAP_ROUND)
 c.translate(34,38)
 s,e,w,tool,opening,yaw=q
 ep=P0+rotation(s)@(P1-P0);wp=ep+rotation(s+e)@(P2-P1);cp=wp+rotation(s+e+w)@(PC-P2)
 base(c);turntable(c,yaw)
 c.save();project_turn(c,yaw,0)
 arm(c,P0,ep,17);arm(c,ep,wp,17);c.restore()
 if abs(yaw)>1e-9:
  c.save();project_turn(c,yaw,34);at(c,P2,wp,s+e+w);head_rear(c);c.restore()
 c.save();project_turn(c,yaw,0);at(c,P2,wp,s+e+w);head(c);c.restore()
 c.save();project_turn(c,yaw,-7);at(c,P2,wp,s+e+w);face(c);c.restore()
 c.save();project_turn(c,yaw,-5);at(c,PC,cp,s+e+w+tool);gripper(c,opening);c.restore()
 c.save();project_turn(c,yaw,-6);cap(c,*ep,29,s+e-14,yaw);cap(c,*wp,33,s+e+w-16,yaw);shoulder(c,yaw);c.restore()

HEAD_OUTLINE=[('M', 125, 70), ('C', 134, 44, 168, 20, 190, 16), ('C', 218, 7, 242, 22, 253, 42), ('C', 262, 56, 263, 59, 271, 66), ('C', 290, 80, 293, 105, 283, 127), ('C', 275, 145, 260, 146, 244, 140), ('C', 228, 137, 206, 139, 185, 128), ('C', 160, 119, 145, 99, 125, 70), ('Z',)]

def trace(c,commands):
 c.new_path()
 for op,*v in commands:
  if op=='M':c.move_to(*v)
  elif op=='L':c.line_to(*v)
  elif op=='C':c.curve_to(*v)
  elif op=='Z':c.close_path()

def turn_point(p,yaw,depth=0):
 # A fixed oblique camera observes the same vertical turntable axis.
 a0=math.radians(25);a=a0+math.radians(yaw);x,y=p
 return np.array([142+(x-142)*math.cos(a)/math.cos(a0)+depth*(math.sin(a)-math.sin(a0)),y+.28*((x-142)*(math.sin(a)-math.sin(a0))/math.cos(a0)+depth*(math.cos(a0)-math.cos(a)))])

class SurfacePen:
 """Draw paths on a curved face surface, not on a flattened sprite plane."""
 def __init__(self,c,warp):self.c=c;self.warp=warp;self.m=np.eye(3);self.stack=[];self.point=np.zeros(2);self.start=np.zeros(2)
 def __getattr__(self,k):return getattr(self.c,k)
 def save(self):self.stack.append(self.m.copy());self.c.save()
 def restore(self):self.m=self.stack.pop();self.c.restore()
 def translate(self,x,y):self.m=self.m@np.array([[1,0,x],[0,1,y],[0,0,1]])
 def scale(self,x,y):self.m=self.m@np.diag([x,y,1])
 def rotate(self,a):self.m=self.m@np.array([[math.cos(a),-math.sin(a),0],[math.sin(a),math.cos(a),0],[0,0,1]])
 def mapped(self,p):return np.array(self.warp((self.m@np.array([*p,1]))[:2]))
 def move_to(self,x,y):self.point=np.array([x,y]);self.start=self.point.copy();self.c.move_to(*self.mapped(self.point))
 def line_to(self,x,y):self.point=np.array([x,y]);self.c.line_to(*self.mapped(self.point))
 def curve_to(self,x1,y1,x2,y2,x3,y3):
  p0=self.point.copy();p1=np.array([x1,y1]);p2=np.array([x2,y2]);p3=np.array([x3,y3])
  for t in np.linspace(0,1,49)[1:]:
   p=(1-t)**3*p0+3*(1-t)**2*t*p1+3*(1-t)*t*t*p2+t**3*p3;self.c.line_to(*self.mapped(p))
  self.point=p3
 def arc(self,x,y,r,a,b):
  # All arcs used here are ellipses built from fresh subpaths.
  angles=np.linspace(a,b,129);self.move_to(x+r*math.cos(a),y+r*math.sin(a))
  for v in angles[1:]:self.line_to(x+r*math.cos(v),y+r*math.sin(v))
 def close_path(self):self.c.close_path();self.point=self.start.copy()
 def set_source(self,pattern):
  # Place drawn gradients in the tangent coordinates of the curved surface.
  anchor=np.array([220.,95.]) if abs(self.m[0,2])+abs(self.m[1,2])<1e-6 else np.zeros(2)
  p0=self.mapped(anchor);px=self.mapped(anchor+[1,0])-p0;py=self.mapped(anchor+[0,1])-p0
  mat=np.array([[px[0],py[0],0],[px[1],py[1],0],[0,0,1.]])
  mat[:2,2]=p0-mat[:2,:2]@anchor
  if abs(np.linalg.det(mat))>1e-8:
   iv=np.linalg.inv(mat);pattern.set_matrix(cairo.Matrix(iv[0,0],iv[1,0],iv[0,1],iv[1,1],iv[0,2],iv[1,2]))
  self.c.set_source(pattern)

def volumetric_head(c,yaw):
 center=np.array([210.,79.]);new=turn_point(center,yaw);a0=math.radians(25);a=a0+math.radians(yaw)
 width=math.sqrt(math.cos(a)**2+(.82*math.sin(a))**2)/math.sqrt(math.cos(a0)**2+(.82*math.sin(a0))**2)
 # The outer shell stays round while its crown, seams and visor rotate together.
 c.save();c.translate(*new);c.scale(width,1);c.translate(*(-center))
 path(c,HEAD_OUTLINE,linear(151,25,259,142,[(0,'#edff88'),(.2,'#dfff23'),(.7,'#d7fc13'),(1,'#b2d30c')]),width=2)
 c.restore()
 def surface(p):
  x,y=p;u=(x-center[0])/92.;v=(y-center[1])/84.
  depth=64*math.sqrt(max(.01,1-u*u-v*v))
  return (new[0]+(x-center[0])*math.cos(a)/math.cos(a0)+depth*(math.sin(a)-math.sin(a0)),new[1]+y-center[1]+.28*((x-center[0])*(math.sin(a)-math.sin(a0))/math.cos(a0)+depth*(math.cos(a0)-math.cos(a))))
 c.save();c.save();c.translate(*new);c.scale(width,1);c.translate(*(-center));trace(c,HEAD_OUTLINE);c.restore();c.clip()
 pen=SurfacePen(c,surface);head_details(pen);face(pen)
 c.restore()

def volume_arm(c,p,endpoint,w,yaw):
 # Rigid prism: draw the exposed side walls between front and rear planes.
 v=endpoint-p;length=np.linalg.norm(v);n=np.array([-v[1],v[0]])/length
 for side in [-w,w+15]:
  pa=p+n*side;pb=endpoint+n*side
  points=[turn_point(pa,yaw,18),turn_point(pb,yaw,18),turn_point(pb,yaw,0),turn_point(pa,yaw,0)]
  path(c,[('M',*points[0])]+[('L',*z) for z in points[1:]]+[('Z',)],'#3d4036',width=1.1)
 arm(SurfacePen(c,lambda p:turn_point(p,yaw)),p,endpoint,w)

def joint_barrel(c,p,r,angle,yaw):
 # Smooth, closed cylinder bridges the near metal disk and the far arm plane.
 from scipy.spatial import ConvexHull
 a0=math.radians(25);a=a0+math.radians(yaw);sx=math.cos(a)/math.cos(a0)
 points=[]
 for depth in [-22,18]:
  blend=min(abs(yaw)/16,1);blend=blend*blend*(3-2*blend)
  center=turn_point(p,yaw,depth)+(rotation(angle)@np.array([8.,0.]))*(1-blend)
  taper=1-.12*blend if depth==18 else 1
  for t in np.linspace(0,math.tau,64,endpoint=False):
   off=rotation(angle)@np.array([(r+1)*taper*math.cos(t),(r+.5)*taper*math.sin(t)])
   points.append(center+np.array([sx*off[0],off[1]]))
 points=np.array(points);hull=points[ConvexHull(points).vertices]
 path(c,[('M',*hull[0])]+[('L',*z) for z in hull[1:]]+[('Z',)],linear(*hull.min(axis=0),*hull.max(axis=0),[(0,'#747569'),(.3,'#41463b'),(.7,'#282d27'),(1,'#5b6051')]),width=1.6)

def volume_cap(c,p,r,angle,yaw):
 core=turn_point(p,yaw);front=turn_point(p,yaw,-22)
 a0=math.radians(25);a=a0+math.radians(yaw);sx=math.cos(a)/math.cos(a0)
 c.save();c.translate(*front);c.scale(sx,1)
 cap(c,0,0,r,angle,back_offset=8-(front[0]-core[0])/sx)
 c.restore()

def draw(c,q,bg=True):
 yaw=q[5]
 if abs(yaw)<1e-8:
  draw_previous(c,q,bg);return
 if bg:color(c,'#fffdf5');c.paint()
 c.set_line_join(cairo.LINE_JOIN_ROUND);c.set_line_cap(cairo.LINE_CAP_ROUND);c.translate(34,38)
 base(c);turntable(c,yaw)
 for p,r,ang in [(P0,31,-14),(P1,29,-14),(P2,33,-16)]:joint_barrel(c,p,r,ang,yaw)
 volume_arm(c,P0,P1,17,yaw);volume_arm(c,P1,P2,17,yaw)
 volumetric_head(c,yaw)
 # Palm depth is distinct from the skull and the two jaw housings.
 gp=turn_point(PC,yaw,25);a0=math.radians(25);a=a0+math.radians(yaw)
 volume_width=math.sqrt(math.cos(a)**2+(.8*math.sin(a))**2)/math.sqrt(math.cos(a0)**2+(.8*math.sin(a0))**2)
 c.save();c.translate(*gp);c.scale(volume_width,1);c.translate(*(-PC));gripper(c,0);c.restore()
 volume_cap(c,P1,29,-14,yaw);volume_cap(c,P2,33,-16,yaw)
 sc=np.array([151.,208.]);sp=turn_point(sc,yaw)
 c.save();c.translate(*sp);c.scale(volume_width,1);c.translate(*(-sc));shoulder_shell(c);c.restore()
 volume_cap(c,P0,31,-14,yaw)

Z=(0,0,0,0,0,0)
clips=[
 ('01_nod','点头打拍',[Z,(14,-8,16,0,0,0),Z,(14,-8,16,0,0,0),Z]),
 ('02_lift','抬臂舒展',[Z,(22,-36,14,0,0,0),Z]),
 ('03_dip','深蹲回升',[Z,(-20,34,-14,0,0,0),Z]),
 ('04_reach','前探伸展',[Z,(22,8,-30,0,0,0),Z]),
 ('05_base_turn','底盘回转',[Z,(0,0,0,0,0,-28.4),Z,(0,0,0,0,0,28.4),Z]),
 ('06_sway','大幅摇摆',[Z,(20,0,-20,0,0,0),Z,(-20,0,20,0,0,0),Z]),
 ('07_elbow','折肘展开',[Z,(0,-28,28,0,0,0),Z,(0,28,-28,0,0,0),Z]),
 ('08_wrist','抬头俯看',[Z,(0,0,-24,0,0,0),Z,(12,-6,12,0,0,0),Z]),
 ('09_grip','夹爪开合',[Z,(0,0,0,0,32,0),Z,(0,0,0,0,32,0),Z]),
 ('10_wave','大幅波浪',[Z,(16,0,-8,0,0,0),(16,-24,8,0,0,0),(0,-24,24,0,0,0),Z])]
_splines={}
def curve(idx):
 if idx not in _splines:
  keys=np.array(clips[idx][2]);ends=[(1,np.zeros(6)),(2,np.zeros(6))]
  _splines[idx]=make_interp_spline(np.linspace(0,1,len(keys)),keys,k=5,bc_type=(ends,ends))
 return _splines[idx]
def pose(idx,t):return [float(v) for v in curve(idx)(t)]

def render(q,transparent=False,size=SIZE):
 surface=cairo.ImageSurface(cairo.FORMAT_ARGB32,size*2,size*2);c=cairo.Context(surface);c.scale(size*2/VIEW,size*2/VIEW);draw(c,q,not transparent);surface.flush()
 arr=np.frombuffer(surface.get_data(),np.uint8).reshape(size*2,size*2,4)[:,:,[2,1,0,3]].copy()
 # Cairo premultiplies alpha; export straight-alpha PNGs.
 if transparent:
  alpha=arr[:,:,3:4].astype(float);arr[:,:,:3]=np.minimum(255,arr[:,:,:3].astype(float)*255/np.maximum(alpha,1)).astype(np.uint8)
 im=Image.fromarray(arr,'RGBA').resize((size,size),Image.Resampling.LANCZOS)
 return im if transparent else im.convert('RGB')
def render_clip(idx):
 ident,name,keys=clips[idx];d=R/'frames'/ident;d.mkdir(exist_ok=True);poses=[]
 for i in range(120):
  q=pose(idx,i/119);poses.append(q);render(q,transparent=True).save(d/f'{i:03d}.png')
 print('Drawn '+ident,flush=True)
 return {'id':ident,'name':name,'joint_order':['shoulder','elbow','wrist','tool','claw_open','base_yaw'],'duration_s':2.4,'frame_duration_ms':20,'coordinate_units':['deg','deg','deg','deg','opening_command','deg'],'jaw_travel_per_command_reference_px':.6,'samples':poses}

if __name__=='__main__':
 if '--preview' in sys.argv:
  render([0]*6).save(R/'neutral.png')
  svg=cairo.SVGSurface(str(R/'source'/'neutral.svg'),640,640);c=cairo.Context(svg);c.scale(640/VIEW,640/VIEW);draw(c,[0]*6,False);svg.finish()
  print('Preview ready',flush=True)
 else:
  for folder in ['frames','data','gifs','mp4']: (R/folder).mkdir(exist_ok=True)
  from concurrent.futures import ProcessPoolExecutor
  with ProcessPoolExecutor(max_workers=3) as pool: results=list(pool.map(render_clip,range(10)))
  (R/'data'/'motions.json').write_text(json.dumps(results,ensure_ascii=False,indent=2))
