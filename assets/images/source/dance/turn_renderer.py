"""One camera, one rigid yaw transform, depth-buffer visibility.
All surface artwork is freshly drawn from the approved 2D vector paths.
The reference photograph and earlier 3D character are not used.
"""
import os
os.environ['PYOPENGL_PLATFORM']='egl'
os.environ['EGL_PLATFORM']='surfaceless'
import math,ctypes,pathlib
import numpy as np
import cairo
from scipy.ndimage import distance_transform_edt,gaussian_filter
from PIL import Image,ImageFilter
from OpenGL import EGL,GL as g
import artwork as a
ROOT=pathlib.Path(__file__).resolve().parent.parent
TEX=1200;S=2;W=H=1280
D=850.;E=math.radians(11);CE=math.cos(E);SE=math.sin(E)

def init():
 display=EGL.eglGetDisplay(EGL.EGL_DEFAULT_DISPLAY);major=EGL.EGLint();minor=EGL.EGLint()
 assert EGL.eglInitialize(display,major,minor)
 attrs=(EGL.EGLint*17)(EGL.EGL_SURFACE_TYPE,EGL.EGL_PBUFFER_BIT,EGL.EGL_RENDERABLE_TYPE,EGL.EGL_OPENGL_BIT,EGL.EGL_RED_SIZE,8,EGL.EGL_GREEN_SIZE,8,EGL.EGL_BLUE_SIZE,8,EGL.EGL_ALPHA_SIZE,8,EGL.EGL_DEPTH_SIZE,24,EGL.EGL_NONE,0,0)
 configs=(EGL.EGLConfig*1)();n=EGL.EGLint();assert EGL.eglChooseConfig(display,attrs,configs,1,n)
 assert EGL.eglBindAPI(EGL.EGL_OPENGL_API)
 surf=EGL.eglCreatePbufferSurface(display,configs[0],(EGL.EGLint*5)(EGL.EGL_WIDTH,W,EGL.EGL_HEIGHT,H,EGL.EGL_NONE))
 ctx=EGL.eglCreateContext(display,configs[0],EGL.EGL_NO_CONTEXT,None)
 assert EGL.eglMakeCurrent(display,surf,surf,ctx)
 g.glViewport(0,0,W,H);g.glEnable(g.GL_DEPTH_TEST);g.glDepthFunc(g.GL_LEQUAL)
 g.glEnable(g.GL_BLEND);g.glBlendFuncSeparate(g.GL_SRC_ALPHA,g.GL_ONE_MINUS_SRC_ALPHA,g.GL_ONE,g.GL_ONE_MINUS_SRC_ALPHA)
 g.glEnable(g.GL_ALPHA_TEST);g.glAlphaFunc(g.GL_GREATER,.08)
 g.glDisable(g.GL_CULL_FACE)
 return display,surf,ctx

def texture(draw):
 surf=cairo.ImageSurface(cairo.FORMAT_ARGB32,TEX,TEX);c=cairo.Context(surf);c.scale(TEX/400,TEX/400)
 c.set_line_join(cairo.LINE_JOIN_ROUND);c.set_line_cap(cairo.LINE_CAP_ROUND);draw(c);surf.flush()
 arr=np.frombuffer(surf.get_data(),np.uint8).reshape(TEX,TEX,4)[:,:,[2,1,0,3]].copy()
 alpha=arr[:,:,3:4].astype(float);arr[:,:,:3]=np.minimum(255,arr[:,:,:3].astype(float)*255/np.maximum(alpha,1)).astype(np.uint8)
 return arr

def lift(x,y,z):
 cy=(239-y)*(D-z)/D;return np.array([(x-142)*(D-z)/D,CE*cy+SE*z,-SE*cy+CE*z])

def upload(arr):
 tex=g.glGenTextures(1);g.glBindTexture(g.GL_TEXTURE_2D,tex)
 g.glTexParameteri(g.GL_TEXTURE_2D,g.GL_TEXTURE_MIN_FILTER,g.GL_LINEAR);g.glTexParameteri(g.GL_TEXTURE_2D,g.GL_TEXTURE_MAG_FILTER,g.GL_LINEAR)
 g.glTexParameteri(g.GL_TEXTURE_2D,g.GL_TEXTURE_WRAP_S,g.GL_CLAMP_TO_EDGE);g.glTexParameteri(g.GL_TEXTURE_2D,g.GL_TEXTURE_WRAP_T,g.GL_CLAMP_TO_EDGE)
 g.glTexImage2D(g.GL_TEXTURE_2D,0,g.GL_RGBA,TEX,TEX,0,g.GL_RGBA,g.GL_UNSIGNED_BYTE,arr)
 return tex

class Part:
 def __init__(self,name,draw,front,back,offset=(0,0),dome=0,body='#333a2c'):
  self.name=name;self.arr=texture(draw);self.tex=upload(self.arr)
  mask=np.array(Image.fromarray(self.arr[:,:,3]).resize((400,400),Image.Resampling.BOX))>32
  dist=gaussian_filter(distance_transform_edt(mask),1.5);maxdist=dist.max()
  self.center=(front,back);self.offset=offset;self.dome=dome;self.body=body
  def depth(x,y,side):
   if dome:
    xx=min(398.99,max(0,x));yy=min(398.99,max(0,y));ix=int(xx);iy=int(yy);fx=xx-ix;fy=yy-iy
    dd=(1-fy)*((1-fx)*dist[iy,ix]+fx*dist[iy,ix+1])+fy*((1-fx)*dist[iy+1,ix]+fx*dist[iy+1,ix+1])
    bulge=math.sin(math.pi*.5*min(1,max(0,dd/(maxdist+1e-10))))
    return front+dome*bulge if side==0 else front-dome*.75*bulge
   return front if side==0 else back
  ys,xs=np.where(mask);l=max(0,int(xs.min())-2);r=min(399,int(xs.max())+2);t=max(0,int(ys.min())-2);b=min(399,int(ys.max())+2)
  vertices=[];uvs=[]
  # Front and back use a fixed dense grid; geometry never changes with yaw.
  for side in [0,1]:
   for y in range(t,b,1):
    for x in range(l,r,1):
     if not mask[max(0,y-1):min(400,y+4),max(0,x-1):min(400,x+4)].any():continue
     for xx,yy in [(x,y),(x+1,y),(x+1,y+1),(x,y),(x+1,y+1),(x,y+1)]:
      dx,dy=offset if side else (0,0)
      vertices.append(lift(xx+dx,yy+dy,depth(xx,yy,side)));uvs.append((xx/400,yy/400))
   if side==0:self.front_count=len(vertices)
  self.v=np.array(vertices,dtype=np.float32);self.uv=np.array(uvs,dtype=np.float32)
  # Close the silhouette with an actual side wall for extruded rigid pieces.
  walls=[]
  for yy,xx in zip(*np.where(mask)):
   for dy,dx,p,q in [(-1,0,(xx-.5,yy-.5),(xx+.5,yy-.5)),(0,1,(xx+.5,yy-.5),(xx+.5,yy+.5)),(1,0,(xx+.5,yy+.5),(xx-.5,yy+.5)),(0,-1,(xx-.5,yy+.5),(xx-.5,yy-.5))]:
    ny,nx=yy+dy,xx+dx
    if 0<=ny<400 and 0<=nx<400 and mask[ny,nx]:continue
    corners=[lift(*p,depth(*p,0)),lift(*q,depth(*q,0)),lift(q[0]+offset[0],q[1]+offset[1],depth(*q,1)),lift(p[0]+offset[0],p[1]+offset[1],depth(*p,1))]
    for k in [0,1,2,0,2,3]:walls.append(corners[k])
  self.walls=np.array(walls,dtype=np.float32)
  backarr=self.arr.copy();backarr[:,:,:3]=np.array(a.rgb(body))*255
  self.backtex=upload(backarr)
 def draw(self):
  g.glEnableClientState(g.GL_VERTEX_ARRAY);g.glEnableClientState(g.GL_TEXTURE_COORD_ARRAY);g.glEnable(g.GL_TEXTURE_2D);g.glColor4f(1,1,1,1)
  g.glVertexPointer(3,g.GL_FLOAT,0,self.v);g.glTexCoordPointer(2,g.GL_FLOAT,0,self.uv)
  g.glBindTexture(g.GL_TEXTURE_2D,self.backtex);g.glDrawArrays(g.GL_TRIANGLES,self.front_count,len(self.v)-self.front_count)
  g.glBindTexture(g.GL_TEXTURE_2D,self.tex);g.glDrawArrays(g.GL_TRIANGLES,0,self.front_count)
  g.glDisableClientState(g.GL_TEXTURE_COORD_ARRAY);g.glDisable(g.GL_TEXTURE_2D)
  if len(self.walls):
   g.glColor4f(*a.rgb(self.body),1);g.glVertexPointer(3,g.GL_FLOAT,0,self.walls);g.glDrawArrays(g.GL_TRIANGLES,0,len(self.walls))
  g.glDisableClientState(g.GL_VERTEX_ARRAY)

def make_parts():
 def head(c):a.head(c);a.face(c)
 return [
 Part('lower_link',lambda c:a.arm(c,a.P0,a.P1,17),-12,-28,(4,-1),body='#41463b'),
 Part('upper_link',lambda c:a.arm(c,a.P1,a.P2,17),-12,-28,(4,-1),body='#41463b'),
 Part('head',head,-38,-60,dome=65,body='#badb1c'),
 Part('palm_and_jaws',lambda c:a.gripper(c,0),48,34,(2,-.5),dome=0,body='#34382e'),
 Part('shoulder_shell',a.shoulder_shell,3,-30,(5,-1),dome=12,body='#42483a'),
 Part('elbow',lambda c:a.cap(c,*a.P1,29,-14),42,4,(6,-1),body='#353b31'),
 Part('wrist',lambda c:a.cap(c,*a.P2,33,-16),42,4,(6,-1),body='#353b31'),
 Part('shoulder',lambda c:a.cap(c,*a.P0,31,-14),42,4,(6,-1),body='#353b31')]

def project_world(points,yaw):
 p=np.asarray(points,dtype=float);t=math.radians(yaw)
 x=p[...,0]*math.cos(t)+p[...,2]*math.sin(t);z=-p[...,0]*math.sin(t)+p[...,2]*math.cos(t)
 cy=CE*p[...,1]-SE*z;cz=SE*p[...,1]+CE*z
 return np.stack([142+D*x/(D-cz),239-D*cy/(D-cz)],axis=-1)

def draw_ring_marks(c,yaw):
 for angle in [-66,0,66]:
  t=math.radians(angle);points=[]
  for radial,tangential in [(-1.4,-2.2),(-1.4,2.2),(1.4,2.2),(1.4,-2.2)]:
   points.append([(55+radial)*math.sin(t)+tangential*math.cos(t),.5,(55+radial)*math.cos(t)-tangential*math.sin(t)])
  p=project_world(points,yaw)
  a.path(c,[('M',*p[0])]+[('L',*v) for v in p[1:]]+[('Z',)],'#d9fa32',None)

CONTEXT=None;PARTS=None

def render(yaw,size=640,transparent=False):
 global CONTEXT,PARTS
 if CONTEXT is None:CONTEXT=init();PARTS=make_parts()
 g.glClearColor(0,0,0,0);g.glClear(g.GL_COLOR_BUFFER_BIT|g.GL_DEPTH_BUFFER_BIT)
 g.glMatrixMode(g.GL_PROJECTION);g.glLoadIdentity()
 # Off-axis perspective keeps the rotation axis at original screen position.
 near=10.;g.glFrustum(-176/D*near,224/D*near,-123/D*near,277/D*near,near,2000.)
 g.glMatrixMode(g.GL_MODELVIEW);g.glLoadIdentity();g.glTranslatef(0,0,-D);g.glRotatef(math.degrees(E),1,0,0);g.glRotatef(yaw,0,1,0)
 for p in PARTS:p.draw()
 g.glFinish();raw=g.glReadPixels(0,0,W,H,g.GL_RGBA,g.GL_UNSIGNED_BYTE)
 arr=np.frombuffer(raw,np.uint8).reshape(H,W,4)[::-1].copy()
 # Background/grounded base are independently drawn once from the original paths.
 surf=cairo.ImageSurface(cairo.FORMAT_ARGB32,W,H);c=cairo.Context(surf);c.scale(W/400,H/400);
 if not transparent:a.color(c,'#fffdf5');c.paint()
 c.translate(34,38);a.base(c);draw_ring_marks(c,yaw);surf.flush()
 bg=np.frombuffer(surf.get_data(),np.uint8).reshape(H,W,4)[:,:,[2,1,0,3]].copy()
 if transparent:
  ba=bg[:,:,3:4].astype(float);bg[:,:,:3]=np.minimum(255,bg[:,:,:3].astype(float)*255/np.maximum(ba,1)).astype(np.uint8)
 alpha=arr[:,:,3:4].astype(float);arr[:,:,:3]=np.minimum(255,arr[:,:,:3].astype(float)*255/np.maximum(alpha,1)).astype(np.uint8)
 fg=Image.fromarray(arr,'RGBA');out=Image.fromarray(bg,'RGBA')
 outline=Image.new('RGBA',(W,H),(23,25,22,255));outline.putalpha(fg.getchannel('A').filter(ImageFilter.MaxFilter(5)));out.alpha_composite(outline);out.alpha_composite(fg)
 return out.resize((size,size),Image.Resampling.LANCZOS).convert('RGBA' if transparent else 'RGB')

if __name__=='__main__':
 sheet=Image.new('RGB',(1600,320),'#fffdf5')
 for j,yaw in enumerate([-32,-16,0,16,32]):
  im=render(yaw);im.save(ROOT/'previews'/f'view_{j}.png');sheet.paste(im.resize((320,320)),(j*320,0))
 sheet.save(ROOT/'previews/turn_sheet.png');print('Preview ready',flush=True)
