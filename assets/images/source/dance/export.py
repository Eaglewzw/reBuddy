from pathlib import Path
import sys,json,math,subprocess,gc
import numpy as np
from PIL import Image,ImageDraw,ImageFont,ImageChops
from draw_robot import clips,curve,pose,P0,P1,P2,PC,rotation
R=Path(__file__).resolve().parent.parent
for f in ['webp','previews']: (R/f).mkdir(exist_ok=True)
font=ImageFont.truetype('/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc',18)
BG=(255,253,245)
def flat(im):
 bg=Image.new('RGBA',im.size,(*BG,255));bg.alpha_composite(im);return bg.convert('RGB')
def savegif(frames,path,ms=20):
 atlas=Image.new('RGB',(256*6,256*2),BG)
 for j,k in enumerate(np.linspace(0,len(frames)-1,12,dtype=int)):
  atlas.paste(frames[k].resize((256,256),Image.Resampling.LANCZOS),(j%6*256,j//6*256))
 palette=atlas.quantize(colors=256);pframes=[f.quantize(palette=palette,dither=Image.Dither.NONE) for f in frames]
 pframes[0].save(path,save_all=True,append_images=pframes[1:],duration=ms,loop=0,optimize=False,disposal=1)
 try:
  check=Image.open(path)
  for i in range(check.n_frames):check.seek(i);check.load()
 except OSError:
  # Fallback encoder for a Pillow GIF delta-stream failure.
  w,h=frames[0].size
  cmd=['ffmpeg','-hide_banner','-loglevel','error','-y','-f','rawvideo','-pixel_format','rgb24','-video_size',f'{w}x{h}','-framerate',str(1000/ms),'-i','pipe:0','-filter_complex','[0:v]split[a][b];[a]palettegen=stats_mode=full[p];[b][p]paletteuse=dither=none','-loop','0',str(path)]
  proc=subprocess.Popen(cmd,stdin=subprocess.PIPE)
  for f in frames:proc.stdin.write(f.tobytes())
  proc.stdin.close();assert proc.wait()==0

def valid_gif(path):
 try:
  im=Image.open(path)
  for i in range(im.n_frames):im.seek(i);im.load()
  return True
 except (OSError,FileNotFoundError):return False

small=[];report={'method':'complete vector paths redrawn for each frame; no bitmap source assets','resolution':[640,640],'fps':50,'duration_seconds':2.4,'clips':[]}
for idx,(ident,name,keys) in enumerate(clips):
 files=sorted((R/'frames'/ident).glob('*.png'));assert len(files)==120
 rgba=[Image.open(p).convert('RGBA') for p in files];frames=[flat(f) for f in rgba]
 assert ImageChops.difference(frames[0],frames[-1]).getbbox() is None
 ready=valid_gif(R/'gifs'/f'{ident}.gif') and '--resume' in sys.argv and (R/'mp4'/f'{ident}.mp4').exists() and (R/'mp4'/f'{ident}.mp4').stat().st_size>0
 if '--repair-two' in sys.argv and ident in {'05_base_turn'}: ready=False
 if not ready: savegif(frames,R/'gifs'/f'{ident}.gif')
 if not ready: rgba[0].save(R/'webp'/f'{ident}.webp',save_all=True,append_images=rgba[1:],duration=20,loop=0,lossless=True,method=1)
 frames[30].save(R/'previews'/f'{ident}.png')
 small.append([f.resize((240,240),Image.Resampling.LANCZOS) for f in frames])
 # Feed exact full-color composites to ffmpeg; no color-key transparency.
 cmd=['ffmpeg','-hide_banner','-loglevel','error','-y','-f','rawvideo','-pixel_format','rgb24','-video_size','640x640','-framerate','50','-i','pipe:0','-an','-c:v','libx264','-crf','17','-pix_fmt','yuv420p','-movflags','+faststart',str(R/'mp4'/f'{ident}.mp4')]
 if not ready:
  proc=subprocess.Popen(cmd,stdin=subprocess.PIPE)
  for f in frames:proc.stdin.write(f.tobytes())
  proc.stdin.close();assert proc.wait()==0

 spl=curve(idx);ts=np.linspace(0,1,2401);qs=spl(ts);speed=spl(ts,1)/2.38;acc=spl(ts,2)/2.38**2
 limits=np.array([30,40,40,20,35,35]);assert (abs(qs).max(axis=0)<=limits).all()
 assert abs(speed).max()<120;assert abs(acc).max()<550
 assert np.max(abs(spl([0,1],1)))<1e-8 and np.max(abs(spl([0,1],2)))<1e-8
 length_err=0
 for q in qs[::10]:
  s,e,w,t=q[:4];ep=P0+rotation(s)@(P1-P0);wp=ep+rotation(s+e)@(P2-P1)
  length_err=max(length_err,abs(np.linalg.norm(ep-P0)-np.linalg.norm(P1-P0)),abs(np.linalg.norm(wp-ep)-np.linalg.norm(P2-P1)))
 boxes=[f.getchannel('A').getbbox() for f in rgba]
 margin=min(min(x,y,640-r,640-b) for x,y,r,b in boxes)
 assert margin>=8
 gif=Image.open(R/'gifs'/f'{ident}.gif');duration=0;start=gif.convert('RGB')
 for k in range(gif.n_frames):gif.seek(k);duration+=gif.info['duration']
 assert duration==2400 and ImageChops.difference(start,gif.convert('RGB')).getbbox() is None
 # Grounded base: bottom region remains pixel-identical throughout all movements.
 fixed=frames[0].crop((0,480,640,640))
 assert all(ImageChops.difference(fixed,f.crop((0,480,640,640))).getbbox() is None for f in frames)
 report['clips'].append({'id':ident,'duration_ms':duration,'gif_encoded_frames':gif.n_frames,'identical_boundary_images':True,'fixed_base':True,'boundary_velocity_acceleration_zero':True,'max_link_length_error':float(length_err),'minimum_canvas_margin_px':margin,'coordinate_units':['deg','deg','deg','deg','opening_command','deg'],'coordinate_max_speed_per_s':abs(speed).max(axis=0).round(3).tolist(),'coordinate_max_acceleration_per_s2':abs(acc).max(axis=0).round(3).tolist(),'coordinate_min':qs.min(axis=0).round(3).tolist(),'coordinate_max':qs.max(axis=0).round(3).tolist(),'jaw_motion':{'axis':'horizontal','each_jaw_max_travel_reference_px':round(float(qs[:,4].max()*.6),3),'vertical_displacement':0,'orientation_change_degrees':0,'each_jaw_max_speed_reference_px_s':round(float(abs(speed[:,4]).max()*.6),3),'each_jaw_max_acceleration_reference_px_s2':round(float(abs(acc[:,4]).max()*.6),3)}})
 print('Exported and verified',ident,flush=True)
 del rgba,frames;gc.collect()
boards=[]
for i in range(120):
 out=Image.new('RGB',(1200,560),'#eff0e5');d=ImageDraw.Draw(out)
 for j,(_,name,_) in enumerate(clips):
  x=j%5*240;y=j//5*280;out.paste(small[j][i],(x,y));d.text((x+14,y+247),f'{j+1:02d} {name}',font=font,fill='#2f3927')
 boards.append(out)
savegif(boards,R/'overview.gif');boards[30].save(R/'overview.png');del boards;gc.collect()
# Pose verification sheet: five stages for each complete action.
board=Image.new('RGB',(1200,2400),BG)
for j in range(10):
 for k,phase in enumerate([0,30,60,90,119]):board.paste(small[j][phase],(k*240,j*240))
board.save(R/'previews'/'pose_checks.jpg')
# Compact combined GIF; full-color demonstration at original resolution as MP4.
sequence=[0,1,4,8,3,5,6,9,7,2];demo=[]
for j in sequence:
 demo.extend([x.resize((360,360),Image.Resampling.LANCZOS) for x in small[j]])
savegif(demo,R/'choreography_demo.gif');del demo,small;gc.collect()
concat=R/'data'/'demo_concat.txt';concat.write_text(''.join("file '../mp4/"+clips[j][0]+".mp4'\n" for j in sequence))
subprocess.run(['ffmpeg','-hide_banner','-loglevel','error','-y','-f','concat','-safe','0','-i',str(concat),'-c','copy','-movflags','+faststart',str(R/'choreography_demo.mp4')],check=True)
report['motion_design_limits']={'angular_joint_order':['shoulder','elbow','wrist','tool','base_yaw'],'relative_angle_abs_degrees':[30,40,40,20,35],'speed_degrees_per_second':120,'acceleration_degrees_per_second2':550,'jaw_max_travel_reference_px':21,'jaw_max_speed_reference_px_s':72,'jaw_max_acceleration_reference_px_s2':330,'note':'Animation design values, not measured actuator ratings. Reference canvas is 400 units, final canvas is 640 pixels.'}
report['scope']='2D joint kinematics, loop continuity, canvas bounds and fixed-base pixels verified. Visual key-pose inspection required. No hardware torque or 3D collision certification.'
(R/'data'/'validation.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
rows=''.join(f'<figure><video autoplay muted loop playsinline poster="previews/{ident}.png" src="mp4/{ident}.mp4"></video><figcaption>{i+1:02d} {name}<span><a href="gifs/{ident}.gif" download>GIF</a> · <a href="webp/{ident}.webp" download>透明 WebP</a></span></figcaption></figure>' for i,(ident,name,_) in enumerate(clips))
(R/'index.html').write_text('''<!doctype html><html lang="zh"><meta charset="utf-8"><title>小机械臂 · 统一透视回转版</title><style>body{margin:40px;background:#eff0e5;color:#2e3927;font:16px system-ui}h1{font-size:32px;margin-bottom:8px}p{line-height:1.8}main{display:grid;grid-template-columns:repeat(auto-fit,minmax(260px,1fr));gap:18px}figure{margin:0;border-radius:18px;overflow:hidden;background:#fffdf5}video{width:100%;display:block}figcaption{padding:15px 18px;font-weight:600}span{display:block;font-weight:400;margin-top:8px}a{color:#55751d}.demo{max-width:640px}</style><h1>小机械臂 · 统一透视回转版</h1><p>完整矢量重绘 · 加大幅度 · 统一透视回转 · 左右夹爪开合 · 柔和腮红 · 640 × 640 · 50 FPS · 2.4 秒 / 组<br>每组首尾回到同一待机姿态。下方播放全彩视频，另提供 GIF 和透明 WebP。</p><main>'''+rows+'''</main><h2>编舞组合示例</h2><p>点头 → 抬臂 → 底盘回转 → 夹爪开合 → 前探 → 摇摆 → 折肘 → 波浪 → 抬头俯看 → 下沉</p><video class="demo" src="choreography_demo.mp4" controls loop></video></html>''')
print('All deliverables ready',flush=True)
