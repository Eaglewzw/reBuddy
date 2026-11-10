from pathlib import Path
import zipfile,json
from PIL import Image
R=Path(__file__).resolve().parent.parent
report=json.loads((R/'data/validation.json').read_text())
for clip in report['clips']:
 im=Image.open(R/'webp'/f"{clip['id']}.webp");total=0
 for i in range(im.n_frames):im.seek(i);im.load();total+=im.info['duration']
 assert total==2400 and im.convert('RGBA').getpixel((0,0))[3]==0
 clip['webp_duration_ms']=total
report['turn_geometry_checks']='perspective_checks.json'
(R/'data/validation.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
out=R.parents[1]/'robot_dance_2d_perspective_v4.zip'
with zipfile.ZipFile(out,'w',zipfile.ZIP_DEFLATED) as z:
 for p in R.rglob('*'):
  if not p.is_file() or {'frames','__pycache__'}.intersection(p.relative_to(R).parts):continue
  if p.parent.name=='previews' and p.name not in {'perspective_keyframes.png','pose_checks.jpg'} and p.stem not in [x['id'] for x in report['clips']]:continue
  z.write(p,Path('robot_dance_2d_perspective_v4')/p.relative_to(R))
with zipfile.ZipFile(out) as z:assert z.testzip() is None
print('Complete and verified:',out,round(out.stat().st_size/1024**2,1),'MiB',flush=True)
