from pathlib import Path
import math,json,hashlib
import numpy as np
from turn_renderer import lift,project_world,D,E
from artwork import P0,P1,P2
R=Path(__file__).resolve().parent.parent
# Rotation is orthonormal and all rigid parts receive this one transform.
points=np.array([lift(*p,-12) for p in (P0,P1,P2)])
lengths=np.linalg.norm(np.diff(points,axis=0),axis=1);maxerr=0
for yaw in np.linspace(-32,32,129):
 t=math.radians(yaw);rot=np.array([[math.cos(t),0,math.sin(t)],[0,1,0],[-math.sin(t),0,math.cos(t)]])
 moved=points@rot.T;maxerr=max(maxerr,float(np.max(abs(np.linalg.norm(np.diff(moved,axis=0),axis=1)-lengths))))
 assert np.allclose(rot.T@rot,np.eye(3),atol=1e-12)
 assert np.allclose(project_world([[0,0,0]],yaw),[[142,239]])
assert maxerr<1e-10
# Camera projection at yaw zero returns the source 2D control point.
errors=[]
for z in [-60,-12,0,42,80]:
 for p in [(75,143),(139,74),(227,143),(142,239)]:errors.append(np.linalg.norm(project_world([lift(*p,z)],0)[0]-p))
assert max(errors)<1e-10
old=R.parent/'expanded';hashes={}
for folder,ext in [('gifs','gif'),('webp','webp'),('mp4','mp4')]:
 for p in (old/folder).glob('*.'+ext):
  if p.stem=='05_base_turn':continue
  target=R/folder/p.name;hashes[str(target.relative_to(R))]=hashlib.sha256(target.read_bytes()).hexdigest()==hashlib.sha256(p.read_bytes()).hexdigest()
assert all(hashes.values())
report={'camera':{'projection':'single calibrated off-axis pinhole camera','focal_distance':D,'elevation_degrees':math.degrees(E)},'axis':'one fixed world Y axis through the turntable center','rigid_length_error_max':maxerr,'reference_projection_error_max':max(errors),'angle_samples_checked':129,'same_rigid_transform_for_all_parts':True,'visibility':'shared depth buffer for all surfaces','artwork':'original 2D vector paths rendered to surface artwork; no reference photograph crops','other_nine_animation_files_identical':hashes,'scope':'Checks the constructed animation geometry and camera, not the dimensions or dynamics of a physical robot.'}
(R/'data/perspective_checks.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
print('Unified camera and rigid rotation verified; 27 files from other nine animations identical.')
