#!/usr/bin/env python3
"""Export a self-contained RS visual URDF and true link transforms; no CAN I/O."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import shutil
import sys
import xml.etree.ElementTree as ET

import numpy as np

from rs_gravity_record import ROOT, DEFAULT_URDF, DEFAULT_CALIBRATION
from rs_dance_virtual_collision import DEFAULT_COLLISION_URDF

MODEL = ROOT/'assets/models/rebot_rs'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def prepare_model():
    control = ET.parse(DEFAULT_URDF).getroot()
    visual = ET.parse(DEFAULT_COLLISION_URDF).getroot()
    MODEL.mkdir(parents=True, exist_ok=True)
    (MODEL/'meshes').mkdir(exist_ok=True)
    geometry = []
    source_links = {link.get('name'): link for link in visual.findall('link')}
    for link in control.findall('link'):
        name = link.get('name')
        if name not in source_links:
            continue
        for index, original in enumerate(source_links[name].findall('visual')):
            node = ET.fromstring(ET.tostring(original))
            mesh = node.find('geometry/mesh')
            if mesh is None:
                raise ValueError('Expected the installed CAD mesh visuals')
            filename = Path(mesh.get('filename')).name
            source = DEFAULT_COLLISION_URDF.parent.parent/'meshes'/filename
            target = MODEL/'meshes'/filename
            shutil.copyfile(source, target)
            mesh.set('filename', 'meshes/'+filename)
            link.append(node)
            origin = node.find('origin')
            geometry.append(dict(name=f'{name}_{index}', link=name, file='meshes/'+filename,
                                 xyz=[float(x) for x in origin.get('xyz','0 0 0').split()],
                                 rpy=[float(x) for x in origin.get('rpy','0 0 0').split()],
                                 scale=[float(x) for x in mesh.get('scale','1 1 1').split()],
                                 source_sha256=sha(source)))
    ET.indent(control)
    ET.ElementTree(control).write(MODEL/'rebot_rs_visual.urdf', encoding='utf-8', xml_declaration=True)
    result = dict(schema='rebuddy.rs.visual_model.v1', kinematics_source_sha256=sha(DEFAULT_URDF),
                  visual_source_sha256=sha(DEFAULT_COLLISION_URDF), geometry=geometry,
                  visual_urdf_sha256=sha(MODEL/'rebot_rs_visual.urdf'),
                  note='Original six-axis RS transforms; source finger sliders fixed closed. Decorative head is separate from CAD geometry.')
    (MODEL/'manifest.json').write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=ROOT/'build/dance-3d')
    args=parser.parse_args()
    import pinocchio as pin
    from rs_dance_library import load_library, DEFAULT_OUTPUT
    model_info=prepare_model()
    model=pin.buildModelFromUrdf(str(MODEL/'rebot_rs_visual.urdf'))
    reference=pin.buildModelFromUrdf(str(DEFAULT_URDF))
    if model.nq != 6:raise ValueError('Only the six-axis closed-gripper rig is expected')
    data=model.createData(); ref_data=reference.createData()
    links=list(dict.fromkeys(g['link'] for g in model_info['geometry']))
    links=list(dict.fromkeys(links+['gripper_end','link1','link2','link3','link4','link5','link6']))
    manifest,plans,_,_,_=load_library(DEFAULT_CALIBRATION,DEFAULT_URDF)
    actions=[]; max_error=0.
    for code in ('08','09'):
        plan=plans[code]; spec=next(a for a in manifest['actions'] if int(a['number'])==int(code))
        count=math.ceil(plan.duration/.08)
        ticks=np.rint(np.linspace(0,plan.duration*100,count+1)).astype(int)
        times=ticks[:-1]/100.;times[-1]=plan.duration
        rows=[]
        for time in times:
            q,_=plan.at(float(time))
            pin.framesForwardKinematics(model,data,np.asarray(q[:6]))
            pin.framesForwardKinematics(reference,ref_data,np.asarray(q[:6]))
            transforms={}
            for link in links:
                value=data.oMf[model.getFrameId(link)].homogeneous
                reference_value=ref_data.oMf[reference.getFrameId(link)].homogeneous
                error=float(np.max(abs(value-reference_value)))
                if error>1e-10:raise ValueError(f'Kinematic model changed: {link}')
                max_error=max(max_error,error);transforms[link]=value.tolist()
            rows.append(dict(time_s=float(time),q_rad=list(q),links=transforms))
        rotations=[np.array(row['links']['gripper_end'])[:3,:3] for row in rows]
        drift=max(float(np.linalg.norm(pin.log3(r@rotations[0].T)))*180/math.pi for r in rotations)
        actions.append(dict(number=code,name=spec['name'],duration_s=plan.duration,
                            duration_ms=int(ticks[-1]*10),frame_durations_ms=(np.diff(ticks)*10).tolist(),
                            trajectory_sha256=spec['sha256'],frames=rows,
                            tcp_orientation_drift_deg=drift))
        if code=='09':
            design=json.loads((DEFAULT_OUTPUT/spec['file']).read_text())['motion_design']
            u=np.array(design['plane_u']);v=np.array(design['plane_v'])
            actions[-1]['camera']=dict(direction=np.cross(u,v).tolist(),up=v.tolist(),
                                       target=[.09,0,.34],parallel_scale=.44)
    # Camera fixed in the base frame. Y is mostly horizontal, Z is vertical;
    # the actual Y/Z eight is shown as infinity, never screen-rotated upright.
    result=dict(schema='rebuddy.dance_3d.v1',model=str(MODEL.relative_to(ROOT)),
                model_manifest_sha256=sha(MODEL/'manifest.json'),offline=True,
                frame_transforms_verified_max_error=max_error,
                camera=dict(direction=[.78,-.62,.26],up=[0,0,1],target=[.09,0,.34],parallel_scale=.44),
                actions=actions,physical_trajectories_changed=False)
    args.output.mkdir(parents=True,exist_ok=True)
    (args.output/'motion.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    print(f'Exported {len(model_info["geometry"])} visual meshes; exact FK error {max_error:g}')
    for action in actions:print(action['number'],action['duration_s'],'s, TCP orientation drift',action['tcp_orientation_drift_deg'],'deg')


if __name__=='__main__':main()
