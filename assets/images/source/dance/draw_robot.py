from artwork import *
from artwork import render as render_original

def render_clip(idx):
 if idx!=4:
  from artwork import render_clip as original
  return original(idx)
 from turn_renderer import render as render_turn
 ident,name,keys=clips[idx];d=R/'frames'/ident;d.mkdir(parents=True,exist_ok=True);poses=[]
 for i in range(120):
  q=pose(idx,i/119);poses.append(q);render_turn(q[5],transparent=True).save(d/f'{i:03d}.png')
  if i%30==0:print('Turn frame',i,flush=True)
 return {'id':ident,'name':name,'joint_order':['shoulder','elbow','wrist','tool','claw_open','base_yaw'],'duration_s':2.4,'frame_duration_ms':20,'coordinate_units':['deg','deg','deg','deg','opening_command','deg'],'jaw_travel_per_command_reference_px':.6,'samples':poses}

if __name__=='__main__':
 for folder in ['frames','data','gifs','mp4']:(R/folder).mkdir(exist_ok=True)
 indices=[4] if '--turn-only' in sys.argv else range(10)
 results={idx:render_clip(idx) for idx in indices}
 p=R/'data/motions.json'
 data=json.loads(p.read_text()) if p.exists() else [None]*10
 for idx,item in results.items():data[idx]=item
 p.write_text(json.dumps(data,ensure_ascii=False,indent=2))
 print('Render complete',flush=True)
