#!/usr/bin/env python3
"""Build a local, dependency-free WebGL viewer for the actual RS CAD rig."""
import json
import base64
from pathlib import Path

import numpy as np
import vtk
from vtk.util.numpy_support import vtk_to_numpy

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'build/dance-3d'

HTML=r'''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>reBuddy · 可旋转三维运动核对</title><style>
body{margin:0;background:#fffdf5;color:#26382d;font:16px system-ui}main{max-width:1100px;margin:auto;padding:20px}
canvas{width:100%;height:65vh;min-height:360px;background:#fffdf5;touch-action:none;border:1px solid #e4e5d7;border-radius:16px}
button,select{font:inherit;padding:9px 15px;margin:4px;border:1px solid #bdcbb1;border-radius:8px;background:#eef5e6;color:#284c31;cursor:pointer}
input{width:min(70%,650px)}small{color:#647568}p{line-height:1.6}#status{font-variant-numeric:tabular-nums}
</style><main><h1>真实六轴 · 三维运动核对</h1><p>拖动旋转视角，滚轮缩放。模型采用 RS 原始 CAD，运动来自当前动作数据；蓝线为真实 TCP 路径。</p>
<select id="action"><option value="0">08 稳定摇摆</option><option value="1">09 空间∞字</option></select><button id="play">暂停</button><button id="reset">重置视角</button>
<canvas id="view"></canvas><p><input id="time" type="range" min="0" max="1000" value="0"><span id="status"></span></p>
<p><small>本地浏览器计算与显示，不连接 CAN。头部装饰外壳请看<a href="index.html">绿色角色 GIF 对照</a>；此处用真实 CAD 避免外壳遮挡腕部。</small></p>
<script id="scene" type="application/json">__SCENE__</script><script>
const data=JSON.parse(document.querySelector('#scene').textContent),g=data.gltf,motion=data.motion,bind=data.bindings;
const canvas=document.querySelector('#view'),gl=canvas.getContext('webgl2',{antialias:true,alpha:false});
if(!gl){document.querySelector('#status').textContent='此浏览器未启用 WebGL 2，请查看 GIF 对照。';throw Error('WebGL 2 unavailable');}
const vs=`#version 300 es
in vec3 p;uniform mat4 model;uniform vec3 target,right,up,front;uniform float scale,aspect;
out vec3 position;void main(){vec3 world=(model*vec4(p,1)).xyz-target;position=world;
gl_Position=vec4(dot(world,right)/scale/aspect,dot(world,up)/scale,-dot(world,front)/2.,1);gl_PointSize=7.;}`;
const fs=`#version 300 es
precision highp float;in vec3 position;uniform vec4 color;uniform bool unlit;out vec4 frag;
void main(){float light=unlit?1.:(.45+.55*max(dot(normalize(cross(dFdx(position),dFdy(position))),normalize(vec3(.6,-1.,1.5))),0.));frag=vec4(color.rgb*light,color.a);}`;
function shader(type,source){let s=gl.createShader(type);gl.shaderSource(s,source);gl.compileShader(s);if(!gl.getShaderParameter(s,gl.COMPILE_STATUS))throw Error(gl.getShaderInfoLog(s));return s;}
const program=gl.createProgram();gl.attachShader(program,shader(gl.VERTEX_SHADER,vs));gl.attachShader(program,shader(gl.FRAGMENT_SHADER,fs));gl.linkProgram(program);
if(!gl.getProgramParameter(program,gl.LINK_STATUS))throw Error(gl.getProgramInfoLog(program));gl.useProgram(program);
const at={p:gl.getAttribLocation(program,'p')},u={};
for(const k of ['model','target','right','up','front','scale','aspect','color','unlit'])u[k]=gl.getUniformLocation(program,k);
const raw=g.buffers.map(b=>{if(!b.uri.startsWith('data:'))throw Error('Expected inline model');return Uint8Array.from(atob(b.uri.split(',')[1]),c=>c.charCodeAt(0));});
function accessor(index,target){let a=g.accessors[index],v=g.bufferViews[a.bufferView],bytes=raw[v.buffer].subarray(v.byteOffset||0,(v.byteOffset||0)+v.byteLength),b=gl.createBuffer();gl.bindBuffer(target,b);gl.bufferData(target,bytes,gl.STATIC_DRAW);return {buffer:b,offset:a.byteOffset||0,stride:v.byteStride||0,count:a.count,type:a.componentType};}
const meshes=g.meshes.map(mesh=>mesh.primitives.map(p=>({p:accessor(p.attributes.POSITION,gl.ARRAY_BUFFER),i:accessor(p.indices,gl.ELEMENT_ARRAY_BUFFER),color:g.materials[p.material].pbrMetallicRoughness.baseColorFactor||[.6,.7,.6,1]})));
function norm(v){let l=Math.hypot(...v);return v.map(x=>x/l);}function cross(a,b){return [a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];}
function mul(a,b){return a.map(row=>b[0].map((_,j)=>row.reduce((s,x,k)=>s+x*b[k][j],0)));}
function uniformMatrix(m){return new Float32Array(m[0].flatMap((_,j)=>m.map(row=>row[j])));}
const identity=[[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]];
let action=motion.actions[0],playing=true,stamp=0,previous=performance.now(),yaw=0,pitch=0,scale=.44,viewUp=[0,0,1],target=[.09,0,.34],pathBuffer;
function reset(){let c=action.camera||motion.camera,d=norm(c.direction);yaw=Math.atan2(d[1],d[0]);pitch=Math.asin(d[2]);scale=c.parallel_scale;viewUp=c.up;target=c.target;}
function choose(){action=motion.actions[+document.querySelector('#action').value];stamp=0;reset();let points=action.frames.flatMap(f=>[f.links.gripper_end[0][3],f.links.gripper_end[1][3],f.links.gripper_end[2][3]]);if(pathBuffer)gl.deleteBuffer(pathBuffer);pathBuffer=gl.createBuffer();gl.bindBuffer(gl.ARRAY_BUFFER,pathBuffer);gl.bufferData(gl.ARRAY_BUFFER,new Float32Array(points),gl.STATIC_DRAW);}
document.querySelector('#action').value=location.hash==='#09'?'1':'0';choose();document.querySelector('#action').onchange=choose;document.querySelector('#reset').onclick=reset;
document.querySelector('#play').onclick=()=>{playing=!playing;document.querySelector('#play').textContent=playing?'暂停':'播放';};
document.querySelector('#time').oninput=e=>{stamp=+e.target.value/1000*action.duration_s;playing=false;document.querySelector('#play').textContent='播放';};
let pointer=null;canvas.onpointerdown=e=>{pointer=[e.clientX,e.clientY];canvas.setPointerCapture(e.pointerId);};canvas.onpointerup=()=>pointer=null;
canvas.onpointermove=e=>{if(!pointer)return;yaw-=(e.clientX-pointer[0])*.008;pitch=Math.max(-1.3,Math.min(1.3,pitch+(e.clientY-pointer[1])*.008));viewUp=[0,0,1];pointer=[e.clientX,e.clientY];};
canvas.addEventListener('wheel',e=>{e.preventDefault();scale=Math.min(1.2,Math.max(.2,scale*Math.exp(e.deltaY*.001)));},{passive:false});
function attribute(slot,a){gl.bindBuffer(gl.ARRAY_BUFFER,a.buffer);gl.enableVertexAttribArray(slot);gl.vertexAttribPointer(slot,3,a.type,false,a.stride,a.offset);}
function draw(now){let dt=(now-previous)/1000;previous=now;if(playing)stamp=(stamp+Math.max(0,Math.min(dt,.1)))%action.duration_s;
let width=Math.round(canvas.clientWidth*devicePixelRatio),height=Math.round(canvas.clientHeight*devicePixelRatio);if(canvas.width!==width||canvas.height!==height){canvas.width=width;canvas.height=height;}gl.viewport(0,0,width,height);
gl.clearColor(1,253/255,245/255,1);gl.clear(gl.COLOR_BUFFER_BIT|gl.DEPTH_BUFFER_BIT);gl.enable(gl.DEPTH_TEST);gl.disable(gl.BLEND);gl.uniform1i(u.unlit,0);
let front=[Math.cos(pitch)*Math.cos(yaw),Math.cos(pitch)*Math.sin(yaw),Math.sin(pitch)],right=norm(cross(viewUp,front)),up=cross(front,right);
gl.uniform3fv(u.target,target);gl.uniform3fv(u.front,front);gl.uniform3fv(u.right,right);gl.uniform3fv(u.up,up);gl.uniform1f(u.scale,scale);gl.uniform1f(u.aspect,width/height);
let index=0;for(let i=1;i<action.frames.length;i++){if(action.frames[i].time_s>stamp)break;index=i;}let row=action.frames[index];
for(let binding of bind.bindings){let model=mul(row.links[binding.frame],binding.local_transform);gl.uniformMatrix4fv(u.model,false,uniformMatrix(model));for(let mesh of meshes[binding.mesh]){attribute(at.p,mesh.p);gl.bindBuffer(gl.ELEMENT_ARRAY_BUFFER,mesh.i.buffer);gl.uniform4fv(u.color,mesh.color);gl.drawElements(gl.TRIANGLES,mesh.i.count,mesh.i.type,mesh.i.offset);}}
if(action.number==='09'){gl.disable(gl.DEPTH_TEST);gl.enable(gl.BLEND);gl.blendFunc(gl.SRC_ALPHA,gl.ONE_MINUS_SRC_ALPHA);gl.uniform1i(u.unlit,1);gl.uniformMatrix4fv(u.model,false,uniformMatrix(identity));gl.bindBuffer(gl.ARRAY_BUFFER,pathBuffer);gl.enableVertexAttribArray(at.p);gl.vertexAttribPointer(at.p,3,gl.FLOAT,false,0,0);gl.uniform4fv(u.color,[.05,.5,.65,.4]);gl.drawArrays(gl.LINE_STRIP,0,action.frames.length);gl.uniform4fv(u.color,[.1,.8,1,1]);gl.drawArrays(gl.LINE_STRIP,0,index+1);gl.drawArrays(gl.POINTS,index,1);}
document.querySelector('#time').value=Math.round(stamp/action.duration_s*1000);document.querySelector('#status').textContent=` ${stamp.toFixed(2)} / ${action.duration_s.toFixed(2)} 秒 · J1 ${(row.q_rad[0]*180/Math.PI).toFixed(1)}°`;
requestAnimationFrame(draw);}requestAnimationFrame(draw);
</script></main></html>'''


def lightweight_model():
    """Only simplify browser display geometry; original model/GIFs stay intact."""
    model=json.loads((OUT/'cad/model.gltf').read_text())
    source=json.loads((ROOT/'assets/models/rebot_rs/manifest.json').read_text())
    buffers=[];views=[];accessors=[]
    for index,row in enumerate(source['geometry']):
        reader=vtk.vtkSTLReader();reader.SetFileName(str(ROOT/'assets/models/rebot_rs'/row['file']));reader.Update()
        poly=reader.GetOutput()
        if poly.GetNumberOfPolys()>4000:
            decimate=vtk.vtkQuadricDecimation();decimate.SetInputData(poly)
            decimate.SetTargetReduction(1-4000/poly.GetNumberOfPolys());decimate.Update();poly=decimate.GetOutput()
        positions=vtk_to_numpy(poly.GetPoints().GetData()).astype('<f4')
        cells=vtk_to_numpy(poly.GetPolys().GetData()).reshape(-1,4)
        if not np.all(cells[:,0]==3):raise ValueError('Viewer requires triangles')
        indices=cells[:,1:].astype('<u4').reshape(-1)
        pair=[]
        for values,kind,components in [(positions,5126,'VEC3'),(indices,5125,'SCALAR')]:
            raw=values.tobytes();buffer_id=len(buffers);view_id=len(views);accessor_id=len(accessors)
            buffers.append(dict(byteLength=len(raw),uri='data:application/octet-stream;base64,'+base64.b64encode(raw).decode()))
            views.append(dict(buffer=buffer_id,byteOffset=0,byteLength=len(raw)))
            accessors.append(dict(bufferView=view_id,byteOffset=0,componentType=kind,type=components,count=len(values)))
            pair.append(accessor_id)
        primitive=model['meshes'][index]['primitives'][0]
        primitive['attributes']={'POSITION':pair[0]};primitive['indices']=pair[1]
    model['buffers']=buffers;model['bufferViews']=views;model['accessors']=accessors
    return model


def main():
    payload=dict(gltf=lightweight_model(),
                 bindings=json.loads((OUT/'cad/model_bindings.json').read_text()),
                 motion=json.loads((OUT/'motion.json').read_text()))
    text=json.dumps(payload,separators=(',',':')).replace('</','<\/')
    (OUT/'viewer.html').write_text(HTML.replace('__SCENE__',text))
    print('Wrote self-contained 3D viewer:',OUT/'viewer.html')


if __name__=='__main__':main()
