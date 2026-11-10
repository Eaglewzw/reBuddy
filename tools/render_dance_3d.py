#!/usr/bin/env python3
"""Render true RS link transforms with VTK. Run with system Python under Xvfb.

No inverse 2D rig, frame deformation, motor commands or online dependencies.
CAD mode uses the installed RS meshes; cartoon mode uses toy shells on the
same rigid links and the original eye artwork on the actual tool frame.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys

import numpy as np
from PIL import Image, ImageDraw, ImageFont
import vtk
from vtk.util.numpy_support import vtk_to_numpy, numpy_to_vtk

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools'))
from render_dance_cartoon import write_gif, BG, FONT


def matrix(values):
    result=vtk.vtkMatrix4x4()
    for i in range(4):
        for j in range(4):result.SetElement(i,j,float(values[i][j]))
    return result


def origin_transform(row):
    from scipy.spatial.transform import Rotation
    value=np.eye(4)
    value[:3,:3]=Rotation.from_euler('xyz',row['rpy']).as_matrix()@np.diag(row['scale'])
    value[:3,3]=row['xyz']
    return value


def texture_image(eye_scale=1.):
    """Reuse the original vector eye/cheek strokes on a front-facing visor."""
    import cairo
    sys.path.insert(0,str(ROOT/'assets/images/source/dance'))
    import artwork as art
    surface=cairo.ImageSurface(cairo.FORMAT_ARGB32,512,400)
    c=cairo.Context(surface);art.color(c,'#18241e');c.paint()
    c.scale(2,2)
    art.eye(c,78,101,22*eye_scale,31*eye_scale,0);art.eye(c,179,101,22*eye_scale,31*eye_scale,0)
    art.line(c,[('M',64,54),('C',72,46,84,46,91,54)],'#edfff3',3)
    art.line(c,[('M',165,54),('C',173,46,185,46,192,54)],'#edfff3',3)
    for x in (53,201):
        art.ellipse(c,x,145,15,9,art.radial(x,145,17,[(0,'#ff9c94',.65),(1,'#ff817e',0)]))
        for off in (-6,0,6):art.line(c,[('M',x+off,141),('L',x+off+2,148)],'#ff8677',2)
    surface.flush()
    rgb=np.frombuffer(surface.get_data(),np.uint8).reshape(400,512,4)[:,:,[2,1,0]].copy()
    image=vtk.vtkImageData();image.SetDimensions(512,400,1)
    image.GetPointData().SetScalars(numpy_to_vtk(np.flipud(rgb).reshape(-1,3),deep=True,array_type=vtk.VTK_UNSIGNED_CHAR))
    texture=vtk.vtkTexture();texture.SetInputData(image);texture.InterpolateOn()
    return texture,Image.fromarray(rgb)


class Scene:
    def __init__(self,manifest,mode,size=640):
        self.mode=mode;self.size=size;self.bindings=[];self.head_actors=[]
        model=ROOT/manifest['model'];self.model=json.loads((model/'manifest.json').read_text())
        self.renderer=vtk.vtkRenderer();self.renderer.SetBackground(*(x/255 for x in BG));self.renderer.SetBackgroundAlpha(0)
        self.window=vtk.vtkRenderWindow();self.window.SetOffScreenRendering(1)
        self.window.SetAlphaBitPlanes(1);self.window.SetMultiSamples(0)
        self.window.SetSize(size,size);self.window.AddRenderer(self.renderer)
        self.renderer.SetUseFXAA(True)
        cam=self.renderer.GetActiveCamera();cfg=manifest['camera'];self.default_camera=cfg
        target=np.array(cfg['target']);direction=np.array(cfg['direction']);direction/=np.linalg.norm(direction)
        cam.SetPosition(*(target+direction*2));cam.SetFocalPoint(*target);cam.SetViewUp(*cfg['up'])
        cam.ParallelProjectionOn();cam.SetParallelScale(cfg['parallel_scale']);cam.SetClippingRange(.1,5)
        self.renderer.SetAmbient(.5,.5,.5)
        for direction,intensity in [([1,-1,2],.8),([-1,-.3,.8],.4)]:
            light=vtk.vtkLight();light.SetLightTypeToSceneLight();light.SetPosition(*direction)
            light.SetFocalPoint(*target);light.SetIntensity(intensity);self.renderer.AddLight(light)
        for row in self.model['geometry']:
            if mode=='cartoon':continue
            source=model/row['file']
            if hashlib.sha256(source.read_bytes()).hexdigest()!=row['source_sha256']:raise ValueError(f'Changed mesh: {source}')
            reader=vtk.vtkSTLReader();reader.SetFileName(str(source));reader.Update()
            normals=vtk.vtkPolyDataNormals();normals.SetInputConnection(reader.GetOutputPort())
            normals.SetFeatureAngle(55);normals.SplittingOn();normals.Update()
            name=source.name
            color=(.76,.96,.09) if 'green' in name else (.10,.12,.11) if 'black' in name or 'motor' in name else (.68,.70,.66)
            if name in ('base_link.STL','link1.STL'):color=(.19,.21,.19)
            actor=self.add_poly(normals.GetOutput(),color)
            self.bindings.append((actor,row['link'],origin_transform(row)))
        if mode=='cartoon':
            self.add_shells()
            self.add_head()
        # A separate overlay layer makes the trajectory annotation legible even
        # where the arm occludes it. Its coordinates remain actual world TCPs.
        self.window.SetNumberOfLayers(2)
        self.overlay=vtk.vtkRenderer();self.overlay.SetLayer(1);self.overlay.SetActiveCamera(cam)
        self.overlay.SetPreserveDepthBuffer(False);self.overlay.InteractiveOff();self.window.AddRenderer(self.overlay)
        self.trace_actors=[]
        self.capture=vtk.vtkWindowToImageFilter();self.capture.SetInput(self.window)
        self.capture.SetInputBufferTypeToRGBA();self.capture.ReadFrontBufferOff()

    def add_poly(self,poly,color,renderer=None):
        mapper=vtk.vtkPolyDataMapper();mapper.SetInputData(poly)
        actor=vtk.vtkActor();actor.SetMapper(mapper);prop=actor.GetProperty()
        prop.SetColor(*color);prop.SetAmbient(.3);prop.SetDiffuse(.7)
        prop.SetSpecular(.22);prop.SetSpecularPower(30);prop.SetInterpolationToPhong()
        (renderer or self.renderer).AddActor(actor)
        return actor

    def cylinder(self,start,end,radius,color,link='base_link'):
        start=np.array(start,float);end=np.array(end,float);axis=end-start;length=np.linalg.norm(axis)
        if length<1e-8:raise ValueError('Empty decorative cylinder')
        direction=axis/length;seed=np.array([1.,0,0]) if abs(direction[0])<.9 else np.array([0.,0,1.])
        x=np.cross(direction,seed);x/=np.linalg.norm(x);z=np.cross(x,direction)
        value=np.eye(4);value[:3,:3]=np.column_stack([x,direction,z]);value[:3,3]=(start+end)/2
        primitive=vtk.vtkCylinderSource();primitive.SetRadius(radius);primitive.SetHeight(length)
        primitive.SetResolution(64);primitive.CappingOn();primitive.Update()
        actor=self.add_poly(primitive.GetOutput(),color);self.bindings.append((actor,link,value))
        return actor

    def add_shells(self):
        """Toy shells use the real joint centers and rigid URDF link frames."""
        green=(.82,1.,.075);dark=(.12,.14,.12);silver=(.82,.80,.71)
        self.cylinder([0,0,.008],[0,0,.072],.105,dark)
        self.cylinder([0,0,.073],[0,0,.081],.102,silver)
        self.cylinder([0,0,.016],[0,0,.065],.106,green)
        self.cylinder([0,0,0],[.020343,.027237,.07],.051,dark,'link1')
        for link,end,radius in [('link2',[-.236,0,0],.037),('link3',[.228,-.072746,.0045],.037),
                                ('link4',[.087,-.048,-.03075],.029),('link5',[.0365,0,.048],.024)]:
            self.cylinder([0,0,0],end,radius,dark,link)
            offset=np.array([0,0,-radius*.65])
            self.cylinder(offset,np.array(end)+offset,radius*.7,green,link)
        for link,radius in [('link2',.051),('link3',.049),('link4',.043),('link5',.028),('link6',.028)]:
            self.cylinder([0,0,-.043],[0,0,.043],radius,dark,link)
            for sign in (-1,1):
                self.cylinder([0,0,sign*.043],[0,0,sign*.049],radius*.94,silver,link)
                cap=self.cylinder([0,0,sign*.049],[0,0,sign*.051],radius*.72,(.91,.89,.81),link)
                cap.GetProperty().SetSpecular(.7);cap.GetProperty().SetSpecularPower(55)
        for side in (-1,1):
            self.cylinder([-.085,side*.025,0],[-.035,side*.018,0],.017,green,'gripper_end')
            self.cylinder([-.035,side*.018,0],[.002,side*.009,0],.014,dark,'gripper_end')

    def add_head(self):
        # Decorative shell mounted rigidly on the gripper_end frame. It never
        # independently faces the camera or cancels the tool's true rotation.
        center=np.array([-.075,0,.085]);radii=np.array([.075,.085,.080])
        sphere=vtk.vtkSphereSource();sphere.SetThetaResolution(64);sphere.SetPhiResolution(48);sphere.Update()
        transform=vtk.vtkTransform();transform.Translate(*center);transform.Scale(*(radii*2))
        filt=vtk.vtkTransformPolyDataFilter();filt.SetInputConnection(sphere.GetOutputPort());filt.SetTransform(transform);filt.Update()
        self.head_actors.append(self.add_poly(filt.GetOutput(),(.82,1.,.08)))
        # Rounded visor surface on the front of the ellipsoid, with UV texture.
        points=vtk.vtkPoints();uv=vtk.vtkFloatArray();uv.SetNumberOfComponents(2);faces=vtk.vtkCellArray()
        rings=24;segments=96
        for j in range(rings+1):
            radius=j/rings
            for i in range(segments+1):
                angle=2*math.pi*i/segments
                y=.074*radius*math.cos(angle);z=.059*radius*math.sin(angle)
                x=radii[0]*math.sqrt(max(0,1-(y/radii[1])**2-(z/radii[2])**2))+.001
                points.InsertNextPoint(*(center+[x,y,z]));uv.InsertNextTuple((.5-y/.148,.5+z/.118))
        for j in range(rings):
            for i in range(segments):
                a=j*(segments+1)+i;b=a+segments+1
                for ids in [(a,b,b+1),(a,b+1,a+1)]:
                    faces.InsertNextCell(3)
                    for index in ids:faces.InsertCellPoint(index)
        mesh=vtk.vtkPolyData();mesh.SetPoints(points);mesh.SetPolys(faces);mesh.GetPointData().SetTCoords(uv)
        actor=self.add_poly(mesh,(1,1,1));texture,self.face_image=texture_image();actor.SetTexture(texture)
        actor.GetProperty().SetAmbient(.85);actor.GetProperty().SetDiffuse(.15);actor.GetProperty().SetSpecular(0)
        self.head_actors.append(actor)

    def set_pose(self,row):
        for actor,link,origin in self.bindings:
            actor.SetUserMatrix(matrix(np.array(row['links'][link])@origin))
        for actor in self.head_actors:actor.SetUserMatrix(matrix(row['links']['gripper_end']))

    def set_camera(self,cfg):
        cam=self.renderer.GetActiveCamera();target=np.array(cfg['target'])
        direction=np.array(cfg['direction']);direction/=np.linalg.norm(direction)
        cam.SetPosition(*(target+direction*2));cam.SetFocalPoint(*target)
        cam.SetViewUp(*cfg['up']);cam.SetParallelScale(cfg['parallel_scale'])

    def line(self,points,color,width,opacity):
        pts=vtk.vtkPoints()
        for p in points:pts.InsertNextPoint(*p)
        cells=vtk.vtkCellArray();cells.InsertNextCell(len(points))
        for i in range(len(points)):cells.InsertCellPoint(i)
        mesh=vtk.vtkPolyData();mesh.SetPoints(pts);mesh.SetLines(cells)
        actor=self.add_poly(mesh,color,self.overlay);actor.GetProperty().SetLighting(False)
        actor.GetProperty().SetLineWidth(width);actor.GetProperty().SetOpacity(opacity)
        self.trace_actors.append(actor)

    def set_trace(self,action,index):
        for actor in self.trace_actors:self.overlay.RemoveActor(actor)
        self.trace_actors=[]
        if action['number']!='09':return
        points=np.array([row['links']['gripper_end'] for row in action['frames']])[:,:3,3]
        t=action['frames'][index]['time_s'];fade=min(1,t/.3,(action['duration_s']-t)/.3)
        if fade<=0:return
        self.line(points,(.10,.50,.63),2,.45*fade)
        if index:self.line(points[:index+1],(.12,.84,1),3,fade)
        sphere=vtk.vtkSphereSource();sphere.SetCenter(*points[index]);sphere.SetRadius(.004);sphere.Update()
        actor=self.add_poly(sphere.GetOutput(),(.9,1,1),self.overlay);actor.GetProperty().LightingOff()
        self.trace_actors.append(actor)

    def image(self):
        self.window.Render();self.capture.Modified();self.capture.Update()
        raw=vtk_to_numpy(self.capture.GetOutput().GetPointData().GetScalars()).reshape(self.size,self.size,4)
        return Image.fromarray(np.flipud(raw).copy(),'RGBA')

    def export_model(self,path):
        self.window.Render()
        exporter=vtk.vtkGLTFExporter();exporter.SetRenderWindow(self.window)
        exporter.SetFileName(str(path));exporter.InlineDataOn();exporter.SaveNormalOn();exporter.Write()
        bindings=[dict(mesh=i,frame=link,local_transform=origin.tolist())
                  for i,(_,link,origin) in enumerate(self.bindings)]
        bindings.extend(dict(mesh=len(self.bindings)+i,frame='gripper_end',local_transform=np.eye(4).tolist())
                        for i in range(len(self.head_actors)))
        path.with_name('model_bindings.json').write_text(json.dumps(dict(
            schema='rebuddy.gltf.link_bindings.v1',model=path.name,bindings=bindings,
            transform_rule='node world matrix = motion frame links[frame] @ local_transform',
            units='m',head_is_decorative=self.mode=='cartoon'),indent=2)+'\n')


def gallery(out,manifest):
    cards=[]
    whole_turn=(out/'whole_turn/manifest.json').exists() and (out/'whole_turn/preview/08.gif').exists()
    multiview=(out/'multiview'/'manifest.json').exists() and all((out/'multiview'/'preview'/f'{code}.gif').exists() for code in ('08','09'))
    # The volume candidate was rejected for changing the character's style.
    # Its presence on disk must never override the original 2D artwork.
    reference=all((out/'reference'/'preview'/f'{code}.gif').exists() for code in ('08','09'))
    toon3d=all((out/'toon3d'/'preview'/f'{code}.gif').exists() for code in ('08','09'))
    illustrated=not toon3d and all((out/'illustrated'/'preview'/f'{code}.gif').exists() for code in ('08','09'))
    download_folder='reference' if reference else 'toon3d' if toon3d else 'illustrated' if illustrated else 'cartoon'
    skin_folder=download_folder+'/preview' if reference or toon3d or illustrated else 'cartoon'
    skin_title='完整原角色 · 二维卡通动画' if reference else '原画细节 · 二维卡通画面' if toon3d else '原版绿色角色：3D动作数据驱动' if illustrated else '3D绿色角色：真实关节运动'
    if multiview:
        download_folder='multiview';skin_folder='multiview/preview'
        skin_title='原角色多视角二维绘图'
    for action in manifest['actions']:
        code=action['number']
        action_skin='whole_turn/preview' if whole_turn and code=='08' else skin_folder
        action_title='完整角色多视角二维绘图' if whole_turn and code=='08' else skin_title
        version=hashlib.sha256((out/action_skin/f'{code}.gif').read_bytes()).hexdigest()[:12]
        cards.append(f'<section><h2>{code} {action["name"]} · {action["duration_s"]:.2f}秒</h2><div class="pair"><figure><img src="{action_skin}/{code}.gif?v={version}"><figcaption>{action_title}</figcaption></figure><figure><img src="cad/{code}.gif"><figcaption>RS原始CAD：核对运动与补偿</figcaption></figure></div></section>')
    (out/'index.html').write_text('<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>reBuddy 3D动作验证</title><style>body{background:#fffdf5;color:#26342a;font:16px system-ui;margin:24px auto;max-width:1100px;padding:0 20px}.pair{display:flex;flex-wrap:wrap;gap:16px}figure{margin:0;flex:1;min-width:280px}img{width:100%;max-width:520px}figcaption{padding:8px}a{color:#376b43}p{line-height:1.7}</style><h1>08 / 09 · 真实三维动作验证</h1><p>固定相机，按原动作时间逐帧驱动六轴。08保留底座摆动和腕部朝向补偿；09显示横向∞轨迹，蓝线与光点来自模型TCP。</p><p><a href="viewer.html">打开可旋转三维预览：拖动视角、暂停、拖动时间轴</a></p>'+''.join(cards)+'<p>绿色角色的头壳是装饰模型，刚性安装在工具坐标系；真实尺寸请看右侧CAD。夹爪保持闭合。本页用于离线运动核对，未连接机械臂。</p><p><a href="cartoon/model.gltf">下载绿色3D模型</a> · <a href="cad/model.gltf">下载RS CAD模型</a> · <a href="motion.json">关节与连杆变换数据</a></p></html>')
    if toon3d:
        page=(out/'index.html').read_text().replace('cartoon/model.gltf', 'toon3d/model.gltf')
        page=page.replace('绿色角色的头壳是装饰模型，刚性安装在工具坐标系；真实尺寸请看右侧CAD。夹爪保持闭合。本页用于离线运动核对，未连接机械臂。', '左侧复用其他动作的面罩、月牙眼、眉毛腮红和银色关节原画，使用原角色的固定绘画配色，不使用立体光照。运动与遮挡保持已确认的投影；右侧为CAD动作对照。')
        (out/'index.html').write_text(page)
    elif illustrated:
        page=(out/'index.html').read_text()
        page=page.replace('绿色角色的头壳是装饰模型，刚性安装在工具坐标系；真实尺寸请看右侧CAD。夹爪保持闭合。本页用于离线运动核对，未连接机械臂。',
                          '左侧复用原版绿色卡通原画：三维关节位移和末端姿态驱动原比例部件，腕部藏在大头内部；右侧保留原始CAD用于核对动作。夹爪保持闭合。')
        page=page.replace('下载绿色3D模型', '旧版简化3D外壳模型（非当前卡通原画）')
        (out/'index.html').write_text(page)
    if reference:
        page=(out/'index.html').read_text()
        page=page.replace('左侧复用其他动作的面罩、月牙眼、眉毛腮红和银色关节原画，使用原角色的固定绘画配色，不使用立体光照。运动与遮挡保持已确认的投影；右侧为CAD动作对照。', '左侧完整复用其他动作的原角色，待机画面一致；保留原连杆长度与角色比例，09轨迹按整段固定比例适配。右侧为实机模型对照。')
        page=page.replace('下载绿色3D模型', '运动参考模型（非左侧原画）')
        (out/'index.html').write_text(page)
    if multiview:
        page=(out/'index.html').read_text()
        page=page.replace('左侧完整复用其他动作的原角色，待机画面一致；保留原连杆长度与角色比例，09轨迹按整段固定比例适配。右侧为实机模型对照。',
                          '左侧为多视角二维绘图：以原画为基准补绘转身视角，原始待机图作为中性姿态，头部保持已确认版本；身体按旧GIF的正面、斜侧面和侧面重画，底座保留原画。视角之间按二维特征点衔接，不使用立体外壳或光照。末端屏幕路径与时间轴保持原适配结果，内部关节为二维角色重定姿。右侧仅作实机动作对照。')
        page=page.replace('固定相机，按原动作时间逐帧驱动六轴。08保留底座摆动和腕部朝向补偿；09显示横向∞轨迹，蓝线与光点来自模型TCP。',
                          '08保留原两轮摇摆节奏，以不同方向的二维绘图表现转身。09保留现有横向∞路径、逐步描线和光点。左侧是二维角色适配，不是右侧CAD的直接渲染。')
        (out/'index.html').write_text(page)
    if whole_turn:
        import re
        page=(out/'index.html').read_text()
        page=re.sub(r'<p>左侧为多视角二维绘图：.*?</p>',
                    '<p>08由完整角色的多视角原画制作，底盘、肩肘、连杆与头部共同绘制，保持原两轮摇摆节奏。09保留当前横向∞版本。二维绘图用于角色动画；右侧为实机模型对照。</p>',page)
        (out/'index.html').write_text(page)
    # Update the page already used for review, while keeping the standalone
    # 3D candidates separate from the firmware's uniform 2D resource family.
    original=ROOT/'build/dance-original'
    if (original/'manifest.json').exists() and all((out/'cartoon'/f'{code}.gif').exists() for code in ('08','09')):
        import re
        from render_dance_cartoon import write_gallery
        write_gallery(original,json.loads((original/'manifest.json').read_text()))
        page=(original/'index.html').read_text()
        for code in ('08','09'):
            action_skin='whole_turn/preview' if whole_turn and code=='08' else skin_folder
            action_download='whole_turn' if whole_turn and code=='08' else download_folder
            version=hashlib.sha256((out/action_skin/f'{code}.gif').read_bytes()).hexdigest()[:12]
            page=re.sub(r'(src|href)="(?:preview/)?'+code+r'\.gif(?:\?[^\"]*)?"',
                        lambda match:f'{match.group(1)}="../dance-3d/{action_skin if match.group(1)=="src" else action_download}/{code}.gif?v={version}"',page)
            if not (illustrated or toon3d):
                page=re.sub(r'(<a href="\.\./dance-3d/cartoon/'+code+r'\.gif[^\"]*">)[^<]+',
                            r'\g<1>640px 三维 GIF',page)
        page=page.replace('这是原画二维动画适配；末端路径和节奏来自当前实机动作，不代表真实机构的三维几何仿真。',
                          '08、09 由三维关节位移和末端姿态驱动原版卡通部件，保留原角色比例；其他动作使用原二维动画适配。')
        if toon3d:
            page=page.replace('08、09 由三维关节位移和末端姿态驱动原版卡通部件，保留原角色比例；其他动作使用原二维动画适配。', '08、09 已改为二维卡通插画效果；动作投影和连杆长度不变，其他动作保持原版画面。')
        note='<p style="padding:12px;background:#e5f0dc;border-radius:10px">08 / 09 已调整：原版绿色卡通外观，保留3D动作轨迹；09为横向∞。<a href="../dance-3d/index.html">查看CAD对照</a> · <a href="../dance-3d/viewer.html">旋转3D模型</a>。本次更新为预览素材，尚未替换固件。</p>' if illustrated else '<p style="padding:12px;background:#e5f0dc;border-radius:10px">08 / 09 已更新为真实关节驱动的三维验证版，其他动作仍为原二维素材。<a href="../dance-3d/index.html">查看CAD对照</a> · <a href="../dance-3d/viewer.html">旋转3D模型</a>。三维候选尚未替换固件素材。</p>'
        if toon3d:
            note='<p style="padding:12px;background:#e5f0dc;border-radius:10px">08 / 09 已调整原角色画风：复用原面罩、眼睛和银色关节绘图，保留当前动作投影和横向∞轨迹。<a href="../dance-3d/index.html">查看CAD对照</a>。当前为预览素材，尚未替换固件。</p>'
        if reference:
            page=page.replace('08、09 已改为二维卡通插画效果；动作投影和连杆长度不变，其他动作保持原版画面。', '08、09 与其他动作共用完整原角色；保留原画比例，动作轨迹按固定比例适配。')
            note='<p style="padding:12px;background:#e5f0dc;border-radius:10px">08 / 09 已撤回3D外壳，恢复原角色的2D原画版；09保留横向∞。这是外观回退，转身的“贴纸感”仍待通过同角色多视角绘图改善。</p>'
        if multiview:
            page=page.replace('08、09 与其他动作共用完整原角色；保留原画比例，动作轨迹按固定比例适配。',
                              '08、09 按原角色补绘多个二维转身视角，原待机图作为首尾；头部保持已确认版本，肩壳、连杆和肘部按旧GIF重画多视角，底座、末端路径与节奏保持不变。')
            note='<p style="padding:12px;background:#e5f0dc;border-radius:10px">08 / 09 身体已按旧GIF重画多视角：肩壳、绿色面板和肘部遮挡随转身变化；保留已确认的头部、动作节奏与09横向∞。<a href="../dance-multiview/index.html">查看旧GIF参考与关键帧</a>。当前为预览素材。</p>'
        if whole_turn:
            page=re.sub(r'<small>.*?</small>', '<small>08采用完整角色的多视角绘图，09保留横向∞动画，其他动作保持原版画面。</small>',page,count=1)
            note='<p style="padding:12px;background:#e5f0dc;border-radius:10px">08 已重做为整体多视角动画：底盘、肩肘、连杆和头部一起绘制。<a href="../dance-whole-turn/index.html">查看完整原画、过渡帧和转身预览</a>。09保持原版本。</p>'
        page=page.replace('<h1>reBuddy · Dance 舞蹈预览</h1>','<h1>reBuddy · Dance 舞蹈预览</h1>'+note)
        (original/'index.html').write_text(page)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input',type=Path,default=ROOT/'build/dance-3d/motion.json')
    parser.add_argument('--mode',choices=['cad','cartoon','both'],default='both')
    parser.add_argument('--stills-only',action='store_true')
    args=parser.parse_args();manifest=json.loads(args.input.read_text());out=args.input.parent
    for mode in (['cad','cartoon'] if args.mode=='both' else [args.mode]):
        folder=out/mode;folder.mkdir(exist_ok=True);scene=Scene(manifest,mode)
        scene.set_pose(manifest['actions'][0]['frames'][0]);scene.image().save(folder/'idle.png')
        scene.export_model(folder/'model.gltf')
        if mode=='cartoon':scene.face_image.save(folder/'face.png')
        for action in manifest['actions']:
            scene.set_camera(action.get('camera',manifest['camera']))
            code=action['number'];frames=[]
            if code=='08':
                joints=np.array([row['q_rad'][0] for row in action['frames']])
                representatives=[0,int(joints.argmin()),int(joints.argmax())]
            else:representatives=[0,len(action['frames'])//4,len(action['frames'])*3//4]
            indices=representatives if args.stills_only else range(len(action['frames']))
            for index in indices:
                scene.set_pose(action['frames'][index]);scene.set_trace(action,index);rgba=scene.image()
                base=Image.new('RGBA',rgba.size,BG+(255,));base.alpha_composite(rgba);frames.append(base)
                if index in representatives:base.save(folder/f'{code}-{index:03d}.png')
            if not args.stills_only:write_gif(frames,folder/f'{code}.gif',action['frame_durations_ms'])
            print(mode,code,'rendered',len(frames),flush=True)
        scene.window.Finalize()
    gallery(out,manifest)


if __name__=='__main__':main()
