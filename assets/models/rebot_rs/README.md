当前08入口为 `tools/render_dance_whole_turn.py`：使用完整角色多视角原画，保留原两轮摇摆相位与时长。头部、身体和底盘在同一幅原画中，旧的头部屏幕坐标不再单独约束整体角色。素材与提示词见 `assets/images/source/dance/rigid_turn/`。09仍采用下述 `multiview/` 版本；下文08的头身分部绘图仅为历史方案。

# RS 三维视觉模型

当前 08 / 09 使用 `multiview/` 多视角二维绘图，入口为 `tools/render_dance_multiview.py`。以原绿色角色补绘多个转身方向；待机方向直接用原 `artwork.py` 绘制，其他头部视角由内置 imagegen 按原画补绘。当前已确认头部不变；身体由 `tools/dance_body_views.py` 参考旧GIF重画正面、斜侧面和侧面关键视角，插值轮廓、绿色肩壳和遮挡，底座仍用原矢量绘图。没有三维外壳、材质或光照渲染。

```bash
# 仅在修改转身图集后重新切片、对齐
python3 -B tools/prepare_dance_multiview.py
# 使用现有 motion.json 重绘08和09
python3 -B tools/render_dance_multiview.py
```

需要 Cairo、NumPy、SciPy、Pillow；画廊模块还使用已有的 VTK 导入，但此流程不创建三维场景。视角过渡按二维轮廓、眼睛、关节盖和夹爪特征点插值，每帧只选一张对齐后的原画，不把两张画交叉淡化，避免重影。原待机绘图用作40度中性视角，首尾像素与其他动作的待机图相同。角度是绘图视角的索引，不是测量得到的真实头壳几何。

08 保留原 J1 两轮往返时间轴；09 保留之前已确认的末端屏幕位置、姿态、整段缩放、横向∞和逐步描线。身体按二维关键视角布置连杆，并连接到已确认的头部铰链；它是二维动画适配，不能当作 CAD 逐关节投影。真实运动数据不改。

素材、图集登记及提示词位于 `assets/images/source/dance/multiview/`。生成记录为 `build/dance-3d/multiview/manifest.json`；对照页为 `build/dance-multiview/index.html`，全库入口仍为 `build/dance-original/index.html`。原 GitHub 转身参考为提交 `39097535d956281d3f7fca27d89527a52146327d` 的 `05_base_turn.gif`（离线副本 `github_turn_reference.gif`）；同提交的 `08_wrist.gif` 是俯仰动作。

`reference/` 是先前只保留单一原画朝向的适配版本。`volume/`、`render_dance_volume.py` 和 `dance_character_volume.py` 的立体外壳已被用户否定，不能重新替换当前二维预览；旧几何检查通过不表示外观获认可。

下文的 `toon3d/`、`illustrated/` 是之前的外观试验与运动核对资料，不再作为原预览页08、09的显示素材。

这是离线动画模型，不会连接 CAN。`rebot_rs_visual.urdf` 保留控制模型的六个关节原点、轴向、连杆长度和 TCP 定义，并合入本机 JoyReBot 的 23 个 CAD 外观部件；两片夹爪保持闭合。源文件校验值和各部件的连接关系见 `manifest.json`，网格保存在 `meshes/`。这些外观来自用户本机工程，未附独立的网格许可证；项目代码许可证不自动覆盖这些网格。

来源：

- 控制模型：`lerobot-teleoperator-rebot-vr/.../urdf/rebot_b601_rs.urdf`。
- CAD 外观及网格：`JoyReBot/src/joyrebot_gazebo_sim/urdf/rebot_b601_rs.urdf` 及同工程 `meshes/`。

生成 08、09 的真实连杆变换及离线 GIF：

```bash
cd /home/verser/esp32/reBuddy
/home/verser/Python/lerobot/.venv/bin/python -B tools/export_dance_3d.py
xvfb-run -a -s '-screen 0 800x800x24 -nolisten tcp' python3 -B tools/render_dance_3d.py
python3 -B tools/build_dance_3d_viewer.py
xvfb-run -a -s '-screen 0 1100x1100x24 -nolisten tcp' python3 -B tools/render_dance_toon3d.py
```

需要项目已有的机器人 Python 环境（Pinocchio）和系统 Python 的 VTK、Cairo、NumPy、SciPy、Pillow。无需联网安装，也无需机械臂供电。打开 `build/dance-3d/index.html` 查看原始 CAD 和绿色角色对照；`viewer.html` 为无网络依赖的交互式 CAD 查看器，可拖动视角、暂停和拖动时间轴。

历史 `toon3d/` 试验使用三维实体卡通模型。连杆、关节盖、头壳和手爪均为固定几何网格，按 `世界变换 = 真实连杆FK × 固定安装变换` 驱动；不使用分部件二维锚点、拉伸或朝向相机的广告牌。整幅画面使用同一个透视相机、固定焦距和深度遮挡。外观调整沿用已认可的相机角度和 30.642073° 视场角，播放中不缩放。09 的轨迹直接使用实际 TCP 三维位置，和机械臂一起投影。

该历史模型的显示风格为二维卡通插画：全部材质关闭光照和镜面反光。面罩、月牙眼、眉毛及腮红直接由原 `artwork.face` 绘制，关节盖直接由 `artwork.cap` 绘制，恢复原图的多层圆环、笔触和奶油色边线。头壳、绿色面板和深色外壳使用原画固定的绘画配色渐变，颜色不由灯光或法线计算。`STYLE.render_style` 为 `original_artwork_unlit_illustration`；三维模型只提供运动投影与遮挡。

外壳沿用原角色的绿色、黑色面罩、月牙眼睛、金属圆关节、圆底座和圆润夹爪设计。面部与关节盖直接复用原矢量绘图函数；原画的轮廓已重建为立体外壳，因此不是原图逐像素复制，外壳也不用于物理碰撞检查。

外观比例集中在 `render_dance_toon3d.py` 的 `STYLE` 中：底座半径由 0.128 m 加宽至 0.180 m，并改为有直立侧面的厚圆鼓；三处主要关节半径为 0.086 / 0.080 / 0.084 m。头壳压低、拉宽，面部改为整套原画纹样；取消额外的装饰耳盖，直接用加宽的真实腕部关节盖连接头壳。以上仅改变可视网格，不改 URDF 关节原点、运动数据、时间轴或相机。

`toon3d/08.gif`、`09.gif` 为透明 GIF；`preview/` 为浅色背景预览，`rgba/` 为透明源帧。`model.gltf` 和 `model_bindings.json` 提供完整静态网格与刚性绑定，逐帧动画仍使用 `motion.json`。生成记录见 `toon3d/manifest.json`，检查记录见 `assets/verification/dance_toon3d.json`。仅生成预览，不会写入固件资源。

`illustrated/` 和 `render_dance_illustrated.py` 是上一版二维锚点适配实验，连杆会随投影锚点伸缩，已退出当前预览入口。不要用于判断三维投影效果；其历史检查记录仅适用于旧输出。

上一版 `cartoon/` 是绑定在真实关节上的简化三维装饰外壳，保留用于对照；其中头壳刚性安装在 `gripper_end`，不会自动朝向相机。它不再是原预览页显示的角色。

每个版本导出 `model.gltf` 和 `model_bindings.json`：前者是带材质的完整三维待机模型，后者记录网格到各连杆的绑定。`motion.json` 包含逐帧关节角和世界变换，可按绑定关系驱动模型。GIF 是固定相机渲染，不等同于动力学仿真或实机验收。

交互查看器为提高浏览器速度，仅简化显示网格；完整模型和 GIF 的几何不变。离线检查记录在 `assets/verification/dance_3d.json`，包括逐帧运动学、横向轨迹、GIF 时长与浏览器交互检查。

本目录仅供电脑离线建模，未加入固件构建或 Flash 资源。
