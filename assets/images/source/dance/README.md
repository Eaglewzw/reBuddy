# 原舞蹈角色素材

这些文件来自用户本机的 `robot_dance_2d_perspective_v4.zip`，保留原脸部、金属关节、连杆和夹爪画法。原压缩包、八个源文件及匹配旧 GIF 的 SHA-256 记录在 `provenance.json`。其中的本机路径仅用于溯源，重建不需要访问该压缩包。

原包的 `motions.json`、`export.py` 等文件用于保存来源，不是当前十组动作的入口。当前入口为仓库的 `tools/render_dance_cartoon.py`：读取 `record/dance_library`，通过二维关节映射驱动这里的 `artwork.py`，输出透明 GIF、RGBA PNG 和预览页。08、09 现由 `tools/render_dance_multiview.py` 单独生成并覆盖预览入口：以同一角色的多视角二维绘图表现转身，素材在 `multiview/`。08 保留原两轮摇摆时序，09 保留当前相机已确认的横向∞屏幕路径、淡色引导线、逐步亮线和光点，首尾保持原待机图。旧 `render_dance_cartoon.py` 内的08/09映射属于历史版本，不作为当前预览依据。10 使用固定虚拟支点的二维约束。这些仅是卡通动作适配，不修改实机数据，也不是实机三维几何仿真。

在仓库根目录生成全部十组及开场、收尾：

```bash
/home/verser/Python/lerobot/.venv/bin/python -B tools/render_dance_cartoon.py
```

轨迹导出使用项目已有的机器人 Python 环境、URDF 和校准文件；原画渲染调用系统 `/usr/bin/python3`，需要 Cairo、NumPy、SciPy、Pillow 和 Noto CJK 字体。默认输出为 `build/dance-original/`，打开其中的 `index.html` 即可查看全库动画。程序不连接 CAN，也不使能机械臂。

已经导出 `motion.json` 后，可只重绘图像，无需模型和机器人环境：

```bash
/usr/bin/python3 -B tools/render_dance_cartoon.py --render-json build/dance-original/motion.json
```

审核后按 `assets/gifs/dance/README.md` 安装素材、生成屏幕资源并构建固件。640×640 原始 GIF 和 RGBA 输入不随板端压缩参数降质。

08当前使用 `rigid_turn/` 中完整角色的多视角原画，由 `tools/render_dance_whole_turn.py` 生成；其原摇摆相位和时长保留，头身分部组合版已退出08当前预览。09继续使用 `multiview/`。
