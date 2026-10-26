# 六种新增舞蹈动作

本组围绕身体节拍设计：底盘转拍、侧摆重拍、上下律动、左右交替、连环波浪、舒展收拍。每段有回中收尾，供编排重复和组合；不是日常情绪动作的改名。

[body-atlas.png](body-atlas.png) 使用内置 ImageGen（builtin 模式）生成，三个关节的屈伸改变身体姿态；[初始提示词](prompt.txt)、[夹爪与结构修正](correction-prompt.txt)、[方向修正](pose-correction-prompt.txt) 均随工程保存。房间复用现有素材。底盘转向与左右夹指交替使用[三视角与夹指图层](../../shared/gripper/README.md)。

[rig.json](rig.json) 保存裁剪边界、统一采样比例、落地中心和姿态时间轴。各姿态只按同一个比例采样，然后平移配准到落地中心；不按每帧包围盒重新缩放。图集行间距并不完全相等，因此显式指定实际留白边界，避免裁掉抬起的头部。夹指按固定尺寸绕原安装点旋转。

[poses.png](poses.png) 展示六种动作各三帧。[GIF 文件夹](../../../../gifs/dance/README.md) 含独立片段与 24 秒组合。

复现：

```sh
python3 tools/export_dance_gifs.py
python3 tools/build_screen_preview.py
```

导出实现在 [dance_motion.py](../../../../../tools/dance_motion.py)，固定 320×240、平均 8 FPS；短动作 2 秒、波浪与收拍 4 秒。关键姿态采用停留与切换，尚不是连续三维关节插值；底盘采用三个采样方向。源画不同姿态的轮廓存在手绘差异，固定采样比例并不代表几何精确的机械模型。节奏为屏幕设计估计，未做实机轨迹或电机速度标定。
