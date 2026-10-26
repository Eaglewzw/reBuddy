# 快速夹爪动作原画与绑定

[turntable.png](turntable.png) 通过内置 ImageGen（builtin 模式）编辑前版三视角原画，左侧原夹指抬到双眼下方的面部中央；[完整提示词](prompt.txt)。右侧使用[前版独立夹指素材](../shared/gripper/jaws.png)，按固定比例装配后绕安装点旋转。左侧固定在身体图层中，不参与挥动。

[rig.json](rig.json) 记录固定侧、活动侧、安装点、节拍及视角顺序；[poses.png](poses.png) 是最终 GIF 的六帧抽样。

在仓库根目录复现：

```sh
python3 tools/export_claw_sway_preview.py --revision claw-sway-v6
```

结果位于 [独立预览目录](../../../gifs/cat-v6/README.md)。导出检查帧数、总时长、固定背景、安装尺度及活动侧像素变化；底盘三个视角是设计采样，并非连续三维渲染。未接入实机控制。
