# 快速夹爪动作原画与绑定

[turntable.png](turntable.png) 保留 V6 原始三视角原画和捂鼻造型，原提示词见 [prompt.txt](prompt.txt)。当前使用 [body-solid.png](body-solid.png)：左侧完整夹爪继续固定捂鼻，右侧原来的空心外壳已移除。右侧使用 [jaws-solid.png](jaws-solid.png) 中与捂鼻侧同造型的完整夹爪，外壳和黑色爪面一起绕腕部转动，尺寸按捂鼻侧校准，始终保持固定比例。

本次使用内置 ImageGen 编辑源图；提示词见 [身体修正](body-solid-prompt.txt)、[完整夹爪](jaws-solid-prompt.txt)。这是 V6 的夹爪结构修正版，等待用户视觉确认，未替换固件中的独立猫步 Demo。

[rig.json](rig.json) 记录固定侧、活动侧、安装点、节拍及视角顺序；[poses.png](poses.png) 是最终 GIF 的六帧抽样。

在仓库根目录复现：

```sh
python3 tools/export_claw_sway_preview.py --revision claw-sway-v6
```

结果位于 [独立预览目录](../../../gifs/cat-v6/README.md)。导出检查帧数、总时长、固定背景、安装尺度及活动侧像素变化；底盘三个视角是设计采样，并非连续三维渲染。未接入实机控制。
