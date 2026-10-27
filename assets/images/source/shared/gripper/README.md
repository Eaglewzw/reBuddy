# 共用身体与完整夹爪原画

当前十个基本舞蹈直接复用这里的 `body-flush.png` 与 `jaws-solid.png`，以 daily 外观为基准。有效绑定在 [primitives](../../dance/primitives/README.md)：身体采样比例 156/724，完整夹爪 0.073；本目录的 `rig.json` 及下文旧参数仅作为历史参考，不用于当前导出。

旧版最终导出使用 [body-flush.png](body-flush.png) 和 [jaws-solid.png](jaws-solid.png)：身体已去除脸部下方突出的黑色连接座，每侧夹爪的黄绿色外壳和黑色爪面作为同一个完整图层转动。两侧采用同一固定采样比例，随底盘转向保持整体结构，不再让黑色爪尖在静止外壳上单独转动，也不朝内穿插。源图由内置 ImageGen 编辑，提示词见 [去除连接座](body-flush-prompt.txt) 和 [完整夹爪](jaws-solid-prompt.txt)。

尺寸以 [`daily/01_待机眨眼.gif`](../../../../gifs/daily/01_待机眨眼.gif) 为基准，在 320×240 原始显示尺寸下对比。完整单侧部件使用固定安装比例 0.31，未旋转时约 32×32 像素；两安装点横向间隔 60 源像素（显示约 14 像素）。此旧版设置用于“底盘转拍”及引用它们的“活力律动”合集。其余旧版舞蹈使用已绘制在身体原画中的夹爪，按相同画面尺寸一起复核。

尺寸检查以完整壳体和爪面为单位。不同原画的透视、角度和遮挡会使可见轮廓略有差异，不能把旋转后的外接框逐帧强行缩放到同一个尺寸。daily 原图与播放节拍保持不变。

- [turntable.png](turntable.png)：保留的原始三视角参考，旧版曾改用 `body-solid.png`，最终改为 `body-flush.png`。来源为内置 ImageGen，提示词见 [视角](turntable-prompt.txt)、[方向修正](turntable-correction-prompt.txt) 和 [面板修正](base-panel-prompt.txt)。
- [jaws.png](jaws.png)：早期只包含黑色爪尖的参考素材，旧版最终导出已停用；[原提示词](jaw-prompt.txt)。V6 使用自己的完整捂鼻造型图层，见 [V6 源图](../../cat-v6/README.md)。
- [rig.json](rig.json)：旧版舞蹈装配使用的固定采样比例、旋转中心与安装点。

夹爪排列按用户提供的 [参考截图](../../../reference/gripper-layout-reference.png) 调整：近侧（画面左侧）最后绘制，远侧略向右、向上移动，静止角度改为自然并排。两侧黑色爪面保持可辨认的轮廓，根部允许自然的透视遮挡。底盘转向时序保持不变；整侧转动，不拆分外壳与爪面。[旧版对比图](../../../previews/dance-gripper-size-comparison.png) 展示 daily 基准、调整前和调整后。

当前舞蹈导出工具不重建这里的旧动作；V6 使用独立的 `python3 tools/export_claw_sway_preview.py`。部件不做动态缩放，底盘方向是三个离散采样视角。

夹爪根部直接贴合脸部下缘，由脸部前景图层遮住根部小凸耳。三个视角分别设置安装点；没有额外的黑色圆柱连接座。完整夹爪尺寸、两侧间距和动作节拍保持不变。[衔接对比](../../../previews/dance-wrist-connection.png)。`body-solid.png` 是带连接座的历史源图，当前不再参与导出。
