# 三关节三表情原画

采用内置 imagegen，根据[角色参考](../../reference/character_reference.png)的待机、开心、好奇三种表情，以及[三关节角色基准](../daily/base/idle-master.png)制作。完整生成指令在 [prompt.txt](prompt.txt)，原画为 [atlas.png](atlas.png)。

每个表情单独导出一个循环 GIF，均为 4 秒、32 个显示帧、8 FPS。它们分别是[待机](../../../gifs/expressions/01_三关节待机.gif)、[开心](../../../gifs/expressions/02_三关节开心.gif)、[好奇](../../../gifs/expressions/03_三关节好奇.gif)。三表情不再串在同一个 GIF 中。

选帧都保留靠近底座的肩关节、后侧肘关节和头侧腕关节三个银色圆盘，以及同一完整合拢的双指夹爪。待机选 0、1 格呈现竖椭圆亮眼和眨眼；开心选 0、3、4 格呈现笑眼、抬头抬爪；好奇选 0、12、8 格呈现上望和轻微倾头。为避免夹爪张开后被看成两只手，没有采用第 5 格及第 9–11 格。

[时间轴](timing.json)按选定姿态原路回位，每段首尾都复用第 0 格。关键表情停留多帧，帧延迟交替为 120/130 ms。导出只做图集统一采样和底座整数平移，不做逐帧缩放。

[关键姿态](keyframes.png)可查看三个独立 GIF 的代表帧。原画细节仍有细微差异，关节数量和夹爪形态经过视觉核对，尚不代表可执行的机械臂运动学。

重新导出：`python3 tools/export_expression_preview.py`。旧的合并 GIF 已移入构建目录 `build/expressions-combined-previous/`，不再出现在发布文件夹。
