# 睡眠呼吸源图

`breath-atlas.png` 由内置 image_gen 根据 `input-sleep.png` 生成，提示词见 [prompt.txt](prompt.txt)。当前使用第一行 4 张吸气姿态，倒序复用为呼气，避免第二行的局部形状漂移。

`tools/sleep_assets.py` 统一缩放、整数位移对齐底座；`tools/build_screen_preview.py` 将其合成房间背景并放入运行帧 24～27。源时间轴为 4.2 秒；当前全局时长倍率为 2，实机呼吸周期为活泼 8.4 秒、慵懒 10.92 秒；Z 由界面代码绘制，从大到小上浮淡出。

已交付的纯背景 GIF 及校验值保存在 [assets/gifs](../../gifs/README.md)。它们保留当时的评估结果；当前小屋版可通过 `tools/export_action_gifs.py` 重新导出到新目录。
