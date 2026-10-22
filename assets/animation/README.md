# 固件动画资源

- `frames.bin`：33 张 160×160 RGB565 关键姿态，每帧包含 12 字节 LVGL 头，角色背景已与小屋合成。
- `room.bin`：320×240 RGB565 小屋背景。
- `manifest.json`：原图校验值、帧清单、手爪触摸区域、动作时序及睡眠循环。
- `colorbars.bin`：RGB565 格式校验资源，主机测试使用，不嵌入应用。

运行时使用 `main/pet_animation_table.c` 选择姿态，表情绘制在原图中，睡眠 Z 由 UI 绘制。两帧缓存位于 PSRAM，资源失效时回退到待机或占位显示。

使用 `python3 tools/build_screen_preview.py` 从工程内源图重新导出这些资源及 C 帧表。行为参数使用独立的 `tools/generate_pet_config.py` 生成。文件夹中的 GIF 评估素材另见 [assets/gifs](../gifs/README.md)。

封闭在头部与连杆之间的空隙通过 [背景区域配置](../design/screen-preview-v1/cavity-background.json) 单独识别，合成同一屏幕位置的房间像素。白色眼睛与金属高光仍属于角色，保留不变。
