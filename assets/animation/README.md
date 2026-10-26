# 固件动画资源

当前资源版本为 `gif-v2-rgb565`，由 [10 个日常 GIF](../gifs/daily/README.md) 和 [13 个舞蹈／猫步 GIF](../gifs/dance/README.md) 生成。

- `frames.bin`：完整 320×240 GIF 帧转换为 RGB565 后去重；每个动作保存独立基准帧，其他帧优先保存经 zlib 压缩的像素差异。表情、问号、Z 与小屋背景均来自 GIF。
- `room.bin`：320×240 RGB565 小屋背景，用于基础界面与回退显示。
- `manifest.json`：源 GIF 校验值、去重帧及压缩大小、手爪触摸区域、动作映射与原始帧时序。

运行时由 `main/pet_animation_table.c` 的时间轴选择图片。两块显示帧缓存加一块基准帧缓存，共占 460,800 字节 PSRAM，比整帧压缩多用 153,600 字节。差异帧只依赖独立基准帧，不串联依赖，支持跳帧、倒放与暂停恢复。压缩解码后的 RGB565 像素与整帧版完全一致。保留 GIF 的 120/130 ms 帧时间，平均 8 FPS；硬件实际刷新率需实机验证。资源失效时回退到待机或占位显示。

使用 `python3 tools/build_screen_preview.py` 从当前 GIF 重新生成资源及 C 帧表，再重新编译固件。若修改原画或时间轴，先运行 `python3 tools/export_gifs_v2.py` 更新 GIF。行为参数使用独立的 `tools/generate_pet_config.py` 生成。

连杆空隙的墙面颜色已在 GIF 导出时通过 [逐姿态种子配置](../config/preview_cavity_seeds.json) 修复；固件转换保留这些像素，不另外重新抠图。
