# 工具

| 命令 | 输出 |
| --- | --- |
| `python3 tools/generate_pet_config.py` | 当前行为 JSON → `rb_pet_config.h`；`--check` 只检查 |
| `python3 tools/build_screen_preview.py` | 当前 10 个日常与 10 个基本舞蹈 GIF → 去重压缩的 RGB565 帧、小屋背景、清单与 C 帧表 |
| `python3 tools/build_room_font.py` | Noto Sans CJK → 中文菜单字形子集 |
| `python3 tools/export_gifs_v2.py` | 新版源图和时间轴 → `assets/gifs/daily/` 的 10 个动作 GIF |
| `python3 tools/export_dance_gifs.py` | 完整角色姿态图集 → 十个独立基本舞蹈 GIF，不生成组合总览 |
| `python3 tools/export_expression_preview.py` | 三关节待机、开心、好奇 → 三个独立表情 GIF，不替换触摸动作 |
| `python3 tools/render_approved_probe.py` | 已选 probe-v2 → `build/approved-probe-preview/`，不改运行资源 |
| `python3 tools/render_previews.py build/host build/ui-preview` | 实际 LVGL 测试截图 → PNG |
| `python3 tools/package_firmware.py --build build/firmware` | 固件、配置、说明与校验 → `dist/reBuddy-esp32c5/` |

图像工具依赖可用 `python3 -m pip install -r tools/requirements.txt` 安装。字体生成需要 Noto Sans CJK；GIF 效果字形使用 DejaVu Sans。打包在带 esptool 的 IDF Python 环境运行。输出路径可按各工具 `--help` 调整；所有工具均不自动烧录或推送。

新版 GIF 导出按 [逐姿态种子配置](../config/preview_cavity_seeds.json) 清除连杆空隙的源图底色，保留金属高光与眼睛。修改源图或时间轴后，先导出 GIF，再运行 `build_screen_preview.py` 转换并重新编译；不能仅替换 GIF 文件后直接烧录旧固件。

舞蹈原画位于 `assets/images/source/dance/individual/`。每张源图集包含 12 个完整角色姿态，时间轴筛选可用姿态并重复停留，输出 3／6 秒、平均 8 FPS 的动画；每图集固定采样比例，底盘落点为 `(155, 190)`，只平移完整角色，不拼接运动部件或添加缩放。图像生成原画的局部尺寸和动作质量仍待视觉验收；导出成功不等于通过硬件验证。
