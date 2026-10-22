# 工具

| 命令 | 输出 |
| --- | --- |
| `python3 tools/generate_pet_config.py` | 当前行为 JSON → `rb_pet_config.h`；`--check` 只检查 |
| `python3 tools/build_screen_preview.py` | 当前源图 → 28 帧 RGB565、小屋背景、色条、清单与 C 帧表 |
| `python3 tools/build_room_font.py` | Noto Sans CJK → 中文菜单字形子集 |
| `python3 tools/export_action_gifs.py` | 当前小屋版 10 个 GIF → `build/action-gifs/`，只接受空输出目录 |
| `python3 tools/render_approved_probe.py` | 已选 probe-v2 → `build/approved-probe-preview/`，不改运行资源 |
| `python3 tools/render_previews.py build-host build/ui-preview` | 实际 LVGL 测试截图 → PNG |
| `python3 tools/package_firmware.py --build build` | 固件、配置、说明与校验 → `dist/reBuddy-esp32c5/` |

图像工具依赖可用 `python3 -m pip install -r tools/requirements.txt` 安装。字体生成需要 Noto Sans CJK；GIF 默认使用 DejaVu Sans，可指定 `--font`。打包在带 esptool 的 IDF Python 环境运行。输出路径可按各工具 `--help` 调整；所有工具均不自动烧录或推送。
