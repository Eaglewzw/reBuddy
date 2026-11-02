# 工具

| 命令 | 输出 |
| --- | --- |
| `python3 tools/generate_pet_config.py` | 当前行为 JSON → `rb_pet_config.h`；`--check` 只检查 |
| `python3 tools/build_screen_preview.py` | 8 个日常、10 个舞蹈与开场收尾 GIF → 去重压缩的 RGB565 帧、小屋背景、清单与 C 帧表 |
| `python3 tools/check_flash_budget.py --app build/reBuddy.bin` | 检查整个应用与动画预算，保留未来机械臂空间 |
| `python3 tools/build_room_font.py` | Noto Sans CJK → 中文菜单字形子集 |
| `python3 tools/export_gifs_v2.py` | 新版源图和时间轴 → `assets/gifs/daily/` 的 8 个动作 GIF |
| `python3 tools/export_expression_preview.py` | 三关节待机、开心、好奇 → 三个独立表情 GIF，不替换触摸动作 |
| `python3 tools/render_previews.py build/host build/ui-preview` | 实际 LVGL 测试截图 → PNG |
| `python3 tools/package_firmware.py --build build/firmware` | 固件、配置、说明与校验 → `dist/reBuddy-esp32c5/` |

图像工具依赖可用 `python3 -m pip install -r tools/requirements.txt` 安装。字体生成需要 Noto Sans CJK；GIF 效果字形使用 DejaVu Sans。打包在带 esptool 的 IDF Python 环境运行。输出路径可按各工具 `--help` 调整；所有工具均不自动烧录或推送。

新版 GIF 导出按 [逐姿态种子配置](../config/preview_cavity_seeds.json) 清除连杆空隙的源图底色，保留金属高光与眼睛。修改源图或时间轴后，先导出 GIF，再运行 `build_screen_preview.py` 转换并重新编译；不能仅替换 GIF 文件后直接烧录旧固件。

`python3 tools/rs_record_playback.py record/trajectory-1.tch` 仅离线检查录制格式与轨迹约束，需要 NumPy/SciPy。这个旧 TCH 播放器的实机播放因已观察到的下落而锁定，见 [异常记录](RS_PLAYBACK_INCIDENT.md)。

`./tools/play_daily inspect record/gravity/<会话目录>/<片段>.csv` 离线检查新重力录制 CSV；`./tools/play_daily play <同一CSV路径> --supported` 在独立承重支撑下，从当前姿态平滑移动到录制起点、确认到位后试播一次；过渡不包含避障，夹爪保持当前开度。保留同目录元数据和摘要，不需转换 TCH。首次 Ctrl+C 先回零再失能，回零中再次 Ctrl+C 会中止；自然播放结束、SIGTERM 或故障仍失能。见 [CSV 播放说明](GRAVITY_RECORDING.md#直接播放-csv)。

`tools/daily` 提供中文日常动作命令终端，支持列表、离线设计检查、只读电机回读与明确拒绝未验收的播放请求。使用方法和当前限制见 [日常动作终端](DAILY_COMMANDS.md)。

`tools/record_daily` 提供本地 RS 重力补偿拖动录制：默认离线检查，`mock` 验证数据导出，`record --supported` 进入实机交互录制。操作、输出和停机行为见 [重力补偿录制](GRAVITY_RECORDING.md)。
