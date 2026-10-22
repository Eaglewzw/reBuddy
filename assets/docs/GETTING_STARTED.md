# 开发与体验指南

当前为 ESP32-C5 纯触摸屏桌宠。使用 ESP-IDF 5.5.6、LVGL 9.5.0，板级组件随仓库提供，首次构建只下载公开依赖。

## 构建与烧录

```bash
source /path/to/esp-idf/export.sh
idf.py -B build/firmware build
# 由操作者选择实际设备并执行：
idf.py -B build/firmware -p /dev/ttyACM0 flash monitor
```

目标与默认配置见 `sdkconfig.defaults`。已有 `sdkconfig` 不会被 defaults 自动覆盖，需要通过 menuconfig 更新；本版 `CONFIG_ESP_LCD_TOUCH_MAX_BUTTONS=0`。串口不固定为 `/dev/ttyACM0`，请按实际连接选择。

## 体验操作

1. 角色识别区域为 x=80～239、y=40～199；轻点并松手探头。
2. 缓慢抚摸身体可轻蹭；在脸下方的手爪轻柔来回滑动，累计约 0.6 秒有效移动即可请求开心回应。若正在轻蹭，会先平滑结束当前动作；按住不动、摸头或摸眼睛不累计开心。手爪区域随显示姿态变化。开心结束后有 15 秒冷却，需松手至少 2 秒并等待冷却结束才可再次累计。[区域示意](../previews/claw-petting-guide.svg)
3. 1.2 秒内轻点三次收拢，再累计约 1.2 秒有效抚摸展开。展开后先松手再进行下一次互动。
4. 快划触发歪头，同一接触只触发一次；松手后可开始新手势。
5. 无操作 60 秒或设置选择休息后持续睡眠，呼吸和 Z 循环；只有长按左下角恢复按钮才唤醒。
6. 左下暂停后再次点击继续；右下设置切换活泼／慵懒、进入休息。点击齿轮或菜单外区域关闭设置。

## 修改参数与素材

安装 Pillow 后使用下列独立工具：

```bash
python3 tools/generate_pet_config.py
python3 tools/build_screen_preview.py
python3 tools/build_room_font.py
python3 tools/export_action_gifs.py --output build/action-gifs
```

行为参数在 `assets/config/behavior_defaults.json`；`animation_playback.duration_scale=2.0` 表示身体动作、轻蹭回位和睡眠呼吸时长加倍，播放速度为原来的一半。活泼动作时长为基准 × 0.85 × 2，慵懒为基准 × 1.3 × 2；触摸识别时限与暂停响应独立。GIF 默认按活泼风格导出，`--style lazy` 可导出慵懒风格。

逐姿态手爪区域在 `assets/config/claw_touch_regions.json`。图像导出使用 24 格姿态原图和 4 张睡眠姿态，统一缩放、对齐底座，不生成形变补间。中文字体工具需要 Noto Sans CJK，GIF 工具默认使用 DejaVu Sans，也可用 `--font` 指定字体。

工程已保存生成资源，普通构建不需要 Pillow、图片生成服务或本地字体文件。已选 probe-v2 可使用 `tools/render_approved_probe.py` 在 `build/approved-probe-preview/` 单独重现。

## 测试与打包

```bash
cmake -S tests/host -B build/host
cmake --build build/host -j8
ctest --test-dir build/host --output-on-failure
python3 tools/render_previews.py build/host build/ui-preview
# 使用带 esptool 的 ESP-IDF Python 环境：
python3 tools/package_firmware.py --build build/firmware
```

打包输出 `dist/reBuddy-esp32c5/`。目录必须为空，也可指定新的 `--output`。包内 `flash.sh` 先校验 SHA256，再按分区地址写入引导、分区表和应用，保留 NVS/PHY 区间；可选 `merged.bin` 从 0x0 写入会覆盖间隙数据。工具只打包，不烧录、不推送。测试与实机验收说明见 [TESTING.md](TESTING.md)。
