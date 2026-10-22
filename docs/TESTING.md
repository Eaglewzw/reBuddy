# 测试与证据

```bash
cmake -S tests/host -B build-host
cmake --build build-host -j8
ctest --test-dir build-host --output-on-failure
python3 tools/render_previews.py build-host build/ui-preview
```

行为测试无需 ESP-IDF。真实 UI 使用 `managed_components/lvgl__lvgl`，或配置 `-DREBUDDY_LVGL_DIR=/path/to/lvgl-9.5.0`。没有 LVGL 时跳过界面测试，其余测试仍可运行。

| 当前用例 | 覆盖范围 |
| --- | --- |
| reference_behavior | 按下/抬起去抖、松手探头、静止不计时、轻蹭退出、快速划动一次/接触、三击收拢、安抚恢复与点击中断、开心冷却/重置及待处理开心被点击取消、停顿/中间速度带、越界取消、入睡中唤醒与接触消费、STOP/急停显式复位、迟到 tick、风格、30 分钟模拟输入 |
| ui_rgb565 | 正式 LVGL UI + 双40行RGB565缓冲，真实指针事件，设置、休息/唤醒、停止冻结、长按复位、输入取消、对象边界、100次不变更新零像素发送、重复操作无持续堆增长；坏头/缺帧/缓存不足回退 |
| board_input | 全屏坐标映射、主触点ID锁定/丢失取消/故障、按键消抖、STOP优先 |
| animation_resources | PNG来源校验、所有二进制帧头/范围/像素哈希、底座固定、片段索引与锚点、RGB565色条和生成配置一致 |
| 旧模块14项回归 | 旧动作配置/轨迹/风格/手势/生命感及机械臂接口错误路径；不代表当前应用仍运行旧行为 |

`ui_rgb565` 生成九张 PPM，转换后 `build/ui-preview/contact-sheet.png` 为现有旧原型UI渲染截图。截图可检查布局，但不证明动态锚点全程无漂移或实屏颜色正确。

本轮记录：`verification/reference-host-tests.txt`、`verification/reference-build-c5.txt`、`verification/reference-budget.txt`。本轮共 18 项（包含 LVGL 时），以最终日志为准。更早版本的重复日志已清理。目录整理后的回归结果见 `verification/cleanup-host-tests.txt`。

模拟30分钟在电脑上快速推进时间；真实触摸、PSRAM分配压力、SPI时序、电源及硬件急停均需实机复测。实机验收模板见 [ACCEPTANCE.md](ACCEPTANCE.md)，统计口径见 [DISPLAY_PERFORMANCE.md](DISPLAY_PERFORMANCE.md)。
