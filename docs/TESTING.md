# 测试与验收

```bash
cmake -S tests/host -B build-host
cmake --build build-host -j8
ctest --test-dir build-host --output-on-failure
python3 tools/render_previews.py build-host build/ui-preview
```

行为核心使用标准 C，不依赖 ESP-IDF。界面测试需要 `managed_components/lvgl__lvgl`，也可指定 `-DREBUDDY_LVGL_DIR=/path/to/lvgl-9.5.0`；没有 LVGL 时仅跳过 UI 测试。

| 测试 | 覆盖 |
| --- | --- |
| reference_behavior | 触摸去抖、轻点、抚摸、快划、收拢／安抚、头部限定开心、冷却和中断、持续睡眠、停止恢复、30 分钟模拟 |
| ui_rgb565 | 真实 LVGL 指针事件、左右下角入口、菜单开合、头部／身体／眼睛区别、睡眠呼吸和 Z、唤醒、输入取消、边界和稳定堆内存 |
| board_input | 全屏触摸映射、主触点 ID、丢失和故障取消 |
| behavior_config | 当前 JSON 与生成参数头一致 |
| animation_resources | 原图和帧校验、RGB565 格式、房间边缘、头部多边形、动作时序、睡眠底座及色条 |
| project_assets | 10 个当前 GIF 校验、已选原图与预览未变、文档链接完整 |

带 LVGL 时共 6 项，不再运行不属于当前应用的旧电机和关节引擎测试。UI 测试输出实际 PPM 截图，转换后的 `build/ui-preview/contact-sheet.png` 用于布局检查。图片生成和模拟测试不能代替实屏动画效果、触摸手感或硬件连续运行验收。

最新整理验证见 [current.json](verification/current.json)，实机记录使用 [ACCEPTANCE.md](ACCEPTANCE.md)。代码仓库只保留当前验证摘要，构建和测试日志输出到被忽略的 `build/`。
