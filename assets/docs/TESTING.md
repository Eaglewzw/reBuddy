# 测试与验收


```bash
cmake -S tests/host -B build/host
cmake --build build/host -j8
ctest --test-dir build/host --output-on-failure
python3 tools/render_previews.py build/host build/ui-preview
```

行为核心使用标准 C，不依赖 ESP-IDF。界面测试需要 `managed_components/lvgl__lvgl`，也可指定 `-DREBUDDY_LVGL_DIR=/path/to/lvgl-9.5.0`；没有 LVGL 时仅跳过 UI 测试。

| 测试 | 覆盖 |
| --- | --- |
| reference_behavior | 触摸去抖、轻点、抚摸、快划、收拢／安抚、手爪限定开心、冷却和中断、持续睡眠、停止恢复、30 分钟模拟 |
| ui_rgb565 | 真实 LVGL 指针事件、左右下角入口、菜单开合、手爪／身体／眼睛区别、睡眠呼吸和 Z、唤醒、输入取消、边界和稳定堆内存 |
| board_input | 全屏触摸映射、主触点 ID、丢失和故障取消 |
| behavior_config | 当前 JSON 与生成参数头一致 |
| animation_resources | 当前 GIF 解码到 RGB565 的逐帧对应、帧去重与校验、原始 120/130 ms 时间轴、动作映射、手爪触摸区域及房间资源 |
| project_assets | 当前 10 个动作 GIF 、三个独立表情 GIF 及十三个表演／猫步 GIF 校验、必要源素材与文档链接完整 |
| gif_visual_regressions | 待机闭眼无残留、身体稳定、连杆空隙正确透出背景、GIF 总时长和帧延迟 |
| choreography_model | 稳定动作 ID、名称和参数边界、添加排序、视觉过渡、停留、节奏时间轴、停止与完成；动作时长与固件资源一致 |
| choreography_storage | 十槽双记录，保留旧三槽、新增舞蹈 ID 保存与重启、版本与 CRC、失败和中断写入、损坏回退、槽位 ID 校验、加载不写入 |
| dance_assets | 十三个动画确有变化、8 FPS、24 秒串联与源片段一致、背景固定、素材哈希和底座定位 |
| daily_suspend | 编辑期间冻结动作和入睡计时、清除旧手势、恢复睡眠与停止状态、急停优先 |
| choreography_ui | 真实 LVGL 触摸编辑、排序与容量、十二类表演素材、独立猫步预览、保存失败、停止与完成、故障、与日常模式隔离 |

带 LVGL 时共 12 项。UI 测试输出实际 PPM 截图，转换后的 `build/ui-preview/contact-sheet.png` 和 `choreography.png` 用于布局检查。图片生成和模拟测试不能代替实屏动画效果、触摸手感或硬件连续运行验收。

最新整理验证见 [current.json](../verification/current.json)，实机记录使用 [ACCEPTANCE.md](ACCEPTANCE.md)。代码仓库只保留当前验证摘要，构建和测试日志输出到被忽略的 `build/`。
