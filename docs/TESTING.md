# 验证方法与边界

核心测试编译正式的行为、手势、动作表和机械臂接口代码。机械臂故障测试只替换底层反馈、目标发送与停止，固件不包含测试假反馈。

| 用例 | 验证内容 |
| --- | --- |
| config / action_schema | 非法配置拒绝、时间单调性、JSON 模式与生成文件一致 |
| actions / styles | 六动作×两种风格、完成与回位、幅度/时长差异、风格切换边界 |
| interrupt / control_timeout | 中断时当前指令连续、每 tick 增量限速、控制延迟锁存停止 |
| sleep / stop | 自动困倦/休息/唤醒、停止清除延迟手势、显式复位且不自动恢复 |
| gestures | 单击、双击/连击、慢抚摸、来回划动、单向快划不误判、取消/无效触摸 |
| arm_timeout / arm_limit / arm_send | 无校准/起始偏差拒绝、反馈过期、非法数值、发送失败、只停止一次、急停与禁止自动重启 |
| soak | 30 分钟模拟时间、90,000 个 20ms tick、密集事件/风格切换/停止复位；核心不使用堆分配 |
| board_input | 全屏坐标映射、边界拒绝、按键消抖、停止优先级、启动与总线故障恢复时不误触发 |
| ui_rgb565（有 LVGL 时） | 同一份正式 UI、真实 LVGL 9.5、RGB565 40 行缓冲、指针点击、按钮和长按、边界检查、重复操作无堆增长、输入设备复位取消手势且不触发按钮 |

UI 用例生成 `idle / curious / happy / lazy / sleep / stopped.ppm`。`tools/render_previews.py` 转成 PNG 与预览总览。画面来自正式 LVGL 渲染，不是设计稿。

```bash
cmake -S tests/host -B build-host -G Ninja
cmake --build build-host -j8
ctest --test-dir build-host --output-on-failure
```

主机测试无需 ESP-IDF。LVGL 不存在时运行 14 项核心、板级逻辑和配置检查；可用 `-DREBUDDY_LVGL_DIR=/path/to/lvgl-9.5.0` 增加第 15 项界面测试。固件与主机测试均仅使用本仓库源码和公开依赖。

验证结果见 [主机测试记录](verification/host-tests.txt) 和 [独立固件构建记录](verification/build-c5.txt)。独立性验证使用单独源码目录和全新构建目录，检查组件路径与编译输入不存在外部应用依赖；此项不替代实机验证。

模拟时间测试在电脑上加速执行，不是墙钟等待 30 分钟，也没有覆盖真实电机、总线、电源、触摸芯片或实体急停链路。最终硬件验收按照 `ACCEPTANCE.md` 记录。
