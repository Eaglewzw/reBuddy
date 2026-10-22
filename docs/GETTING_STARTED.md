# 开发与体验指南

选定素材与旧固件的区别见 [ASSET_STATUS.md](ASSET_STATUS.md)。当前可构建的旧原型实现依据 [资料包对照记录](REFERENCE_IMPLEMENTATION.md)，ESP-IDF 5.5.5 / ESP32-C5 / LVGL 9.5.0。板级支持随仓库提供；首次构建需下载公开组件。

## 构建与烧录

```bash
source /path/to/esp-idf-v5.5.5/export.sh
idf.py -B build reconfigure build
# 需要上板测试时执行，串口按实际设备选择：
idf.py -B build -p /dev/ttyACM0 flash monitor
```

已有 sdkconfig 不会被 defaults 自动覆盖，需通过 `idf.py -B build menuconfig` 调整。旧构建目录仍可使用，但所有命令应指定同一 `-B`。当前产物为 `build/reBuddy.bin`；本轮开发没有执行烧录。

此机器采用 EIM 工具布局时，可使用已安装环境运行（路径按机器调整）：

```bash
export IDF_PATH=/home/verser/.espressif/v5.5.5/esp-idf
export IDF_TOOLS_PATH=/home/verser/.espressif/tools
export IDF_PYTHON_ENV_PATH=/home/verser/.espressif/tools/python/v5.5.5/venv
export PATH="$IDF_PYTHON_ENV_PATH/bin:$IDF_TOOLS_PATH/ninja/1.12.1:$IDF_TOOLS_PATH/riscv32-esp-elf/esp-14.2.0_20260121/riscv32-esp-elf/bin:$PATH"
python3 "$IDF_PATH/tools/idf.py" -B build build
```

## 测试操作

1. 角色区域为 x=80～239、y=40～199；轻点并松手应探头，静止按住只跟随视线。
2. 缓慢往返滑动 1 秒后松手，观察轻蹭回位；有效移动累计约 3 秒出现一次开心。停住不计时。
3. 1.2 秒内轻点三次，收拢后继续点击不应再次退缩；温柔抚摸约 1.2 秒后展开，该接触不会立即开心。
4. 快速划动触发歪头，同一接触继续滑动不应反复触发。
5. 互动结束后无操作 60 秒入睡，首次角色接触只唤醒；空白区域不会唤醒。
6. 设置中切换 LIVELY / LAZY、REST / WAKE。STOP、BACK 或急停应冻结身体；急停释放后需长按 RESET 恢复。

手势阈值是资料包初始值；实际触摸需要持续略长于有效时长门槛，以覆盖去抖、速度估计和换向。风格仅改变播放节奏；ENTER 不再绕过抚摸条件触发开心。CAN 保持待机，日志中的动作事件为 Mock。

## 修改配置和素材

当前配置为 `config/behavior_defaults.json`，原图为 `assets/reference/`。安装 Pillow 后运行：

```bash
python3 tools/build_pet_assets.py
idf.py -B build build
```

工具验证原图哈希，生成 `assets/animation/frames.bin`、清单、色条、C 帧表与 `rb_pet_config.h`。当前布局/帧率/队列容量等结构性参数有检查；改变这些结构需同步改播放器。旧角色转换工具已清理；`config/actions.json`仍用于历史回归。用户选定的probe-v2使用 `tools/render_approved_probe.py` 重现，两个工具处理不同素材。

## 打包

在带 esptool 的 IDF Python 环境中运行：

```bash
python3 tools/package_firmware.py --build build
```

输出 `dist/reBuddy-reference-preview/`，含合并固件、分区/引导/应用、配置、资源清单、状态说明、需求对照及 SHA256SUMS。打包清单明确标记仍使用旧原型，尚未集成probe-v2；不附与固件不符的选定GIF。旧分发包已清理，运行该命令才重新生成。它只在本地整理文件，不烧录、不发布。
