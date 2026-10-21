# 开发指南

## 环境与依赖

行为核心测试需要 Git、CMake、Ninja、C/C++ 编译器和 Python 3，无需开发板或 ESP-IDF。测试与界面预览的配置见 [TESTING.md](TESTING.md)。

屏幕固件使用 ESP-IDF 5.5.5，目标为 `esp32c5`。组件版本在 [main/idf_component.yml](../main/idf_component.yml) 中固定；首次构建需要访问 Espressif 组件注册表。

板级实现包含在 [components/rebuddy_board/](../components/rebuddy_board/) 中，只依赖公开的 ESP-IDF、LVGL、Espressif LVGL Port 和 GT911 触摸组件。无需克隆其他应用仓库，无需私人仓库权限。

## 构建屏幕固件

以下命令从 reBuddy 仓库根目录执行。先安装 ESP-IDF 5.5.5 及其 ESP32-C5 工具链，并将示例 SDK 路径替换为实际安装位置。

```bash
source /path/to/esp-idf-v5.5.5/export.sh
idf.py -B build-c5 build
```

如果从使用外部板级依赖的旧版本升级，请使用全新构建目录和配置，避免旧缓存引用已移走的路径：

```bash
idf.py -B build-standalone -DSDKCONFIG=build-standalone/sdkconfig build
```

使用该目录时，后续 `flash`、`monitor`、`menuconfig` 均使用 `-B build-standalone`；打包使用 `python3 tools/package_firmware.py --build build-standalone`。

硬件配置、接线和首次上板检查见 [HARDWARE.md](HARDWARE.md)。已有 `sdkconfig` 时，修改 `sdkconfig.defaults` 不会覆盖已有配置，需使用 `idf.py -B build-c5 menuconfig` 调整。

## 烧录与操作

确认板卡型号和串口后烧录；下面的 `/dev/ttyACM0` 为示例：

```bash
idf.py -B build-c5 -p /dev/ttyACM0 flash monitor
```

当前固件为屏幕预览版本，CAN 收发器保持待机，不启动总线控制器、不使能电机或发送 RS 运动命令。真实机械臂接入与标定见 [MOTION.md](MOTION.md)。

角色区域支持单击、慢抚摸、连击、快速来回划动和触摸唤醒。开发版另有以下控制入口：

| 操作 | 功能 |
| --- | --- |
| `HAPPY` / 实体 ENTER | 开心小舞蹈预览 |
| `LIVELY` / `LAZY` / 波轮上下 | 切换行为风格，参数在下一动作生效 |
| `REST` | 进入打瞌睡与休息流程 |
| `STOP` / 实体 BACK / GPIO28 | 锁存软件停止，清除待处理手势 |
| 长按 `RESET` | 清除软件停止；不自动恢复动作，外部急停仍有效 |

默认无操作 30 秒进入困倦，60 秒开始打瞌睡。界面目前使用英文。软件停止的控制边界见 [MOTION.md](MOTION.md)。

## 修改动作与角色资源

动作以 [config/actions.json](../config/actions.json) 为唯一数据源。修改后生成并检查 C 动作表：

```bash
python3 tools/generate_actions.py
python3 tools/generate_actions.py --check
```

角色原图位于 [assets/character/](../assets/character/)，来源及制作方式见 [GENERATION.md](../assets/character/GENERATION.md)。替换原图后运行：

```bash
python3 tools/pack_character.py
```

该工具生成固定尺寸的 ARGB8888 C 资源；表情仍由 LVGL 实时绘制。更新资源后需要重新编译固件。

## 整理固件产物

完成构建后，在已激活的 IDF Python 环境中执行：

```bash
python3 tools/package_firmware.py
```

默认输出到 `dist/reBuddy-v0.1-preview/`，包含合并固件、应用、引导与分区文件、动作配置、预览图、烧录说明和校验值。构建产物不随源码提交；需要 ZIP 文件时可自行压缩该目录。
