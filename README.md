# reBuddy

基于 **ESP32-C5 + 触摸屏 + reBot RS 机械臂** 的桌面伙伴。黄绿色小机器人用表情回应触摸，并计划通过机械臂表达情绪。

**当前为 v0.1 项目初版：屏幕互动 Demo、行为核心、动作配置和主机测试已实现；真实机械臂的动作适配与姿态标定尚未完成。固件固定显示 `PREVIEW`，不使能电机、不发送 RS 运动命令。**

![LVGL 实际渲染预览](docs/preview/contact-sheet.png)

## 功能与交互

| 方案功能 | 当前实现 |
| --- | --- |
| 屏幕角色 | 按用户设定图生成的 reBot RS 立体角色，黄绿色外壳、银色关节和前置夹爪；LVGL 实时绘制眼睛、眉毛、腮红及表情 |
| 触摸互动 | 点击、缓慢抚摸、快速来回划动、连续点击；抬手后 350ms 区分单击与连击 |
| 表情与身体联动 | 共用关键帧时间轴；七关节相对偏移预览、五次平滑插值和速度限幅；动作中断从当前指令姿态衔接 |
| 两种风格 | 活泼 / 慵懒；慵懒幅度 55%、时长 165%、速度上限 55%，切换于下一动作生效 |
| 待机与休息 | 默认 30s 困倦、60s 开始打瞌睡，随后休息；触摸唤醒 |

| 操作 | 回应 |
| --- | --- |
| 点击角色 | 探头看你 |
| 缓慢抚摸角色 | 舒服地蹭蹭 |
| 连续点击角色 | 害羞缩回去 |
| `HAPPY` / 实体 ENTER | 开心小舞蹈 |
| 在角色上快速来回划动 | 歪头不理解；单向快速划动不会误判为来回划动 |
| `REST` / 无操作达到时间 | 打瞌睡；睡眠时触摸唤醒 |
| `LIVELY` / `LAZY` / 波轮上下 | 切换活泼、慵懒风格 |
| `STOP` / 实体 BACK / GPIO28 急停 | 锁存停止，丢弃待处理手势 |
| 长按 `RESET` | 清除软件停止；不自动恢复动作，外部急停仍有效 |

屏幕初版使用英文，避免引入中文字库；方案、动作说明和开发文档均为中文。动作编舞列入第四周的可选项。

## 本机构建

项目目录：`/home/verser/esp32/reBuddy`。当前基线是 **ESP-IDF 5.5.5**，目标 `esp32c5`。5.5.4 不符合参考驱动的依赖声明；6.0.1 缺少该驱动使用的 I²C 超时字段，因此本项目固定到 5.5 分支。第三方依赖固定在 [组件清单](main/idf_component.yml)。

```bash
cd /home/verser/esp32/reBuddy
mkdir -p ../.references
# 已有外部参考仓库时无需再次克隆
git clone git@github.com:Eaglewzw/esp32c5_ui_console.git ../.references/esp32c5_ui_console
git -C ../.references/esp32c5_ui_console checkout ba288ae49c80f19b10d14dd9d428f63e3dd41486
source /path/to/esp-idf-v5.5.5/export.sh
idf.py -B build-c5 build
```

参考仓库单独放在项目外的 `../.references/esp32c5_ui_console`，不作为子模块或源码副本加入 reBuddy。也可在首次构建时使用 `idf.py -B build-c5 -DREBUDDY_CONSOLE_DIR=/其他位置/esp32c5_ui_console build` 指定外部路径。

首次构建需连接 Espressif 组件注册表。已有 `sdkconfig` 时，修改 `sdkconfig.defaults` 不会覆盖已有选择，需用 `idf.py menuconfig` 调整。

硬件按私人参考仓库的 XIAO ESP32-C5 CAN Console 配置：320×240 横屏、ST7789、GT911、PCA9535，8 MiB Flash 和 8 MiB Quad PSRAM。接线见 [硬件说明](docs/HARDWARE.md)。其他 C5 板需要替换 BSP 和内存配置。

接入板卡后按实际串口烧录：

```bash
idf.py -B build-c5 -p /dev/ttyACM0 flash monitor
```

`/dev/ttyACM0` 为示例。当前未连接板卡，本次没有烧录或验证实机触摸。

本地已生成屏幕 Demo 固件包 `dist/reBuddy-v0.1-preview.zip`（构建产物不随源码推送），含合并固件、分区/引导文件、动作配置、校验值和烧录说明。应用固件 667,696 bytes，合并固件 733,232 bytes；这是预览版本，不能驱动六个实体动作。打包可运行 `python3 tools/package_firmware.py`（使用已激活的 IDF Python 环境）。

## 测试和预览

不需要 ESP-IDF、屏幕或私人仓库即可运行核心测试：

```bash
cmake -S tests/host -B build-host -G Ninja
cmake --build build-host -j8
ctest --test-dir build-host --output-on-failure
```

如果已经下载 `managed_components/lvgl__lvgl`，会自动增加真实 LVGL RGB565 触摸测试。也可用 `-DREBUDDY_LVGL_DIR=/path/to/lvgl-9.5.0` 指定源码。截图由同一份正式 UI 代码渲染：

```bash
python3 tools/render_previews.py build-host docs/preview
```

预览转换工具需要 Pillow。主机 30 分钟测试是 **90,000 个 20ms tick 的模拟时间**，不能作为真机连续 30 分钟验收证据。详细边界与复现方法见 [测试说明](docs/TESTING.md)。

## 开发入口

| 路径 | 内容 |
| --- | --- |
| `components/rebuddy_core/` | 与 SDK 无关的行为、手势、动作与机械臂接口 |
| `main/pet_ui.c` | LVGL 角色、表情及触摸控制 |
| `assets/character/` | 透明 reBot RS 角色原图、生成提示词和导出说明 |
| `main/main.c` | 板级初始化、20ms 更新、实体输入、急停和内存日志 |
| `config/actions.json` | 六个动作的唯一数据源，当前均未实机标定 |
| `tests/host/` | 核心、模拟长跑和真实 LVGL 触摸测试 |
| `docs/` | 原始方案、一个月计划、动作接入、验收与界面截图 |

修改动作配置后运行：

```bash
python3 tools/generate_actions.py
python3 tools/generate_actions.py --check
```

角色资源使用内置 image_gen 根据用户设定图生成，完整提示词见 [资源记录](assets/character/GENERATION.md)。`tools/pack_character.py` 导出固定尺寸 ARGB8888 C 数据，102,784 bytes 驻留 Flash，保留透明度，无需运行时 PNG 解码。替换原图后运行 `python3 tools/pack_character.py` 并重编译。

后续实机工作从 [动作和控制适配](docs/MOTION.md) 开始；月度安排见 [开发计划](docs/ROADMAP.md)，验收记录见 [验收清单](docs/ACCEPTANCE.md)。

## 私人参考与开源边界

项目外部的 `../.references/esp32c5_ui_console` 使用提交 `ba288ae49c80f19b10d14dd9d428f63e3dd41486`，来自你指定的私人仓库 `Eaglewzw/esp32c5_ui_console`。当前仅直接编译它的板级、显示、输入、内存和 CAN 服务；没有链接原控制台页面或 RS 运动控制器。

公开仓库不包含私人参考仓库、子模块指针或其源码副本；固件构建通过可配置的外部路径引用板级依赖。**没有该私人仓库权限的用户可运行核心测试，但不能构建整机固件。** 后续对外独立构建前，需要把允许公开的 BSP 单独开放或实现独立适配。

本项目新增代码采用 Apache-2.0；外部板级依赖及第三方组件分别遵循各自授权，见 [第三方说明](THIRD_PARTY_NOTICES.md)。
