# 资源索引

项目文档、配置、图片、GIF 和来源说明统一保存在这里。代码位于项目根目录的 `main/`、`components/`、`tools/`、`tests/`。

| 目录 | 内容 |
| --- | --- |
| [docs](docs/GETTING_STARTED.md) | 开发、硬件、测试、项目规划、素材制作与第三方说明 |
| [config](config/behavior_defaults.json) | 当前行为参数与逐姿态手爪触摸区域 |
| [animation](animation/README.md) | 固件使用的 28 帧 RGB565、房间背景和动作清单 |
| [gifs](gifs/README.md) | 当前小屋版 10 个独立动作 GIF，按默认活泼风格导出 |
| [previews](previews/current.png) | 当前实机布局截图与手爪触摸示意 |
| [design](design/actions/README.md) | 动作分镜、角色/房间/睡眠源图、提示词与设计草案 |
| [reference](reference/character_reference.png) | 用户原图、互动分镜和原始资料包文字 |
| [approved](approved/probe-v2/source.png) | 已选定的待机与探头原图，保留一份 160 像素探头预览 |
| [fonts](fonts/OFL.txt) | 字体许可证；编译用字形 C 文件在 `main/` |
| [verification](verification/current.json) | 当前版本构建和测试结果 |

当前运行资源来自 `design/screen-preview-v1/` 的 24 张姿态与 `design/sleep-v2/` 的 4 张呼吸姿态，小屋背景来自 `design/room-v1/`。`approved/probe-v2/` 和 `design/probe-v3/` 是造型与分镜参考，未直接用于固件。

源图、固件二进制和 GIF 各有用途：源图用于编辑，二进制供设备读取，GIF 便于评估。可再生的大尺寸探头对比图由 `tools/render_approved_probe.py` 输出到 `build/approved-probe-preview/`，不重复保存于源码。原始资料文字保留原内容，其中旧路径和交互规则以 [当前开发指南](docs/GETTING_STARTED.md) 为准。

表情已烘焙在角色图中，问号与 Z 由代码绘制。当前仍为稀疏关键姿态；原始图片未附独立许可证，代码许可证不自动覆盖图片。参见 [第三方说明](docs/THIRD_PARTY_NOTICES.md)。
