# 资源索引

项目文档、配置、图片、GIF 和来源说明统一保存在这里。代码位于项目根目录的 `main/`、`components/`、`tools/`、`tests/`。

| 目录 | 内容 |
| --- | --- |
| [docs](docs/GETTING_STARTED.md) | 开发、硬件、测试、项目规划、素材制作与第三方说明 |
| [config](config/behavior_defaults.json) | 当前行为参数与逐姿态手爪触摸区域 |
| [animation](animation/README.md) | 从当前 GIF 转换的 RGB565 帧、房间背景和动作清单 |
| [gifs-v2](gifs-v2/README.md) | 固件当前使用的 10 个独立动作 GIF，平均 8 FPS；另含三个独立表情预览 |
| [previews](previews/current.png) | 当前实机布局截图与手爪触摸示意 |
| [design](design/actions/README.md) | 动作分镜、角色/房间/睡眠源图、提示词与设计草案 |
| [reference](reference/character_reference.png) | 用户原图、互动分镜和原始资料包文字 |
| [approved](approved/probe-v2/source.png) | 已选定的待机与探头原图，保留一份 160 像素探头预览 |
| [fonts](fonts/OFL.txt) | 字体许可证；编译用字形 C 文件在 `main/` |
| [verification](verification/current.json) | 当前版本构建和测试结果 |

当前运行资源由 [gifs-v2](gifs-v2/README.md) 的 10 个动作 GIF 转换而来，原画与时间轴见 [interaction-v3-8fps](design/interaction-v3-8fps/README.md)，小屋背景来自 `design/room-v1/`。旧版 GIF 已删除；历史设计源图保留作为制作参考。三个独立表情 GIF 仍为评估素材，不替换触摸动作。

源图、固件二进制和 GIF 各有用途：源图用于编辑，GIF 用于评估并作为固件动画源，二进制供设备读取。可再生的大尺寸探头对比图由 `tools/render_approved_probe.py` 输出到 `build/approved-probe-preview/`，不重复保存于源码。原始资料文字保留原内容，其中旧路径和交互规则以 [当前开发指南](docs/GETTING_STARTED.md) 为准。

表情、问号与 Z 均已绘入 GIF，并随帧转换到固件。当前仍为关键姿态动画；原始图片未附独立许可证，代码许可证不自动覆盖图片。参见 [第三方说明](docs/THIRD_PARTY_NOTICES.md)。
