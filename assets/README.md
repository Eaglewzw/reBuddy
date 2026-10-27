# 资源目录

所有可播放 GIF 集中在 `gifs/`；PNG、SVG 和设计原画集中在 `images/`。配置、文档、固件动画和许可证各自归类，代码仍位于工程根目录的 `main/`、`components/`、`tools/`、`tests/`。

| 目录 | 内容与入口 |
| --- | --- |
| [gifs](gifs/README.md) | 日常动作、舞蹈、独立表情、已选 V6、原始动作参考 |
| [images](images/README.md) | 设计源图、角色参考、界面截图和触摸示意 |
| [docs](docs/GETTING_STARTED.md) | 开发说明、编舞说明、验收标准、产品需求和动作分镜 |
| [config](config/behavior_defaults.json) | 行为参数、逐姿态触摸区域和背景处理配置 |
| [animation](animation/README.md) | 从 GIF 生成、供固件读取的 RGB565 二进制与清单 |
| [fonts](fonts/OFL.txt) | 字体许可证；编译字形代码位于 `main/` |
| [verification](verification/current.json) | 当前构建与测试结果 |

常用入口：[已选 V6 猫步](gifs/cat-v6/01_左夹指捂鼻_右夹指快挥.gif)、[十个独立基本舞蹈动作](gifs/dance/README.md)、[日常 GIF](gifs/daily/README.md)、[产品需求](docs/reBuddy_产品需求说明.md)、[编舞操作](docs/CHOREOGRAPHY.md)。

查找动画先进入 `gifs/`；修改外观先进入 `images/source/`。源图、对应提示词、绑定参数与时间轴放在同一个动作目录中，避免同一动作的制作文件散落。文件名保留动作含义，V6 明确保留版本标识；共用图层按用途命名。

运行资源设计为 10 个日常 GIF 和 10 个基本舞蹈 GIF。本轮舞蹈素材改为十个独立完整角色动作，待视觉验收，未制作新组合或完成硬件验证；是否已同步到固件应核对运行清单中的来源哈希。独立表情、V6 与参考 GIF 不进入固件。旧舞蹈、`cat-demo` 和“左右交替”不再导出或接入运行；V6 继续单独保留。当前运行来源以 `animation/manifest.json` 为准。

原始资料文字保留原内容，历史交互规则以 [开发指南](docs/GETTING_STARTED.md) 为准。素材许可与来源见 [第三方说明](docs/THIRD_PARTY_NOTICES.md)。
