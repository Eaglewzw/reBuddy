# 机械臂动作数据

当前只维护两套可执行动作库：

| 路径 | 内容 | 说明 |
| --- | --- | --- |
| `daily_ready/` | Daily 当前轨迹与清单 | [日常动作](../assets/docs/DAILY_COMMANDS.md) |
| `dance_library/` | Dance 当前 10 组动作、开场、收尾 | [基础舞蹈](../assets/docs/DANCE_LIBRARY.md) |

`daily_design.json` 是 Daily 的阶段与屏幕设计资料。
旧 `dance/` 原始录制已清理；它不是当前 Dance 播放依赖。下次录制时工具会自动重新创建该目录。
Daily 播放直接使用 `daily_ready/manifest.json` 中的待机/零位姿态及库内轨迹，不再读取 `gravity/` 原始录制。来源路径和校验和仅保留为制作记录。
重新录制或执行 `daily prepare` 时仍需原始 CSV、元数据和待机预设；现有轨迹不会被伪装成原始采样。
Daily 与 Dance 共用六轴待机姿态及开场/收尾路径；启动均通过同一控制器，从实测位置平滑到待机。

早期四段 TCH/CSV、其清单、旧抬头试播方案和旧点头派生文件已删除。
旧基础舞蹈、showcase 和精选历史库也已清理。今后录制后临时生成的 `dance_prepared/` 仅供素材处理，不作为额外动作库。
旧录制及 `build/record-review/`、`build/dance-trials/` 历史日志已清理；下次运行会自动创建新的日志目录。文档中的事故摘要保留，所提到的旧日志路径仅作历史记录。
