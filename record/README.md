# 机械臂动作数据

当前只维护两套可执行动作库：

| 路径 | 内容 | 说明 |
| --- | --- | --- |
| `daily_ready/` | Daily 当前轨迹与清单 | [日常动作](../assets/docs/DAILY_COMMANDS.md) |
| `dance_library/` | Dance 当前 10 组动作、开场、收尾 | [基础舞蹈](../assets/docs/DANCE_LIBRARY.md) |

`daily_design.json` 是 Daily 的阶段与屏幕设计资料。
旧 `dance/` 原始录制已清理；它不是当前 Dance 播放依赖。下次录制时工具会自动重新创建该目录。
Daily 录制工具保存到 `gravity/`，其中的来源 CSV 和待机预设仍是 Daily 加载校验的依赖。
截至本次整理，`gravity/` 已缺失，需要恢复匹配清单校验和的原始文件才能运行 Daily；不能用生成轨迹冒充原始采样。

早期四段 TCH/CSV、其清单、旧抬头试播方案和旧点头派生文件已删除。
旧基础舞蹈、showcase 和精选历史库也已清理。今后录制后临时生成的 `dance_prepared/` 仅供素材处理，不作为额外动作库。
旧录制及 `build/record-review/`、`build/dance-trials/` 历史日志已清理；下次运行会自动创建新的日志目录。文档中的事故摘要保留，所提到的旧日志路径仅作历史记录。
