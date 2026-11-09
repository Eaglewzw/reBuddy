# 动作数据

| 路径 | 内容 | 使用方式 |
| --- | --- | --- |
| `dance/` | 当前基础舞蹈录制，包含独立姿态预设 | [录制说明](../assets/docs/DANCE_RECORDING.md) |
| `dance_designed/` | 生成的基础舞蹈，保留各版本用于比较 | [基础舞蹈试播](../assets/docs/DANCE_DESIGNED.md) |
| `dance_showcase/` | 独立舞台编舞，包含回眸、涟漪和夹爪鼓点等动作 | [舞台动作试播](../assets/docs/DANCE_SHOWCASE.md) |
| `gravity/` | Daily 原始采样、标定身份、会话摘要及姿态预设 | 保留 CSV 与同目录 metadata、summary |
| `daily_ready/` | 当前已处理的 Daily 轨迹和清单 | 由准备脚本生成，供播放及 C5 导出读取 |
| `daily_ready_history/` | 本地旧版处理结果 | 不纳入 Git，保留用于调参对比 |
| `daily_design.json` | Daily 动作设计和阶段语义 | 设计资料，不是执行轨迹 |
| `trajectory-*.tch`、`trajectory-*.csv`、`manifest.json` | 早期校零前录制及来源记录 | 历史诊断，不作为新舞蹈录制输入 |

原始采样不能由程序重新生成，因此不按“旧文件”删除。部分失败会话虽无 CSV，仍记录故障和校零身份。各会话的 `complete`、校验和与标定信息决定能否进一步处理，文件存在不等于轨迹已经验收。

高频 `session.jsonl`、`can.jsonl` 仅在本机保留，不提交；CSV、metadata 和 summary 保留用于追溯。`build/record-review/` 是额外诊断证据，也不属于可无条件清除的普通编译缓存。
