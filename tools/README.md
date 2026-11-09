# 工具入口与模块

从这里选择命令；底层 Python 模块供入口复用，不需要逐个启动。

| 入口 | 用途 | 说明 |
| --- | --- | --- |
| `record_dance` | 九个基础舞蹈的拖动录制 | [录制操作](../assets/docs/DANCE_RECORDING.md)；09 夹爪留待程序生成 |
| `record_daily` | Daily 重力补偿录制 | [录制与 CSV 播放](../assets/docs/GRAVITY_RECORDING.md) |
| `play_daily` | 单份 CSV 检查与试播 | 默认先查看 `--help`，实机播放需明确选择 |
| `play_dance` | 舞蹈数据整理与保护保持试播 | `prepare 原始.csv`、`inspect 处理后.json`、`play 处理后.json --supported`；保留时长并缩小幅度 |
| `dance` | 基础舞蹈逐组试播 | [当前 10 组动作](../assets/docs/DANCE_LIBRARY.md)；`--list` 菜单，`--check` 离线检查，`--supported` 实机终端 |
| `daily` | 中文 Daily 动作终端 | [动作命令](../assets/docs/DAILY_COMMANDS.md) |

## 实机工具分层

| 模块 | 职责 |
| --- | --- |
| `rs_dance_record.py`、`daily_actions.py` | 动作名称、交互命令和录制规则 |
| `rs_gravity_record.py` | 重力补偿、反馈采样、日志、片段导出 |
| `rs_motion.py` | 通用曲线、极值检查与时间缩放，不发送电机指令 |
| `rs_gravity_play.py` | CSV 导入、运动控制和单段播放 |
| `rs_dance_play.py` | 独立生成舞蹈处理版本，校验来源，保留节奏并约束幅度 |
| `rs_dance_library.py`、`rs_dance_console.py` | 唯一动作库的加载、身份校验、逐组播放与保护保持 |
| `rs_dance_motion.py`、`rs_dance_plan.py` | 关节样条、Daily 首尾、90°/2 秒夹爪脉冲、同步播放轨迹 |
| `rs_dance_nod.py`、`rs_dance_cute.py`、`rs_dance_steady.py` | 当前双点头、拟人动作和双来回摇摆的生成与验证 |
| `rs_dance_spatial.py` | 蛇形波、稳定朝向与空间 8 字的运动学检查 |
| `rs_dance_pivot_sway.py`、`rs_dance_pivot_join.py` | 当前 10：定点左右上下及连续过渡 |
| `rs_dance_speed.py` | 仅 10 的试播倍率，默认 1.0 |
| `render_dance_design.py` | 当前动作库的骨架预览；不连接 CAN，夹爪只显示电机角度 |
| `rs_dance_pivot_ik.py`、`rs_dance_pivot_kinematics.py`、`rs_dance_pivot_trajectory.py`、`rs_dance_virtual_collision.py` | 定点动作共享 IK、工作区、样条与模型自碰撞检查 |
| `rs_idle_entry.py`、`rs_zero_return.py`、`rs_recovery_policy.py` | 到互动姿态、回零和恢复策略 |
| `rs_protective_stop.py` | 录制与播放制动、固定姿态保持，等待显式退出 |
| `rs_daily_prepare.py`、`rs_daily_session.py` | Daily 数据准备和连续动作会话 |
| `rs_daily_timing.py` | 保留原路径，按 GIF 目标在现有动态限值内重定时 |
| `rs_record_playback.py`、`rs_feedback_audit.py` | CAN 传输、反馈核对、旧 TCH 检查 |
| `export_daily_c5.py` | 将已准备的 Daily 动作导出到固件 |

当前仅维护 Daily、Dance 两套动作库，旧抬头试播入口和早期录制已删除。`rs_record_playback.py` 提供两套库共用的 CAN 硬件与校验函数，仍需保留；其中旧 TCH 实机播放保持锁定。通用轨迹算法位于 `rs_motion.py`。

## 资源工具

`build_screen_preview.py` 生成固件动画，复用 `dance_assets.py`、`cavity_background.py` 和 `interpolate_turn.py`。其余生成、预览、字体、预算与打包命令见 [工具说明](../assets/docs/TOOLS.md)。

命令入口保持在本目录，避免修改已有终端命令。Python 缓存不属于源码，可随时删除；诊断日志和原始录制不要混入工具目录。
