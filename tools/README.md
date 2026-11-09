# 工具入口与模块

从这里选择命令；底层 Python 模块供入口复用，不需要逐个启动。

| 入口 | 用途 | 说明 |
| --- | --- | --- |
| `record_dance` | 九个基础舞蹈的拖动录制 | [录制操作](../assets/docs/DANCE_RECORDING.md)；09 夹爪留待程序生成 |
| `record_daily` | Daily 重力补偿录制 | [录制与 CSV 播放](../assets/docs/GRAVITY_RECORDING.md) |
| `play_daily` | 单份 CSV 检查与试播 | 默认先查看 `--help`，实机播放需明确选择 |
| `play_dance` | 舞蹈数据整理与保护保持试播 | `prepare 原始.csv`、`inspect 处理后.json`、`play 处理后.json --supported`；保留时长并缩小幅度 |
| `dance` | 基础与舞台动作，按编号逐个试播 | 默认[基础连续快版 v6](../assets/docs/DANCE_DESIGNED.md)，保留 v5 的 `--bundle` 入口；`--check` 离线检查，`--supported` 进入终端；`--showcase` 选择[舞台 v10](../assets/docs/DANCE_SHOWCASE.md)，[21 定点左右上下](../assets/docs/DANCE_FIXED_TCP.md)P 下移 12 cm，左右 ±20°、上下 ±18°，默认正常速度约 16.74 秒，旧舞台版本保留 |
| `daily` | 中文 Daily 动作终端 | [动作命令](../assets/docs/DAILY_COMMANDS.md) |

## 实机工具分层

| 模块 | 职责 |
| --- | --- |
| `rs_dance_record.py`、`daily_actions.py` | 动作名称、交互命令和录制规则 |
| `rs_gravity_record.py` | 重力补偿、反馈采样、日志、片段导出 |
| `rs_motion.py` | 通用曲线、极值检查与时间缩放，不发送电机指令 |
| `rs_gravity_play.py` | CSV 导入、运动控制和单段播放 |
| `rs_dance_play.py` | 独立生成舞蹈处理版本，校验来源，保留节奏并约束幅度 |
| `rs_dance_design.py`、`rs_dance_console.py` | 12 组节拍与姿态设计、独立数据校验和持续试播终端 |
| `rs_dance_showcase.py`、`rs_dance_showcase_extra.py` | 独立舞台动作编排与模型检查，含蛇形波、世界朝向补偿和空间 8 字，保留基础库 |
| `rs_dance_pivot_sway.py` | v7/v8/v9 的 21：左右、上下各一次；v9 进一步降低 P、调整准备倾角并分程配速 |
| `rs_dance_pivot_join.py` | v10 的 21：左右转上下连续衔接，保留 v9 幅度、速度与总时长 |
| `rs_dance_pivot_body.py` | v5 的 21：`BODY_CONFIG`、显式 J1 摆动、固定位置且自由朝向的 IK、样条与验证 |
| `rs_dance_pivot.py`、`rs_dance_pivot_primitives.py`、`rs_dance_pivot_kinematics.py`、`rs_dance_pivot_trajectory.py` | 保留[旧 v4 朝向动作](../assets/docs/DANCE_FIXED_TCP_V4.md)，提供可复用 TCP/工作区与曲线模块 |
| `rs_dance_speed.py` | 运行时缩放轨迹时间；21 的 v9/v10 默认 1.0 倍；旧 v7/v8 保留 0.3 倍，v5 保留 1.0 倍 |
| `render_dance_design.py` | 按库格式生成基础或舞台动作的 URDF 骨架预览，不连接 CAN |
| `plot_showcase_paths.py` | 根据骨架预览采样绘制 19 世界朝向变化、20 空间 8 字轨迹、21 TCP 位置偏差与姿态，不连接 CAN |
| `plot_virtual_pivot.py` | 离线绘制虚拟支点的局部可达扫描、分段 q/v/a 与 P/TCP 路径；扫描图可独立于动作库生成 |
| `rs_dance_virtual_pivot.py`、`rs_dance_virtual_pivot_primitives.py`、`rs_dance_virtual_pivot_scan.py`、`rs_dance_virtual_collision.py` | [虚拟支点扫描和独立备选](../assets/docs/DANCE_VIRTUAL_PIVOT.md)：四段结构、包络、原始网格自碰撞检查；默认库为 v10，不能把入口工具轴小幅备选当作世界竖直大锥 |
| `rs_idle_entry.py`、`rs_zero_return.py`、`rs_recovery_policy.py` | 到互动姿态、回零和恢复策略 |
| `rs_protective_stop.py` | 录制与播放制动、固定姿态保持，等待显式退出 |
| `rs_daily_prepare.py`、`rs_daily_session.py` | Daily 数据准备和连续动作会话 |
| `rs_record_playback.py`、`rs_feedback_audit.py` | CAN 传输、反馈核对、旧 TCH 检查 |
| `export_daily_c5.py` | 将已准备的 Daily 动作导出到固件 |

`rs_record_playback.py` 的旧 TCH 实机入口仍锁定，但其硬件与校验函数正在使用，不能删除。`rs_probe_trial.py` 保留旧试播诊断和兼容导入，执行入口仍锁定；当前轨迹算法已独立至 `rs_motion.py`，当前播放不再依赖旧试播模块。

## 资源工具

`build_screen_preview.py` 生成固件动画，复用 `dance_assets.py`、`cavity_background.py` 和 `interpolate_turn.py`。其余生成、预览、字体、预算与打包命令见 [工具说明](../assets/docs/TOOLS.md)。

命令入口保持在本目录，避免修改已有终端命令。Python 缓存不属于源码，可随时删除；诊断日志和原始录制不要混入工具目录。
