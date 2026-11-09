# 21 定点转动

这是保留的舞台 v4 版本说明。当前默认 v5 已改为固定 TCP 位置、允许朝向变化的身体摆动，见[当前说明](DANCE_FIXED_TCP.md)。复现本文旧动作时，播放命令需显式增加 `--bundle ../record/dance_showcase/v4`；旧版默认仍为 30% 速度。

这个动作让末端围绕同一个配置 TCP 转动：点头、锥形绕转、姿态 8 字，然后定格、回到原姿态。肩肘腕需要一起配合，不能只让腕部转动，否则工具偏移会让 TCP 画出小圈。

这里的 TCP 是 URDF 中 `gripper_end` 坐标系加一个局部平移偏移，计算为 `p_tcp = p_frame + R_frame × tcp_offset`。默认偏移为零时，固定的是模型坐标系原点；它不自动等于夹爪指尖，也不代表已经做过工具中心标定。加装笔、灯或其他工具后，需要使用正确的偏移与模型，不能直接把原预览当作工具尖端轨迹。

## 运行

先离线查看和检查：

```bash
cd /home/verser/esp32/reBuddy/tools
./dance --showcase --bundle ../record/dance_showcase/v4 --list
./dance --showcase --bundle ../record/dance_showcase/v4 --check
```

现场防坠支撑和动作空间准备完成后，单独播放：

```bash
./dance 21 --showcase --bundle ../record/dance_showcase/v4 --supported
```

也可以用 `./dance --showcase --bundle ../record/dance_showcase/v4 --supported` 进入终端，再输入 `21`。完成后保持力矩，等待下一编号；`stop` / `estop` 请求制动保持，`home` / `quit` 回零并确认到位后失能退出。供电、CAN 或驱动失效时软件无法保证承重，仍依赖现场防坠支撑。

首次执行默认使用名义轨迹的 30% 速度。骨架动画与模型曲线展示名义时间，两者时长不同。当前 CAN 控制周期约 20 毫秒，即 50 Hz；设计模板中的 200 Hz 不是当前实机执行频率，离线加密采样也不会把控制器变成 200 Hz。

`--pivot-speed` 只改变 `21` 的执行速度，范围为 0.1～1.0。例如 `./dance 21 --showcase --bundle ../record/dance_showcase/v4 --pivot-speed 0.3 --supported` 与旧版默认首次试播相同，时长约为名义轨迹的 3.33 倍。它不会缩小姿态幅度，也不会改变其他舞台动作。请求更高 BPM 属于重新规划；调整执行倍率则沿已有路径改变时间，两种操作需要区分。

## 模型和实现

本机标定：

```text
/home/verser/.cache/huggingface/lerobot/calibration/robots/rebot_b601_rs_follower/rebot_b601_rs_vr.json
```

本机 URDF：

```text
/home/verser/Python/lerobot-teleoperator-rebot-vr/src/lerobot_teleoperator_rebot_vr/urdf/rebot_b601_rs.urdf
```

实现按职责拆分：

| 模块 | 职责 |
| --- | --- |
| [rs_dance_pivot_primitives.py](../../tools/rs_dance_pivot_primitives.py) | 集中 `CONFIG`、点头/锥形/姿态 8 字基元和节拍时间轴 |
| [rs_dance_pivot_kinematics.py](../../tools/rs_dance_pivot_kinematics.py) | TCP 变换、连续逆运动学、工作区和奇异性检查 |
| [rs_dance_pivot_trajectory.py](../../tools/rs_dance_pivot_trajectory.py) | 采样曲线、定格与 `PivotSegment` 求值 |
| [rs_dance_pivot.py](../../tools/rs_dance_pivot.py) | 编排生成、联合验证、离线 CLI |
| [rs_dance_showcase.py](../../tools/rs_dance_showcase.py) | 舞台动作库生成、读取与校验 |
| [rs_dance_console.py](../../tools/rs_dance_console.py)、[rs_dance_speed.py](../../tools/rs_dance_speed.py) | 复用现有 CAN 试播与保护保持，缩放 21 的执行时间 |
| [render_dance_design.py](../../tools/render_dance_design.py)、[plot_showcase_paths.py](../../tools/plot_showcase_paths.py) | 离线骨架动画与模型诊断图 |

每个离线采样点的逆运动学以上一解为初值，以保持关节分支连续；求解后还需要验证限位、速度、加速度、模型力矩和固定点偏差。规划段首尾以零速度收束。离线求解通过，不代表工具中心已标定，也不替代碰撞或实机跟踪验证。

默认姿态指向轴是模型 TCP 的局部 `+Z`，不是已经标定的实体夹爪朝向。锥形目标围绕世界 `+Z` 构造；需要改变指向轴时应先确认模型坐标定义。

## 节拍和参数

动作默认分为 16 拍，请求初值为 100 BPM，对应理想时长 9.6 秒。规划器会根据关节与模型检查统一拉长时间，并尝试配置中的幅度比例；因此生成文件中的实际 BPM、最终幅度比例才是采用的结果。再乘实机 30% 执行倍率后，实际试播时间进一步增加。

当前 v4 生成结果保留全幅点头 ±15°、锥形半角 20°及姿态 8 字 12°/10°，采用幅度比例 1.0。100 BPM 请求下的点头关节速度、加速度超过既有规划预算，所以整体时间拉长约 2.495 倍：

| 时间口径 | 全动作时长 | 节奏 | 两拍定格时长 |
| --- | --- | --- | --- |
| 请求初值 | 9.60 秒 | 100 BPM | 1.20 秒 |
| 通过模型检查的名义轨迹 | 23.95 秒 | 40.08 BPM | 2.99 秒 |
| 默认 30% 首次试播 | 79.85 秒 | 约 12.02 BPM | 约 9.98 秒 |

表中是动作本身的时长，不包含起身、到位确认或回零。当前名义定格区间约为第 17.97～20.96 秒；默认 30% 试播时约为第 59.89～69.87 秒。固定点在模型世界坐标中约为 `[0.32525, -0.00161, 0.43112]` 米，来自统一互动姿态和零 TCP 偏移；这是模型数据，不能代替对实体指尖的测量。

| 拍子 | 运动 |
| --- | --- |
| 1～4 拍 | 点头正负摆动，过渡到锥形姿态 |
| 5～8 拍 | 完成一周锥形朝向变化 |
| 9～12 拍 | 绕固定 TCP 做姿态 8 字 |
| 13～14 拍 | 维持当前姿态两拍 |
| 15～16 拍 | 平滑回到原姿态 |

集中默认配置位于 `tools/rs_dance_pivot_primitives.py` 的 `CONFIG`，主模块保留导入接口。调参通过 JSON 覆盖已知键；未知键会拒绝，不直接编辑生成后的动作 JSON。

| 配置项 | 默认值 | 用途与调节注意 |
| --- | --- | --- |
| `bpm` | `100` | 请求节奏；实际节奏可能因约束降低 |
| `beat_sections` | `[4, 4, 4, 2, 2]` | 五段各占拍数，均需为正数；默认总计 16 拍 |
| `beats` | `16` | 自动取五段拍数之和；若显式提供，必须与总和相等 |
| `sample_period_s` | `0.02` | 当前固定 50 Hz 规划采样，不是 200 Hz |
| `tcp_offset_m` | `[0, 0, 0]` | `gripper_end` 局部坐标中的 TCP 平移，单位米 |
| `fixed_point_xyz_m` | `null` | 默认由互动姿态的 TCP 计算；指定点若不匹配当前入口则拒绝，不会瞬跳过去 |
| `pointing_axis_local` | `[0, 0, 1]` | 单位姿态指向向量，先确认模型轴定义 |
| `nod_deg` | `15` | 点头正负幅度，最终还乘幅度比例 |
| `overshoot_ratio` | `0.08` | 点头过渡进度越过目标后回弹，默认真实超调 8%；允许 0～0.3，不是直接把角幅乘 1.08 |
| `cone_deg` | `20` | 锥形目标半角，最终还乘幅度比例 |
| `figure8_deg` | `[12, 10]` | 姿态 8 字局部 Y/X 两轴角幅，不是 TCP 位移 |
| `nod_transition_fractions` | `[0.29, 0.43, 0.28]` | 点头阶段三个过渡的时间占比，正数且和为 1 |
| `angular_progress` | `"smooth_cruise"` | 朝向运动的时间进度；也支持 `"smoothstep5"` |
| `angular_ramp_fraction` | `0.15` | 巡航进度每侧加减速时间比例，允许大于 0 且不超过 0.5 |
| `amplitude_scales` | `[1, 0.85, 0.7, 0.5]` | 依次尝试的幅度比例；生成结果记录最终采用值 |
| `workspace_reach_fraction` | `0.7` | 相对链长上界的保守外半径比例，不能超过 0.7 |
| `workspace_inner_fraction` | `0.15` | 保守内半径比例，需小于外半径比例 |
| `workspace_min_z_m` | `0.15` | 模型 TCP 的最低世界 Z，不能替代桌面碰撞模型 |
| `joint_margin_deg` | `10` | 保留的关节限位余量 |
| `jac_characteristic_length_m` | `0.3` | 完整六维雅可比的平移/旋转量纲归一化尺度 |
| `jac_sigma_min` | `0.08` | 归一化雅可比最小奇异值门槛 |
| `jac_condition_max` | `40` | 归一化雅可比条件数上限 |
| `position_tolerance_m` | `0.0005` | 模型 TCP 偏差验收阈值，不是实机精度承诺 |
| `orientation_tolerance_deg` | `0.2` | 模型姿态误差验收阈值 |
| `max_duration_s` | `60` | 自动拉长后的名义时长上限 |

通常先调 `bpm` 和三个姿态幅度，再查看生成结果。工作区、雅可比与误差阈值属于模型检查，不能为了让不可行轨迹通过而随意放宽。TCP 固定点从统一互动姿态和配置偏移计算；`fixed_point_xyz_m` 用于明确指定并核对该点，不会增加一段搬运到任意新位置的轨迹。

默认 8% 回弹按每段转移幅度计算：0°→+15°段会多走约 1.2°再回到目标；+15°→−15°的转移幅度为 30°，因此会多走约 2.4°。这不是所有点头极值统一附加 1.2°。

例如将希望尝试的参数保存为 `record/pivot_config.json`：

```json
{
  "bpm": 100,
  "nod_deg": 12,
  "cone_deg": 16,
  "figure8_deg": [10, 8]
}
```

再独立生成新目录，原版动作不覆盖：

```bash
cd /home/verser/esp32/reBuddy
/home/verser/Python/lerobot/.venv/bin/python -B tools/rs_dance_showcase.py --library-version v4 --output record/dance_showcase/pivot_custom --pivot-config record/pivot_config.json
cd tools
./dance --showcase --bundle ../record/dance_showcase/pivot_custom --check
./dance 21 --showcase --bundle ../record/dance_showcase/pivot_custom --supported
```

每次改变配置使用一个新输出目录。检查 `21_fixed_tcp_pivot.json` 中的 `pivot_design` 和 `validation.pivot`：请求/实际 BPM、实际时长、幅度比例、TCP 固定点、模型偏差、工作区和定格区间都记录在那里。

## 离线预览

```bash
cd /home/verser/esp32/reBuddy
/home/verser/Python/lerobot/.venv/bin/python -B tools/render_dance_design.py --library record/dance_showcase/v4 --output build/dance-showcase-preview/v4 --fps 20
MPLCONFIGDIR=/tmp/rebuddy-matplotlib /usr/bin/python3 -s -B tools/plot_showcase_paths.py build/dance-showcase-preview/v4
```

输出目录中的 `21.gif` 显示名义速度下的骨架动作，橙色十字标记配置 TCP 的固定点。`21-fixed-tcp.png` 对照 TCP 的模型位置偏差、末端姿态变化和六关节配合；`21-tip-samples.json` 保存作图样本。骨架和曲线均来自配置中的 TCP 偏移，而不是无条件使用模型末端原点。

20fps 是预览显示帧率，50Hz 是当前执行周期，默认 30% 是沿轨迹播放的速度倍率，三者互不等同。预览不含外壳、桌面或工具碰撞几何，也没有模拟电机闭环、机构弹性和工具实际负载。修改了工具偏移后应重新生成预览与模型检查，不能沿用旧图判断新工具。

## 常见问题

| 现象 | 优先检查 |
| --- | --- |
| 夹爪指尖仍在画小圈 | 比较指尖与 `gripper_end` 的差别；确认偏移坐标系、方向和米/毫米单位，不要用腕轴中心代替工具尖端 |
| 模型 TCP 固定，实机却漂移 | 检查工具偏移、零位、关节反馈、机构间隙与模型；先分清模型误差和跟踪误差，再决定是否需要重新标定 |
| IK 不收敛或接近奇异位形 | 缩小姿态幅度，检查固定点是否过近/过远或接近伸直、折叠位置；不能用更大的控制增益掩盖不可达目标 |
| 关节出现跳变 | 检查上一解是否用作初值、角度展开及解分支；跳变轨迹应离线拒绝，不交给执行器硬追 |
| 动作比 100 BPM 慢 | 区分请求 BPM、规划限值下的实际 BPM，以及实机默认 30% 速度；三者不是同一个数 |
| 点头顺畅，但锥形段卡住 | 检查该段腕部雅可比、关节速度/力矩和实测跟踪；只针对具体问题调整幅度或工作点 |
| 正常转动中突然定住 | 先查终端和日志是否进入保护保持；不要把故障停住当作编排中的两拍定格 |
| 想更快执行 | 先确认现有动作的模型与实测表现，再调集中配置并重新生成验证；不要直接删掉速度、加速度或跟踪检查 |

“使用约 70% 工作区”在当前实现中是保守几何筛选：外半径取运动链长度上界的 0.7 倍，再叠加内半径、最低高度、关节余量和归一化六维雅可比检查。它不代表已证明可达体积的 70%，也不是碰撞空间的保证。不同工具偏移、夹爪姿态和关节限位都会改变可用区域。

基础动作曾出现低速停住再追赶，现有日志不足以把原因确定为摩擦或某一个闭环参数。本动作的笛卡尔规划不能证明这个实机问题已被消除。
