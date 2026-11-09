# 21 固定 TCP 身体摆动

这是保留的舞台 v5 版本说明。新虚拟支点版本见[当前说明](DANCE_FIXED_TCP.md)。复现本文动作时，请显式指定 `--bundle ../record/dance_showcase/v5`，其默认倍率仍为 1.0。

舞台 v5 的 `21` 改为：**TCP 位置保持在同一点，底盘 J1 明显左右摆，肩肘腕一起补偿；末端朝向可以变化。** 它从统一互动姿态出发，完成一轮身体摆动后回到原姿态，中间没有定格。

旧 v4 是规定朝向的点头、锥形和姿态 8 字，J1 运动较小；默认 30% 试播约 79.85 秒。该版本和说明仍保留在 [DANCE_FIXED_TCP_V4.md](DANCE_FIXED_TCP_V4.md)。新版不再沿用这套朝向编排或默认降速。

最近一次 v4 实机日志 `build/dance-trials/session-1794049816734515810/` 中，J1 目标总幅约 8.07°、实际总幅约 7.51°。这说明旧版并非完全没有发送或响应 J1 运动，而是幅度小、耗时长，视觉上不明显；新版将 J1 总幅明确提高到 40°。

## 运行与版本

```bash
cd /home/verser/esp32/reBuddy/tools
./dance --showcase --bundle ../record/dance_showcase/v5 --check
./dance 21 --showcase --bundle ../record/dance_showcase/v5 --supported
```

第一条只做离线检查；第二条用于现场防坠支撑和动作空间已准备好的实机试播。也可用 `./dance --showcase --bundle ../record/dance_showcase/v5 --supported` 进入终端，再输入 `21`。

v5 新动作默认 **1.0 倍速度**，一轮 **6.00 秒，8 拍、80 BPM**，预览与默认执行采用相同轨迹时间。到位确认、起身或回零时间不包含在这 6 秒内。`--pivot-speed` 只改变 `21` 的执行时间，不改变幅度，允许 0.1～1.0；例如显式指定 `--pivot-speed 0.5` 时为 12 秒。

比较旧 v4 时使用：

```bash
./dance 21 --showcase --bundle ../record/dance_showcase/v4 --supported
```

旧版本默认仍为 0.3 倍；显式的 `--pivot-speed` 会覆盖相应版本的默认值。选旧库不会把旧动作误当成新版 6 秒动作。

动作完成后保持力矩，等待下一编号。`stop` / `estop` 请求制动保持；`home` / `quit` 回零并确认到位后失能退出。软件保持依赖有效供电、反馈和 CAN，不能替代现场防坠支撑。

## 实际设计范围

J1 的左右摆动是显式轨迹目标；其余五轴通过位置逆运动学补偿，使用关节姿态偏好选择连续的解。**没有固定末端朝向约束**，所以模型朝向会随身体摆动变化。

| 关节 | 一轮最大角减最小角 |
| --- | --- |
| J1 底盘 | 40.00°，相对起始左右各约 20° |
| J2 肩 | 约 5.01° |
| J3 肘 | 约 7.60° |
| J4 腕俯仰 | 约 9.64° |
| J5 腕偏转 | 约 66.75° |
| J6 腕滚转 | 30.00° |

J7 保持闭合。模型计算中朝向相对起始最多变化约 19.06°，这是自由朝向设计的结果，不是 TCP 位置漂移。上述范围与固定点检查来自计划轨迹；新版尚未实机验收，不能作为实际精度或碰撞验证。

TCP 定义仍是 `gripper_end` 坐标系加局部平移：`p_tcp = p_frame + R_frame × tcp_offset`。默认偏移为零，固定的是 URDF 模型末端原点，不自动等于实体夹爪指尖。加装笔、灯或工具后，需要确认工具偏移与模型；当前没有自动工具中心标定或 LED 控制。

## 模块与配置

| 模块 | 职责 |
| --- | --- |
| [rs_dance_pivot_body.py](../../tools/rs_dance_pivot_body.py) | 新版 `BODY_CONFIG`、显式 J1 轨迹、位置 IK、连续样条、生成与校验 |
| [rs_dance_pivot_kinematics.py](../../tools/rs_dance_pivot_kinematics.py) | TCP 变换、工作区和雅可比检查 |
| [rs_dance_pivot_trajectory.py](../../tools/rs_dance_pivot_trajectory.py) | 复用密集曲线求值 |
| [rs_dance_showcase.py](../../tools/rs_dance_showcase.py) | 版本化动作库生成、读取与校验 |
| [rs_dance_console.py](../../tools/rs_dance_console.py)、[rs_dance_speed.py](../../tools/rs_dance_speed.py) | CAN 试播、保护保持、按动作版本选择默认执行倍率 |
| [rs_dance_pivot.py](../../tools/rs_dance_pivot.py)、[rs_dance_pivot_primitives.py](../../tools/rs_dance_pivot_primitives.py) | 保留旧 v4 的朝向动作与 `CONFIG`，不是新版 `BODY_CONFIG` |

每个采样点以上一解作为初值；首尾位置回到互动姿态，速度和加速度为零。生成器会检查关节限位、运动范围、速度、加速度、模型力矩、TCP 位置误差和解分支连续性。

本机默认资源：

```text
标定：/home/verser/.cache/huggingface/lerobot/calibration/robots/rebot_b601_rs_follower/rebot_b601_rs_vr.json
URDF：/home/verser/Python/lerobot-teleoperator-rebot-vr/src/lerobot_teleoperator_rebot_vr/urdf/rebot_b601_rs.urdf
```

| `BODY_CONFIG` 配置项 | 默认值 | 用途 |
| --- | --- | --- |
| `beats` | `8` | 计划拍数，BPM 由拍数与实际时长计算 |
| `requested_duration_s` | `6` | 请求时长；若受约束，生成结果可能加长 |
| `sample_period_s` | `0.02` | 当前固定 50 Hz，不是模板中的 200 Hz |
| `base_amplitude_deg` | `20` | J1 相对起始的单侧幅度 |
| `amplitude_scales` | `[1, 0.85, 0.7]` | 依次尝试的幅度比例，生成结果记录实际采用值 |
| `phase_ramp_fraction` | `0.15` | 相位进度每侧平滑加减速占比 |
| `shoulder_preference_deg` | `-10` | 肩关节偏好，不是最终运动幅度保证 |
| `elbow_preference_deg` | `14` | 肘关节偏好 |
| `wrist_flex_preference_deg` | `-8` | 腕俯仰偏好 |
| `wrist_yaw_base_ratio` | `-1.4` | 腕偏转偏好与 J1 幅度的比例 |
| `wrist_roll_preference_deg` | `15` | 腕滚转偏好 |
| `position_weight` / `posture_weight` | `1000` / `0.05` | 位置误差与关节偏好的求解权重；位置仍需通过独立误差检查 |
| `tcp_offset_m` | `[0, 0, 0]` | `gripper_end` 局部坐标中的 TCP 平移，单位米 |
| `fixed_point_xyz_m` | `null` | 默认由入口 TCP 计算；显式值需匹配入口，不会瞬移到新点 |
| `workspace_reach_fraction` / `workspace_inner_fraction` | `0.7` / `0.15` | 链长上界对应的保守径向区间 |
| `workspace_min_z_m` | `0.15` | 模型 TCP 的最低世界 Z |
| `joint_margin_deg` | `10` | 关节限位余量 |
| `jac_characteristic_length_m` | `0.3` | 六维雅可比的量纲归一化尺度 |
| `jac_sigma_min` / `jac_condition_max` | `0.08` / `40` | 奇异性检查门槛 |
| `position_tolerance_m` | `0.0005` | 模型 TCP 偏差验收阈值，不是实机精度承诺 |
| `max_duration_s` | `10` | 自动调时后的名义时长上限 |

通常先调 `base_amplitude_deg`、`requested_duration_s`，再看实际生成结果。J2～J6 的偏好会受固定位置约束调整，不能把偏好值当作最终关节摆幅。工作区、雅可比、力矩与误差检查不能为追求效果随意放宽。

例如把以下参数保存为 `record/pivot_body_config.json`：

```json
{
  "base_amplitude_deg": 18,
  "requested_duration_s": 6.5
}
```

独立生成新目录，原数据保留：

```bash
cd /home/verser/esp32/reBuddy
/home/verser/Python/lerobot/.venv/bin/python -B tools/rs_dance_showcase.py --library-version v5 --output record/dance_showcase/body_custom --pivot-config record/pivot_body_config.json
cd tools
./dance --showcase --bundle ../record/dance_showcase/body_custom --check
./dance 21 --showcase --bundle ../record/dance_showcase/body_custom --supported
```

每次改变配置使用新输出目录。新版结果查看 `21_fixed_tcp_pivot.json` 中的 `body_design` 和 `validation.pivot_body`；旧版 `pivot_design` 对应另一套朝向动作，不能混用配置键。

## 预览与排查

```bash
cd /home/verser/esp32/reBuddy
/home/verser/Python/lerobot/.venv/bin/python -B tools/render_dance_design.py --library record/dance_showcase/v5 --output build/dance-showcase-preview/v5 --fps 20
MPLCONFIGDIR=/tmp/rebuddy-matplotlib /usr/bin/python3 -s -B tools/plot_showcase_paths.py build/dance-showcase-preview/v5 --compare build/dance-showcase-preview/v4
```

`21.gif` 标记 TCP 固定点；`21-fixed-tcp.png` 显示 TCP 模型位置偏差、自由朝向变化与六轴运动；`21-body-comparison.png` 对比 v4/v5 的六轴峰峰范围和 TCP 模型误差。预览为 20fps，执行约 50Hz，速度倍率为 1.0，三个概念不同。没有外壳、桌面、工具碰撞几何，也没有模拟实际电机闭环和机构弹性。

| 现象 | 优先检查 |
| --- | --- |
| J1 仍不明显、动作接近 80 秒 | 查看是否选了旧 v4，或显式传入很小的 `--pivot-speed`；v5 默认 J1 总幅 40°、6 秒 |
| 头部朝向在变化 | 新版只固定位置，朝向变化是允许的；需要旧朝向动作时显式选择 v4 |
| 指尖在画小圈 | 检查实体指尖与模型 `gripper_end` 的差别，以及偏移坐标系和米/毫米单位 |
| 模型固定、实机漂移 | 分别检查 TCP/零位标定、实测关节跟踪、间隙及工具负载；模型检查不是实测误差保证 |
| 改偏好后 J2/J3 幅度不等于填写值 | 偏好是软约束，求解器需要调整其他轴以固定位置；以生成的各轴范围为准 |
| IK 不收敛或接近奇异位形 | 缩小 J1 幅度，检查入口姿态和 TCP 偏移；不要用更大控制增益掩盖不可达目标 |
| 关节突然跳变或运动途中停住 | 查看是否触发分支/跟踪保护；新版中间没有计划定格，不能用舞蹈节奏解释异常停住 |
| 请求 6 秒但生成更长 | 查 `body_design.actual_duration_s` 与实际幅度比例；约束可能要求延长时间 |

“70% 工作区”是链长上界的径向过滤，再叠加高度、关节余量和雅可比检查，不代表可达体积的 70% 或已验证的碰撞空间。既有低速停住再追赶问题仍需结合实机日志判断，本次改变运动目标不能证明硬件卡滞已经消除。
