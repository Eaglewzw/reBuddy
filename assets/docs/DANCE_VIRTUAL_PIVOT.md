# 虚拟支点：模型与离线扫描

这次目标是固定空间中的虚拟点 `P`，让工具绕 `P` 转动。它与 [v5 的固定 TCP 身体摆动](DANCE_FIXED_TCP_V5.md) 不同：虚拟支点模式下，模型 TCP 本身允许移动。

本文保留 v6 长版实验记录；当前默认已改为 [v10 连续大幅低位摆动](DANCE_FIXED_TCP.md)。只有虚拟支点的完整路径、动态和碰撞检查通过后，才能发布新动作；扫描中找到几个姿态，不等于已经有可播放的四段舞蹈。

## 坐标约定

模型 TCP 使用 `gripper_end` 原点。默认工具轴为该坐标系局部 `+X`，虚拟点在其前方 0.20 米：

```text
P = p_gripper_end + R_gripper_end × [0.20, 0, 0]
```

扫描文件中的 `tcp_offset_m` 与 `virtual_offset_m` 都表示从 `gripper_end` **直接到 P** 的同一个偏移，不要叠加两次。`model_tcp_offset_m=[0,0,0]` 指模型 TCP。将偏移设为零，P 才与模型 TCP 重合。工具轴和工具长度是模型配置，不代表已经标定了实体夹爪指尖或安装了 20 厘米实体工具。

用户要求的锥轴是世界 `+Z`。绕入口工具轴的圆锥是另一种动作，不能在世界竖直方向求解失败后悄悄替代。目标方案包含四段：大圆锥两圈、钟摆、螺旋、逐拍定格；默认请求 16 拍、100 BPM、实机初始 30% 倍率。实际时长和可用幅度必须由最终路径检查决定，目前不能把请求值当作已生成结果。

## 扫描能说明什么

[rs_dance_virtual_pivot_scan.py](../../tools/rs_dance_virtual_pivot_scan.py) 对候选 P、锥角、方位角、工具 Roll 进行离散扫描，每个姿态使用有限个确定性初值求解。输出格式为 `rebuddy.virtual_pivot_scan.v1`，包含完整配置、模型身份、碰撞资源身份、逐点结果与每个 P 的汇总。

每个采样结果记录 `accepted`、失败原因、位置/朝向误差和尝试的初值数。未通过表示“这次采样和所用初值没有找到满足条件的解”，不证明该位置全局不可达。即使一圈所有离散方位都通过，也仍需验证相邻样本之间的连续路径、关节分支、速度、加速度、模型力矩和起止过渡。

本轮严格世界竖直轴扫描输出为 `build/dance-virtual-pivot/world-z-scan.json`。四个候选 P、四个锥角 40°/50°/60°/70°、八个方位及三个 Roll，共 384 个格点，每格最多七个初值。具体 P 坐标、模型和结果以文件内容为准；不同 P 的样本不应合并成同一个可达包络。

已完成的扫描结果如下：

| 模式与 P | 通过样本 | 完整采样圆环 |
| --- | --- | --- |
| 世界竖直，入口偏移 P ≈ `[0.5239,-0.0067,0.4086]` 米 | 0 / 96 | 无 |
| 世界竖直，P = `[0.1,0,0.7]` 米 | 0 / 96 | 无 |
| 世界竖直，P = `[0.2,0,0.75]` 米 | 18 / 96 | 无 |
| 世界竖直，P = `[0.3,0,0.65]` 米 | 3 / 96 | 无 |
| 入口工具轴备选，入口偏移 P | 40 / 48 | 5°、10°、15°，均为 Roll 0° |

严格世界竖直轴扫描没有找到完整采样圆环，不能据此发布所要求的 50°～60°大锥四段动作。备选扫描为 `build/dance-virtual-pivot/idle-tool-scan.json`，它绕入口工具轴，**不是世界竖直**；其 15°完整采样环可以作为另一个小幅方案的候选，按 75% 取 11.25°也仍需完整路径验证，不能等同于原大幅请求。

备选中有个别 30°姿态通过，但最大完整采样圆环只有 15°。文件中的 `max_accepted_sample_cone_deg` 与 `max_complete_sampled_ring_cone_deg` 分别记录这两件事，不能拿某个方向的最大角度充当一整圈都可用的幅度。

## 已另存的小幅备选

`record/dance_showcase/v6_virtual_preview/` 已生成一套 **入口工具轴** 备选，当前默认舞台库为 v10。它完成四段结构，但没有满足原先“世界竖直轴、50°～60°大幅、100 BPM”的全部要求，不能作为原请求已完成来描述。

备选使用 15°完整采样环的 75%，名义锥角和钟摆幅度为 11.25°，螺旋内角缩为 2.25°。真实回弹比例为 3%，极值约 11.925°，即采样环幅度的 79.5%；不是把“75%”当成包括回弹的实际全程上限。

各四拍段分别调时，避免一个最慢段拖长所有其他段。请求仍为 100 BPM，实际是变速节拍：

| 四拍段 | 名义时间区间 | 时长 | 实际 BPM |
| --- | --- | --- | --- |
| 绕入口工具轴锥形两圈 | 2.01～14.83 秒 | 12.82 秒 | 18.72 |
| 平面钟摆与回弹 | 14.83～22.04 秒 | 7.20 秒 | 33.32 |
| 螺旋扩张与收回 | 22.04～33.92 秒 | 11.89 秒 | 20.19 |
| 四次逐拍定格 | 33.92～42.21 秒 | 8.28 秒 | 28.98 |

舞段共 40.20 秒，进入/返回各约 2.01 秒，总名义时长 **44.22 秒**。默认 30% 执行约 **147.39 秒**，仍然较慢；没有通过放宽速度、加速度或力矩预算来追赶请求的 100 BPM。时间不包含额外的连接、起身和到位确认。

逐段解析极值统计保存在 `build/dance-virtual-pivot/trajectory-statistics.json`，带动作文件 SHA；它与预览采样曲线分开，避免用低帧率图形估计真正峰值：

| 段 | 六轴中的最大速度 | 六轴中的最大加速度 |
| --- | --- | --- |
| 锥形两圈 | 57.01°/s | 63.66°/s² |
| 平面钟摆 | 57.01°/s | 57.40°/s² |
| 螺旋 | 57.01°/s | 72.07°/s² |
| 逐拍定格 | 53.06°/s | 113.45°/s² |

表中是名义轨迹。30% 执行时速度乘 0.3，加速度乘 0.09；统计文件同时记录各关节的缩放结果。它们仍是规划计算，不是实机反馈测量。

P 只在上述舞段内保持固定。进入准备姿态和返回互动姿态是额外过渡，不应把过渡中的虚拟点位移误判为舞段固定点错误。前三段末各有 0.15 拍计划停顿，第四段四次定格；这些计划停顿与控制器异常卡滞不同。

离线检查此备选，需显式选择独立目录：

```bash
cd /home/verser/esp32/reBuddy/tools
./dance --showcase --bundle ../record/dance_showcase/v6_virtual_preview --check
```

若选择查看这个小幅备选的实机效果，在已准备现场支撑和动作空间时使用：

```bash
./dance 21 --showcase --bundle ../record/dance_showcase/v6_virtual_preview --supported
```

该命令默认 0.3 倍；直接运行原来的 `./dance 21 --showcase --supported` 选择 v10 的连续大幅低位左右上下动作。备选尚未实机验收。

[rs_dance_virtual_collision.py](../../tools/rs_dance_virtual_collision.py) 使用本机原始碰撞 URDF、网格和 SRDF，并记录文件摘要。它检查模型自碰撞，夹爪按闭合状态处理；周边桌面、线缆、人员及新加工具没有被自动建模。采样自碰撞检查不等于完整连续路径已通过，也不等于现场碰撞验收。

## 独立绘制扫描图

配置集中在 `tools/rs_dance_virtual_pivot_scan.py` 的 `VIRTUAL_CONFIG`。例如将以下覆盖项保存为 `record/virtual-pivot-config.json`，明确保留世界竖直锥轴：

```json
{
  "cone_axis": "world_z",
  "tcp_offset_m": [0.20, 0, 0],
  "requested_cone_deg": 50,
  "scan_cone_deg": [40, 50, 60, 70],
  "P_candidates_m": [[0.1, 0, 0.7], [0.2, 0, 0.75], [0.3, 0, 0.65]]
}
```

扫描与动作生成分开执行：

```bash
cd /home/verser/esp32/reBuddy
/home/verser/Python/lerobot/.venv/bin/python -B tools/rs_dance_virtual_pivot.py --scan --config record/virtual-pivot-config.json --output build/dance-virtual-pivot/my-scan.json
/home/verser/Python/lerobot/.venv/bin/python -B tools/rs_dance_showcase.py --library-version v6 --pivot-scan build/dance-virtual-pivot/my-scan.json --output record/dance_showcase/virtual_custom
```

第二步读取扫描配置并核对标定、URDF 身份；找不到满足条件的完整路径时会拒绝生成，不会自动换轴或播放另一种动作。本轮严格世界竖直扫描没有可用完整圆环，因此不能把这条生成命令理解为当前已能播放原请求。

扫描 JSON 完整写出后，可直接作图，不需要动作库或机械臂连接：

```bash
cd /home/verser/esp32/reBuddy
MPLCONFIGDIR=/tmp/rebuddy-matplotlib /usr/bin/python3 -s -B tools/plot_virtual_pivot.py --scan build/dance-virtual-pivot/world-z-scan.json --output build/dance-virtual-pivot/world-z-envelope.png
```

多候选 P 时输出 `world-z-envelope-point-XX.png`，分别显示角度/方位/Roll 采样和通过情况；全失败网格也会保留。`world-z-envelope.index.json` 记录各图对应的 P、通过数量和源 JSON 的 SHA，避免把图与另一轮扫描混用。

四段轨迹通过并导出后，预览工具还可生成每段六轴 q/v/a、虚拟 P 位置偏差、TCP 绕 P 路径和骨架动画。图中橙色标记是虚拟 P，工具轴延长线连向 P，TCP 保留独立标记；不会把 TCP 的允许移动误画成固定点误差。尚未生成并验证的路径不提供虚构的运动曲线或动画。

已生成的入口工具轴备选预览可复现为：

```bash
cd /home/verser/esp32/reBuddy
/home/verser/Python/lerobot/.venv/bin/python -B tools/render_dance_design.py --library record/dance_showcase/v6_virtual_preview --output build/dance-showcase-preview/v6_virtual_preview --fps 20
MPLCONFIGDIR=/tmp/rebuddy-matplotlib /usr/bin/python3 -s -B tools/plot_virtual_pivot.py build/dance-showcase-preview/v6_virtual_preview --library record/dance_showcase/v6_virtual_preview --scan build/dance-virtual-pivot/idle-tool-scan.json
```

目录中 `21.gif` 使用名义时间，`21-phase-01-qva.png`～`21-phase-04-qva.png` 按生成文件的实际 `phase_ranges_s` 分段，不均分变速动作。`21-virtual-p-path.png` 显示模型 TCP 绕 P 的路径和虚拟 P 误差，灰区为进入/返回过渡；`21-reach-envelope.png` 是此备选对应 P 和入口工具轴的离散扫描图。

## 调整顺序

先核对模型工具轴、虚拟距离和 P 坐标，再检查扫描失败原因。位置不可达、关节余量不足、奇异性、模型自碰撞是不同问题；不能只提高求解迭代次数或放宽误差阈值来绕过它们。若需更换 P，应增加经过检查的进入/退出过程，而不是直接跳到新点。

| 优先级 | 参数与处理 | 本轮依据 |
| --- | --- | --- |
| 1 | 调整 `P_world_m` 或增加 `P_candidates_m`，保持 `cone_axis=world_z` 重扫 | `[0.2,0,0.75]` 的通过单点较多，但仍没有完整圆环；不能直接拿它试转 |
| 2 | 将 `tcp_offset_m` 从 `[0.20,0,0]` 分别试算 `[0.15,0,0]`、`[0.10,0,0]` | 缩短虚拟半径可能减轻关节补偿；这些值尚未验证，需重新扫描和生成 |
| 3 | 增加 `scan_roll_deg`、准备姿态初值，再加密方位角 | 寻找其他可连续的关节解；单点成功后仍需前一帧初值的完整路径验证 |
| 4 | 幅度通过后再调整圈数、拍型与 `bpm` | 固定 P 会放大关节运动。增大请求 BPM 不会越过动态限制；当前按四拍段分别伸时 |
| 5 | 最后调整 `overshoot_ratio` 与停顿 | 当前 3% 回弹使实际最大幅度为采样包络的 79.5%；修改后重新核对峰值 |

| 现象 | 检查与处理 |
| --- | --- |
| IK 误差超过 2 mm | 看失败方向与关节余量，优先改 P、偏移或姿态；不把容差放宽当作修复 |
| 奇异度或限位拒绝 | 调整准备姿态、P 或幅度，重新验证整圈及进入/退出 |
| 自碰撞拒绝 | 查看报告中的碰撞 link 对，改变路径；保留原有碰撞对检查 |
| 标定、URDF 或网格摘要不匹配 | 核对实际使用的模型和零点，再重新扫描、生成；不要改摘要绕过检查 |
| 播放比请求 BPM 慢 | 查看逐段速度与加速度统计，同时区分名义时长和默认 30% 的首播时长 |

## 完整代码分工

| 文件 | 内容 |
| --- | --- |
| [rs_dance_virtual_pivot_scan.py](../../tools/rs_dance_virtual_pivot_scan.py) | 集中默认配置、包络网格、带边界的信赖域最小二乘 IK、奇异与工作区检查 |
| [rs_dance_virtual_pivot_primitives.py](../../tools/rs_dance_virtual_pivot_primitives.py) | 圆锥、平面钟摆、过冲、螺旋和按拍定格 |
| [rs_dance_virtual_pivot.py](../../tools/rs_dance_virtual_pivot.py) | 时间轴、逐帧求解、分段配速、固定 P 验证与离线命令入口 |
| [rs_dance_virtual_collision.py](../../tools/rs_dance_virtual_collision.py) | 原始三角网格自碰撞、坐标一致性、资源摘要检查 |
| [rs_dance_showcase.py](../../tools/rs_dance_showcase.py) | 扫描结果接入、动作库生成及加载校验 |
| [rs_dance_console.py](../../tools/rs_dance_console.py)、[rs_dance_speed.py](../../tools/rs_dance_speed.py) | 真实控制入口、保护保持与默认 30% 时间缩放 |

严格配置与显式备选分别在 [dance_virtual_pivot_strict.json](../../tools/config/dance_virtual_pivot_strict.json)、[dance_virtual_pivot_preview.json](../../tools/config/dance_virtual_pivot_preview.json)。模型动力学使用原有夹爪质量，没有把背景中的 1.5 kg 当作已验证的附加负载。规划中的 P 误差不是实体指尖误差测量。

“70% 工作区”仍是链长上界的保守径向筛选，配合高度、关节余量和雅可比检查，不是已验证可达体积的 70%。现有控制周期约 50 Hz，预览帧率与轨迹执行倍率是另外的概念。所有扫描及绘图都在离线执行，不发送 CAN 命令。
