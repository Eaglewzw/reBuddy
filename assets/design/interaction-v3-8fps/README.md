# 灵动可爱新版素材

这是当前 GIF 动画的设计源素材，输出位于 [gifs-v2](../../gifs-v2/README.md)。固件使用当前 GIF 转换的 RGB565 帧，不在设备上裁切原画图集。

使用内置 imagegen 工具，参考 [批准的角色基准](../../approved/idle_master.png) 和 [互动图](../../reference/interaction_storyboard.png)，分别生成每个动作的 4×4 原画图集。实际图集尺寸、SHA-256 和固定采样比例由 [导出清单](../../gifs-v2/manifest.json) 记录，不假定生成结果一定符合提示词的目标分辨率。

| 原画 | 完整提示词 | 用途 |
| --- | --- | --- |
| probe-atlas.png | [probe-prompt.txt](probe-prompt.txt) | 旧版轻微探头图集，保留为角色参考来源；不再用于新版抬头导出 |
| probe-look-up-atlas.png | [probe-look-up-prompt.txt](probe-look-up-prompt.txt) | 头部围绕上方关节仰起、面屏朝上注视，再回位；选取 0、4、5、6、7、8 格并反向返回，排除前段近似整体上移的 2、3 格 |
| nuzzle-atlas.png | [nuzzle-prompt.txt](nuzzle-prompt.txt) | 保留的旧轻蹭原画；前两排为亲昵表情参考，后两排翻转视角未采用；不再用于当前新版导出 |
| nuzzle-clearer-atlas.png | [nuzzle-clearer-prompt.txt](nuzzle-clearer-prompt.txt) | 保留的幅度增强版；当前改用互动图参考版 |
| [旧互动图蹭蹭原画](../nuzzle-storyboard/atlas.png) | [初始提示词](../nuzzle-storyboard/prompt.txt)、[定向修正](../nuzzle-storyboard/refine-prompt.txt) | 保留的上一稿，已被固定肩肘版本替换 |
| [固定肩肘轻蹭原画](../nuzzle-cheek/atlas.png) | [初始提示词](../nuzzle-cheek/prompt.txt)、[定向修正](../nuzzle-cheek/refine-prompt.txt) | 只采用第一排 4 个姿态，肩肘保持位置，笑眼头侧短程往返两次 |
| tilt-atlas.png | [tilt-prompt.txt](tilt-prompt.txt) | 保留的旧小幅歪头原画；当前使用清楚侧歪版 |
| [清楚侧歪原画](../tilt-clear/atlas.png) | [制作说明与提示词](../tilt-clear/README.md) | 固定肩肘，头壳和面屏一起侧滚，峰值停留 1.25 秒再回正；2×2 图集、三个有效姿态 |
| withdraw-atlas.png | [withdraw-prompt.txt](withdraw-prompt.txt) | 收拢，选定第 12 格为终态，排除后段闭眼似睡姿态 |
| recover-atlas.png | [recover-prompt.txt](recover-prompt.txt) | 从收拢展开，选用连续展开姿态 |
| happy-atlas.png | [happy-prompt.txt](happy-prompt.txt) | 保留的旧开心原画；当前改用舒展版 |
| [旧舒展开合原画](../happy-open/atlas.png) | [提示词](../happy-open/prompt.txt) | 保留的上一稿，当前不再采用其夹爪开合 |
| [完整夹爪开心原画](../happy-pose/atlas.png) | [提示词](../happy-pose/prompt.txt) | 保留的闭合夹爪姿态稿，作为同步开合版的舒展姿态参考 |
| [旧双指同步开合原画](../happy-symmetric-clap/atlas.png) | [关键帧提示词](../happy-symmetric-clap/keyframes-prompt.txt)、[同步修正](../happy-symmetric-clap/symmetric-correction-prompt.txt) | 保留的固定舒展姿态夹爪试稿；当前改用完整起身过程 |
| [完整起身开心原画](../happy-rise-clap/atlas.png) | [提示词](../happy-rise-clap/prompt.txt)、[起身修正](../happy-rise-clap/rise-correction-prompt.txt) | 同一图集逐步起身、完全舒展、双指轻开合两次，再沿原姿态回位 |
| rest-atlas.png | [rest-prompt.txt](rest-prompt.txt) | 无腮红困倦入睡；唤醒选用同一姿态序列反向展开 |
| idle-atlas.png | [idle-prompt.txt](idle-prompt.txt) | 按完整黑色面屏内部蒙版合成眨眼，保留相同外壳与身体像素；不用会截断右眼的矩形裁剪 |
| breath-atlas.png | [breath-prompt.txt](breath-prompt.txt) | 无腮红熟睡微动，选取前半程并原路返回 |

`idle-master.png`、`folded-master.png` 和 `sleep-master.png` 是从新图集中提取的生成参考，分别来自 probe 第 0 格、withdraw 第 12 格和 rest 第 15 格。所有编号从 0 开始。只提取参考图，没有重新拟合角色尺寸。

[timing.json](timing.json) 保存每个动作的选帧、每姿态停留拍数、总时长、循环方式和机械臂时间预算假设。8 FPS 是显示时间基准，停留段会保持同一姿态；图片帧不直接转换为电机角度。

导出工具为 `tools/export_gifs_v2.py`。调色板在整段动画内固定，背景沿用现有小屋。连杆空隙使用 [逐姿态背景种子](../../config/preview_cavity_seeds.json) 去掉源图底色，并绑定源图校验值；更换原画后必须重新核对。孤立装饰线被排除。只做固定采样和底座位置对齐，不使用逐帧缩放来伪造动作。

当前 GIF 已作为固件动画源。原画细节的一致性、连接过渡和实机运动同步尚不能视为已验收。
