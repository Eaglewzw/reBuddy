# 设计源文件

| 目录 | 用途 |
| --- | --- |
| [daily/base](daily/base/README.md) | 日常基础图集与完整时间轴 |
| [daily/nuzzle](daily/nuzzle/README.md) | 固定肩肘的轻蹭原画 |
| [daily/tilt](daily/tilt/README.md) | 疑惑歪头原画与独立关键姿态 |
| [daily/happy](daily/happy/README.md) | 起身、舒展、夹指开合的开心回应 |
| [dance/individual](dance/individual/README.md) | 当前十个独立基本舞蹈：完整角色图集、提示词与筛选时间轴 |
| [dance/primitives](dance/primitives/README.md) | 已弃用分层方案的制作参考，不再用于当前 GIF 导出 |
| [cat-v6](cat-v6/README.md) | 已选 V6 捂鼻、向上扇动的源图和绑定参数 |
| [shared/gripper](shared/gripper/README.md) | 保留的夹爪外观与安装参考，非当前十动作导出入口 |
| [expressions](expressions/README.md) | 三个独立表情的图集和时间轴 |
| [room](room/README.md) | 桌宠小屋背景 |

每组原画的提示词、来源说明和参数随原画保存。`poses.png` 等检查图仅用于观察姿态；最终 GIF 在 [gifs](../../gifs/README.md)。

所有工具均从工程根目录运行。日常、表情、舞蹈和 V6 分别使用 `tools/export_gifs_v2.py`、`tools/export_expression_preview.py`、`tools/export_dance_gifs.py`、`tools/export_claw_sway_preview.py`。需要更新固件资源时，再运行 `tools/build_screen_preview.py`。
