# 设计源文件

| 目录 | 用途 |
| --- | --- |
| [daily/base](daily/base/README.md) | 日常基础图集与完整时间轴 |
| [daily/nuzzle](daily/nuzzle/README.md) | 固定肩肘的轻蹭原画 |
| [daily/tilt](daily/tilt/README.md) | 疑惑歪头原画与独立关键姿态 |
| [daily/happy](daily/happy/README.md) | 起身、舒展、夹指开合的开心回应 |
| [dance/basic](dance/basic/README.md) | 基础六种舞蹈与当前运行时猫步图层 |
| [dance/extended](dance/extended/README.md) | 新增六种舞蹈、关键姿态与时间轴 |
| [cat-v6](cat-v6/README.md) | 已选 V6 捂鼻、向上扇动的源图和绑定参数 |
| [shared/gripper](shared/gripper/README.md) | 舞蹈与 V6 共用的底盘视角、左右夹指和安装参数 |
| [expressions](expressions/README.md) | 三个独立表情的图集和时间轴 |
| [room](room/README.md) | 桌宠小屋背景 |

每组原画的提示词、来源说明和参数随原画保存。`poses.png` 等检查图仅用于观察姿态；最终 GIF 在 [gifs](../../gifs/README.md)。

所有工具均从工程根目录运行。日常、表情、舞蹈和 V6 分别使用 `tools/export_gifs_v2.py`、`tools/export_expression_preview.py`、`tools/export_dance_gifs.py`、`tools/export_claw_sway_preview.py`。需要更新固件资源时，再运行 `tools/build_screen_preview.py`。
