# GIF 动画

所有 GIF 集中在此目录，按用途分类；各目录中的说明与清单记录时长、来源和使用状态。

| 目录 | 内容 | 固件使用状态 |
| --- | --- | --- |
| [daily](daily/README.md) | 10 个日常动作：待机、抬头、轻蹭、歪头、收拢、展开、开心、入睡、熟睡、唤醒 | 已使用 |
| [dance](dance/README.md) | 10 个独立完整角色基本舞蹈 GIF | 新一轮素材待视觉验收；固件同步状态以运行资源清单为准 |
| [expressions](expressions/README.md) | 三关节待机、开心、好奇，三个独立 GIF | 补充预览 |
| [cat-v6](cat-v6/README.md) | 已选 V6：左侧捂鼻、右侧向上快挥、底盘转向 | 已选保留，独立预览 |
| [reference](reference/README.md) | 原始月薪猫动作参考、已选探头两姿态参考 | 制作参考 |

直接查看：[V6 猫步](cat-v6/01_左夹指捂鼻_右夹指快挥.gif) · [十个独立基本舞蹈动作](dance/README.md)。

制作源文件统一位于 [images/source](../images/source/README.md)。不要把原始图集或临时导出图片放入 GIF 目录；临时对比输出放到工程 `build/`。

早期完整角色重绘与尺寸稳定性试验保留在 [dance-preview](dance-preview/README.md)，不是本轮十个动作的交付入口。本轮请逐段查看 `dance`，先确认动作、局部尺寸与衔接，再制作组合。每段从 12 格源图集中筛选可用完整姿态，加重复停留构成 3／6 秒、平均 8 FPS 动画，尚未完成视觉和硬件验收。
