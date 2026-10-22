# 项目目标与范围

在 ESP32-C5 和现有屏幕硬件上制作 reBot RS 桌宠：角色住在小屋中，通过触摸与预设关键姿态互动。交付源码、配置、动画资源、固件和运行说明，目标包括可现场体验及视频展示。

当前实现轻点探头、抚摸轻蹭、快划歪头、三击收拢、安抚展开、摸头开心、持续睡眠及长按唤醒；两种风格共用资源。仅使用触摸屏，电机动作和编舞不属于本阶段。

原始资料包正文保留在 [reference/development_spec.txt](reference/development_spec.txt)，其中硬件按键、独立眼睛和普通触摸唤醒等条目已由后续交互要求替代。当前行为以 [开发指南](GETTING_STARTED.md)、`config/behavior_defaults.json` 和运行代码为准，视觉设计参考 [动作分镜](ui/actions-v1/README.md)。
