# 探头 v3 · 新分镜草案

本轮用户允许重新设计探头。本图为待审阅概念分镜，未获选定、未接入固件。历史 probe-v2 的原图、清单与 GIF 均保留。

- 输入：`assets/approved/idle_master.png`（角色身份）与 `assets/reference/interaction_storyboard.png`（互动意图）。
- 工具：内置 `image_gen.imagegen`，一次生成；[完整提示词](prompt.txt)。
- 输出：[四格分镜](storyboard-draft.png)；哈希与状态见 [manifest.json](manifest.json)。
- 分段：注意 0–120 ms；前探 120–450 ms；观察 450–750 ms；回位 750–1200 ms。

当前图可用于讨论眼神先行、肘部打开与小幅靠近的方向。格间底座落点、比例和回位轮廓仍有差异，必须在制作动画前统一；这四格不是可直接播放的连续帧，也不是已完成的 ESP32 资源。正式制作需锁定镜头与底座、保持连杆长度、补齐不同姿态和眼睛锚点，再缩到 160×160 检查。
