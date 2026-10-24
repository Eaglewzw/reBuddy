# 按互动图修订的蹭蹭原画

参考 [互动示意图](../../reference/interaction_storyboard.png) 的第 02 格：笑眼、头侧轻靠、亲昵蹭蹭。角色结构沿用 [三关节角色基准](../interaction-v3-8fps/idle-master.png)，肩部、肘部、头部连接处三个银色关节都保留。

采用内置 imagegen 制作。首轮 [提示词](prompt.txt) 对应 [初始图集](initial-atlas.png)；随后用 [定向修正提示词](refine-prompt.txt) 增强头部侧倾，得到最终 [atlas.png](atlas.png)。没有通过图像缩放来增大动作。

建议选帧见 [timing-suggestion.json](timing-suggestion.json)：8 FPS，6 秒，先笑眼侧倾轻靠，稍退后再蹭一次，沿原姿态回中。选择前 8 格反向回位；未采用后半程更像向下点头的姿态。推荐选帧保证同侧视角与首尾同图，但生成原画的零件细节仍需随最终 GIF 检查。

本目录只提供原画与选帧建议，不更改当前 GIF、公共导出工具或固件。机械臂时长为设计预算，没有从画面像素换算实际关节角度。
