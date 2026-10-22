# 素材状态

| 资源 | 用途 |
| --- | --- |
| `assets/reference/` | 用户提供的完整角色设定和互动分镜，原样保留 |
| `assets/approved/` | 用户选定的待机基准与 probe-v2 两姿态源图；对应预览保留在 `docs/preview/probe-v2/`，未作为当前运行帧 |
| `assets/design/screen-preview-v1/` | 当前 24 张关键姿态的原图和生成提示词 |
| `assets/design/sleep-v2/` | 当前 4 张睡眠呼吸姿态、输入图和提示词 |
| `assets/design/room-v1/` | 当前小屋背景原图和提示词 |
| `assets/design/probe-v3/` | 动作分镜草案，保留为设计参考，不参与固件构建 |
| `assets/animation/` | 从上述运行源图导出的 28 帧、背景和动作清单 |
| `assets/gifs/` | 当前小屋版 10 个动作评估 GIF，与固件共用姿态和问号节奏 |
| `assets/fonts/` | 中文字形来源的 OFL 许可证 |

当前表情直接绘制在帧图中，疑惑问号与睡眠 Z 单独绘制。动画仍是稀疏关键姿态，局部关节结构与衔接尚需细化。原始图片没有附带独立开放许可证，代码许可证不自动覆盖图片。

[动画标准](ANIMATION_FIDELITY.md) · [动作分镜设计](ui/actions-v1/README.md) · [独立 GIF](../assets/gifs/README.md)
