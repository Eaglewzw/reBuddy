# 完整角色逐帧原画试验

以 daily 的 [idle-master.png](../../daily/base/idle-master.png) 为角色参考，使用内置 ImageGen 绘制完整姿态，避免旋转裁切部件造成的拼接感。

- `groove-atlas.png`：当前 4×4 完整姿态图集。
- [首次生成要求](prompt.txt)：角色比例、完整绘制、转向与节奏要求。
- [朝向修正要求](refine-prompt.txt)：排除突然镜像翻转。
- [尺寸修正要求](size-repair-prompt.txt)：约束底座、头部及关节尺寸。

原画仍存在局部尺寸差异。当前只选用第 1、2、3 格（从 0 开始），完整角色按 daily 的底座宽度标定，再回放完整姿态。没有头部／手爪分层变换，也不将不同姿态插值为拉伸动画。选帧、离线标定比例与输出状态记录在 [预览清单](../../../../gifs/dance-preview/manifest.json)。

这不是最终十段舞蹈原画，不作为实机轨迹。未采用的格子不进入当前预览。

```bash
python3 tools/export_whole_dance_preview.py
```
