> 此图集和形变插值方案已退出08预览。当前采用 [rigid_turn](../rigid_turn/README.md) 的完整姿态逐格播放，不再使用光流、网格或混合帧。

# 08 完整角色转身原画

当前08使用这一套完整角色原画；09不变。图集通过内置 **image_gen** 生成，参考原角色 `build/dance-original/neutral-original.png`，肩肘、连杆、头部、夹爪和底盘在每一格中共同绘制。

- `turnaround.png`：最终九视角透明图集。
- `prompt.txt`、`front_prompt.txt`：完整绘图和正面视角修正的原始提示词。
- `views/`、`views.json`：完整角色切片、整体等比缩放及平移参数、素材校验值。
- `landmarks.json`：原画里的关节、眼睛、夹爪和外壳对应点；坐标位于图集原始单元格中。
- `registered_sheet.png`：原画检查图。

先运行 `python3 -B tools/prepare_dance_integrated_turn.py`，再运行 `PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1 /usr/bin/python3 -B tools/render_dance_whole_turn.py`。每幅完整原画通过同一平滑变换生成中间姿态，再做双向光流对齐与五次连续融合。使用稀疏关节/眼睛/夹爪中心的正则化薄板样条，不再使用可能局部翻折的三角网格。主变换位移梯度的谱范数不超过0.30；残差光流不超过0.15，且位移不超过10像素。两者都有连续、正向的采样映射。不再裁切肩壳、覆盖连接区域或粘贴另一帧的底盘；角色的各部位始终来自同一完整视角绘图。

输入时钟与摇摆相位继续来自 `build/dance-3d/motion.json`，共110帧、两轮摇摆、GIF时长8.74秒。最低视角约5°，没有越过正面；该段沿用12°整幅原画变形到目标姿态，避免肘部遮挡切换。绘图是二维角色表现，不是实机运动学仿真。

输出仍位于 `build/dance-3d/whole_turn/`，主预览为 `build/dance-original/index.html`，原画对照为 `build/dance-whole-turn/index.html`。仅更新电脑端预览，不覆盖固件资源。质量记录为 `assets/verification/dance_whole_turn.json`。

视角标定将40°的起止姿态对齐到中间那张完整原画，避免静止姿态仍在两幅画之间混合。已依据原图亮部轮廓修正眼睛定位点。实际渲染的形变最大值记录在输出 manifest 的 `deformation_report`，对应质量检查也写入 verification 文件。
