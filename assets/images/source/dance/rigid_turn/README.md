# 08 完整姿态逐格转身

当前08使用16张完整角色原画直接逐格播放。渲染阶段没有网格变形、薄板样条、光流、半透明混合或局部零件覆盖。每个RGBA播放帧与选中的注册原画逐像素相同，避免算法把圆形关节、直连杆或脸部拉成橡皮形状。

原画由内置 **image_gen** 生成，参考已确认的绿色卡通角色和旧GitHub转身的正面姿态。每格包含完整底盘、肩肘、连杆、头部和夹爪，沿用二维线稿画风。

- `front.png`、`side.png`：两张4×3透明原画图集，共24张完整姿态。
- `front_prompt.txt`、`side_prompt.txt`：完整绘图提示词。
- `quarter_reference.png`：第一组最后一格的完整角色，供第二组衔接参考。
- `views/`、`views.json`：留存的19张完整姿态（播放仅采用前16张，排除56°/59°/62°）、视角、整幅等比缩放及平移参数、校验值。
- `registered_sheet.png`：采用姿态的检查图。

准备阶段只对每格完整原画做一次等比缩放和整体平移，把底盘基线对齐到同一位置，不拉伸局部关节、连杆或头部。第一组40°与第二组40°重复，只保留第一组；第二组65°以上的极侧面会使脸部过窄，不进入播放。

屏幕视角映射为输入5°/40°/75°对应原画5°/40°/52°，仅收小侧转终点，保留40°起始姿态。实机动作数据不改。按原时钟的110个采样选取最接近的完整姿态，保留两次来回、8.74秒和相同首尾。GIF编码器可合并重复帧，实际编码帧数小于110，但总时长不变。此版本采用逐格观感，不通过插值伪造平滑帧。

重建：

```bash
python3 -B tools/prepare_dance_rigid_turn.py
python3 -B tools/render_dance_whole_turn.py
```

主预览仍为 `build/dance-original/index.html`，对照页为 `build/dance-whole-turn/index.html`，输出在 `build/dance-3d/whole_turn/`。09 保持原版本。渲染命令只生成预览，不控制实机；当前确认版已通过 `python3 tools/install_dance_cartoon.py --reviewed` 安装到正式 GIF/RGBA 目录，并重建固件动画资源。质量记录见 `assets/verification/dance_whole_turn.json`。
