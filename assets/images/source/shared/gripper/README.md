# 共用底盘与夹指图层

这里保存当前舞蹈和已选 V6 共用的底盘与夹指源文件。它们最初为 V5 制作，现按共用用途归类；旧 V5 GIF 已删除。

- [turntable.png](turntable.png)：三个底盘视角，供“底盘转拍”和“左右交替”使用。来源为内置 ImageGen，提示词见 [视角](turntable-prompt.txt)、[方向修正](turntable-correction-prompt.txt) 和 [面板修正](base-panel-prompt.txt)。
- [jaws.png](jaws.png)：原夹爪左右两侧的独立黑色夹指，供上述舞蹈与 [V6](../../cat-v6/README.md) 共用；[生成提示词](jaw-prompt.txt)。
- [rig.json](rig.json)：舞蹈装配使用的固定采样比例、旋转中心与安装点。

生成当前舞蹈使用 `python3 tools/export_dance_gifs.py`，生成 V6 使用 `python3 tools/export_claw_sway_preview.py`。部件不做动态缩放，底盘方向是三个离散采样视角。
