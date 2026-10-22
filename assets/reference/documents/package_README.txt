reBot RS 桌宠开发资料包

先阅读 Word 开发说明，再检查现有工程。
assets/character_reference.png：选定角色原图。
assets/interaction_storyboard.png：六种交互动作示意原图。
behavior_defaults.json：与文档对应的可调初始参数，尚未实机验证。

无需重新设计或生成角色形象。原图可用于提取已有姿态与表情，但不是完整动画序列；中间帧、干净脸部底图、眼睛图层和 RGB565 资源仍需制作。不要把整张宣传图直接当成角色帧。
当前先做屏幕与触摸，CAN 保持待机，机械臂动作接口默认 Mock。文档中的屏幕像素和角度不得直接换算为机械臂关节指令。
所有原始 PNG 按字节复制，未重新生成或压缩。
