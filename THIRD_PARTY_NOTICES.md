# 第三方与参考来源

- 私人硬件参考：`git@github.com:Eaglewzw/esp32c5_ui_console.git`，提交 `ba288ae49c80f19b10d14dd9d428f63e3dd41486`。独立存放于 reBuddy 项目目录之外，构建时通过外部路径引用，不作为子模块或源码副本发布。该仓库的可见性和源码再分发授权独立于本项目；本项目没有替其授予公开权。
- ESP-IDF：Espressif 官方 SDK，遵循其仓库许可证；本次使用 5.5.5。
- LVGL 9.5.0：MIT，许可证见组件 `LICENCE.txt`。
- Espressif `esp_lvgl_port 2.9.0`、`esp_lcd_touch 1.2.1`、`esp_lcd_touch_gt911 1.2.1`：许可证随组件分发，以各组件 `license.txt` 为准。
- 原始需求：用户提供的《基于 ESP32-C5 的 reBot RS 机械臂桌宠方案书》，副本见 `docs/PROJECT_BRIEF.md`。

角色外观以用户提供的设定图为参考，由内置 image_gen 生成透明立体素材，生成提示词和原图保存在 `assets/character/`；眼睛、眉毛、腮红与表情由 LVGL 绘制。没有复制私人控制台的品牌图片、字体资源或整套 UI 页面。
