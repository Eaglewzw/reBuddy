# 第三方与参考来源

- ESP-IDF：Espressif 官方 SDK，遵循其仓库许可证；构建基线为 5.5.5。
- LVGL 9.5.0：MIT，许可证见组件 `LICENCE.txt`。
- Espressif `esp_lvgl_port 2.9.0`、`esp_lcd_touch 1.2.1`、`esp_lcd_touch_gt911 1.2.1`：许可证随组件分发，以各组件 `license.txt` 为准。
- 原始需求：用户提供的《基于 ESP32-C5 的 reBot RS 机械臂桌宠方案书》，副本见 `docs/PROJECT_BRIEF.md`。

角色外观以用户提供的设定图为参考，由内置 image_gen 生成透明立体素材，生成提示词和原图保存在 `assets/character/`；眼睛、眉毛、腮红与表情由 LVGL 绘制。没有复制私人控制台的品牌图片、字体资源或整套 UI 页面。

板级组件 `components/rebuddy_board/` 为本项目实现，接线依据目标板卡配置；不包含或链接私人应用仓库。驱动接口与寄存器依据以下公开资料：

- [ESP-IDF SPI LCD 文档](https://docs.espressif.com/projects/esp-idf/en/v5.5.1/esp32c5/api-reference/peripherals/lcd/spi_lcd.html)：ST7789 面板与 SPI 接口。
- [TI PCA9535 数据手册](https://www.ti.com/lit/ds/symlink/pca9535.pdf)：输入、输出、极性和方向寄存器。
- [Espressif 显示与触摸组件](https://github.com/espressif/esp-bsp/tree/master/components)：LVGL Port 与 GT911 的公开接口。
