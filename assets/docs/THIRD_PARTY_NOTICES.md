# 第三方与参考来源

- ESP-IDF：Espressif 官方 SDK，遵循其仓库许可证；构建基线为 5.5.6。
- LVGL 9.5.0：MIT，许可证见组件 `LICENCE.txt`。
- Espressif `esp_lvgl_port 2.9.0`、`esp_lcd_touch 1.2.1`、`esp_lcd_touch_gt911 1.2.1`：许可证随组件分发，以各组件 `license.txt` 为准。
- 原始需求：用户提供的《基于 ESP32-C5 的 reBot RS 机械臂桌宠方案书》，项目范围与原始资料入口见 `assets/docs/PROJECT.md`。

当前角色直接来源于用户提供的《reBot RS 桌宠开发资料包》，原始 PNG 位于 `assets/images/reference/`，出处及校验值见 `assets/config/behavior_defaults.json` 和 `assets/animation/manifest.json`。当前运行帧来源于 [当前 GIF 清单](../gifs/daily/manifest.json) 中记录的有效图集；来源与导出说明在 [日常素材说明](../images/source/daily/base/README.md)。表情烘焙在图中，不叠加矢量眼睛。原始图未附独立许可证，代码许可证不自动覆盖第三方原图。未使用的历史生成角色已清理。已选角色基准保留在 `assets/images/reference/approved/`。

板级组件 `components/rebuddy_board/` 为本项目实现，接线依据目标板卡配置；不包含或链接私人应用仓库。驱动接口与寄存器依据以下公开资料：

- [ESP-IDF SPI LCD 文档](https://docs.espressif.com/projects/esp-idf/en/v5.5.1/esp32c5/api-reference/peripherals/lcd/spi_lcd.html)：ST7789 面板与 SPI 接口。
- [TI PCA9535 数据手册](https://www.ti.com/lit/ds/symlink/pca9535.pdf)：输入、输出、极性和方向寄存器。
- [Espressif 显示与触摸组件](https://github.com/espressif/esp-bsp/tree/master/components)：LVGL Port 与 GT911 的公开接口。

当前抬头使用 `probe-look-up-atlas.png`，开心使用 `happy-rise-clap/atlas.png`；各动作以 GIF 清单记录的路径与校验值为准。内部背景空隙的合成修正不修改原始 PNG。

当前睡眠使用日常素材目录中的无腮红 `rest-atlas.png` 和 `breath-atlas.png`；已删除不再使用的旧版睡眠修补脚本与源图。

当前十组舞蹈来自用户提供的 `2D机械臂_当前10组合集.zip`，仅导入 `gifs/` 中的原始文件，未导入对比稿。来源及逐文件校验见 `assets/gifs/dance/manifest.json`，转换说明见该目录 README；代码许可证不自动覆盖这些素材。
