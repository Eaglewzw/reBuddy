# 硬件与参考版本

当前板卡配置来自用户指定的 `Eaglewzw/esp32c5_ui_console`，固定提交 `ba288ae49c80f19b10d14dd9d428f63e3dd41486`。这是已有实现的接线依据，本项目尚未完成实机复核。

| 项目 | 配置 |
| --- | --- |
| 开发板 | XIAO ESP32-C5 CAN Console |
| 显示屏 | ST7789；原生 240×320，横屏 320×240；RGB565；SPI 20 MHz |
| 触摸 | GT911，I²C 地址 0x5D |
| 扩展器 | PCA9535；LCD、触摸复位、背光、实体输入和 CAN 待机控制共用 |
| Flash / PSRAM | 8 MiB / 8 MiB Quad 40 MHz，沿用参考板配置 |
| 机械臂 | reBot RS；ID1–3 RS06、ID4–7 RS00；前六轴为机械臂，J7 为夹爪 |
| CAN | 两路，1 Mbps；初版固定监听模式 |

| 信号 | GPIO / 扩展器 |
| --- | --- |
| LCD SCLK / MOSI / CS / D-C | 8 / 10 / 7 / 25 |
| I²C SDA / SCL | 23 / 24 |
| GT911 INT / RESET | GPIO0 / PCA9535 P0.1 |
| LCD RESET / 背光 | PCA9535 P0.0 / P0.5 |
| 波轮 A / B / PUSH | PCA9535 P0.2 / P0.3 / P0.4 |
| ENTER / BACK | PCA9535 P0.6 / P0.7 |
| PCA9535 INT | GPIO9 |
| 急停 | GPIO28，低有效 |
| CAN1 TX / RX | GPIO5 / GPIO3 |
| CAN2 TX / RX | GPIO2 / GPIO4 |

横屏触摸变换沿用驱动的 `(x, y) = (raw_y, 239 - raw_x)`；不要在桌宠 UI 再次旋转触摸坐标。字符和截图使用逻辑 RGB565，SPI 字节序交由参考显示组件处理。

启动顺序为急停输入、共享 I²C、PCA9535、ST7789/GT911/LVGL、NVS、CAN、实体输入、桌宠 UI。显示初始化异常由参考板级模块每 2 秒重试。日志通过 USB Serial/JTAG 输出，每 30 秒打印内存状态。

当前 PSRAM 缺失会按正式参考配置中止启动。更换无 PSRAM 板卡时应独立修改配置并重新测量内存；不能沿用此固件直接声称适配。

## 首次上板检查

1. 确认 Flash / PSRAM 容量和电源；先测试屏幕互动版本。
2. 检查显示朝向、角色点击、四角触摸及实体按键。
3. 验证 GPIO28 按下后显示 STOPPED，释放后仍需手动清除。
4. 记录 30 分钟的内部堆、PSRAM 堆、最低剩余与最大连续块。
5. 完成动作适配与标定后另行验证机械臂；当前软件急停只控制本 Demo，未宣称能够切断电机电源。
