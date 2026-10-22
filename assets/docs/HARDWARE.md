# 硬件与输入

本项目自带 `rebuddy_board`，针对 XIAO ESP32-C5 CAN Console 使用 ST7789 显示和 GT911 触摸，没有私人应用仓库依赖。当前仅触摸屏交互，不配置或读取实体按键、方向键、确认键、返回键或 GPIO28 急停输入；CAN 收发器保持待机，没有电机控制后端。

| 项目 | 配置 |
| --- | --- |
| LCD | ST7789，原生 240×320，横屏 320×240，RGB565，SPI 40 MHz |
| 触摸 | GT911，I²C 地址 0x5D，额外触摸按键数量为 0 |
| 扩展器 | PCA9535，地址 0x21，仅用于必需输出 |
| Flash / PSRAM | 8 MiB / 8 MiB Quad 40 MHz |
| LCD SCLK / MOSI / CS / D-C | GPIO 8 / 10 / 7 / 25 |
| I²C SDA / SCL | GPIO 23 / 24 |
| GT911 INT / RESET | GPIO0 / PCA9535 P0.1 |
| LCD RESET / 背光 | PCA9535 P0.0 / P0.5 |
| CAN 待机 / 状态灯关闭 | PCA9535 P1.0～P1.1 / P1.2～P1.4 拉高 |

PCA9535 输出锁存器先写入再切换方向；GT911 在复位期间拉低 INT 选择 0x5D 地址。初始化顺序为共享 I²C、PCA9535 输出、LCD、GT911、LVGL，随后创建桌宠 UI；没有按键轮询任务。

触摸映射为 `(x, y) = (raw_y, 239 - raw_x)`，不要在 UI 再次旋转。触摸每 20ms 轮询，事务超时 20ms。读失败会取消本次手势，松手后可重新触摸，不锁住动画。显示缓冲为两块 320×40 RGB565 DMA 内存，共 51,200 字节；帧缓存使用 PSRAM。

启动失败会报错中止，PSRAM 缺失也会按此板卡配置中止启动。更换屏幕或内存配置需重新验证。首次上板确认颜色、朝向、四角触摸、手爪区域、睡醒流程及 30 分钟运行情况。显示和内存日志口径见 [DISPLAY_PERFORMANCE.md](DISPLAY_PERFORMANCE.md)。
