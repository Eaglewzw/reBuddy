# 硬件与输入

本项目自带 `rebuddy_board`，针对 XIAO ESP32-C5 CAN Console 使用 ST7789 显示和 GT911 触摸，没有私人应用仓库依赖。实体 ENTER 接为 HOME 回零，接法参考 `esp32c5_ui_console` 的 PCA9535 P0.6（低电平按下）；其他实体键及GPIO28急停未接入。默认daily控制构建上电保持SLEEP且不使能，点击屏幕唤醒后才开始动作。

| 项目 | 配置 |
| --- | --- |
| LCD | ST7789，原生 240×320，横屏 320×240，RGB565，SPI 40 MHz |
| 触摸 | GT911，I²C 地址 0x5D，额外触摸按键数量为 0 |
| 扩展器 | PCA9535，地址 0x21，必需输出与 ENTER 输入 |
| 实体 ENTER | PCA9535 P0.6，低电平按下，触发 HOME |
| Flash / PSRAM | 8 MiB / 8 MiB Quad 40 MHz |
| LCD SCLK / MOSI / CS / D-C | GPIO 8 / 10 / 7 / 25 |
| I²C SDA / SCL | GPIO 23 / 24 |
| GT911 INT / RESET | GPIO0 / PCA9535 P0.1 |
| LCD RESET / 背光 | PCA9535 P0.0 / P0.5 |
| CAN 待机 / 状态灯关闭 | PCA9535 P1.0～P1.1 / P1.2～P1.4 拉高 |

PCA9535 输出锁存器先写入再切换方向；GT911 在复位期间拉低 INT 选择 0x5D 地址。初始化顺序为共享 I²C、PCA9535 输出、LCD、GT911、LVGL，随后创建桌宠 UI；daily控制初始化成功后，独立低优先级任务约20ms读取一次ENTER输入，稳定30ms去抖。上电按住或I²C读取失败后必须稳定松开才能重新触发；按键读取不会修改扩展器输出。

触摸映射为 `(x, y) = (raw_y, 239 - raw_x)`，不要在 UI 再次旋转。触摸每 20ms 轮询，事务超时 20ms。读失败会取消本次手势，松手后可重新触摸，不锁住动画。显示缓冲为两块 320×40 RGB565 DMA 内存，共 51,200 字节；帧缓存使用 PSRAM。

启动失败会报错中止，PSRAM 缺失也会按此板卡配置中止启动。更换屏幕或内存配置需重新验证。首次上板确认颜色、朝向、四角触摸、手爪区域、睡醒流程及 30 分钟运行情况。显示和内存日志口径见 [DISPLAY_PERFORMANCE.md](DISPLAY_PERFORMANCE.md)。

## 机械臂控制组件接入

`components/rebuddy_arm` 提供 reBuddy 的启动、只读状态及daily执行协调器。底层五个 `can_console_*` 组件来自本机 `esp32c5_ui_console` 的固定源码快照，来源提交及逐文件 SHA-256 见 [arm_components.json](../config/arm_components.json)。构建不访问原工程目录，也不引入原工程 UI、USB 命令台、网络、示教文件分区或第二套屏幕/I²C 初始化。

daily控制构建调用 `rb_arm_daily_start()` 和 `rb_arm_init_control()`，选择 RS 模型，在 CAN1/TWAI0 上绑定 ID 1–3 为 rs-06、ID 4–7 为 rs-00，控制周期20ms。前六路为运动关节，第七路为独立夹爪。仅屏幕/监听构建仍调用 `rb_arm_init(monitor)`。八项daily动作、回零和异常恢复策略见 [日常动作说明](DAILY_COMMANDS.md)；编舞仍是屏幕预览。

仅屏幕构建（关闭 `CONFIG_REBUDDY_ARM_DAILY_CONTROL`）下，默认 `CONFIG_REBUDDY_ARM_MONITOR` 关闭：CAN1 与 CAN2 待机，无周期控制发送。需要接收已有状态广播时，在 menuconfig 的 `reBuddy arm components` 中启用该选项；CAN1 使用 GPIO5 TX / GPIO3 RX、Classic CAN 1 Mbps、硬件 LISTEN_ONLY 模式。此模式不发送探测、不订阅状态、不发送 ACK，因此电机未主动报告或没有其他节点 ACK 时可能收不到有效反馈。它不能用于控制机械臂，也不改变 CAN2。

`rb_arm_get_status()` 返回七路原始电机弧度、弧度/秒和力矩，包含夹爪电机角度而非归一化开度。反馈只在总线可用、来源匹配、位置/速度/力矩有限且反馈年龄不超过 100 ms 时标记有效；缺失或过期数据为 NaN。掩码不代表标定、使能或可运动；七路数据也不是同时采样的物理快照。主任务每 30 秒记录组件与反馈状态。

初始化为一次性流程，部分失败会锁定失败状态并尝试让 CAN1 回到待机，不重试上游单例初始化；屏幕可继续工作。适配层初始化必须在板级初始化之后、其他调用方启动之前完成，之后由串行控制任务执行daily请求，并提供并发只读查询。规划工作任务独立于 LVGL，无动作时不生成轨迹。

daily动作已接入七轴指令、反馈保护及停止链路，尚需实机验收。实体 ENTER 和屏幕 HOME 都是软件回零请求，不是物理急停；GPIO28急停尚未接入。ENTER不触发使能，不写入编码器零点；只有屏幕唤醒入口请求使能。
