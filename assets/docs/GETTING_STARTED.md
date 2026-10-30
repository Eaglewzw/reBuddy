# 开发与体验指南

当前为 ESP32-C5 纯触摸屏桌宠。使用 ESP-IDF 5.5.6、LVGL 9.5.0，板级组件随仓库提供，首次构建只下载公开依赖。

## 构建与烧录

```bash
source /path/to/esp-idf/export.sh
idf.py -B build/firmware build
# 由操作者选择实际设备并执行：
idf.py -B build/firmware -p /dev/ttyACM0 flash monitor
```

目标与默认配置见 `sdkconfig.defaults`。已有 `sdkconfig` 不会被 defaults 自动覆盖，需要通过 menuconfig 更新；本版 `CONFIG_ESP_LCD_TOUCH_MAX_BUTTONS=0`。串口不固定为 `/dev/ttyACM0`，请按实际连接选择。

## 体验操作

1. 角色识别区域为 x=80～239、y=40～199；轻点并松手探头。
2. 缓慢抚摸身体可轻蹭；在脸下方的手爪轻柔来回滑动，累计约 0.6 秒有效移动即可请求开心回应。若正在轻蹭，会先平滑结束当前动作；按住不动、摸头或摸眼睛不累计开心。手爪区域随显示姿态变化。开心结束后有 15 秒冷却，需松手至少 2 秒并等待冷却结束才可再次累计。[区域示意](../images/previews/claw-petting-guide.svg)
3. 1.2 秒内轻点三次收拢，再累计约 1.2 秒有效抚摸展开。展开后先松手再进行下一次互动。
4. 快划触发歪头，同一接触只触发一次；松手后可开始新手势。
5. 无操作 60 秒或设置选择休息后持续睡眠，呼吸和 Z 循环；只有长按左下角恢复按钮才唤醒。
6. 左下暂停后再次点击继续；右下设置切换活泼／慵懒、进入休息或打开“编排”。点击齿轮或菜单外区域关闭设置。

## 编舞体验

1. 在设置中点击“编排”，选择空槽位新建，或打开已经保存的作品。最多保存 10 个作品，可上下滚动列表；可选择十组独立舞蹈动作。
2. 添加动作或停留，点击步骤卡片调整停留、前移、后移或删除。每个作品最多 8 步，新增动作插在当前选中项之后。停留积木按半拍调整、固定一次，十组表演动作每段 2.4 秒，可重复 1–3 次。
3. 选择慢、标准或快节奏，点击“预览”再选择“屏幕预览”，自动加入开场和收尾，不占步骤。当前可用十组舞蹈动作与停留；“同步表演”不可用。
4. 预览中点击“停止预览”保持当前帧；停止或播放结束后可返回编辑或手动重播。作品不会自动循环或启动电机。
5. 点击“保存”将作品写入设备。未保存直接返回时，可以保存并返回、放弃修改或继续编辑。重建已有作品需要确认，只有再次保存才替换原作品。


编辑期间日常动画、手势请求与无操作计时暂停，退出后恢复原来的日常状态；正在睡觉的桌宠继续睡觉。加载、重启和退出页面不会自动续播作品。具体时长、小屋背景、存储恢复与当前边界见 [编舞说明](CHOREOGRAPHY.md)。

## 修改参数与素材

安装 Pillow 后使用下列独立工具：

```bash
python3 tools/generate_pet_config.py
python3 tools/export_gifs_v2.py
python3 tools/build_screen_preview.py
python3 tools/build_room_font.py
```


编舞的步数、停留范围和时间步长定义在 `components/rebuddy_core/include/rb_choreo.h`，动作目录与预览节奏在对应的 `src/rb_choreo.c`。作品存储使用 `main/choreo_store.c`；只在用户保存时写入 NVS，每个槽位保留两份校验记录。不应为初始化失败而直接擦除 NVS，避免丢失已保存作品。

日常行为参数在 `assets/config/behavior_defaults.json`。当前全局时长倍率与两种风格倍率均为 `1.0`，保留新版 GIF 的平均 8 FPS 与动作总时长；活泼和慵懒暂时不改变动画速度。开心回应为 8 秒，其他时长见 [当前 GIF](../gifs/daily/README.md)。触摸识别时限与暂停响应独立。编舞页的慢／标准／快是另外的屏幕预览参数，不修改日常风格倍率。



`build_screen_preview.py` 保留日常 GIF 的像素与时间轴，舞蹈采用 64 色、预计算遮罩、50 ms 采样和有限深度索引差分。十组动作使用共同中立首尾，底盘回转增加首尾连接但仍有稀疏姿态限制。字体生成需要 Noto Sans CJK。更多空间预算、播放阶段及机械臂过渡接口见 [DANCE_OPTIMIZATION.md](DANCE_OPTIMIZATION.md)。

工程已保存生成资源，普通构建不需要 Pillow、图片生成服务或本地字体文件。

## 测试与打包

```bash
cmake -S tests/host -B build/host
cmake --build build/host -j8
ctest --test-dir build/host --output-on-failure
python3 tools/render_previews.py build/host build/ui-preview
# 使用带 esptool 的 ESP-IDF Python 环境：
python3 tools/package_firmware.py --build build/firmware
```

打包输出 `dist/reBuddy-esp32c5/`。目录必须为空，也可指定新的 `--output`。包内 `flash.sh` 先校验 SHA256，再按分区地址写入引导、分区表和应用，保留 NVS/PHY 区间；可选 `merged.bin` 从 0x0 写入会覆盖间隙数据。工具只打包，不烧录、不推送。测试与实机验收说明见 [TESTING.md](TESTING.md)。
