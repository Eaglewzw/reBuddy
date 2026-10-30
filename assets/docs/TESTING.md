# 测试与验收


```bash
cmake -S tests/host -B build/host
cmake --build build/host -j8
ctest --test-dir build/host --output-on-failure
python3 tools/render_previews.py build/host build/ui-preview
```

行为核心使用标准 C，不依赖 ESP-IDF。界面测试需要 `managed_components/lvgl__lvgl`，也可指定 `-DREBUDDY_LVGL_DIR=/path/to/lvgl-9.5.0`；没有 LVGL 时仅跳过 UI 测试。

| 测试 | 覆盖 |
| --- | --- |
| reference_behavior | 触摸去抖、轻点、抚摸、快划、收拢／安抚、手爪限定开心、冷却和中断、持续睡眠、停止恢复、30 分钟模拟 |
| ui_rgb565 | 真实 LVGL 指针事件、左右下角入口、菜单开合、手爪／身体／眼睛区别、睡眠呼吸和 Z、唤醒、输入取消、边界和稳定堆内存 |
| board_input | 全屏触摸映射、主触点 ID、丢失和故障取消 |
| behavior_config | 当前 JSON 与生成参数头一致 |
| animation_resources | 当前 GIF 解码到 RGB565 的逐帧对应、帧去重与校验、原始 120/130 ms 时间轴、动作映射、手爪触摸区域及房间资源 |
| project_assets | 当前 10 个日常 GIF、10 个选定舞蹈 GIF、三个独立表情 GIF 资源校验、必要源素材与文档链接完整 |
| gif_visual_regressions | 待机闭眼无残留、身体稳定、连杆空隙正确透出背景、GIF 总时长和帧延迟 |
| choreography_model | 稳定动作 ID、名称和参数边界、添加排序、拒绝日常动作、停留、节奏时间轴、停止与完成；动作时长与固件资源一致 |
| choreography_storage | 十槽双记录，保留旧三槽、拒绝缺失素材及停用动作编号、版本与 CRC、失败和中断写入、损坏回退、槽位 ID 校验、加载不写入 |
| asset_cleanup | 废弃舞蹈资源和生成器不存在，运行清单仅包含当前日常与选定舞蹈动画 |
| daily_suspend | 编辑期间冻结动作和入睡计时、清除旧手势、恢复睡眠与停止状态、急停优先 |
| choreography_ui | 真实 LVGL 触摸编辑、排序与容量、缺失素材条目与示例禁用、保存失败、停止与完成、故障、与日常模式隔离 |

带 LVGL 时共 19 项。UI 测试输出实际 PPM 截图，转换后的 `build/ui-preview/contact-sheet.png` 和 `choreography.png` 用于布局检查。图片生成和模拟测试不能代替实屏动画效果、触摸手感或硬件连续运行验收。

最新整理验证见 [current.json](../verification/current.json)，实机记录使用 [ACCEPTANCE.md](ACCEPTANCE.md)。代码仓库只保留当前验证摘要，构建和测试日志输出到被忽略的 `build/`。

舞蹈专项：十组动作均可通过触摸选择与保存，拒绝日常及旧舞蹈 ID。所有舞蹈帧验证小屋背景覆盖两侧、外部背景来自 room.bin，浅色边缘按覆盖率混合、深色轮廓与绿色面板不变，合成后原始帧缓存保持完整。

编排滚动回归：使用触摸按下、连续移动、释放事件，分别从右侧空白和卡片开始上下滑动作品列表、动作库、步骤列表；验证末项完整可见、可返回顶部，且滑动不触发卡片操作。该测试不调用程序滚动接口替代手势。

边缘去白验证：已知 25% 覆盖率的人工轮廓对照小屋背景检查混合颜色，验证内部白色高光与深色轮廓保持不变；十组舞蹈逐帧检查软边修改只发生在外部背景邻接处，原始解码缓存不变。

开场收尾与压缩：自动段不占步骤，校验不同节奏、停止、重播、收尾末帧及禁止手动保存；色表编码逐帧独立解码并比对转换像素，损坏色表／压缩流安全回退；舞蹈减色后的平均每通道误差上限为 4/255，日常仍按原像素验证。

舞蹈优化新增 `motion_bridge` 与 `flash_budget`。逐帧资源测试覆盖 64 色索引／覆盖率、最多七层差分、所有中立首尾组合；C 解码器独立核对全部 CRC，正序与倒序访问并检查循环引用和压缩流损坏。UI 测试覆盖半拍调整和旧作品兼容。详情见 [DANCE_OPTIMIZATION.md](DANCE_OPTIMIZATION.md)。历史段落中的 256 色与仅单层差分描述已由本轮舞蹈格式替代，日常差分仍为单层。

`dance_cavities` 覆盖 25 个含封闭空隙的时间轴采样，检查空隙透明、眼睛／关节高光不透明，并拒绝过期的种子坐标、面积或边界。


控制组件接入增加四项主机测试：`arm_component_snapshot` 校验固定组件源码；`arm_adapter` 使用实际适配层与上游公开头文件，覆盖默认待机、硬件只听模式、七路型号映射、重复初始化/部分失败隔离、反馈时效及总线异常；`arm_vendor_joint_path` 与 `arm_vendor_teach` 在本仓库源码上运行参考工程的轨迹与七通道示教回归。主机替身不访问硬件。固件还需分别验证默认待机和 `CONFIG_REBUDDY_ARM_MONITOR=y` 配置的构建；监听与真实电机控制仍需上板验收。
