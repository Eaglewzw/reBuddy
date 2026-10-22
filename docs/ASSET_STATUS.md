# 当前素材状态与目录约定

## 已选择保留：probe-v2

用户反馈“这个版本比之前的好，保留这个版本”。这里记录的是两姿态视觉基线的选择，不代表连续动画或固件整体验收通过。

| 文件 | 用途 |
| --- | --- |
| `assets/approved/probe-v2/source.png` | 从用户提供的 `1.png` 原样复制；左为待机、右为探头到位 |
| `assets/approved/probe-v2/manifest.json` | 选定状态、原图SHA256、导出文件SHA256和两帧700ms节奏 |
| `assets/approved/idle_master.png` | 网页生成时使用的原始待机基准，保留来源链 |
| `docs/preview/probe-v2/idle.png`、`probe.png` | 从选定原图拆出并对齐的姿态，供后续补帧使用 |
| `docs/preview/probe-v2/comparison.png` | 待机和探头的静态对照 |
| `docs/preview/probe-v2/probe-v2-poses.gif` | 选定的600×600往返预览 |
| `docs/preview/probe-v2/probe-v2-poses-160.gif` | 同一预览的160×160版本 |

原图SHA256：`611660c885e33fdf36f547c2e67c65a034bc6a5d58ecd36f023c53e47dff56ad`。下载目录中的 `1.png` 和 `2.png` 字节相同，工程只导入一份。两个GIF在整理前后保持字节一致。

当前只有两张关键姿态，无中间帧、无图像拉伸补间、无固件集成。重现工具为 `tools/render_approved_probe.py`，只做背景提取、统一落点与输出；不会改动原图或旧固件帧。

## 三类素材各自保存

- `assets/reference/`：用户原始角色图和六格互动图；用于外观和动作含义约束。
- `assets/approved/` 与 `docs/preview/probe-v2/`：当前选定素材与预览；后续优先在此基线上补帧。
- `assets/animation/`：旧技术原型，当前CMake/UI/资源测试仍需要它。不能把它当作用户选定的probe-v2，也不能直接删除而破坏构建。

`tools/build_pet_assets.py` 只重建旧技术原型；它不会替换选定素材。若要查看其诊断图，显式使用 `--preview-dir build/asset-review`。正式UI截图使用 `tools/render_previews.py build-host build/ui-preview`。临时/可再生预览放到构建目录，不再散放到文档预览根目录。

## 本轮清理范围

删除了被替代的旧探头12帧GIF及拆帧、参考图裁切的多尺寸临时副本、旧UI预览、未链接的旧生成角色及对应C资源/转换工具、旧动作草案文档、过期的验证日志和旧分发包。目录级清单与数量见 [整理记录](verification/cleanup-summary.json)。

保留当前源码、回归测试、参数、板级配置、分区、依赖及构建缓存；原始下载目录不在本次清理范围。旧行为/机械臂接口代码仍有测试引用，未以“当前暂不开发机械臂”为由删除。

下一步从已选择的两个姿态补充连贯的关节过渡，确认视觉效果后再替换运行时资源。
