# 应用代码

| 目录 | 职责 | 文件 |
| --- | --- | --- |
| `app/` | 应用启动、任务调度、屏幕与机械臂协调 | `main.c`、`app_screen.*`、`app_arm.*`、`app_diagnostics.*` |
| `ui/` | 小屋交互与编舞页面 | `pet_ui.*`、`choreo_ui.*` |
| `storage/` | 编舞作品的 NVS 保存、恢复和校验 | `choreo_store.*` |
| `animation/` | 动画帧解码、缓存与背景合成 | `pet_assets.*` |
| `generated/` | 提交到仓库的生成代码 | `pet_animation_table.*`、`room_font.c` |

`CMakeLists.txt`、`Kconfig.projbuild` 和 `idf_component.yml` 保留在组件根目录，供 ESP-IDF 发现。头文件与对应实现放在一起，构建显式提供各模块的 include 路径。

行为、轨迹、硬件与电机后端继续在 `components/`，不要把硬件协议放进 UI。通用逻辑优先放入独立组件，由 `app/main.c` 协调。

`app/main.c` 仅保留板级初始化、机械臂初始化、屏幕启动和周期健康检查。`app_screen.c` 负责宠物状态初始化、资源加载与 LVGL 定时刷新；`app_arm.c` 负责 Daily 事件转发、屏幕同步和 ENTER 输入任务；`app_diagnostics.c` 汇总启动、动画性能和机械臂诊断日志。各模块共用入口持有的同一个宠物状态，初始化顺序保持不变。

生成代码必须保留，普通固件构建直接编译它们：

- `tools/build_screen_preview.py` 写入 `generated/pet_animation_table.c/.h`。
- `tools/build_room_font.py` 递归读取应用文本并写入 `generated/room_font.c`。

运行生成器时使用工程根目录；不要手改生成表，也不要将该目录当作编译缓存删除。历史验证 JSON 中的旧路径表示当时的源码位置，保留用于追溯。
