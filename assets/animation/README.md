# 当前固件仍依赖的旧原型资源

本目录的74帧技术原型尚未替换为用户选定的probe-v2。`main/CMakeLists.txt`嵌入 `frames.bin`，资源管理器与主机测试读取同一文件；`manifest.json`用于资源校验，`colorbars.bin`用于RGB565颜色验证。

保留这些文件以维持现有工程可构建。最新选定素材位于 `assets/approved/probe-v2/`，说明见 `docs/ASSET_STATUS.md`。

`tools/build_pet_assets.py`默认只生成运行资源、参数头和帧表；诊断PNG须通过 `--preview-dir build/asset-review` 显式输出。
