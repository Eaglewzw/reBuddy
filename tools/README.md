# 工具入口

| 命令 | 用途与输出 |
| --- | --- |
| `python3 tools/render_approved_probe.py` | 重现已选定probe-v2；从工程内原图读取，输出到 `docs/preview/probe-v2/` |
| `python3 tools/render_approved_probe.py --output /tmp/probe-preview` | 输出到临时目录，便于检查而不覆盖保留版本 |
| `python3 tools/build_pet_assets.py` | 重建当前固件的旧74帧原型及参数/C帧表；不处理probe-v2 |
| `python3 tools/build_pet_assets.py --preview-dir build/asset-review` | 额外输出旧原型诊断PNG，不混入已选素材 |
| `python3 tools/render_previews.py build-host build/ui-preview` | 将主机LVGL测试PPM转换成旧UI诊断PNG |
| `python3 tools/package_firmware.py --build build` | 本地打包当前固件；明确标记仍为旧原型，不使用probe-v2预览冒充固件截图 |
| `python3 tools/generate_actions.py --check` | 检查仍由历史回归测试使用的旧关节配置与生成代码 |

图片工具需要Pillow；打包工具使用IDF环境中的esptool。默认不烧录、不发布。旧 `pack_character.py` 和临时参考图裁切工具已随无用素材清理。
