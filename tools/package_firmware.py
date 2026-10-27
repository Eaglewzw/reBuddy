#!/usr/bin/env python3
"""Package build artifacts locally; never flash a board or publish a release."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import shlex
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser()
parser.add_argument('--build', type=Path, default=ROOT/'build/firmware')
parser.add_argument('--output', type=Path, default=ROOT/'dist/reBuddy-esp32c5')
args = parser.parse_args()
build = args.build.resolve()
out = args.output.resolve()
flash = json.loads((build/'flasher_args.json').read_text())
description=json.loads((build/'project_description.json').read_text())
behavior=json.loads((ROOT/'assets/config/behavior_defaults.json').read_text())
assets=json.loads((ROOT/'assets/animation/manifest.json').read_text())
app=(build/flash['app']['file']).read_bytes()
if (ROOT/'assets/animation/frames.bin').read_bytes() not in app:
    raise SystemExit('Application does not contain current animation assets; rebuild before packaging.')
if assets.get('asset_version')!='gif-v2-rgb565':
    raise SystemExit('Generate and build gif-v2-rgb565 before packaging.')
if (ROOT/'assets/animation/room.bin').read_bytes() not in app:
    raise SystemExit('Application does not contain the current room background; rebuild first.')
if description['target']!='esp32c5':
    raise SystemExit('Expected an ESP32-C5 build.')
verification=json.loads((ROOT/'assets/verification/current.json').read_text())
if verification['application_sha256']!=hashlib.sha256(app).hexdigest():
    raise SystemExit('Verification summary does not match this application; update build verification before packaging.')
if out.exists() and any(out.iterdir()):
    raise SystemExit('Output directory is not empty. Choose a new --output to avoid stale package files.')
out.mkdir(parents=True, exist_ok=True)
files = []
for address, relative in flash['flash_files'].items():
    source = (build/relative).resolve()
    if not source.is_relative_to(build):
        raise SystemExit(f'Unexpected artifact outside build directory: {relative}')
    destination = out/relative
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, destination)
    files.extend([address, str(destination)])
command = [sys.executable, '-m', 'esptool', '--chip', 'esp32c5', 'merge_bin', '--output', str(out/'merged.bin')]
for key in ['flash_mode', 'flash_freq', 'flash_size']:
    command.extend(['--'+key, str(flash['flash_settings'][key])])
subprocess.run(command+files, check=True)
source=Path(description['config_file'])
shutil.copy2(source,out/'sdkconfig')
for relative in ['LICENSE', 'assets/docs/THIRD_PARTY_NOTICES.md',
                 'assets/config/behavior_defaults.json', 'assets/config/claw_touch_regions.json',
                 'assets/animation/manifest.json', 'assets/fonts/OFL.txt',
                 'assets/docs/GETTING_STARTED.md', 'assets/docs/HARDWARE.md',
                 'assets/docs/TESTING.md', 'assets/docs/ACCEPTANCE.md', 'assets/docs/DISPLAY_PERFORMANCE.md',
                 'assets/docs/CHOREOGRAPHY.md', 'assets/images/previews/choreography.png',
                 'assets/images/previews/current.png', 'assets/images/previews/claw-petting-guide.svg',
                 'assets/verification/current.json']:
    destination=out/relative
    destination.parent.mkdir(parents=True,exist_ok=True)
    shutil.copy2(ROOT/relative,destination)
manifest = {
    'project': 'reBuddy', 'version': description['project_version'], 'target': description['target'],
    'sdk': description['git_revision'], 'asset_version': assets['asset_version'],
    'interaction_revision': assets.get('interaction_revision'),
    'animation_playback': behavior['animation_playback'],
    'animation_compression': assets.get('compression'),
    'ui_revision': 'choreography-v1', 'mode': 'screen_touch_and_choreography_preview', 'hardware_tested': False,
    'choreography': {'slots': 10, 'max_steps': 8, 'schema_version': 1,
                    'available_actions': verification['choreography']['available_actions'],
                    'wait_block': True, 'motor_playback': False},
    'visual_status': assets['visual_status'],
    'animation_source': 'assets/gifs', 'animation_fps': 8,
    'animation_unique_frames': assets['unique_frames'],
    'animation_bytes': assets['bytes'], 'room_sha256': assets['room_sha256'],
    'input_mode': 'touch_only', 'physical_keys': False, 'gpio_estop_input': False,
    'board_component': 'rebuddy_board', 'private_dependencies': False,
    'flash_files': flash['flash_files'], 'flash_settings': flash['flash_settings']}
(out/'manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n')
flash_args = flash['write_flash_args'][:]
for address, relative in flash['flash_files'].items():
    flash_args.extend([address, relative])
(out/'flash_args').write_text(shlex.join(flash_args)+'\n')
sdk_version=description['git_revision'].lstrip('v')
script='''#!/usr/bin/env bash
set -euo pipefail
if [[ $# -ne 1 ]]; then
  echo "Usage: ./flash.sh /dev/ttyACM0" >&2
  exit 2
fi
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
sha256sum --check --status SHA256SUMS
flash_python="${FLASH_PYTHON:-python3}"
if ! "$flash_python" -m esptool version >/dev/null 2>&1; then
  flash_python="${HOME}/.espressif/tools/python/vSDK/venv/bin/python"
fi
if ! "$flash_python" -m esptool version >/dev/null 2>&1; then
  echo "esptool unavailable; set FLASH_PYTHON to your ESP-IDF Python interpreter." >&2
  exit 1
fi
exec "$flash_python" -m esptool --chip esp32c5 --port "$1" --baud 460800 --before default_reset --after hard_reset write_flash @flash_args
'''.replace('vSDK','v'+sdk_version)
(out/'flash.sh').write_text(script)
(out/'flash.sh').chmod(0o755)
preview=ROOT/'assets/images/previews/current.png'
if preview.exists(): shutil.copy2(preview,out/'ui-preview.png')
(out/'README.txt').write_text(
    f'reBuddy {description["project_version"]} / touch-only / ESP32-C5 / ESP-IDF {sdk_version}\n'
    f'当前新版 GIF 动画：{assets["unique_frames"]} 张去重帧，表情、问号和睡眠 Z 均来自当前 GIF。\n'
    f'平均 8 FPS，开心回应 8 秒；全局时长倍率 {behavior["animation_playback"]["duration_scale"]}，活泼与慵懒暂时均为 1.0 倍，触摸识别时限独立。\n'
    '直接触摸角色互动；左下暂停/继续，右下设置；没有英文状态栏。\n'
    '设置→编排：编辑、预览并保存作品，每段最多 8 步，保存 10 段；十六类动作及等待均可用。\n'
    '基础律动为 30 秒可编辑舞蹈，动作库包含 10 种基本舞蹈动作。\n'
    '编排仅使用当前屏幕动画；保存后断电保留，加载不自动播放。详见 assets/docs/CHOREOGRAPHY.md。\n'
    '硬件：XIAO ESP32-C5 CAN Console，8 MiB Flash / 8 MiB Quad PSRAM。\n'
    '已通过主机测试与编译，尚未实际烧录或完成实机 30 分钟验证。\n'
    '在本目录执行：./flash.sh /dev/ttyACM0（将串口替换成实际设备）。\n'
    '脚本先校验文件，再按 flash_args 写入引导、分区和应用，保留 NVS/PHY 区间。\n'
    'merged.bin 为可选整合镜像，从 0x0 写入会覆盖间隙内原有数据；优先用 flash.sh。\n'
    '轻点探头，慢慢抚摸轻蹭，连续三次轻点收拢，抚摸安抚，快划歪头。\n'
    '轻柔来回摸爪累计约 0.6 秒有效移动可触发开心；设置→休息可快速看入睡，持续循环睡眠，长按左下角恢复。\n'
    '左下暂停图标冻结画面，再点击同一位置的播放图标继续。不读取实体按键或 GPIO 急停输入。CAN 保持待机，无电机后端。\n'
    '房间装饰暂不响应触摸；部分动作中间帧仍待细化，详见 assets/docs/GETTING_STARTED.md。\n')
entries=[]
for file in sorted(out.rglob('*')):
    if file.is_file() and file.name!='SHA256SUMS':
        entries.append(f'{hashlib.sha256(file.read_bytes()).hexdigest()}  {file.relative_to(out)}')
(out/'SHA256SUMS').write_text('\n'.join(entries)+'\n')
print(out)
