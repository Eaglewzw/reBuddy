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
parser.add_argument('--build', type=Path, default=ROOT/'build')
parser.add_argument('--output', type=Path, default=ROOT/'dist/reBuddy-esp32c5')
args = parser.parse_args()
build = args.build.resolve()
out = args.output.resolve()
flash = json.loads((build/'flasher_args.json').read_text())
description=json.loads((build/'project_description.json').read_text())
behavior=json.loads((ROOT/'config/behavior_defaults.json').read_text())
assets=json.loads((ROOT/'assets/animation/manifest.json').read_text())
app=(build/flash['app']['file']).read_bytes()
if (ROOT/'assets/animation/frames.bin').read_bytes() not in app:
    raise SystemExit('Application does not contain current animation assets; rebuild before packaging.')
if assets.get('asset_version')!='room-preview-v1':
    raise SystemExit('Generate and build room-preview-v1 before packaging.')
if (ROOT/'assets/animation/room.bin').read_bytes() not in app:
    raise SystemExit('Application does not contain the current room background; rebuild first.')
if description['target']!='esp32c5':
    raise SystemExit('Expected an ESP32-C5 build.')
verification=json.loads((ROOT/'docs/verification/current.json').read_text())
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
for relative in ['LICENSE', 'THIRD_PARTY_NOTICES.md',
                 'config/behavior_defaults.json', 'config/design/head_touch_regions.json',
                 'assets/animation/manifest.json', 'assets/fonts/OFL.txt',
                 'docs/GETTING_STARTED.md', 'docs/HARDWARE.md',
                 'docs/TESTING.md', 'docs/ACCEPTANCE.md', 'docs/DISPLAY_PERFORMANCE.md',
                 'docs/preview/current.png', 'docs/preview/head-petting-guide.png',
                 'docs/verification/current.json']:
    destination=out/relative
    destination.parent.mkdir(parents=True,exist_ok=True)
    shutil.copy2(ROOT/relative,destination)
manifest = {
    'project': 'reBuddy', 'version': description['project_version'], 'target': description['target'],
    'sdk': description['git_revision'], 'asset_version': assets['asset_version'],
    'animation_playback': behavior['animation_playback'],
    'ui_revision': 'touch-only', 'mode': 'screen_touch_key_pose_preview', 'hardware_tested': False,
    'visual_status': assets['visual_status'], 'approved_probe_v2_integrated': False,
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
preview=ROOT/'docs/preview/current.png'
if preview.exists(): shutil.copy2(preview,out/'ui-preview.png')
(out/'README.txt').write_text(
    f'reBuddy {description["project_version"]} / touch-only / ESP32-C5 / ESP-IDF {sdk_version}\n'
    '小屋预览：窗户、木地板、地毯与充电角；28 张姿态、睡眠呼吸和动态 Z。\n'
    f'动画时长倍率：{behavior["animation_playback"]["duration_scale"]}；GIF 默认匹配活泼风格，触摸识别时限独立。\n'
    '直接触摸角色互动；左下暂停/继续，右下设置；没有英文状态栏。\n'
    '硬件：XIAO ESP32-C5 CAN Console，8 MiB Flash / 8 MiB Quad PSRAM。\n'
    '已通过主机测试与编译，尚未实际烧录或完成实机 30 分钟验证。\n'
    '在本目录执行：./flash.sh /dev/ttyACM0（将串口替换成实际设备）。\n'
    '脚本先校验文件，再按 flash_args 写入引导、分区和应用，保留 NVS/PHY 区间。\n'
    'merged.bin 为可选整合镜像，从 0x0 写入会覆盖间隙内原有数据；优先用 flash.sh。\n'
    '轻点探头，慢慢抚摸轻蹭，连续三次轻点收拢，抚摸安抚，快划歪头。\n'
    '持续有效抚摸头顶约 3 秒开心；设置→休息可快速看入睡，持续循环睡眠，长按左下角恢复。\n'
    '左下暂停图标冻结画面，再点击同一位置的播放图标继续。不读取实体按键或 GPIO 急停输入。CAN 保持待机，无电机后端。\n'
    '房间装饰暂不响应触摸；部分动作中间帧仍待细化，详见 docs/GETTING_STARTED.md。\n')
entries=[]
for file in sorted(out.rglob('*')):
    if file.is_file() and file.name!='SHA256SUMS':
        entries.append(f'{hashlib.sha256(file.read_bytes()).hexdigest()}  {file.relative_to(out)}')
(out/'SHA256SUMS').write_text('\n'.join(entries)+'\n')
print(out)
