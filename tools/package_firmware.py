#!/usr/bin/env python3
"""Package build artifacts locally; never flash a board or publish a release."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser()
parser.add_argument('--build', type=Path, default=ROOT/'build-c5')
parser.add_argument('--output', type=Path, default=ROOT/'dist/reBuddy-v0.1-preview')
args = parser.parse_args()
build = args.build.resolve()
out = args.output.resolve()
flash = json.loads((build/'flasher_args.json').read_text())
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
command = [sys.executable, '-m', 'esptool', '--chip', 'esp32c5', 'merge-bin', '--output', str(out/'merged.bin')]
for key in ['flash_mode', 'flash_freq', 'flash_size']:
    command.extend(['--'+key.replace('_','-'), str(flash['flash_settings'][key])])
subprocess.run(command+files, check=True)
for name in ['sdkconfig', 'project_description.json']:
    if name == 'sdkconfig':
        source = build/name
        if source.exists(): shutil.copy2(source,out/name)
shutil.copy2(ROOT/'config/actions.json',out/'actions.json')
shutil.copy2(ROOT/'docs/preview/contact-sheet.png',out/'screen-preview.png')
manifest = {
    'project': 'reBuddy', 'version': '0.1.0', 'target': 'esp32c5', 'sdk': '5.5.5',
    'mode': 'preview_only_no_motor_commands', 'hardware_tested': False,
    'reference_commit': 'ba288ae49c80f19b10d14dd9d428f63e3dd41486',
    'flash_files': flash['flash_files'], 'flash_settings': flash['flash_settings']}
(out/'manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n')
(out/'README.txt').write_text(
    'reBuddy 0.1.0 / ESP32-C5 / ESP-IDF 5.5.5\n'
    'Screen preview only. No RS motor enable or movement commands. Not hardware-tested.\n'
    'Target board: XIAO ESP32-C5 CAN Console, 8 MiB Flash, 8 MiB Quad PSRAM.\n'
    'Flash with the correct serial port using esptool:\n'
    'python -m esptool --chip esp32c5 --port /dev/ttyACM0 write-flash 0x0 merged.bin\n'
    'This changes the board firmware. Use only after checking the target hardware.\n')
entries=[]
for file in sorted(out.rglob('*')):
    if file.is_file() and file.name!='SHA256SUMS':
        entries.append(f'{hashlib.sha256(file.read_bytes()).hexdigest()}  {file.relative_to(out)}')
(out/'SHA256SUMS').write_text('\n'.join(entries)+'\n')
print(out)
