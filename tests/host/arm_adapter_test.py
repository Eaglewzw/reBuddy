"""Compile the real adapter against public vendor headers; isolate each boot."""
from pathlib import Path
import os
import subprocess
import tempfile
from arm_vendor_runtime_support import write_headers

ROOT = Path(__file__).resolve().parents[2]
with tempfile.TemporaryDirectory() as tmp:
    folder = Path(tmp)
    write_headers(folder)
    executable = folder/'arm_adapter'
    includes = [folder, ROOT/'components/rebuddy_board/include']
    includes += [ROOT/'components'/name/'include' for name in (
        'rebuddy_arm', 'can_console_bsp', 'can_console_can', 'can_console_motor',
        'can_console_arm_motion', 'can_console_ik')]
    subprocess.run([os.environ.get('CC', 'gcc'), '-std=c11', '-Wall', '-Wextra', '-Werror',
        '-UNDEBUG', *[f'-I{p}' for p in includes],
        str(ROOT/'components/rebuddy_arm/src/rb_arm.c'),
        str(ROOT/'tests/host/arm_adapter_test.c'), '-lm', '-o', str(executable)], check=True)
    for scenario in ['standby', 'monitor', 'feedback', 'fail-model', 'fail-pins', *[f'fail-{i}' for i in range(1, 16)]]:
        subprocess.run([str(executable), scenario], check=True)
print('PASS adapter boot, seven-channel feedback, freshness, bus faults and every init failure')
