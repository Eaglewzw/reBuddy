"""Exercise board serial parser and Daily/Dance UI ownership without hardware."""
from pathlib import Path
import subprocess,tempfile
from arm_vendor_runtime_support import write_headers
ROOT=Path(__file__).resolve().parents[2]
with tempfile.TemporaryDirectory() as tmp:
    folder=Path(tmp);write_headers(folder)
    (folder/'sdkconfig.h').write_text('#define CONFIG_REBUDDY_ARM_DAILY_CONTROL 1\n#define CONFIG_REBUDDY_ARM_DANCE_CONTROL 1\n')
    (folder/'esp_random.h').write_text('#include <stdint.h>\nuint32_t esp_random(void);\n')
    includes=[folder,ROOT/'main/app',ROOT/'main/ui']+[ROOT/'components'/n/'include' for n in
        ('rebuddy_core','rebuddy_arm','rebuddy_board','can_console_arm_motion','can_console_motor','can_console_ik')]
    src=ROOT/'components/rebuddy_core/src'
    subprocess.run(['gcc','-std=c11','-D_POSIX_C_SOURCE=200809L','-Wall','-Wextra','-Werror',
                    *[f'-I{p}' for p in includes],str(ROOT/'tests/host/dance_app_test.c'),
                    *[str(src/n) for n in ('rb_pet.c','rb_choreo.c','rb_daily.c','rb_daily_data.c','rb_dance.c','rb_dance_data.c')],
                    str(ROOT/'components/rebuddy_board/src/rb_board_input.c'),'-lm','-o',str(folder/'app')],check=True)
    subprocess.run([str(folder/'app')],check=True)
