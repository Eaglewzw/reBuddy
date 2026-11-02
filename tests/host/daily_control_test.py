"""Real daily executor and gravity model with deterministic motor-service faults."""
from pathlib import Path
import os, subprocess, tempfile
from arm_vendor_runtime_support import write_headers
ROOT=Path(__file__).resolve().parents[2]
COMP=ROOT/'components'
SOURCES=['rebuddy_arm/src/rb_daily_control.c','rebuddy_core/src/rb_daily.c','rebuddy_core/src/rb_daily_data.c',
         'rebuddy_arm/src/rb_daily_screen.c','rebuddy_core/src/rb_pet.c',
         'can_console_arm_motion/src/arm_gravity.c','can_console_arm_motion/src/arm_gravity_model.c',
         'can_console_ik/src/can_console_ik.c','can_console_ik/src/can_console_pose_math.c']
with tempfile.TemporaryDirectory() as tmp:
    folder=Path(tmp);write_headers(folder)
    binary=folder/'control'
    includes=[folder]+[COMP/name/'include' for name in ['rebuddy_arm','rebuddy_core','can_console_motor','can_console_arm_motion','can_console_ik']]
    subprocess.run([os.environ.get('CC','gcc'),'-std=c11','-Wall','-Wextra','-Werror','-Wno-sign-compare','-UNDEBUG',
                    *[f'-I{x}' for x in includes],*[str(COMP/x) for x in SOURCES],
                    str(ROOT/'tests/host/daily_control_test.c'),'-lm','-o',str(binary)],check=True)
    for scenario in ['screen_lifecycle','boot_wake','stale_start','already_enabled','all_actions','tx_stall','tx_failed','stale_running','bus_off',
                     'fault','nan','ui_loss','budget','rest_arrival_tolerance','rest_arrival_timeout','temperature','disable_unconfirmed',
                     'home_budget','following_recovery','recovery_stale','recovery_timeout','home_arrival_timeout',
                     'feedback_frame_spacing','hold_ramp_delivery','home_request_idempotent']:
        subprocess.run([str(binary),scenario],check=True)
print('PASS native executor lifecycle, shared seven-axis delivery, measured arrival, fault/stop paths')
