"""Compare reused native RS statics with the calibrated PC Pinocchio model."""
from pathlib import Path
import ctypes, os, subprocess, sys, tempfile
import numpy as np
from arm_vendor_runtime_support import write_headers
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT/'tools'))
from rs_daily_prepare import load_bundle
from rs_gravity_record import DEFAULT_CALIBRATION,DEFAULT_URDF
manifest,plans,gravity,identity=load_bundle(DEFAULT_CALIBRATION,DEFAULT_URDF)
with tempfile.TemporaryDirectory() as temp:
    path=Path(temp);write_headers(path)
    wrapper=path/'wrap.c'
    wrapper.write_text('''#include "can_console_arm_gravity.h"
#include <stdint.h>
int64_t esp_timer_get_time(void){return 0;}
int daily_gravity(const float q[6],float t[6]) {
    can_console_arm_gravity_t g;can_console_arm_gravity_config_t c;
    can_console_arm_gravity_config_set_defaults(&c);c.model=CAN_CONSOLE_IK_MODEL_REBOTARM_RS;
    return can_console_arm_gravity_init(&g,&c) || can_console_arm_gravity_compute(&g,q,t);
}
''')
    comp=ROOT/'components'
    sources=['can_console_arm_motion/src/arm_gravity.c','can_console_arm_motion/src/arm_gravity_model.c',
             'can_console_ik/src/can_console_ik.c','can_console_ik/src/can_console_pose_math.c']
    libpath=path/'gravity.so'
    subprocess.run([os.environ.get('CC','gcc'),'-shared','-fPIC','-O2',f'-I{path}',
                    f'-I{comp}/can_console_arm_motion/include',f'-I{comp}/can_console_ik/include',
                    *[str(comp/p) for p in sources],str(wrapper),'-lm','-o',str(libpath)],check=True)
    lib=ctypes.CDLL(str(libpath));ptr=ctypes.POINTER(ctypes.c_float)
    lib.daily_gravity.argtypes=[ptr,ptr];lib.daily_gravity.restype=ctypes.c_int
    points=[np.zeros(6),np.array(manifest['idle_rad'][:6])]
    for plan in plans.values():points.extend(plan.at(t)[0][:6] for t in np.linspace(0,plan.duration,100))
    rng=np.random.default_rng(20);points.extend(rng.uniform(gravity.lower,gravity.upper,size=(1000,6)))
    peak=0.
    for q in points:
        actual=(ctypes.c_float*6)();q=np.array(q,dtype=np.float32)
        assert lib.daily_gravity((ctypes.c_float*6)(*q),actual)==0
        peak=max(peak,float(np.max(abs(np.array(actual)-gravity(q)))))
    assert peak<5e-4,peak
    print(f'PASS native/PC gravity max error {peak:.8f} Nm over {len(points)} configurations')
