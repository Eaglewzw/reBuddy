#!/usr/bin/env python3
"""Real joint-path math, planner, queue and consumer; no board access."""
from pathlib import Path
import os
import subprocess
import tempfile
from arm_vendor_runtime_support import write_headers

ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent
MOTION = ROOT / "components/can_console_arm_motion"
IK = ROOT / "components/can_console_ik"

if __name__ == "__main__":
    with tempfile.TemporaryDirectory() as directory:
        folder = Path(directory)
        write_headers(folder)
        output = folder / "joint_path"
        sanitizer = os.environ.get("SANITIZE") == "1"
        flags = ["-fsanitize=address,undefined", "-fno-omit-frame-pointer"] if sanitizer else []
        subprocess.run([os.environ.get("CC", "gcc"), "-std=gnu17", "-O1" if sanitizer else "-O2",
                        "-g", "-Wall", "-Wextra", "-Werror", "-Wno-sign-compare", "-pthread", *flags,
                        *[f"-I{p}" for p in (folder, MOTION, MOTION / "include", IK / "include")],
                        str(HERE / "arm_vendor_test_joint_path.c"), str(HERE / "arm_vendor_runtime_support.c"),
                        str(MOTION / "src/arm_joint_path.c"), str(MOTION / "src/arm_stop.c"), str(MOTION / "src/arm_trajectory.c"),
                        str(IK / "src/can_console_ik.c"), str(IK / "src/can_console_pose_math.c"),
                        "-Wl,--wrap=malloc,--wrap=calloc,--wrap=realloc",
                        "-Wl,--wrap=can_console_ik_solve,--wrap=can_console_ik_forward",
                        "-lm", "-o", str(output)], check=True)
        subprocess.run([str(output)], check=True, timeout=40)
