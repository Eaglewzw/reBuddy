#!/usr/bin/env python3
"""Compile the real allocation-free recorder, without IDF or hardware."""
from pathlib import Path
import os
import subprocess
import tempfile
from arm_vendor_runtime_support import write_headers

ROOT = Path(__file__).resolve().parents[2]
if __name__ == "__main__":
    with tempfile.TemporaryDirectory() as directory:
        folder = Path(directory)
        write_headers(folder)
        output = folder / "teach"
        subprocess.run([os.environ.get("CC", "gcc"), "-std=gnu17", "-O2", "-Wall", "-Wextra", "-Werror",
                        f"-I{folder}", f"-I{ROOT / 'components/can_console_arm_motion/include'}",
                        str(ROOT / "components/can_console_arm_motion/src/arm_teach.c"),
                        str(Path(__file__).with_suffix(".c")), "-lm", "-o", str(output)], check=True)
        subprocess.run([str(output)], check=True)
