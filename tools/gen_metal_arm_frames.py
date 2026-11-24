#!/usr/bin/env python3
"""The Metal Arm's frame fixture: tools/gen_arm_frames.py --product metal.

    ~/metal-arm-sdk/.venv/bin/python tools/gen_metal_arm_frames.py [--no-gravity]
"""

import runpy
import sys
from pathlib import Path

sys.argv = [sys.argv[0], "--product", "metal", *sys.argv[1:]]
runpy.run_path(str(Path(__file__).resolve().parent / "gen_arm_frames.py"), run_name="__main__")
