#!/usr/bin/env python3
"""Generate the active behavior header without regenerating images."""
import argparse
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--check', action='store_true')
args = parser.parse_args()
cfg = json.loads((ROOT/'assets/config/behavior_defaults.json').read_text())
lines = ['/* Generated from assets/config/behavior_defaults.json. */', '#pragma once']
for name, value in cfg['gesture'].items():
    lines.append(f'#define RB_CFG_{name.upper()} {value}u')
for name, value in cfg['action_durations_ms'].items():
    lines.append(f'#define RB_MS_{name} {value}u')
for name, value in cfg['style_duration_scale'].items():
    lines.append(f'#define RB_STYLE_{name.upper()} {value}f')
lines.append(f"#define RB_PLAYBACK_DURATION_SCALE {cfg['animation_playback']['duration_scale']}f")
for name, value in cfg['display']['role_rect'].items():
    lines.append(f'#define RB_ROLE_{name.upper()} {value}')
lines.append(f'#define RB_BODY_FPS {cfg["display"]["target_animation_fps"]}u')
output = ROOT/'components/rebuddy_core/include/rb_pet_config.h'
content = '\n'.join(lines)+'\n'
if args.check:
    if output.read_text() != content:
        raise SystemExit('Behavior header is stale: run python3 tools/generate_pet_config.py')
else:
    output.write_text(content)
