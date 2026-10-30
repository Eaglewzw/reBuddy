#!/usr/bin/env python3
"""Fail builds before animation/code growth consumes the future motion reserve."""
import argparse
import json
import csv
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def check(app_bytes, animation_bytes, budget):
    reserve = sum(budget[k] for k in ('future_motion_data_bytes', 'future_code_growth_bytes', 'unallocated_margin_bytes'))
    limit = budget['application_partition_bytes'] - reserve
    if animation_bytes > budget['animation_limit_bytes']:
        raise ValueError(f'Animation uses {animation_bytes} bytes; limit is {budget["animation_limit_bytes"]}')
    if app_bytes > limit:
        raise ValueError(f'Application uses {app_bytes} bytes; limit {limit} preserves {reserve} bytes for motion/code/margin')
    return dict(application_bytes=app_bytes, application_limit_bytes=limit, reserved_bytes=reserve,
                free_bytes=budget['application_partition_bytes']-app_bytes,
                unreserved_headroom_bytes=limit-app_bytes, animation_bytes=animation_bytes)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--app', type=Path, required=True)
    args = parser.parse_args()
    budget = json.loads((ROOT/'assets/config/flash_budget.json').read_text())
    rows=csv.reader(line for line in (ROOT/'partitions.csv').read_text().splitlines()
                    if line.strip() and not line.lstrip().startswith('#'))
    factory=next((row for row in rows if row[0].strip()=='factory'),None)
    if not factory or int(factory[4].strip(),0)!=budget['application_partition_bytes']:
        raise SystemExit('Flash budget no longer matches the factory partition; review the reserve before building.')
    try:
        print(json.dumps(check(args.app.stat().st_size, (ROOT/'assets/animation/frames.bin').stat().st_size, budget), indent=2))
    except ValueError as error:
        raise SystemExit(str(error))
