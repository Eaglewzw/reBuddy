import importlib.util
import json
from pathlib import Path

root=Path(__file__).resolve().parents[2]
spec=importlib.util.spec_from_file_location('budget',root/'tools/check_flash_budget.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
budget=json.loads((root/'assets/config/flash_budget.json').read_text())
reserved=sum(budget[k] for k in ('future_motion_data_bytes','future_code_growth_bytes','unallocated_margin_bytes'))
limit=budget['application_partition_bytes']-reserved
assert module.check(limit,budget['animation_limit_bytes'],budget)['free_bytes']==reserved
for app,animation in ((limit+1,0),(0,budget['animation_limit_bytes']+1)):
    try:module.check(app,animation,budget)
    except ValueError:pass
    else:raise AssertionError('Oversized firmware accepted')
assert (root/'assets/animation/frames.bin').stat().st_size<=budget['animation_limit_bytes']
print('PASS animation budget and future motion/code reserve including exact boundaries')
