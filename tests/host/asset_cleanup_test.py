"""Removed artwork must not survive in export inputs or firmware resources."""
import json
from pathlib import Path

root = Path(__file__).resolve().parents[2]
for path in (
    'assets/gifs/dance-preview', 'assets/gifs/dance-review',
    'assets/gifs/cat-v6', 'assets/images/source/dance',
    'assets/images/source/cat-v6', 'assets/images/source/shared',
    'tools/export_dance_gifs.py', 'tools/export_whole_dance_preview.py',
    'tools/export_claw_sway_preview.py', 'tools/export_bounce_review.py',
    'tools/dance_motion.py',
    'tools/render_approved_probe.py',
    'assets/images/reference/approved/probe-v2',
    'assets/images/source/expressions/keyframes.png',
    'assets/docs/WEB_ANIMATION_ASSET_BRIEF.md',
):
    assert not (root / path).exists(), path
runtime = json.loads((root / 'assets/animation/manifest.json').read_text())
assert len(runtime['source_gifs']) == 22
assert all(path.startswith(('assets/gifs/daily/', 'assets/gifs/dance/')) for path in runtime['source_gifs'])
assert runtime['dance_source'] == 'assets/gifs/dance/manifest.json'
assert all(runtime['choreography_clip_map'][i] == 255 for i in range(23, 33))
print('PASS removed dance artwork/generators absent; only daily and selected dance GIFs enter runtime')

assert not list((root/'assets').rglob('*prompt*.txt')), 'unused generation prompts returned'
for name in ('middle.png', 'moderate.png', 'strong-peak.png', 'pose-analysis.json', 'timing-suggestion.json'):
    assert not (root/'assets/images/source/daily/tilt'/name).exists()
