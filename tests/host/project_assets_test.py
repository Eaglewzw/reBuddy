"""Check current GIFs, approved sources, and local Markdown links (stdlib only)."""
import hashlib
import json
from pathlib import Path
import re
import struct
from urllib.parse import unquote

root = Path(__file__).resolve().parents[2]
gif_dir = root/'assets/gifs'
manifest = json.loads((gif_dir/'manifest.json').read_text())
assert len(manifest['gifs']) == 10
assert manifest['behavior_config_sha256'] == hashlib.sha256((root/'config/behavior_defaults.json').read_bytes()).hexdigest()
assert manifest['source_runtime_sha256'] == hashlib.sha256((root/'assets/animation/frames.bin').read_bytes()).hexdigest()
assert manifest['room_sha256'] == hashlib.sha256((root/'assets/animation/room.bin').read_bytes()).hexdigest()
assert {p.name for p in gif_dir.glob('*.gif')} == {g['file'] for g in manifest['gifs']}
for item in manifest['gifs']:
    data = (gif_dir/item['file']).read_bytes()
    assert hashlib.sha256(data).hexdigest() == item['sha256'], item['file']
    assert data[:6] in (b'GIF87a', b'GIF89a')
    assert struct.unpack_from('<HH', data, 6) == (320, 240)
approved = json.loads((root/'assets/approved/probe-v2/manifest.json').read_text())
for name in ('source', 'idle_master'):
    assert hashlib.sha256((root/approved[name]).read_bytes()).hexdigest() == approved[name+'_sha256']
for export in approved['exports']:
    assert hashlib.sha256((root/export['path']).read_bytes()).hexdigest() == export['sha256']
markdown = [root/'README.md', root/'THIRD_PARTY_NOTICES.md']
for folder in ('docs', 'assets', 'tools'):
    markdown.extend((root/folder).rglob('*.md'))
for doc in markdown:
    for link in re.findall(r'!?\[[^\]]*\]\(([^)]+)\)', doc.read_text()):
        target = link.split('#')[0].strip('<>')
        if not target or re.match(r'[a-z]+:', target):
            continue
        assert (doc.parent/unquote(target)).exists(), f'{doc.relative_to(root)}: {target}'
print('PASS ten current GIF hashes/headers, approved artwork, and local Markdown links')
