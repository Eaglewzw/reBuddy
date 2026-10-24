"""Check current GIFs, approved sources, and local Markdown links (stdlib only)."""
import hashlib
import json
from pathlib import Path
import re
import struct
from urllib.parse import unquote
import xml.etree.ElementTree as ET

root = Path(__file__).resolve().parents[2]
gif_dir = root/'assets/gifs-v2'
manifest = json.loads((gif_dir/'manifest.json').read_text())
assert len(manifest['clips']) == 10
assert not (root/'assets/gifs').exists(), 'obsolete GIF directory is still shipped'
assert {p.name for p in gif_dir.glob('*.gif')} == {g['file'] for g in manifest['clips']}
for folder, report in ((gif_dir, manifest), (gif_dir/'expressions-preview',
        json.loads((gif_dir/'expressions-preview/manifest.json').read_text()))):
    for item in report['clips']:
        data = (folder/item['file']).read_bytes()
        assert hashlib.sha256(data).hexdigest() == item['sha256'], item['file']
        assert data[:6] in (b'GIF87a', b'GIF89a')
        assert struct.unpack_from('<HH', data, 6) == (320, 240)
for source in manifest['sources'].values():
    assert hashlib.sha256((root/source['path']).read_bytes()).hexdigest() == source['sha256']
markdown = [root/'README.md', *sorted((root/'assets').rglob('*.md'))]
for doc in markdown:
    for link in re.findall(r'!?\[[^\]]*\]\(([^)]+)\)', doc.read_text()):
        target = link.split('#')[0].strip('<>')
        if not target or re.match(r'[a-z]+:', target):
            continue
        assert (doc.parent/unquote(target)).exists(), f'{doc.relative_to(root)}: {target}'
print('PASS ten current GIFs, three expression GIFs, current source artwork and local asset links')

for svg in (root/'assets').rglob('*.svg'):
    for element in ET.parse(svg).iter():
        href=element.get('href') or element.get('{http://www.w3.org/1999/xlink}href')
        if href and not href.startswith(('#','data:','http:','https:')):
            assert (svg.parent/href).is_file(), f'{svg}: missing linked image {href}'
