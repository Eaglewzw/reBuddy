"""Check current GIFs, approved sources, and local Markdown links (stdlib only)."""
import hashlib
import json
from pathlib import Path
import re
import struct
from urllib.parse import unquote
import xml.etree.ElementTree as ET

root = Path(__file__).resolve().parents[2]
gif_dir = root/'assets/gifs/daily'
manifest = json.loads((gif_dir/'manifest.json').read_text())
assert len(manifest['clips']) == 10
for obsolete in ('gifs-v2','gifs-preview','design','approved','reference','previews'):
    assert not (root/'assets'/obsolete).exists(), f'obsolete asset directory: {obsolete}'
for path in (root/'assets').rglob('*'):
    if path.suffix.lower() == '.gif':
        assert path.is_relative_to(root/'assets/gifs'), path
    elif path.suffix.lower() in ('.png','.jpg','.jpeg','.webp','.svg'):
        assert path.is_relative_to(root/'assets/images'), path
assert {p.name for p in gif_dir.glob('*.gif')} == {g['file'] for g in manifest['clips']}
for folder, report in ((gif_dir, manifest), (root/'assets/gifs/expressions',
        json.loads((root/'assets/gifs/expressions/manifest.json').read_text())),
        (root/'assets/gifs/dance', json.loads((root/'assets/gifs/dance/manifest.json').read_text()))):
    for item in report['clips']:
        data = (folder/item['file']).read_bytes()
        assert hashlib.sha256(data).hexdigest() == item['sha256'], item['file']
        assert data[:6] in (b'GIF87a', b'GIF89a')
        assert struct.unpack_from('<HH', data, 6) == (320, 240)
dance = json.loads((root/'assets/gifs/dance/manifest.json').read_text())
assert len(dance['clips']) == 10
assert {clip['action_id'] for clip in dance['clips']} == set(range(23, 33))
for source in manifest['sources'].values():
    assert hashlib.sha256((root/source['path']).read_bytes()).hexdigest() == source['sha256']
selected = root/'assets/gifs/cat-v6'
v6 = json.loads((selected/'manifest.json').read_text())
assert v6['status'] in ('user_selected_keep', 'solid_jaw_correction_pending_review')
assert hashlib.sha256((selected/v6['file']).read_bytes()).hexdigest() == v6['sha256']
for source, digest in v6['sources'].items():
    assert hashlib.sha256((root/source).read_bytes()).hexdigest() == digest, source
markdown = [root/'README.md', *sorted((root/'assets').rglob('*.md'))]
for doc in markdown:
    for link in re.findall(r'!?\[[^\]]*\]\(([^)]+)\)', doc.read_text()):
        target = link.split('#')[0].strip('<>')
        if not target or re.match(r'[a-z]+:', target):
            continue
        assert (doc.parent/unquote(target)).exists(), f'{doc.relative_to(root)}: {target}'
print('PASS ten current GIFs, three expression GIFs, ten basic dance GIFs, current source artwork and local asset links')

for svg in (root/'assets').rglob('*.svg'):
    for element in ET.parse(svg).iter():
        href=element.get('href') or element.get('{http://www.w3.org/1999/xlink}href')
        if href and not href.startswith(('#','data:','http:','https:')):
            assert (svg.parent/href).is_file(), f'{svg}: missing linked image {href}'
