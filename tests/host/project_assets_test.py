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
assert len(manifest['clips']) == 8
for obsolete in ('gifs-v2','gifs-preview','design','approved','reference','previews'):
    assert not (root/'assets'/obsolete).exists(), f'obsolete asset directory: {obsolete}'
for path in (root/'assets').rglob('*'):
    if path.suffix.lower() == '.gif':
        if path == root/'assets/images/source/dance/github_turn_reference.gif':
            # Archived drawing reference, never a runtime clip.
            reference=json.loads(path.with_suffix('.json').read_text())
            assert hashlib.sha256(path.read_bytes()).hexdigest()==reference['sha256']
        else:
            assert path.is_relative_to(root/'assets/gifs'), path
    elif path.suffix.lower() in ('.png','.jpg','.jpeg','.webp','.svg'):
        assert path.is_relative_to(root/'assets/images') or (path.suffix.lower()=='.png' and path.is_relative_to(root/'assets/gifs/dance/rgba')), path
assert {p.name for p in gif_dir.glob('*.gif')} == {g['file'] for g in manifest['clips']}
for folder, report in ((gif_dir, manifest), (root/'assets/gifs/expressions',
        json.loads((root/'assets/gifs/expressions/manifest.json').read_text()))):
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
print('PASS eight current GIFs, three expression GIFs, current source artwork and local asset links')

for svg in (root/'assets').rglob('*.svg'):
    for element in ET.parse(svg).iter():
        href=element.get('href') or element.get('{http://www.w3.org/1999/xlink}href')
        if href and not href.startswith(('#','data:','http:','https:')):
            assert (svg.parent/href).is_file(), f'{svg}: missing linked image {href}'

selected = root/'assets/gifs/dance'
report = json.loads((selected/'manifest.json').read_text())
assert len(report['clips']) == 12
assert {p.name for p in selected.glob('*.gif')} == {c['file'] for c in report['clips']}
assert [c['action_id'] for c in report['clips']] == [43]+list(range(45,55))+[44]
library=json.loads((root/'record/dance_library/manifest.json').read_text())
authored={c['number']:c for c in library['actions']}
for c in report['clips']:
    data = (selected/c['file']).read_bytes()
    assert hashlib.sha256(data).hexdigest() == c['sha256']
    assert struct.unpack_from('<HH',data,6) == (640,640)
    assert sum(c['delays_ms']) == c['duration_ms'] == round(authored[c['number']]['duration_s']*100)*10
    assert c['name']==authored[c['number']]['name']
    assert sum(r['duration_ms'] for r in c['rgba_frames'])==c['duration_ms']
