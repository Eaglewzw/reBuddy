"""Pinned component bytes are verifiable without the source checkout."""
import hashlib
import json
from pathlib import Path
root = Path(__file__).resolve().parents[2]
manifest = json.loads((root/'assets/config/arm_components.json').read_text())
for name, digest in manifest['files'].items():
    assert hashlib.sha256((root/name).read_bytes()).hexdigest() == digest, name
for component in manifest['components']:
    actual = {str(p.relative_to(root)) for p in (root/'components'/component).rglob('*') if p.is_file()}
    expected = {p for p in manifest['files'] if p.startswith(f'components/{component}/')}
    assert actual == expected, (component, actual ^ expected)
print('PASS local control component snapshot and source provenance')
