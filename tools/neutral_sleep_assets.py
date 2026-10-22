"""Integrate only cheek patches from imagegen retouches into original poses."""
import hashlib
import json
from pathlib import Path
from PIL import Image, ImageFilter

ROOT = Path(__file__).resolve().parents[1]
RECIPE = ROOT / 'assets/design/sleep-neutral-v1/retouch.json'


def neutral_sleep_poses(images, registration):
    cfg = json.loads(RECIPE.read_text())
    for name, sha in cfg['edit_sha256'].items():
        assert hashlib.sha256((RECIPE.parent / name).read_bytes()).hexdigest() == sha
    atlas = Image.open(RECIPE.parent / cfg['pose_edit']).convert('RGB')
    breath = Image.open(RECIPE.parent / cfg['breath_edit']).convert('RGB')
    assert atlas.size == (1536, 1024) and breath.size == (1774, 887)
    results, masks = {}, {}
    for key, bounds in cfg['cheek_bounds'].items():
        source = int(key)
        if source < 24:
            x, y = source % 6 * 256, source // 6 * 256
            cell = atlas.crop((x, y, x + 256, y + 256))
        else:
            i = source - 24
            cell = breath.crop((round(i * breath.width / 4), 0,
                                round((i + 1) * breath.width / 4), round(breath.height / 2)))
        cell = cell.resize((160, 160), getattr(Image, 'Resampling', Image).LANCZOS)
        edited = Image.new('RGB', (160, 160))
        edited.paste(cell, tuple(registration[source]['offset_px']))
        original = images[source]
        mask = Image.new('L', original.size)
        for x0, y0, x1, y1 in bounds:
            for y in range(y0, y1):
                for x in range(x0, x1):
                    r, g, b = original.getpixel((x, y))
                    if r > max(g, b) + cfg['red_excess']:
                        mask.putpixel((x, y), 255)
        assert mask.getbbox(), f'No blush found in source {source}'
        mask = mask.filter(ImageFilter.MaxFilter(2 * cfg['mask_dilation_px'] + 1))
        # Copy only the tiny retouched cheek areas. Eyes, body, pose and room
        # keep their original pixels even if the generated atlas changed them.
        result = original.copy()
        result.paste(edited, (0, 0), mask)
        results[source], masks[source] = result, mask
    return results, masks, cfg
