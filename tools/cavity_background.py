"""Remove explicitly identified enclosed backdrop regions, preserving highlights."""

BG = (248, 247, 239)


def clear_cavity(image, region, min_channel=140, max_channel_spread=40):
    """Flood only the reviewed cavity; return its pixels for export verification."""
    x0, y0, x1, y1 = region['bounds']
    pending = [tuple(region['seed'])]
    seen, cleared = set(), []
    while pending:
        x, y = pending.pop()
        if (x, y) in seen or not (x0 <= x < x1 and y0 <= y < y1):
            continue
        seen.add((x, y))
        rgb = image.getpixel((x, y))
        if min(rgb) <= min_channel or max(rgb) - min(rgb) >= max_channel_spread:
            continue
        image.putpixel((x, y), BG)
        cleared.append((x, y))
        pending.extend(((x-1, y), (x+1, y), (x, y-1), (x, y+1)))
    if not cleared:
        raise ValueError(f'Cavity seed no longer matches source: {region}')
    return cleared
