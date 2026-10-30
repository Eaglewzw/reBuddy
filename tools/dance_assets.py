"""Offline dance matte and compact, room-independent indexed frames.

The device only inflates indices/coverage and composites pixels. Background
connectivity and edge coverage are never recomputed in the LVGL task.
"""
from collections import deque
import numpy as np
from PIL import Image


def rgb565_array(image):
    rgb = np.asarray(image.convert('RGB'), dtype=np.uint16)
    return ((rgb[..., 0] >> 3) << 11) | ((rgb[..., 1] >> 2) << 5) | (rgb[..., 2] >> 3)


def channels(pixels):
    p = pixels.astype(np.int32)
    return np.stack(((p >> 11) * 255 // 31, ((p >> 5) & 63) * 255 // 63,
                     (p & 31) * 255 // 31), axis=-1)


def coverage(pixels, cavities=()):
    """Exterior and explicitly reviewed cavity floods, preserving highlights."""
    height, width = pixels.shape
    p = pixels.astype(np.int32)
    key = int(p[0, 0])
    candidate = ((abs((p >> 11) - (key >> 11)) <= 1) &
                 (abs(((p >> 5) & 63) - ((key >> 5) & 63)) <= 2) &
                 (abs((p & 31) - (key & 31)) <= 2))
    outside = np.zeros(p.shape, dtype=bool)
    queue = deque()
    for y, x in ([(0, x) for x in range(width)] + [(height-1, x) for x in range(width)] +
                 [(y, 0) for y in range(height)] + [(y, width-1) for y in range(height)]):
        if candidate[y, x] and not outside[y, x]:
            outside[y, x] = True
            queue.append((y, x))
    try:
        from scipy.ndimage import binary_propagation
    except ImportError:
        while queue:
            y, x = queue.popleft()
            for ny, nx in ((y-1, x), (y+1, x), (y, x-1), (y, x+1)):
                if 0 <= ny < height and 0 <= nx < width and candidate[ny, nx] and not outside[ny, nx]:
                    outside[ny, nx] = True
                    queue.append((ny, nx))
    else:
        outside=binary_propagation(outside,mask=candidate)
    # Enclosed linkage gaps are background too, but so are neither eye whites
    # nor metal highlights. Never remove an arbitrary enclosed pale component.
    # Source-specific reviewed seeds are guarded by exact area/bounds checks.
    for cavity in cavities:
        x,y=cavity['seed']
        if not (0<=x<width and 0<=y<height and candidate[y,x] and not outside[y,x]):
            raise ValueError(f'Stale or non-enclosed dance cavity seed: {cavity}')
        region=np.zeros(p.shape,dtype=bool)
        pending=[(x,y)]
        while pending:
            x,y=pending.pop()
            if not (0<=x<width and 0<=y<height) or region[y,x] or not candidate[y,x]:
                continue
            region[y,x]=True
            pending.extend(((x-1,y),(x+1,y),(x,y-1),(x,y+1)))
        ys,xs=np.where(region)
        bounds=[int(xs.min()),int(ys.min()),int(xs.max())+1,int(ys.max())+1]
        if int(region.sum())!=cavity['area'] or bounds!=cavity['bounds'] or np.any(region & outside):
            raise ValueError(f'Dance cavity geometry changed; review required: {cavity}')
        outside |= region
    rgb = channels(pixels)
    darkest = np.full(p.shape, 766)
    foreground = rgb.copy()
    adjacent = np.zeros(p.shape, dtype=bool)
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            if not dx and not dy:
                continue
            exterior = np.roll(outside, (dy, dx), (0, 1))
            neighbor = np.roll(rgb, (dy, dx), (0, 1))
            light = neighbor.sum(axis=-1)
            replace = ~exterior & (light < darkest)
            foreground[replace] = neighbor[replace]
            darkest[replace] = light[replace]
            adjacent |= exterior
    edge = (~outside & adjacent & (darkest <= 300) &
            ((p >> 11) >= 20) & (((p >> 5) & 63) >= 45) & ((p & 31) >= 20))
    edge[[0, -1], :] = False
    edge[:, [0, -1]] = False
    key_rgb = channels(np.array(key, dtype=np.uint16))
    delta = key_rgb - foreground
    denominator = (delta * delta).sum(axis=-1)
    numerator = ((key_rgb-rgb)*delta).sum(axis=-1)
    alpha = np.full(p.shape, 255, dtype=np.uint8)
    valid = edge & (denominator > 0)
    alpha[valid] = np.clip(numerator[valid]*255//denominator[valid], 0, 255)
    alpha = ((alpha.astype(np.uint16)+8)//17*17).clip(0,255).astype(np.uint8)
    alpha[outside] = 0
    return alpha


def indexed_frame(image, palette_rgb, cavities=()):
    # A fresh palette avoids Pillow's mutable RGB->index lookup cache leaking
    # from quantization of an earlier image into the next frame.
    palette = Image.new('P', (1, 1))
    palette.putpalette(palette_rgb)
    indexed = image.quantize(palette=palette, dither=0)
    pixels = rgb565_array(indexed)
    alpha = coverage(pixels,cavities)
    indices = np.asarray(indexed).copy()
    indices[alpha == 0] = 0  # exterior color is irrelevant; improves XOR compression
    return indices.tobytes() + alpha.tobytes(), pixels


def composite(planes, palette565, key, room):
    count = room.size
    data = np.frombuffer(planes, dtype=np.uint8)
    pixels = np.asarray(palette565, dtype=np.uint16)[data[:count]].reshape(room.shape)
    alpha = data[count:].reshape(room.shape).astype(np.int32)
    rgb, bg, matte = channels(pixels), channels(room), channels(np.array(key, dtype=np.uint16))
    # Truncate toward zero, matching C signed integer division.
    rgb += np.trunc((255-alpha[..., None])*(bg-matte)/255).astype(np.int32)
    rgb = np.clip(rgb, 0, 255).astype(np.uint16)
    result = ((rgb[..., 0] >> 3) << 11) | ((rgb[..., 1] >> 2) << 5) | (rgb[..., 2] >> 3)
    result[alpha == 0] = room[alpha == 0]
    result[alpha == 255] = pixels[alpha == 255]
    return result.astype('<u2').tobytes()
