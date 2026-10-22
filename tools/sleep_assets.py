"""Shared authored sleep poses used by GIF export and firmware resources."""
from collections import deque
from PIL import Image
BG = (248, 247, 239)
RESAMPLE = getattr(Image, 'Resampling', Image).LANCZOS

def load_sleep_poses(sleep_source):
    sheet = Image.open(sleep_source).convert('RGB')
    sleep_poses = []
    registrations = []
    for n in range(4):
        # Only the first row is used: it has ordered inhale poses. The reverse
        # path uses these SAME images to avoid the second row's shape drift.
        cell = sheet.crop((round(n * sheet.width / 4), 0,
                           round((n + 1) * sheet.width / 4), round(sheet.height / 2)))
        cell = cell.resize((160, 160), RESAMPLE)
        px = list(cell.getdata())
        queue = deque([(x, y) for x in range(160) for y in (0, 159)] +
                      [(x, y) for y in range(160) for x in (0, 159)])
        visited = bytearray(25600)
        while queue:
            x, y = queue.popleft()
            if not (0 <= x < 160 and 0 <= y < 160): continue
            i = y * 160 + x
            if visited[i]: continue
            visited[i] = 1
            if min(px[i]) > 205 and max(px[i]) - min(px[i]) < 35:
                px[i] = BG
                queue.extend(((x-1, y), (x+1, y), (x, y-1), (x, y+1)))
        cell.putdata(px)
        occupied = [(x, y) for y in range(120, 160) for x in range(160)
                    if min(cell.getpixel((x, y))) < 150]
        bottom = max(y for _, y in occupied)
        foot = [x for x, y in occupied if y >= bottom - 8]
        dx, dy = 80 - (min(foot) + max(foot)) // 2, 151 - bottom
        canvas = Image.new('RGB', (160, 160), BG)
        canvas.paste(cell, (dx, dy))
        sleep_poses.append(canvas)
        registrations.append([dx, dy])
    # Lock the visible lower base. The wrist/claw never enters this bottom band.
    base = sleep_poses[0].crop((0, 139, 160, 160))
    for im in sleep_poses: im.paste(base, (0, 139))
    return sleep_poses, registrations
