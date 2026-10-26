"""Dedicated body-dance poses and original-gripper rig, with fixed camera scale."""
import hashlib
import json
import math
from pathlib import Path
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
R = getattr(Image, 'Resampling', Image)


def source_record(path, rig_path=None):
    record = {'path': str(path.relative_to(ROOT)),
              'sha256': hashlib.sha256(path.read_bytes()).hexdigest()}
    if rig_path:
        record.update(rig=str(rig_path.relative_to(ROOT)),
                      rig_sha256=hashlib.sha256(rig_path.read_bytes()).hexdigest())
    return record


def register(sprite, scale, anchor):
    # The opaque pedestal contact, rather than its rotating coloured panel.
    alpha = sprite.getchannel('A').point(lambda v: 255 if v > 230 else 0)
    bottom = alpha.getbbox()[3]
    # Use only the bottom contact band: lowered jaws can enter the lower 22%
    # of the sprite and must not pull the pedestal registration sideways.
    lower = max(0, bottom-round(sprite.height*.08))
    x0, y0, x1, y1 = alpha.crop((0, lower, sprite.width, sprite.height)).getbbox()
    origin = (anchor[0] - round((x0+x1)*scale/2), anchor[1] - round((lower+y1)*scale))
    geometry = {'fixed_scale': scale, 'floor_center': anchor[0], 'floor_y': anchor[1],
                'floor_width': (x1-x0)*scale, 'origin': origin}
    return origin, geometry


def build_dances(room):
    config = ROOT/'assets/images/source/dance/extended/rig.json'
    rig = json.loads(config.read_text())
    atlas_path = ROOT/rig['atlas']
    atlas = Image.open(atlas_path).convert('RGBA')
    sources = {'body-dance-v2': source_record(atlas_path, config)}
    registration = {'body-dance-v2': {}, 'yaw-dance-v2': {}}
    sprites, specs = {}, []

    def place(part, scale, origin):
        frame = room.copy()
        size = tuple(round(v*scale) for v in part.size)
        frame.alpha_composite(part.resize(size, R.LANCZOS), origin)
        return frame.convert('RGB')

    for i in range(9):
        x, y = i % 3, i // 3
        part = atlas.crop((rig['columns'][x], rig['rows'][y],
                           rig['columns'][x+1], rig['rows'][y+1]))
        origin, geometry = register(part, rig['body_scale'], rig['floor_anchor'])
        registration['body-dance-v2'][i] = geometry
        sprites[('body-dance-v2', i)] = place(part, rig['body_scale'], origin)

    yaw_config = ROOT/rig['yaw_rig']
    yaw = json.loads(yaw_config.read_text())
    body_path, jaw_path = ROOT/yaw['body_source'], ROOT/yaw['jaw_source']
    sources['yaw-dance-v2'] = source_record(body_path, yaw_config)
    sources['jaw-dance-v2'] = source_record(jaw_path, config)
    sheet = Image.open(body_path).convert('RGBA')
    jaw_sheet = Image.open(jaw_path).convert('RGBA')
    radius = 128
    jaws = []
    for box, pivot in zip(yaw['jaw_crops'], yaw['jaw_pivots']):
        part = jaw_sheet.crop(box)
        part = part.resize(tuple(round(v*yaw['jaw_setup_scale']) for v in part.size), R.LANCZOS)
        px, py = [round((pivot[j]-box[j])*yaw['jaw_setup_scale']) for j in (0,1)]
        layer = Image.new('RGBA', (2*radius, 2*radius))
        layer.alpha_composite(part, (radius-px, radius-py))
        jaws.append(layer)

    size = yaw['body_cell_px']
    scale = yaw['body_render_px']/size
    for view in range(3):
        body = sheet.crop((view*size, 0, (view+1)*size, size))
        origin, geometry = register(body, scale, rig['floor_anchor'])
        registration['yaw-dance-v2'][view] = geometry
        for tick in range(16 if view == 1 else 1):
            robot = body.copy()
            # Alternate the original clamp sides around their own sockets.
            envelope = math.sin(math.pi*tick/15)**2
            for j, jaw in enumerate(jaws):
                angle = yaw['jaw_rest_deg'][j]
                if view == 1:
                    angle += 55*envelope*math.sin(2*math.pi*tick/4 + j*math.pi)
                rotated = jaw.rotate(angle, R.BICUBIC, center=(radius,radius))
                x, y = yaw['socket_centers'][view][j]
                robot.alpha_composite(rotated, (x-radius,y-radius))
            frame = place(robot, scale, origin)
            if tick == 0:
                sprites[('yaw-dance-v2', view)] = frame
            if view == 1:
                sprites[('alternating-v2', tick)] = frame

    def add(id, file, atlas, keys, duration):
        assert sum(n for _, n in keys)*125 == duration
        specs.append(dict(id=id, file=file, atlas=atlas, duration_ms=duration,
                          keys=keys, grid=[3,3] if atlas=='body-dance-v2' else [3,1]))

    add('BASE_TURN', '07_底盘转拍.gif', 'yaw-dance-v2',
        [[1,2],[0,2],[1,2],[2,2],[1,2],[0,1],[1,1],[2,2],[1,2]], 2000)
    add('SIDE_BEAT', '08_侧摆重拍.gif', 'body-dance-v2', rig['side_beat'], 2000)
    add('BOUNCE', '09_上下律动.gif', 'body-dance-v2', rig['bounce'], 2000)
    add('ALTERNATE', '10_左右交替.gif', 'alternating-v2', [[i,1] for i in range(16)], 2000)
    add('WAVE', '11_连环波浪.gif', 'body-dance-v2', rig['wave'], 4000)
    add('FINALE', '12_舒展收拍.gif', 'body-dance-v2', rig['finale'], 4000)
    return sprites, specs, sources, registration
