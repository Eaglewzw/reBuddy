#!/usr/bin/env python3
"""Render generated dance plans as offline URDF joint skeletons; never opens CAN.

This is a kinematic preview, not a mesh render, collision check or dynamics
simulation. The supplied URDF fixes the gripper fingers, so J7 is shown as a
motor-angle readout instead of inventing an aperture-to-angle calibration.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from rs_gravity_record import ROOT, DEFAULT_CALIBRATION, DEFAULT_URDF, digest

FONT_PATH = '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
BG = '#f4f5ef'
INK = '#253539'
GREEN = '#38866b'
BLUE = '#4d79a0'


def font(size):
    return ImageFont.truetype(FONT_PATH, size, index=2)


class Skeleton:
    def __init__(self, urdf):
        import pinocchio as pin
        self.pin = pin
        self.model = pin.buildModelFromUrdf(str(urdf))
        if self.model.nq != 6:
            raise ValueError('preview requires the calibrated six-axis RS model')
        self.data = self.model.createData()
        self.tip_id = self.model.getFrameId('gripper_end')
        if self.tip_id >= len(self.model.frames):
            raise ValueError('URDF has no gripper_end frame')

    def points(self, q, tcp_offset=None):
        self.pin.framesForwardKinematics(self.model, self.data, np.asarray(q[:6], float))
        points = [np.zeros(3)]
        points.extend(self.data.oMi[j].translation.copy() for j in range(1, 7))
        points.append(self.data.oMf[self.tip_id].translation.copy())
        tip = self.data.oMf[self.tip_id]
        tcp = tip.translation if tcp_offset is None else tip.translation+tip.rotation @ np.asarray(tcp_offset)
        if tcp_offset is not None:
            points.append(tcp.copy())
        # Orientation axes make wrist rotation visible even with fixed TCP.
        axes = np.array([tcp + tip.rotation[:, j]*.05 for j in range(3)])
        return np.array(points), axes


def project(points):
    points = np.asarray(points)
    # Orthographic 3/4 view; the same projection and scale cover every action.
    return np.column_stack((.82*points[:, 0]-.57*points[:, 1],
                            -.25*points[:, 0]-.36*points[:, 1]-points[:, 2]))


def export_tip_samples(skeleton, plan, path, tcp_offset=None, fixed_point=None,
                       virtual_offset=None):
    """Export model-only diagnostics for world-stabilized and path gestures."""
    times = np.linspace(0., plan.duration, 601)
    positions, rotations, joints, velocities, accelerations, pivots = [], [], [], [], [], []
    skeleton.points(plan.start)
    initial_rotation = skeleton.data.oMf[skeleton.tip_id].rotation.copy()
    for stamp in times:
        q, v = plan.at(float(stamp))
        skeleton.points(q)
        tip = skeleton.data.oMf[skeleton.tip_id]
        tcp = tip.translation if tcp_offset is None else tip.translation+tip.rotation @ np.asarray(tcp_offset)
        positions.append(tcp.tolist())
        if virtual_offset is not None:
            pivots.append((tcp+tip.rotation @ np.asarray(virtual_offset)).tolist())
        # Rotation vector in the world frame relative to the entry orientation.
        rotations.append(np.degrees(skeleton.pin.log3(
            tip.rotation @ initial_rotation.T)).tolist())
        joints.append(np.degrees(q[:6]).tolist())
        velocities.append(np.degrees(v[:6]).tolist())
        accelerations.append(np.degrees(plan.acceleration_at(float(stamp))[:6]).tolist())
    path.write_text(json.dumps(dict(
        schema='rebuddy.dance_tip_preview.v1', model_only=True,
        collision_checked=False, time_s=times.tolist(),
        tcp_frame='gripper_end', tcp_offset_m=tcp_offset,
        fixed_point_m=fixed_point,
        virtual_offset_m=virtual_offset, virtual_p_xyz_m=pivots,
        tip_xyz_m=positions, world_rotation_vector_deg=rotations,
        joint_deg=joints, joint_velocity_deg_s=velocities,
        joint_acceleration_deg_s2=accelerations), ensure_ascii=False, indent=2)+'\n')


def canvas_pose(skeleton, q, bounds, size=(480, 390), ghost=None,
                tcp_offset=None, fixed_point=None, virtual_offset=None):
    image = Image.new('RGB', size, BG)
    draw = ImageDraw.Draw(image)
    left, top, right, bottom = bounds
    scale = min((size[0]-65)/(right-left), (size[1]-125)/(bottom-top))
    offset = np.array([size[0]/2-(left+right)*scale/2,
                       70+(size[1]-125)/2-(top+bottom)*scale/2])

    def pixels(points):
        return [tuple(p) for p in project(points)*scale+offset]

    for coordinate in np.arange(-.4, .41, .1):
        draw.line(pixels([[-.4, coordinate, 0], [.4, coordinate, 0]]), fill='#dfe4dd', width=1)
        draw.line(pixels([[coordinate, -.4, 0], [coordinate, .4, 0]]), fill='#dfe4dd', width=1)
    if ghost is not None:
        points, _ = skeleton.points(ghost, tcp_offset)
        draw.line(pixels(points), fill='#c4cdc7', width=5)
    points, axes = skeleton.points(q, tcp_offset)
    positions = pixels(points)
    draw.line(positions, fill=GREEN, width=8)
    for j, (x, y) in enumerate(positions):
        r = 5 if j else 8
        draw.ellipse((x-r, y-r, x+r, y+r), fill=BG, outline=INK, width=2)
    for endpoint, color in zip(pixels(axes), ('#bc5d57', '#579469', '#608ab4')):
        draw.line([positions[-1], endpoint], fill=color, width=3)
    if virtual_offset is not None:
        virtual_p = points[-1]+skeleton.data.oMf[skeleton.tip_id].rotation @ np.asarray(virtual_offset)
        virtual_pixel = pixels([virtual_p])[0]
        draw.line([positions[-1], virtual_pixel], fill='#b38d46', width=3)
        x, y = positions[-1]
        draw.text((x-28, y+10), 'TCP', font=font(13), fill='#687777')
    if fixed_point is not None:
        x, y = pixels([fixed_point])[0]
        draw.ellipse((x-10, y-10, x+10, y+10), outline='#b76233', width=2)
        draw.line((x-14, y, x+14, y), fill='#b76233', width=2)
        draw.line((x, y-14, x, y+14), fill='#b76233', width=2)
        draw.text((x+14, y-24), '虚拟 P' if virtual_offset is not None else 'TCP 定点',
                  font=font(14), fill='#9b512c')
    return image


def render_library(folder, output, calibration, urdf, fps=12, sheet_only=False):
    folder, output = Path(folder), Path(output)
    schema = json.loads((folder/'manifest.json').read_text()).get('schema')
    if schema == 'rebuddy.dance_library.v1':
        from rs_dance_design import load_library
        title = '机械臂舞蹈姿态'
    elif schema == 'rebuddy.dance_showcase.v1':
        from rs_dance_showcase import load_library
        title = '机械臂舞台动作'
    else:
        raise ValueError(f'unsupported dance preview schema: {schema}')
    skeleton = Skeleton(urdf)
    manifest, plans, _, _, reports = load_library(calibration, urdf, folder)
    codes = [f'{int(action["number"]):02d}' for action in manifest['actions']]
    if not codes or len(codes) != len(set(codes)) or set(codes) != set(plans):
        raise ValueError('preview action index does not match loaded plans')
    if schema == 'rebuddy.dance_showcase.v1' and '11' in codes:
        codes = [code for code in codes if code != '11']+['11']
    loaded = [(code, plans[code], reports[code]) for code in codes]
    pose_options = {code: {} for code in codes}
    if schema == 'rebuddy.dance_showcase.v1' and '21' in codes:
        action = next(a for a in manifest['actions'] if int(a['number']) == 21)
        action_spec = json.loads((folder/action['file']).read_text())
        design = action_spec.get('pivot_sway_design', action_spec.get('virtual_pivot_design', action_spec.get('body_design', action_spec.get('pivot_design'))))
        if design is None:
            raise ValueError('21 preview requires a fixed-TCP design description')
        if 'virtual_pivot_design' in action_spec or 'pivot_sway_design' in action_spec:
            pose_options['21'] = dict(tcp_offset=design['model_tcp_offset_m'],
                                      fixed_point=design['P_world_m'],
                                      virtual_offset=design['virtual_offset_m'])
        else:
            pose_options['21'] = dict(tcp_offset=design['tcp_offset_m'],
                                      fixed_point=design['tcp_anchor_m'])
    # Include the floor and all sampled poses in one constant scene scale.
    all_points = [np.array([[-.35, -.35, 0], [.35, .35, 0]])]
    for code, plan, _ in loaded:
        for stamp in np.linspace(0., plan.duration, 41):
            points, axes = skeleton.points(plan.at(stamp)[0], pose_options[code].get('tcp_offset'))
            all_points.extend((points, axes))
            if pose_options[code].get('virtual_offset') is not None:
                virtual_p = points[-1]+skeleton.data.oMf[skeleton.tip_id].rotation @ np.asarray(pose_options[code]['virtual_offset'])
                all_points.append(np.asarray([virtual_p, pose_options[code]['fixed_point']]))
    projected = project(np.vstack(all_points))
    lo, hi = projected.min(axis=0)-.025, projected.max(axis=0)+.025
    bounds = (*lo, *hi)
    output.mkdir(parents=True, exist_ok=True)
    columns = min(3, len(loaded))
    rows = (len(loaded)+columns-1)//columns
    sheet = Image.new('RGB', (columns*480, rows*390+95), BG)
    heading = ImageDraw.Draw(sheet)
    heading.text((24, 12), f'reBuddy · {len(loaded)} 组{title}', font=font(27), fill=INK)
    heading.text((24, 52), 'URDF 关节骨架预览 · 无碰撞几何 · 不代表实机验收 · J7 仅显示电机角度',
                 font=font(18), fill='#687777')
    index = dict(schema='rebuddy.dance_skeleton_preview.v1', offline=True,
                 library_schema=schema, library=str(folder.resolve()),
                 manifest_sha256=digest(folder/'manifest.json'),
                 frames_per_second=fps, collision_checked=False,
                 model_sha256=digest(urdf), actions=[])
    for tile, (code, plan, report) in enumerate(loaded):
        name = report.get('name', code)
        # Show the most visibly displaced pose; grey outline is its entry pose.
        candidates = np.linspace(0., plan.duration, 101)
        amplitudes = [np.linalg.norm(plan.at(t)[0]-plan.start) for t in candidates]
        representative = float(candidates[int(np.argmax(amplitudes))])
        pose = plan.at(representative)[0]
        still = canvas_pose(skeleton, pose, bounds, ghost=plan.start, **pose_options[code])
        draw = ImageDraw.Draw(still)
        draw.text((16, 12), f'{code}  {name}', font=font(23), fill=INK)
        draw.text((16, 43), f'第 {representative:.1f} 秒  ·  J7 {np.degrees(pose[6]):.1f}°',
                  font=font(16), fill='#687777')
        time_label = '名义 ' if code == '21' else ''
        draw.text((16, 350), f'{time_label}{plan.duration:.2f} 秒  ·  灰线为起始姿态', font=font(17), fill='#687777')
        stem = code
        still.save(output/f'{stem}.png')
        sheet.paste(still, ((tile % columns)*480, 95+(tile//columns)*390))
        item = dict(number=stem, name=name, duration_s=plan.duration,
                    still=f'{stem}.png', representative_s=representative)
        if schema == 'rebuddy.dance_showcase.v1' and code in ('19', '20', '21'):
            sample_file = f'{stem}-tip-samples.json'
            export_tip_samples(skeleton, plan, output/sample_file, **pose_options[code])
            item['tip_samples'] = sample_file
            if code == '21':
                item['tcp_offset_m'] = pose_options[code]['tcp_offset']
                item['fixed_point_m'] = pose_options[code]['fixed_point']
                if pose_options[code].get('virtual_offset') is not None:
                    item['virtual_offset_m'] = pose_options[code]['virtual_offset']
                    item['fixed_point_kind'] = 'virtual_P_during_dance_not_model_TCP'
                    item['dance_interval_s'] = design['dance_interval_s']
                item['time_basis'] = 'nominal_plan_before_trial_speed_scaling'
        if not sheet_only:
            frames = []
            count = max(2, int(round(plan.duration*fps)))
            for stamp in np.linspace(0., plan.duration, count):
                q, _ = plan.at(stamp)
                frame = canvas_pose(skeleton, q, bounds, ghost=plan.start, **pose_options[code])
                draw = ImageDraw.Draw(frame)
                draw.text((16, 12), f'{code}  {name}', font=font(23), fill=INK)
                subtitle = ('名义速度 / 舞段固定 P，TCP 绕行' if pose_options[code].get('virtual_offset') is not None
                            else '名义速度 / 模型 TCP 定点 / 非指尖标定') if code == '21' else '骨架预览 / 无碰撞验证'
                draw.text((16, 42), subtitle, font=font(15), fill='#687777')
                draw.text((16, 338), f'{stamp:4.1f} / {plan.duration:.1f} 秒     J7 {np.degrees(q[6]):.1f}°',
                          font=font(17), fill=INK)
                draw.line((16, 373, 464, 373), fill='#d7ded7', width=5)
                draw.line((16, 373, 16+448*stamp/plan.duration, 373), fill=BLUE, width=5)
                frames.append(frame)
            # GIF delays have 10 ms granularity. Distribute rounding over the
            # whole clip so 12 fps does not silently make every move 4% faster.
            ticks = np.rint(np.linspace(0., plan.duration*100., count+1)).astype(int)
            delays = (np.diff(ticks)*10).tolist()
            frames[0].save(output/f'{stem}.gif', save_all=True, append_images=frames[1:],
                           duration=delays, loop=0, optimize=True)
            item['gif'] = f'{stem}.gif'
        index['actions'].append(item)
    sheet.save(output/'contact-sheet.png')
    (output/'index.json').write_text(json.dumps(index, ensure_ascii=False, indent=2)+'\n')
    return output/'contact-sheet.png'


def main():
    from rs_dance_design import DEFAULT_OUTPUT
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--library', type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument('--output', type=Path, default=ROOT/'build/dance-design-preview')
    parser.add_argument('--calibration', type=Path, default=DEFAULT_CALIBRATION)
    parser.add_argument('--urdf', type=Path, default=DEFAULT_URDF)
    parser.add_argument('--fps', type=int, default=12, choices=range(4, 31))
    parser.add_argument('--sheet-only', action='store_true')
    args = parser.parse_args()
    print(render_library(args.library, args.output, args.calibration, args.urdf, args.fps, args.sheet_only))


if __name__ == '__main__':
    main()
