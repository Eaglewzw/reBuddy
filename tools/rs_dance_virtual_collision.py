"""Offline RS self-collision checks using the locally installed CAD meshes.

This checks the source URDF's collision surfaces, not an observed environment.
The fingers are closed; their motor-to-slider mapping is deliberately not guessed.
"""
from pathlib import Path
import hashlib
import xml.etree.ElementTree as ET

import numpy as np

DEFAULT_COLLISION_URDF = Path('/home/verser/CPP/JoyReBot/src/joyrebot_gazebo_sim/urdf/rebot_b601_rs.urdf')
DEFAULT_SRDF = Path('/home/verser/CPP/reBotArmController_ROS2/src/rebotarm_moveit_config/config/rebotarm_rs.srdf')


def _resource(path):
    path = Path(path).resolve(strict=True)
    return {'path': str(path), 'sha256': hashlib.sha256(path.read_bytes()).hexdigest()}


def validate_identity(expected):
    """Fail closed if any external geometry or collision-policy resource changed."""
    if not isinstance(expected, dict) or expected.get('schema') != 'rebuddy.rs_collision.v1':
        raise ValueError('invalid collision resource identity')
    resources = expected.get('resources', [])
    if not resources:
        raise ValueError('collision resource identity is empty')
    for item in resources:
        if _resource(item['path']) != item:
            raise ValueError(f"collision resource changed: {item['path']}")
    return True


class CollisionAudit:
    def __init__(self, gravity, urdf_path=DEFAULT_COLLISION_URDF, srdf_path=DEFAULT_SRDF):
        self.pin = gravity.pin
        self.urdf_path = Path(urdf_path).resolve(strict=True)
        self.srdf_path = Path(srdf_path).resolve(strict=True)
        tree = ET.parse(self.urdf_path).getroot()
        self.model = self.pin.buildModelFromUrdf(str(self.urdf_path))
        if list(self.model.names)[1:] != ['joint1', 'joint2', 'joint3', 'joint4',
                                        'joint5', 'joint6', 'joint_left', 'joint_right']:
            raise ValueError('collision model joint mapping differs from RS control model')
        if self.model.nq != 8 or gravity.model.nq != 6:
            raise ValueError('collision model requires six arm axes and two closed finger sliders')
        self.geometry = self.pin.buildGeomFromUrdf(
            self.model, str(self.urdf_path), self.pin.GeometryType.COLLISION,
            package_dirs=[str(self.urdf_path.parents[2])])
        if self.geometry.ngeoms != 10:
            raise ValueError('RS collision model must provide all ten link collision meshes')
        self._verify_kinematics(gravity)
        self.geometry.addAllCollisionPairs()
        srdf = ET.parse(self.srdf_path).getroot()
        adjacent = {frozenset((j.find('parent').get('link'), j.find('child').get('link')))
                    for j in tree.findall('joint')}
        mesh_links = {self.model.frames[g.parentFrame].name for g in self.geometry.geometryObjects}
        disabled = []
        for item in srdf.findall('disable_collisions'):
            a, b, reason = item.get('link1'), item.get('link2'), item.get('reason')
            if a not in mesh_links or b not in mesh_links:
                continue  # SRDF also mentions a meshless optional TCP frame.
            pair = frozenset((a, b))
            if not ((reason == 'Adjacent' and pair in adjacent) or
                    (reason == 'Never' and pair == frozenset(('gripper_left', 'gripper_right')))):
                raise ValueError(f'unreviewed collision exclusion: {a}/{b}/{reason}')
            disabled.append({'links': sorted(pair), 'reason': reason})
        self.pin.removeCollisionPairs(self.model, self.geometry, str(self.srdf_path))
        self.data = self.model.createData()
        self.geometry_data = self.geometry.createData()
        paths = [self.urdf_path, self.srdf_path]
        paths.extend(Path(g.meshPath) for g in self.geometry.geometryObjects)
        self.identity = {
            'schema': 'rebuddy.rs_collision.v1',
            'resources': [_resource(p) for p in sorted(set(paths), key=str)],
            'geometry_count': self.geometry.ngeoms,
            'checked_pair_count': len(self.geometry.collisionPairs),
            'disabled_pairs': disabled,
            'scope': 'source URDF collision meshes; closed fingers; robot self-collision only',
            'environment_checked': False, 'physical_geometry_calibrated': False,
        }

    def _verify_kinematics(self, gravity):
        """Compare transforms for every collision link, including closed fingers."""
        data = self.model.createData()
        reference_data = gravity.model.createData()
        names = ['base_link', 'link1', 'link2', 'link3', 'link4', 'link5', 'link6',
                 'gripper_end', 'gripper_left', 'gripper_right']
        for arm in (np.zeros(6), np.array([.2, .8, 1.1, -.3, .25, -.4]),
                    np.array([-.6, 1.5, .7, .4, -.5, .8])):
            self.pin.forwardKinematics(self.model, data, np.r_[arm, 0., 0.])
            self.pin.updateFramePlacements(self.model, data)
            self.pin.forwardKinematics(gravity.model, reference_data, arm)
            self.pin.updateFramePlacements(gravity.model, reference_data)
            for name in names:
                if not self.model.existFrame(name) or not gravity.model.existFrame(name):
                    raise ValueError(f'collision model lacks matching control frame {name}')
                a = data.oMf[self.model.getFrameId(name)]
                b = reference_data.oMf[gravity.model.getFrameId(name)]
                if not (np.allclose(a.translation, b.translation, atol=1e-9, rtol=0.) and
                        np.allclose(a.rotation, b.rotation, atol=1e-9, rtol=0.)):
                    raise ValueError(f'collision/control geometry frame mismatch: {name}')

    def validate_identity(self, expected):
        validate_identity(expected)
        if expected != self.identity:
            raise ValueError('collision resources or pair policy differ from saved trajectory')
        return True

    def check(self, q, distances=False):
        q = np.asarray(q, dtype=float)
        if q.shape != (7,) or not np.all(np.isfinite(q)) or abs(q[6]) > 1e-8:
            raise ValueError('collision audit requires finite seven-axis data with a closed gripper')
        state = np.r_[q[:6], 0., 0.]
        self.pin.computeCollisions(self.model, self.data, self.geometry, self.geometry_data,
                                   state, False)
        pairs = []
        for pair, result in zip(self.geometry.collisionPairs, self.geometry_data.collisionResults):
            if result.isCollision():
                pairs.append([self.geometry.geometryObjects[pair.first].name,
                              self.geometry.geometryObjects[pair.second].name])
        result = {'colliding': bool(pairs), 'pairs': pairs, 'min_distance_m': None}
        if distances:
            self.pin.computeDistances(self.geometry, self.geometry_data)
            result['min_distance_m'] = min(r.min_distance for r in self.geometry_data.distanceResults)
        return result
