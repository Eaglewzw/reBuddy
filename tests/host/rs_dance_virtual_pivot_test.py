"""Virtual-pivot geometry and execution checks; all tests are offline."""
import copy
import json
from pathlib import Path
import sys
import tempfile
import unittest
from types import SimpleNamespace

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tools'))
import rs_dance_virtual_collision as collision
import rs_dance_virtual_pivot_scan as scan
from rs_gravity_record import preflight, DEFAULT_CALIBRATION, DEFAULT_URDF


class CollisionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not all(p.exists() for p in (DEFAULT_CALIBRATION, DEFAULT_URDF,
                                       collision.DEFAULT_COLLISION_URDF, collision.DEFAULT_SRDF)):
            raise unittest.SkipTest('Needs local RS model resources; no CAN is opened')
        cls.gravity, _ = preflight(DEFAULT_CALIBRATION, DEFAULT_URDF)
        cls.audit = collision.CollisionAudit(cls.gravity)
        cls.idle = np.asarray(json.loads((ROOT / 'record/dance_showcase/v5/manifest.json').read_text())['idle_rad'])

    def test_full_mesh_resources_and_explicit_pair_policy(self):
        identity = self.audit.identity
        self.assertEqual(identity['geometry_count'], 10)
        self.assertEqual(identity['checked_pair_count'], 35)
        self.assertEqual(len(identity['resources']), 12)
        self.assertFalse(identity['environment_checked'])
        self.assertFalse(identity['physical_geometry_calibrated'])
        self.assertTrue(self.audit.validate_identity(identity))
        for pair in identity['disabled_pairs']:
            if pair['reason'] == 'Never':
                self.assertEqual(pair['links'], ['gripper_left', 'gripper_right'])
            else:
                self.assertEqual(pair['reason'], 'Adjacent')

    def test_source_geometry_matches_every_control_link(self):
        pin = self.gravity.pin
        other = self.audit.model
        rng = np.random.default_rng(32)
        for _ in range(20):
            q = rng.uniform(self.gravity.model.lowerPositionLimit,
                            self.gravity.model.upperPositionLimit)
            d = other.createData(); reference = self.gravity.model.createData()
            pin.forwardKinematics(other, d, np.r_[q, 0., 0.]); pin.updateFramePlacements(other, d)
            pin.forwardKinematics(self.gravity.model, reference, q)
            pin.updateFramePlacements(self.gravity.model, reference)
            for geom in self.audit.geometry.geometryObjects:
                name = other.frames[geom.parentFrame].name
                expected = reference.oMf[self.gravity.model.getFrameId(name)]
                actual = d.oMf[other.getFrameId(name)]
                np.testing.assert_allclose(actual.translation, expected.translation, atol=1e-9)
                np.testing.assert_allclose(actual.rotation, expected.rotation, atol=1e-9)

    def test_control_coordinate_mismatch_is_rejected(self):
        model = self.gravity.pin.Model(self.gravity.model)
        model.jointPlacements[2].translation[0] += .01
        with self.assertRaisesRegex(ValueError, 'frame mismatch'):
            collision.CollisionAudit(SimpleNamespace(pin=self.gravity.pin, model=model))

    def test_model_positive_x_extends_outwards_beyond_the_closed_fingers(self):
        pin = self.gravity.pin; model = self.audit.model
        data = model.createData(); geometry_data = self.audit.geometry.createData()
        pin.forwardKinematics(model, data, np.r_[self.idle[:6], 0., 0.])
        pin.updateFramePlacements(model, data)
        pin.updateGeometryPlacements(model, data, self.audit.geometry, geometry_data)
        tcp = data.oMf[model.getFrameId('gripper_end')]
        dtype = np.dtype([('normal', '<f4', 3), ('points', '<f4', (3, 3)), ('attr', '<u2')])
        for index, geom in enumerate(self.audit.geometry.geometryObjects):
            name = model.frames[geom.parentFrame].name
            if name not in ('gripper_end', 'gripper_left', 'gripper_right'):
                continue
            raw = Path(geom.meshPath).read_bytes()
            vertices = np.frombuffer(raw[84:], dtype=dtype)['points'].reshape(-1, 3)
            transform = tcp.inverse() * geometry_data.oMg[index]
            local = vertices @ transform.rotation.T + transform.translation
            self.assertLess(local[:, 0].min(), -.07)
            if name == 'gripper_end':
                self.assertLess(local[:, 0].max(), -.07)
            else:
                self.assertAlmostEqual(local[:, 0].max(), 0., delta=.001)

    def test_idle_clearance_and_nonadjacent_actual_mesh_contact(self):
        result = self.audit.check(self.idle, distances=True)
        self.assertFalse(result['colliding'])
        self.assertGreater(result['min_distance_m'], .008)
        q = np.array([1.4623823311, 2.4682419186, .4022768066, -.1557881550,
                      -.4056942039, 2.6800841300, 0.])
        result = self.audit.check(q)
        self.assertTrue(result['colliding'])
        self.assertIn(['base_link_0', 'link5_0'], result['pairs'])
        self.assertIsNone(result['min_distance_m'])

    def test_unknown_gripper_configuration_and_invalid_input_rejected(self):
        for value in (self.idle[:6], np.r_[self.idle[:6], .1],
                      np.r_[np.nan, self.idle[1:]]):
            with self.assertRaises(ValueError):
                self.audit.check(value)

    def test_changed_mesh_hash_and_pair_policy_rejected(self):
        identity = copy.deepcopy(self.audit.identity)
        identity['resources'][0]['sha256'] = '0' * 64
        with self.assertRaisesRegex(ValueError, 'resource changed'):
            self.audit.validate_identity(identity)
        identity = copy.deepcopy(self.audit.identity)
        identity['checked_pair_count'] -= 1
        with self.assertRaisesRegex(ValueError, 'pair policy'):
            self.audit.validate_identity(identity)
        with tempfile.TemporaryDirectory() as folder:
            srdf = Path(folder) / 'unsafe.srdf'
            srdf.write_text('<robot name="RS"><disable_collisions '
                            'link1="base_link" link2="link5" reason="Never"/></robot>')
            with self.assertRaisesRegex(ValueError, 'unreviewed collision exclusion'):
                collision.CollisionAudit(self.gravity, srdf_path=srdf)


class ScanTests(unittest.TestCase):
    def test_failed_world_vertical_scan_never_substitutes_another_axis(self):
        import rs_dance_virtual_pivot as pivot
        cfg = scan.config_dict()
        result = {'config': cfg, 'points': [], 'samples': []}
        with self.assertRaises(pivot.VirtualPivotInfeasible) as error:
            pivot.build_recipe(None, np.zeros(7), cfg, scan=result)
        self.assertIs(error.exception.scan, result)
        self.assertEqual(result['config']['cone_axis'], 'world_z')
        self.assertEqual(result['trajectory_candidate_failures'], [])

    def test_world_vertical_cone_does_not_silently_use_idle_tool_axis(self):
        from scipy.spatial.transform import Rotation
        reference = Rotation.from_euler('xyz', [12., -18., 35.], degrees=True).as_matrix()
        for angle in (20., 50., 60.):
            for azimuth in (0., 90., 180., 270.):
                rotation = scan.desired_rotation(reference, angle, azimuth, 15., axis='world_z')
                theta, phi = np.radians([angle, azimuth])
                expected = [np.sin(theta)*np.cos(phi), np.sin(theta)*np.sin(phi), np.cos(theta)]
                np.testing.assert_allclose(rotation[:, 0], expected, atol=1e-12)
                np.testing.assert_allclose(rotation.T @ rotation, np.eye(3), atol=1e-12)

    def test_invalid_virtual_point_and_scan_configuration_rejected(self):
        for values in ({'tcp_offset_m': [-.2, 0., 0.]}, {'tcp_offset_m': [.2, .1, 0.]},
                       {'tcp_offset_m': [np.nan, 0., 0.]}, {'pointing_axis_local': [0., 0., 1.]},
                       {'sample_period_s': .005}, {'envelope_fraction': 1.1},
                       {'scan_roll_deg': []}, {'max_seed_attempts': 0},
                       {'P_world_m': [0., 0., np.nan]}, {'workspace_reach_fraction': .8}):
            with self.subTest(values=values), self.assertRaises(ValueError):
                scan.config_dict(values)
        self.assertEqual(scan.config_dict()['tcp_offset_m'], [.2, 0., 0.])
        self.assertEqual(scan.config_dict()['cone_axis'], 'world_z')

    def test_single_accepted_sample_is_not_claimed_as_a_certified_path(self):
        if not DEFAULT_CALIBRATION.exists() or not collision.DEFAULT_COLLISION_URDF.exists():
            self.skipTest('Needs local RS geometry; no CAN is opened')
        gravity, identity = preflight(DEFAULT_CALIBRATION, DEFAULT_URDF)
        idle = np.asarray(json.loads((ROOT / 'record/dance_showcase/v5/manifest.json').read_text())['idle_rad'])
        result = scan.scan_envelope(gravity, idle, {
            'cone_axis': 'idle_tool', 'scan_cone_deg': [0.], 'scan_azimuth_deg': [0.],
            'scan_roll_deg': [0.], 'P_candidates_m': [], 'max_seed_attempts': 1}, identity)
        self.assertEqual(len(result['samples']), 1)
        self.assertTrue(result['samples'][0]['accepted'])
        self.assertFalse(result['points'][0]['continuous_path_verified'])
        self.assertFalse(result['hardware_accepted'])
        self.assertIn('not a proof of global unreachability', result['scope'])
        pin = gravity.pin; model = gravity.model; data = model.createData()
        pin.forwardKinematics(model, data, idle[:6]); pin.updateFramePlacements(model, data)
        pose = data.oMf[model.getFrameId('gripper_end')]
        expected = pose.translation + .2 * pose.rotation[:, 0]
        np.testing.assert_allclose(result['initial_virtual_P_world_m'], expected, atol=1e-12)
        np.testing.assert_allclose(result['initial_model_tcp_world_m'], pose.translation, atol=1e-12)


class TrajectoryTests(unittest.TestCase):
    """A deliberately small, explicit alternate-axis fixture; not the user scan."""
    @classmethod
    def setUpClass(cls):
        import rs_dance_virtual_pivot as pivot
        import rs_dance_design as base
        import rs_dance_showcase as showcase
        if not DEFAULT_CALIBRATION.exists() or not collision.DEFAULT_COLLISION_URDF.exists():
            raise unittest.SkipTest('Needs local RS geometry; no CAN is opened')
        cls.gravity, identity = preflight(DEFAULT_CALIBRATION, DEFAULT_URDF)
        cls.idle, *_ = base._anchors(identity)
        cls.pivot = pivot
        cls.stages, cls.spec, _ = pivot.build_recipe(cls.gravity, cls.idle, {
            'cone_axis': 'idle_tool', 'scan_cone_deg': [5.],
            'scan_azimuth_deg': [0., 90., 180., 270.], 'scan_roll_deg': [0.],
            'P_candidates_m': [], 'max_seed_attempts': 1})
        cls.plan = showcase.ShowcasePlan(cls.stages)
        cls.segment = cls.plan.stages[0][1]
        cls.design = cls.spec['virtual_pivot_design']
        cls.cfg = cls.design['config']
        cls.pin = cls.gravity.pin; cls.model = cls.gravity.model
        cls.data = cls.model.createData(); cls.frame = cls.model.getFrameId('gripper_end')

    def fk(self, q):
        self.pin.forwardKinematics(self.model, self.data, q[:6])
        self.pin.updateFramePlacements(self.model, self.data)
        pose = self.data.oMf[self.frame]
        return pose.translation.copy(), pose.rotation.copy()

    def at_beat(self, beat):
        phase = next(p for p in self.segment.phase_ranges_s if beat <= p['end_beat'])
        fraction = (beat-phase['start_beat'])/(phase['end_beat']-phase['start_beat'])
        return phase['start_s'] + fraction*(phase['end_s']-phase['start_s'])

    def test_virtual_point_is_fixed_while_model_tcp_really_moves(self):
        errors = []; tips = []; rotations = []
        for t in np.linspace(*self.segment.dance_interval_s, 1501):
            q, _ = self.plan.at(t); tip, rotation = self.fk(q)
            virtual = tip + rotation @ np.asarray(self.cfg['tcp_offset_m'])
            errors.append(np.linalg.norm(virtual-self.segment.P_world_m))
            tips.append(tip); rotations.append(rotation)
        self.assertLess(max(errors), .002)
        self.assertGreater(max(np.ptp(tips, axis=0)), .015)
        self.assertFalse(self.design['physical_tool_calibrated'])
        self.assertFalse(self.design['hardware_accepted'])
        np.testing.assert_allclose(self.cfg['tcp_offset_m'], [.2, 0., 0.])

    def test_planar_pendulum_crosses_both_sides_without_an_azimuth_circle(self):
        reference = np.asarray(self.design['reference_rotation'])
        # Alternate-axis fixture: the plane is spanned by reference X/Y,
        # with reference Z as its normal. This is independent of the recipe.
        directions = []
        start, end = np.cumsum(self.cfg['beat_sections'])[:2]
        for beat in np.linspace(start, end-self.cfg['phase_pause_beats'], 401):
            q, _ = self.plan.at(self.at_beat(beat))
            _, rotation = self.fk(q)
            directions.append(reference.T @ rotation[:, 0])
        directions = np.asarray(directions)
        self.assertLess(max(abs(directions[:, 2])), .004)
        signed = np.degrees(np.arctan2(directions[:, 1], directions[:, 0]))
        self.assertGreater(signed.max(), 2.)
        self.assertLess(signed.min(), -2.)
        self.assertLessEqual(max(abs(signed)), self.design['selected']['sampled_envelope_cone_deg']+.2)
        # The envelope allowance applies to the rebound peak as well as the
        # nominal target; 75% with a 3% full-stroke overshoot peaks at 79.5%.
        self.assertLessEqual(max(abs(signed))/self.design['selected']['sampled_envelope_cone_deg'], .8)

    def test_exact_freezes_seams_and_motion_budgets(self):
        freezes = 0
        for index, (role, curve) in enumerate(self.segment.stages):
            if role == 'freeze':
                freezes += 1
                self.assertGreater(curve.duration, 0.)
                for t in np.linspace(0., curve.duration, 7):
                    np.testing.assert_allclose(curve.at(t)[0], curve.start, atol=1e-12)
                    np.testing.assert_allclose(curve.at(t)[1], 0., atol=1e-12)
                    np.testing.assert_allclose(curve.acceleration_at(t), 0., atol=1e-12)
            if index:
                left = self.segment.stages[index-1][1]
                np.testing.assert_allclose(left.end, curve.start, atol=1e-8)
                np.testing.assert_allclose(left.at(left.duration)[1], curve.at(0.)[1], atol=1e-8)
                np.testing.assert_allclose(left.acceleration_at(left.duration), curve.acceleration_at(0.), atol=1e-7)
            if hasattr(curve, 'curve'):
                for t in np.unique(curve.curve.spline.t):
                    if 0. < t < curve.duration:
                        np.testing.assert_allclose(curve.jerk_at(t-1e-7), curve.jerk_at(t+1e-7), atol=.005)
        self.assertEqual(freezes, 7)  # Three phrase pauses and four beat freezes.
        self.assertLessEqual(max(self.plan.peak_velocity), 1.+1e-8)
        self.assertLessEqual(max(self.plan.peak_acceleration), 2.+1e-8)
        for t in (0., self.plan.duration):
            np.testing.assert_allclose(self.plan.at(t)[0], self.idle, atol=1e-8)
            np.testing.assert_allclose(self.plan.at(t)[1], 0., atol=1e-8)
            np.testing.assert_allclose(self.plan.acceleration_at(t), 0., atol=1e-7)

    def test_default_trial_is_thirty_percent_and_only_21_is_scaled(self):
        import rs_dance_speed as speed
        other = object(); plans = {'21': self.plan, '01': other}
        self.assertEqual(speed.trial_speed_for(plans), .3)
        for requested, expected in ((None, .3), (.5, .5), (1., 1.)):
            changed = speed.apply_trial_speed(plans, requested)
            self.assertIs(changed['01'], other)
            self.assertAlmostEqual(changed['21'].duration, self.plan.duration/expected)
            for t in np.linspace(0., self.plan.duration, 31):
                q, v = self.plan.at(t); actual_q, actual_v = changed['21'].at(t/expected)
                np.testing.assert_allclose(actual_q, q, atol=1e-10)
                np.testing.assert_allclose(actual_v, v*expected, atol=1e-10)
                np.testing.assert_allclose(changed['21'].acceleration_at(t/expected),
                                           self.plan.acceleration_at(t)*expected**2, atol=1e-9)

    def test_fabricated_amplitude_and_timing_metadata_rejected(self):
        for key in ('actual_cone_deg', 'actual_bpm', 'actual_duration_s', 'dance_duration_s',
                    'requested_cone_deg', 'recommended_playback_speed'):
            for value in (self.design[key]+1., np.nan):
                changed = copy.deepcopy(self.spec)
                changed['virtual_pivot_design'][key] = value
                with self.subTest(key=key, value=value), self.assertRaisesRegex(ValueError, 'metadata'):
                    self.pivot.validate_virtual_pivot(self.plan, self.gravity, self.idle, changed)
        selected = self.design['selected']
        self.assertAlmostEqual(selected['cone_deg'], min(self.cfg['requested_cone_deg'],
                               self.cfg['envelope_fraction']*selected['sampled_envelope_cone_deg']))

    def test_per_phrase_beat_mapping_and_saved_timing_are_checked(self):
        self.assertTrue(self.design['variable_bpm'])
        self.assertEqual(self.design['actual_bpm_kind'], 'average_over_dance')
        for phase in self.segment.phase_ranges_s:
            for fraction in (0., .25, .5, .75, 1.):
                t = phase['start_s'] + fraction*(phase['end_s']-phase['start_s'])
                expected = phase['start_beat'] + fraction*(phase['end_beat']-phase['start_beat'])
                self.assertAlmostEqual(self.segment.beat_at(t), expected, places=9)
        changed = copy.deepcopy(self.spec)
        changed['virtual_pivot_design']['phase_time_factors'][1] += .1
        with self.assertRaisesRegex(ValueError, 'timing factors'):
            self.pivot.validate_virtual_pivot(self.plan, self.gravity, self.idle, changed)
        changed = copy.deepcopy(self.spec)
        changed['virtual_pivot_design']['phase_ranges_s'][1]['actual_bpm'] += 1.
        with self.assertRaisesRegex(ValueError, 'timing metadata'):
            self.pivot.validate_virtual_pivot(self.plan, self.gravity, self.idle, changed)

    def test_real_controller_default_speed_following_delay_retains_caps(self):
        import rs_dance_speed as speed
        from rs_gravity_play import Controller
        from rs_gravity_record import CAPS
        scaled = speed.apply_trial_speed({'21': self.plan})['21']
        for lag in (.04, .06):
            controller = Controller(self.gravity, gripper_velocity_limit=scaled.gripper_velocity_limit)
            controller.velocity_limit[:6] = scaled.velocity_limit
            for index, t in enumerate(np.arange(0., scaled.duration+.02, .02)):
                target, velocity = scaled.at(t)
                measured, _ = scaled.at(max(0., t-lag))
                command = controller.command(measured, 1_000_000_000+index*20_000_000,
                                             target, velocity)
                self.assertTrue(np.all(np.abs(command['estimated_torque_nm']) <= CAPS))


if __name__ == '__main__':
    unittest.main()
