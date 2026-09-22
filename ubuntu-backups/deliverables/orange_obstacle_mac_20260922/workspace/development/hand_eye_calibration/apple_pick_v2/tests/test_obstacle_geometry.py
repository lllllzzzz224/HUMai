import copy
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from obstacle_geometry import (depth_points, validate_observation, build_snapshot,
                               classify_points, scene_delta, cleared_cells, canonical_scene)


def record():
    return dict(capture_stamp_s=100., depth_stamp_s=100., depth_scale_m=.001,
                intrinsics=dict(fx=100., fy=100., ppx=1., ppy=1., width=3, height=3),
                frame_id='base_link', color_frame='camera_color_optical_frame',
                depth_frame='camera_color_optical_frame', camera_to_base=np.eye(4).tolist(),
                stationary=True, joint_span_deg=.002, bracket_s=.1,
                transform_provenance={'synchronized': True, 'source': 'controller bracket + FK + handeye + optical TF'},
                robot_model_sha256='a'*64, joint_positions_rad=[0.]*6)


class GeometryTests(unittest.TestCase):
    def test_depth_units_and_translation(self):
        t = np.eye(4); t[0, 3] = .2
        p = depth_points(np.array([[1000]], np.uint16),
                         dict(fx=100., fy=100., ppx=0., ppy=0.), .001, t)
        np.testing.assert_allclose(p, [[.2, 0., 1.]])

    def test_invalid_observations_are_rejected(self):
        for change in [dict(depth_scale_m=0), dict(depth_scale_m=float('nan')),
                       dict(capture_stamp_s=90.), dict(capture_stamp_s=101.),
                       dict(stationary=False), dict(joint_span_deg=.5),
                       dict(camera_to_base=None), dict(depth_frame='other'),
                       dict(transform_provenance={}), dict(bracket_s=float('nan'))]:
            with self.subTest(change=change), self.assertRaises(ValueError):
                validate_observation(record() | change, now_s=100., max_age_s=2.)
        r = record(); r['camera_to_base'][0][0] = 2.
        with self.assertRaises(ValueError):
            validate_observation(r, 100., 2.)

    def test_nonfinite_and_zero_depth_are_not_points(self):
        p = depth_points(np.array([[0., np.nan, np.inf, 1000.]]),
                         dict(fx=100., fy=100., ppx=0., ppy=0.), .001)
        np.testing.assert_allclose(p, [[.03, 0., 1.]])

    def test_surface_front_back_and_outside_have_distinct_states(self):
        d = np.full((3, 3), 1000, np.uint16)
        states = classify_points([[0, 0, .5], [0, 0, 1.], [0, 0, 1.1], [3, 0, 1.]], d, record(), {})
        self.assertEqual(states.tolist(), ['free', 'occupied', 'unknown', 'unknown'])
        d[1, 1] = 0
        self.assertEqual(classify_points([[0, 0, .5]], d, record(), {})[0], 'unknown')

    def test_out_of_range_depth_cannot_certify_a_free_ray(self):
        for raw in (50, 5000):
            d = np.full((3, 3), raw, np.uint16)
            self.assertEqual(classify_points([[0, 0, .5]], d, record(), {})[0], 'unknown')

    def test_target_mask_never_erases_branch_or_target_geometry(self):
        d = np.full((3, 3), 1000, np.uint16); d[1, 1] = 800
        mask = np.ones_like(d, bool)
        a = build_snapshot(d, record(), {}, mask)
        b = build_snapshot(d, record(), {})
        self.assertEqual(a['occupied_cells'], b['occupied_cells'])
        self.assertFalse(a['executable'])
        self.assertGreater(len(a['target_geometry']['observed_cell_ids']), 0)

    def test_absence_needs_positive_clear_evidence_and_foreign_ids_survive(self):
        old = {'occupied_cells': {'auto_obstacle_0_0_1': [0, 0, 1], 'foreign': [0, 0, 2]}}
        self.assertEqual(scene_delta(old, {'occupied_cells': {}}, set())['remove'], [])
        delta = scene_delta(old, {'occupied_cells': {}}, {'auto_obstacle_0_0_1', 'foreign'})
        self.assertEqual(delta['remove'], ['auto_obstacle_0_0_1'])

    def test_clearance_checks_entire_box_and_requires_valid_depth(self):
        r = record(); r['intrinsics'].update(fx=1., fy=1.)
        old = dict(occupied_cells={'auto_obstacle_0_0_1': [0., 0., .5]},
                   frame_id='base_link', box_size_m=.02)
        d = np.full((3, 3), 1000, np.uint16)
        self.assertEqual(cleared_cells(old, d, r, {}), {'auto_obstacle_0_0_1'})
        d[1, 1] = 0
        self.assertEqual(cleared_cells(old, d, r, {}), set())

    def test_digest_is_order_independent_and_detects_geometry_changes(self):
        a = [{'id':'a', 'size':.01}, {'id':'b', 'size':.02}]
        self.assertEqual(canonical_scene(a), canonical_scene(a[::-1]))
        b = copy.deepcopy(a); b[0]['size'] = .03
        self.assertNotEqual(canonical_scene(a), canonical_scene(b))


if __name__ == '__main__':
    unittest.main()
