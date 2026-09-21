"""Stage 3.5 planning consumes unambiguous sensor output without truth access."""
import unittest
from types import SimpleNamespace

import numpy as np

from scripts.validate_rm65_stage35 import unique_detection_center, joint_margin


class NaturalDetectionSelection(unittest.TestCase):
    def test_only_raw_detection_is_required_and_result_is_owned(self):
        center = np.array([.78, .46, .66])
        selected = unique_detection_center((SimpleNamespace(center_world=center),))
        np.testing.assert_array_equal(selected, center)
        center[:] = 99
        np.testing.assert_array_equal(selected, [.78, .46, .66])

    def test_missing_or_multiple_detections_fail_without_truth_ranking(self):
        item = SimpleNamespace(center_world=np.zeros(3))
        for detections in ((), (item, item)):
            with self.assertRaisesRegex(RuntimeError, 'DETECTION_AMBIGUITY'):
                unique_detection_center(detections)

    def test_invalid_center_is_rejected(self):
        for center in ([0, 1], [0, np.nan, 1], [0, np.inf, 1]):
            with self.assertRaisesRegex(RuntimeError, 'INVALID_DETECTION_CENTER'):
                unique_detection_center((SimpleNamespace(center_world=center),))

    def test_margin_checks_actual_coordinates_without_wrapping(self):
        kin = SimpleNamespace(soft_lower=np.full(6, -1.), soft_upper=np.full(6, 1.))
        np.testing.assert_array_equal(joint_margin(kin, np.zeros(6)), np.ones(6))
        for q in (np.full(6, .95), np.full(6, -1.1), np.full(6, np.nan)):
            with self.assertRaisesRegex(RuntimeError, 'JOINT_MARGIN'):
                joint_margin(kin, q)
