"""Stage 3.5 slew deadline regression; Stage 3.4 remains unchanged."""
import unittest
import importlib
import numpy as np

class ObservationBudget(unittest.TestCase):
    def budget(self,start,target):
        m=importlib.import_module('scripts.validate_rm65_stage35')
        return m.observation_budget(start,target)

    def test_previous_timeout_has_slew_and_separate_settling_budget(self):
        q=[-.4461756944656372,1.2990716695785522,-.9880963563919067,.2896624803543091,-1.523876428604126,.09278970211744308]
        b=self.budget(np.zeros(6),q)
        self.assertEqual(b['slew_frames'],915)
        self.assertEqual(b['settling_frames'],600)
        self.assertEqual(b['ready_deadline_frames'],1515)
        self.assertEqual(b['acquisition_frames'],36)
        self.assertEqual(b['total_frames'],1551)

    def test_zero_negative_and_multi_axis_use_max_per_axis_time(self):
        for start,target,frames in [(np.zeros(6),np.zeros(6),0),([.3]*6,[-.2]*6,300),([0]*6,[.1,-.2,.3,-.4,.8,0],480)]:
            with self.subTest(target=target):self.assertEqual(self.budget(start,target)['slew_frames'],frames)

    def test_nonfinite_or_wrong_shape_rejected_without_mutation(self):
        for target in [[0]*5,[0]*5+[float('nan')],[0]*5+[float('inf')]]:
            start=np.zeros(6);saved=start.copy()
            with self.assertRaises(ValueError):self.budget(start,target)
            np.testing.assert_array_equal(start,saved)

    def test_stage34_original_deadline_still_rejects_at_600(self):
        from scripts.validate_rm65_camera import ObservationCase
        from types import SimpleNamespace
        p=ObservationCase(SimpleNamespace(log_dir=None))
        with self.assertRaisesRegex(RuntimeError,'10 seconds'):
            p.after_frame(SimpleNamespace(frames=600))
