import inspect
import unittest

from single_apple_full_grasp import (
    continuous_main,
    execute_prepared_round,
    mark_motion_started,
    prepare_round,
)


class RoundPhaseBoundaryTest(unittest.TestCase):
    def test_motion_flag_is_false_until_first_execute(self):
        record = {"motion_started": False}
        self.assertFalse(record["motion_started"])
        mark_motion_started(record)
        self.assertTrue(record["motion_started"])

    def test_prepare_phase_contains_no_motion_or_gripper_command(self):
        source = inspect.getsource(prepare_round)
        self.assertNotIn("node.execute(", source)
        self.assertNotIn("set_opening_mm(", source)

    def test_execute_phase_preserves_validated_order(self):
        source = inspect.getsource(execute_prepared_round)
        labels = [
            'mark_motion_started(record)',
            'node.execute(pre_traj, "SCAN -> PREGRASP")',
            'node.execute(verify_traj, "PREGRASP -> VERIFY")',
            'node.execute(grasp_traj, "VERIFY -> GRASP")',
            'set_opening_mm(',
            'node.execute(lift_traj, "GRASP -> LIFT")',
            'node.execute(release_traj, "LIFT -> RELEASE")',
            'node.execute(retreat_traj, "RELEASE -> PREGRASP")',
            'node.execute(scan_trajectory, "PREGRASP -> SCAN")',
        ]
        positions = [source.index(label) for label in labels]
        self.assertEqual(positions, sorted(positions))

    def test_continuous_loop_has_no_enter_and_latches_before_execute(self):
        source = inspect.getsource(continuous_main)
        self.assertNotIn("input(", source)
        self.assertIn(
            "motion_start_callback=session.motion_started",
            source,
        )
        execute_source = inspect.getsource(execute_prepared_round)
        labels = (
            "motion_start_callback()",
            "mark_motion_started(record)",
            'node.execute(pre_traj, "SCAN -> PREGRASP")',
        )
        positions = [execute_source.index(label) for label in labels]
        self.assertEqual(positions, sorted(positions))
        self.assertIn("gate.after_release", source)
        self.assertIn("SessionState.FAULT_LATCHED", source)


if __name__ == "__main__":
    unittest.main()
