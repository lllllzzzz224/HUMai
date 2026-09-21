import unittest
from pathlib import Path

from grasp_cli import parse_cli, resolve_run_mode


CFG = {
    "continuous": {
        "max_rounds": 0,
        "min_position_change_m": 0.030,
        "stable_time_s": 2.0,
        "safety_countdown_s": 3.0,
        "retry_delay_s": 2.0,
        "max_pre_motion_retries": 3,
    },
    "vision": {"max_axis_span_m": 0.010},
}


class GraspCliTest(unittest.TestCase):
    def parse(self, words):
        args, ros_args = parse_cli(words, Path("config.yaml"))
        self.assertEqual(ros_args, [])
        return args

    def test_old_fully_autonomous_remains_single_round(self):
        mode = resolve_run_mode(
            self.parse(
                ["--execute", "--operator-confirmed", "--fully-autonomous"]
            ),
            CFG,
        )
        self.assertFalse(mode.continuous)
        self.assertTrue(mode.auto_grasp)
        self.assertTrue(mode.auto_return)
        self.assertEqual(mode.max_rounds, 1)

    def test_continuous_requires_both_real_robot_gates(self):
        for words in (["--continuous"], ["--continuous", "--execute"]):
            with self.subTest(words=words), self.assertRaisesRegex(
                RuntimeError, "--execute --operator-confirmed"
            ):
                resolve_run_mode(self.parse(words), CFG)

    def test_continuous_rejects_old_mode_flags(self):
        words = [
            "--continuous",
            "--execute",
            "--operator-confirmed",
            "--fully-autonomous",
        ]
        with self.assertRaisesRegex(RuntimeError, "不能组合"):
            resolve_run_mode(self.parse(words), CFG)

    def test_cli_round_limit_overrides_yaml(self):
        words = [
            "--continuous", "--execute", "--operator-confirmed",
            "--max-rounds", "2",
        ]
        mode = resolve_run_mode(self.parse(words), CFG)
