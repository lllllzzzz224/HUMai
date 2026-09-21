import unittest

import numpy as np

from continuous_grasp_state import (
    AppleStabilityGate,
    ContinuousSession,
    ContinuousSettings,
    SessionState,
)


class StabilityGateTest(unittest.TestCase):
    def setUp(self):
        self.settings = ContinuousSettings(
            max_rounds=0,
            min_position_change_m=0.030,
            stable_time_s=2.0,
            safety_countdown_s=3.0,
            retry_delay_s=2.0,
            max_pre_motion_retries=3,
            max_axis_span_m=0.010,
        )
        self.gate = AppleStabilityGate(self.settings)
        self.apple = np.asarray([0.40, -0.20, -0.05])

    def test_first_round_requires_stability_and_countdown(self):
        self.assertEqual(
            self.gate.observe(self.apple, 0.0).state,
            "STABILITY_CHECK",
        )
        self.assertEqual(
            self.gate.observe(self.apple, 2.1).state,
            "SAFETY_COUNTDOWN",
        )
        ready = self.gate.observe(self.apple, 5.1)
        self.assertEqual(ready.state, "READY")
        self.assertTrue(ready.ready)

    def test_cli_max_rounds_overrides_yaml(self):
        settings = ContinuousSettings.from_mapping(
            {
                "max_rounds": 0,
                "min_position_change_m": 0.030,
                "stable_time_s": 2.0,
                "safety_countdown_s": 3.0,
                "retry_delay_s": 2.0,
                "max_pre_motion_retries": 3,
            },
            max_axis_span_m=0.010,
            cli_max_rounds=2,
        )
        self.assertEqual(settings.max_rounds, 2)

    def test_released_position_must_move_before_next_round(self):
        self.gate.after_release(self.apple)
        blocked = self.gate.observe(self.apple + [0.010, 0.0, 0.0], 10.0)
        self.assertEqual(blocked.state, "WAIT_NEW_APPLE")
        self.assertFalse(blocked.ready)
        allowed = self.gate.observe(self.apple + [0.040, 0.0, 0.0], 11.0)
        self.assertEqual(allowed.state, "STABILITY_CHECK")

    def test_jitter_beyond_span_restarts_stability_timer(self):
        self.gate.observe(self.apple, 0.0)
        reset = self.gate.observe(self.apple + [0.011, 0.0, 0.0], 1.9)
        self.assertEqual(reset.stable_elapsed_s, 0.0)
        self.assertEqual(
            self.gate.observe(self.apple + [0.011, 0.0, 0.0], 3.0).state,
            "STABILITY_CHECK",
        )

    def test_missing_target_restarts_stability_timer(self):
        self.gate.observe(self.apple, 0.0)
        self.gate.missing()
        restarted = self.gate.observe(self.apple, 3.0)
        self.assertEqual(restarted.stable_elapsed_s, 0.0)
        self.assertEqual(restarted.state, "STABILITY_CHECK")

    def test_rejected_target_must_move_before_retry(self):
        self.gate.reject(self.apple)
        blocked = self.gate.observe(self.apple + [0.005, 0.0, 0.0], 0.0)
        self.assertEqual(blocked.state, "WAIT_NEW_APPLE")


class ContinuousSessionTest(unittest.TestCase):
    def test_max_rounds_stops_after_requested_count(self):
        session = ContinuousSession(max_rounds=2)
        session.round_completed()
        self.assertEqual(session.state, SessionState.WAIT_NEW_APPLE)
        session.round_completed()
        self.assertEqual(session.state, SessionState.COMPLETE)

    def test_pre_motion_failure_returns_to_wait_state(self):
        session = ContinuousSession(max_rounds=0)
        session.fail("planning failed")
        self.assertEqual(session.state, SessionState.WAIT_NEW_APPLE)
        self.assertFalse(session.terminal)

    def test_failure_after_motion_is_latched(self):
        session = ContinuousSession(max_rounds=0)
        session.motion_started()
        session.fail("execution failed")
        self.assertEqual(session.state, SessionState.FAULT_LATCHED)
        self.assertTrue(session.terminal)

    def test_operator_stop_is_terminal(self):
        session = ContinuousSession(max_rounds=0)
        session.stop("Ctrl+C")
        self.assertEqual(session.state, SessionState.STOPPED_BY_OPERATOR)
        self.assertTrue(session.terminal)


if __name__ == "__main__":
    unittest.main()
