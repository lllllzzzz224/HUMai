"""Stage 3.2 A: structure and target safety; no physics stepping."""
import unittest
import numpy as np
import newton
from treesim import rm65
from treesim import rm65_control as rc


class StructureTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.builder = newton.ModelBuilder()
        rm65.build_fixed(cls.builder, None)
        rc.configure(cls.builder)
        cls.model = cls.builder.finalize(device="cpu")

    def controller(self):
        return rc.JointController(self.model, self.model.control())

    def test_order(self):
        self.assertEqual(rc.NAMES[:6], tuple(f"joint{i}" for i in range(1, 7)))

    def test_missing_and_duplicate_names(self):
        with self.assertRaises(ValueError):
            rc.resolve_names([], rc.NAMES)
        with self.assertRaises(ValueError):
            rc.resolve_names(list(rc.NAMES) + [rc.NAMES[0]], rc.NAMES)

    def test_position_limits(self):
        np.testing.assert_allclose(rc.description()["upper"][:6], [3.1, 2.268, 2.355, 3.1, 2.233, 6.28])

    def test_actual_monitor_uses_official_limits_not_command_limits(self):
        c = self.controller()
        state = self.model.state()
        # Synthetic monitor inputs only: no solver or motion occurs in this test.
        for axis in range(6):
            for side, sign in (('lower', -1), ('upper', 1)):
                q = self.model.joint_q.numpy().copy()
                hard = c.limits[side][axis]
                q[c.qs[axis]] = hard + sign * .00009
                state.joint_q.assign(q)
                c.check_state(state)  # soft-boundary crossing is not hard failure
                q[c.qs[axis]] = hard + sign * .00011
                state.joint_q.assign(q)
                with self.assertRaisesRegex(RuntimeError, 'joint limit violation'):
                    c.check_state(state)

    def test_commands_do_not_rewrite_compiled_hard_limits(self):
        c = self.controller()
        before = [a.numpy().copy() for a in (self.model.joint_limit_lower, self.model.joint_limit_upper)]
        np.testing.assert_allclose(before[0][c.ds], c.limits['lower'], atol=1e-6)
        np.testing.assert_allclose(before[1][c.ds], c.limits['upper'], atol=1e-6)
        for target in ([100]*6, [-100]*6, [.02]*6):
            c.set_arm_targets(target)
        np.testing.assert_array_equal(self.model.joint_limit_lower.numpy(), before[0])
        np.testing.assert_array_equal(self.model.joint_limit_upper.numpy(), before[1])
        np.testing.assert_allclose(c.goal[:6], [.02]*6)
        self.assertFalse(c.soft_clamped.any())

    def test_effort_limits(self):
        d = rc.description()
        self.assertTrue(np.all(rc.EFFORT <= d["effort"]))
        np.testing.assert_allclose(self.model.joint_effort_limit.numpy()[self.controller().ds], rc.EFFORT)

    def test_target_clamp(self):
        c = self.controller()
        self.assertTrue(c.set_arm_targets([100] * 6))
        np.testing.assert_allclose(c.goal[:6], [3.09, 2.258, 2.345, 3.09, 2.223, 6.27])

    def test_soft_clamp_twelve_directions_and_audit(self):
        bounds = [3.09, 2.258, 2.345, 3.09, 2.223, 6.27]
        for axis, bound in enumerate(bounds):
            for sign in (-1, 1):
                with self.subTest(axis=axis, sign=sign):
                    c = self.controller()
                    req = np.zeros(6); req[axis] = sign * (bound + .02)
                    self.assertTrue(c.set_arm_targets(req))
                    expected = np.zeros(6); expected[axis] = sign * bound
                    np.testing.assert_allclose(c.goal[:6], expected, atol=1e-12)
                    np.testing.assert_array_equal(c.raw_arm_request, req)
                    np.testing.assert_array_equal(c.effective_arm_target, c.goal[:6])
                    mask = np.zeros(6, dtype=bool); mask[axis] = True
                    np.testing.assert_array_equal(c.soft_clamped, mask)
                    # Caller mutation must not rewrite the audit record.
                    req[:] = 0
                    self.assertNotEqual(c.raw_arm_request[axis], 0)

    def test_invalid_request_preserves_previous_command_and_audit(self):
        c = self.controller()
        c.set_arm_targets([.02]*6)
        c.update(0)
        names = ('goal', 'sent', 'raw_arm_request', 'effective_arm_target', 'soft_clamped')
        self.assertTrue(all(hasattr(c, name) for name in names), 'command audit is missing')
        saved = {name: getattr(c, name).copy() for name in names}
        buffer = c.control.joint_target_q.numpy().copy()
        for bad in ([0]*5, [np.nan]*6, [np.inf]*6, [-np.inf]*6):
            with self.assertRaises(ValueError):
                c.set_arm_targets(bad)
            for name in names:
                np.testing.assert_array_equal(getattr(c, name), saved[name])
            np.testing.assert_array_equal(c.control.joint_target_q.numpy(), buffer)

    def test_nonfinite_and_dimension_rejected(self):
        c = self.controller()
        for values in ([0]*5, [np.nan]*6, [np.inf]*6):
            with self.assertRaises(ValueError):
                c.set_arm_targets(values)
        np.testing.assert_array_equal(c.goal, np.zeros(8))

    def test_gripper_mapping(self):
        c = self.controller()
        for fraction, q in ((0, .06), (.5, .03), (1, 0)):
            c.set_gripper_open_fraction(fraction)
            np.testing.assert_allclose(c.goal[6:], q)

    def test_gripper_clamp_and_finite(self):
        c = self.controller()
        self.assertTrue(c.set_gripper_open_fraction(-1))
        np.testing.assert_allclose(c.goal[6:], .06)
        self.assertTrue(c.set_gripper_open_fraction(2))
        np.testing.assert_array_equal(c.goal[6:], 0)
        with self.assertRaises(ValueError):
            c.set_gripper_open_fraction(float("nan"))

    def test_once_per_control_frame_and_target_slots(self):
        c = self.controller()
        before = c.control.joint_target_q.numpy()
        c.set_arm_targets([.05]*6)
        c.set_gripper_open_fraction(0)
        c.update(0)
        after = c.control.joint_target_q.numpy()
        np.testing.assert_allclose(after[c.ts], rc.RATE / 60, atol=1e-8)
        mask = np.ones(len(after), dtype=bool)
        mask[c.ts] = False
        np.testing.assert_array_equal(after[mask], before[mask])
        with self.assertRaises(ValueError):
            c.update(0)
        np.testing.assert_array_equal(c.control.joint_target_q.numpy(), after)

    def test_position_mode(self):
        c = self.controller()
        np.testing.assert_array_equal(self.model.joint_target_mode.numpy()[c.ds], int(newton.JointTargetMode.POSITION))

    def test_velocity_metadata(self):
        c = self.controller()
        np.testing.assert_allclose(self.model.joint_velocity_limit.numpy()[c.ds], c.limits["velocity"])

    def test_urdf_axes(self):
        np.testing.assert_array_equal(rc.description()["axis"], [[0,0,1]]*6 + [[0,-1,0], [0,1,0]])

    def test_imported_axes_match_source(self):
        c = self.controller()
        np.testing.assert_allclose(self.model.joint_axis.numpy()[c.ds], rc.description()['axis'], atol=1e-7)


if __name__ == "__main__":
    unittest.main()
