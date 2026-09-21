"""Regression: mechanical limit fixtures must not inherit orchard ground."""
import unittest
import contextlib
import io
import json
from unittest.mock import patch
import numpy as np
from scripts import validate_rm65_control as v


class IsolationRegression(unittest.TestCase):
    def test_installation_gate_reports_mount_even_if_finger_contact_is_first(self):
        v.run_c2()

    def test_installation_rejected_before_controller_takeover(self):
        # Sentinel isolates only actuator takeover; real model/collision audit runs.
        with patch.object(v.rc, 'JointController', side_effect=AssertionError('controller took over unsafe pose')):
            v.run_c2()

    def test_failure_retains_second_substep_sample(self):
        f = v.mechanical_fixture()
        f.trace_substeps = True
        original = f.controller.check_state
        calls = 0
        def stop_on_second(state):
            nonlocal calls
            calls += 1
            result = original(state)
            if calls == 2:
                raise RuntimeError('injected safety stop')
            return result
        captured = io.StringIO()
        with patch.object(f.controller, 'check_state', side_effect=stop_on_second):
            with contextlib.redirect_stdout(captured):
                with self.assertRaisesRegex(RuntimeError, 'injected safety stop'):
                    f.step_frame()
        rows = [json.loads(line) for line in captured.getvalue().splitlines() if line.startswith('{')]
        samples = [r for r in rows if r.get('event') == 'physics_sample']
        self.assertEqual(len(samples), 2, 'failing substep was lost from trace')
        self.assertEqual(samples[-1]['physics_steps'], 2)
        self.assertEqual(f.physics_steps, 2)
        self.assertEqual(samples[-1]['substep'], 1)

    def test_mechanical_fixture_has_no_environment_shapes(self):
        # Fall back to the old C factory to demonstrate the original defect.
        factory = getattr(v, 'mechanical_fixture', v.Fixture)
        f = factory()
        self.assertFalse(np.any(f.m.shape_body.numpy() < 0),
                         'old C mixes ground/environment shapes into mechanical limit validation')

    def test_joint2_environment_rejection_is_not_a_mechanical_limit_failure(self):
        factory = getattr(v, 'mechanical_fixture', v.Fixture)
        q = np.zeros(8); q[1] = -2.258
        try:
            factory(initial=q)
        except RuntimeError as exc:
            self.assertNotIn('ground', str(exc),
                             'old C rejects a mechanical-limit pose because of orchard ground')
            # Self collision is a legitimate separate C3 outcome, not a green
            # mechanical-limit claim; the C3 runner must stop and report it.


if __name__ == '__main__':
    unittest.main()
