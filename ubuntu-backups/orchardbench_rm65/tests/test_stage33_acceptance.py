import unittest
import numpy as np
from tests.test_stage33_safety import scene, buffers
from treesim.rm65_runtime import RMRuntime
from treesim.rm65_planning import PlanningFailure
from treesim.rm65_acceptance import EnvironmentClearance as Clearance, arrived, repeatability, ArrivalWindow

class ArrivalTests(unittest.TestCase):

    def test_tcp_and_speed_gate_including_nonfinite(self):
        self.assertTrue(arrived(0.004, 0.015, np.zeros(6)))
        self.assertFalse(arrived(0.0051, 0.015, np.zeros(6)))
        self.assertFalse(arrived(0.004, 0.018, np.zeros(6)))
        v = np.zeros(6)
        v[4] = 0.00501
        self.assertFalse(arrived(0.004, 0.015, v))
        v[4] = np.nan
        self.assertFalse(arrived(0.004, 0.015, v))

    def test_arrival_requires_45_consecutive_samples_and_resets(self):
        gate = ArrivalWindow()
        for _ in range(44):
            self.assertFalse(gate.update(0.004, 0.015, np.zeros(6)))
        self.assertFalse(gate.update(0.006, 0.015, np.zeros(6)))
        for _ in range(44):
            self.assertFalse(gate.update(0.004, 0.015, np.zeros(6)))
        self.assertTrue(gate.update(0.004, 0.015, np.zeros(6)))
        gate.reset()
        self.assertFalse(gate.update(0.004, 0.015, np.zeros(6)))

    def test_rejects_wrong_velocity_shape(self):
        self.assertFalse(arrived(0.004, 0.015, np.zeros(5)))

    def endpoints(self, errors, spread=0):
        rows = []
        for stage in ('pre_grasp', 'grasp', 'retract', 'home'):
            for i, e in enumerate(errors):
                pose = np.eye(4)
                pose[0, 3] = i * spread
                rows.append(dict(stage=stage, actual_pose_world=pose.tolist(), position_error=e, rotation_error=0.005))
        return rows

    def test_rejects_growing_errors_even_inside_task_tolerance(self):
        with self.assertRaises(PlanningFailure):
            repeatability(self.endpoints([0.003, 0.0031, 0.0032]))

    def test_rejects_endpoint_spread_and_accepts_stable_sequence(self):
        with self.assertRaises(PlanningFailure):
            repeatability(self.endpoints([0.003, 0.0029, 0.003], 0.0011))
        self.assertEqual(len(repeatability(self.endpoints([0.003, 0.0029, 0.003], 0.0001))), 4)

class ClearanceTests(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.tree, cls.sim = scene()
        cls.r = RMRuntime(cls.tree, cls.sim)

    def setUp(self):
        self.before = buffers(self.sim)
        self.audit = Clearance(self.r)
        self.state = self.sim.model.state()
        self.state.assign(self.sim.state_0)

    def tearDown(self):
        self.assertEqual(self.before, buffers(self.sim))

    def move(self, body, delta):
        p = self.state.body_q.numpy()
        p[body, :3] += delta
        self.state.body_q.assign(p)

    def test_exact_installation_exemption_and_no_live_writes(self):
        row, d = self.audit.check(self.state, 'test')
        self.assertGreater(row['distance_m'], 0.005)
        self.assertAlmostEqual(d, 0, places=7)
        a, b = self.audit.install
        self.assertEqual(self.audit.n.body_label[self.audit.sb[self.audit.gmap[a]]], 'rm65_with_gripper_newton/base_link')
        self.assertEqual(self.audit.n.shape_label[self.audit.gmap[b]], 'ground_plane')

    def test_installation_small_penetration_allowed_macro_rejected(self):
        self.move(self.r.audit.body_ids['base_link'], [0, 0, -0.0001])
        self.audit.check(self.state, 'test')
        self.move(self.r.audit.body_ids['base_link'], [0, 0, -0.0002])
        with self.assertRaisesRegex(RuntimeError, 'INSTALLATION_PENETRATION'):
            self.audit.check(self.state, 'test')

    def test_link1_ground_below_5mm_rejected_without_penetration(self):
        self.move(self.r.audit.body_ids['Link1'], [0, 0, -0.1345])
        with self.assertRaisesRegex(RuntimeError, 'ENVIRONMENT_CLEARANCE'):
            self.audit.check(self.state, 'test')

    def test_base_tree_not_exempt(self):
        p = self.state.body_q.numpy()
        tree = next((i for i in range(len(p)) if i not in self.r.audit.audit_body_ids))
        p[tree, :3] = [1.8, 0, 0.06]
        self.state.body_q.assign(p)
        with self.assertRaisesRegex(RuntimeError, 'ENVIRONMENT_CLEARANCE'):
            self.audit.check(self.state, 'test')

    def test_ridgeback_scope_rejected(self):
        old = self.tree.config.robot.mount
        try:
            self.tree.config.robot.mount = 'ridgeback'
            with self.assertRaisesRegex(ValueError, 'SCOPE'):
                Clearance(self.r)
        finally:
            self.tree.config.robot.mount = old

    def test_pruned_minimum_matches_exhaustive_signed_distances(self):
        row, _ = self.audit.check(self.state, 'test')
        brute = min((self.audit.distance(a, b) for a, b in self.audit.pairs))
        self.assertAlmostEqual(row['distance_m'], brute, places=10)
if __name__ == '__main__':
    unittest.main(verbosity=2, failfast=True)
