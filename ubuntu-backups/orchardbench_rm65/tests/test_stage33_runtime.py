"""Stage 3.3 formal RM entry contracts; no harvesting side effects."""
import unittest
from scripts import grow_tree


class EntryContracts(unittest.TestCase):
    def test_static_default_preserves_franka(self):
        a = grow_tree.parse_args([])
        self.assertEqual(getattr(a, 'rm_control', None), 'static')
        c = grow_tree.make_config(a)
        self.assertEqual((c.robot.model, c.robot.mount), ('franka', 'ridgeback'))

    def test_joint_rejects_franka_before_build(self):
        a = grow_tree.parse_args([]); a.rm_control = 'joint'
        with self.assertRaises(ValueError):
            grow_tree.make_config(a)

    def test_joint_rejects_wrong_solver_substeps_apples_warmup(self):
        for attr, value in [('mj_solver', 'newton'), ('solver', 'xpbd'),
                            ('substeps', 2), ('apples', True), ('warmup', 1), ('num_envs', 2)]:
            with self.subTest(attr=attr):
                a = grow_tree.parse_args(['--robot-model','rm65','--mount','fixed'])
                a.rm_control = 'joint'; setattr(a, attr, value)
                with self.assertRaises(ValueError):
                    grow_tree.make_config(a)

    def test_joint_selects_no_camera(self):
        a = grow_tree.parse_args(['--robot-model','rm65','--mount','fixed'])
        a.rm_control = 'joint'
        c = grow_tree.make_config(a)
        self.assertFalse(c.robot.camera)
        self.assertEqual(getattr(c.robot, 'rm_control', None), 'joint')
