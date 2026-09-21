"""Construction-only Stage 3.5 installation; original defaults remain intact."""
import unittest
import numpy as np
from treesim import rm65
from treesim.config import RobotParams
from scripts import grow_tree

class FixedInstallation(unittest.TestCase):
    def test_original_default(self):
        xyz,rpy,source=rm65.fixed_mount(RobotParams())
        self.assertEqual(xyz,rm65.FIXED_MOUNT_XYZ)
        self.assertEqual(rpy,rm65.FIXED_MOUNT_RPY)
        self.assertEqual(source,rm65.FIXED_MOUNT_SOURCE)

    def test_explicit_installation_is_copied_and_upright(self):
        rp=RobotParams(model='rm65',mount='fixed',rm_control='joint')
        rp.stage35_fixed_mount=[1.2,.7,0.,2.8]
        xyz,rpy,source=rm65.fixed_mount(rp)
        rp.stage35_fixed_mount[0]=99
        self.assertEqual(xyz,(1.2,.7,0.))
        self.assertEqual(rpy,(0.,0.,2.8))
        self.assertIn('Stage 3.5',source)

    def test_invalid_scope_and_values(self):
        for model,mount,mode in [('franka','ridgeback','static'),('rm65','ridgeback','joint'),('rm65','fixed','static')]:
            rp=RobotParams(model=model,mount=mount,rm_control=mode)
            rp.stage35_fixed_mount=(1.,0.,0.,0.)
            with self.assertRaises(ValueError):rm65.fixed_mount(rp)
        for v in [(1,2,3),(1,2,3,4,5),(1,0,np.nan,0),(1,0,0,np.inf)]:
            rp=RobotParams(model='rm65',mount='fixed',rm_control='joint');rp.stage35_fixed_mount=v
            with self.assertRaises(ValueError):rm65.fixed_mount(rp)

    def test_formal_cli_opt_in(self):
        a=grow_tree.parse_args(['--robot-model','rm65','--mount','fixed','--rm-control','joint',
                               '--stage35-fixed-mount','1.2','.7','0','2.8'])
        c=grow_tree.make_config(a)
        self.assertEqual(tuple(c.robot.stage35_fixed_mount),(1.2,.7,0.,2.8))
        self.assertIsNone(grow_tree.make_config(grow_tree.parse_args([])).robot.stage35_fixed_mount)
