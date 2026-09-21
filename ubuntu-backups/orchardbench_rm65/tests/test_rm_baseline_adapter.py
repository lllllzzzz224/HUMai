import unittest
import importlib
from types import SimpleNamespace
import numpy as np
from scripts import grow_tree
from treesim.picker import AutoPicker

class BaselineAdapter(unittest.TestCase):
    def module(self):return importlib.import_module('treesim.rm65_picker')

    def test_core_strategy_methods_are_inherited_not_reimplemented(self):
        c=self.module().RMBaselinePicker
        for name in ['_st_reach','_st_grasp','_st_pull']:
            self.assertIs(getattr(c,name),getattr(AutoPicker,name))

    def test_explicit_mode_and_default_preservation(self):
        a=grow_tree.parse_args(['--robot-model','rm65','--mount','fixed','--rm-control','joint','--rm-auto-baseline','--seed','31'])
        c=grow_tree.make_config(a)
        self.assertTrue(c.robot.rm_auto_baseline);self.assertTrue(c.robot.rm_camera);self.assertTrue(c.fruit.enabled)
        self.assertFalse(grow_tree.make_config(grow_tree.parse_args([])).robot.rm_auto_baseline)
        a=grow_tree.parse_args(['--rm-auto-baseline'])
        with self.assertRaises(ValueError):grow_tree.make_config(a)

    def test_hold_uses_tcp_origin_without_changing_spring_parameters(self):
        a=SimpleNamespace(hold_off=None,hold_k=500.,hold_c=12.3)
        self.module().configure_rm_hold(a)
        np.testing.assert_array_equal(np.array(a.hold_off),[0,0,0])
        self.assertEqual((a.hold_k,a.hold_c),(500.,12.3))

    def test_gripper_adapter_sends_real_two_finger_control(self):
        c=self.module().RMBaselinePicker.__new__(self.module().RMBaselinePicker)
        seen=[];c.runtime=SimpleNamespace(controller=SimpleNamespace(set_gripper_open_fraction=seen.append))
        c._q_goal=np.zeros(8);c._q_cmd=np.zeros(8)
        c._fingers(c.FINGER_OPEN);c._fingers(c.FINGER_CLOSED)
        self.assertEqual(seen,[1.,0.]);np.testing.assert_array_equal(c._q_goal[-2:],[.06,.06])

    def test_failure_never_sends_recovery_target(self):
        c=self.module().RMBaselinePicker.__new__(self.module().RMBaselinePicker)
        c.runtime=SimpleNamespace(stopped=False);c.state='REACH';c.done=False;c._frame=3
        with self.assertRaisesRegex(RuntimeError,'IK_FAILED'):c._fail('IK_FAILED')
        self.assertTrue(c.runtime.stopped);self.assertTrue(c.done)

    def test_tracks_only_consume_fresh_samples(self):
        c=self.module().RMBaselinePicker.__new__(self.module().RMBaselinePicker)
        c.cam=SimpleNamespace(last_sample=SimpleNamespace(sample_id=7,detections_world=()))
        c._last_sample_id=7;c._track=[[np.zeros(3),3,0]]
        c._update_tracks();self.assertEqual(c._track[0][1],3)
