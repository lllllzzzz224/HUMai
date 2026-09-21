import unittest
import numpy as np
import tempfile
from types import SimpleNamespace as N
from treesim.rm65_official import GripperAdapter

class GraspCenterTests(unittest.TestCase):
    def test_strategy_revalidation_uses_adapter_center(self):
        from treesim.rm65_official_strategy import LocalGraspStrategy
        from treesim.rm65_run_config import load_config
        for offset,want in [(0.,[1,2,3]),(-.02,[1,2,3.02])]:
            with tempfile.TemporaryDirectory() as d:
                s=LocalGraspStrategy(d,None,config=load_config());plans=[]
                try:
                    s.state='REVALIDATE';s.r=N(frames=0)
                    s.association=N(accept=lambda sample:None)
                    s.gripper=GripperAdapter(self.runtime(),grasp_center_local=(0,0,offset))
                    s.robot=N(actual=lambda:np.eye(4),plan=lambda point,label:plans.append((point,label)))
                    for i in (1,2,3):s.after_camera_sample(s.r,N(sample_id=i,simulation_time=i/5,detections_world=[N(center_world=[1,2,3])]))
                    self.assertEqual(s.state,'GRASP');self.assertEqual(plans[0][1],'GRASP')
                    np.testing.assert_allclose(plans[0][0],want)
                finally:s.close_files()
    def runtime(self):
        self.calls=[]
        return N(audit=N(body_ids={'gripper_tcp':42}),sim=N(apples=N(hold=lambda i,b:self.calls.append((i,b)))))
    def test_default_preserves_original_center(self):
        g=GripperAdapter(self.runtime())
        self.assertTrue(hasattr(g,'grasp_center_local'))
        self.assertEqual(g.grasp_center_local,(0.,0.,0.))
        np.testing.assert_allclose(g.tcp_target_for_fruit([1,2,3],np.eye(4)),[1,2,3])
    def test_rotated_visual_goal_and_hold_share_one_center(self):
        self.assertTrue(hasattr(GripperAdapter,'tcp_target_for_fruit'))
        g=GripperAdapter(self.runtime(),grasp_center_local=(0,0,-.02))
        pose=np.eye(4);pose[:3,:3]=[[0,0,1],[0,1,0],[-1,0,0]]
        target=g.tcp_target_for_fruit([1,2,3],pose)
        np.testing.assert_allclose(target,[1.02,2,3])
        pose[:3,3]=target;np.testing.assert_allclose(g.grasp_center_world(pose),[1,2,3])
        g.hold(27,distance=.004,closed=True,stable=True)
        np.testing.assert_allclose(list(g.r.sim.apples.hold_off),[0,0,-.02],atol=1e-9)
        self.assertEqual(self.calls,[(27,42)])
    def test_invalid_centers_fail_closed(self):
        self.assertTrue(hasattr(GripperAdapter,'tcp_target_for_fruit'))
        for center in [(0,0),[0,0,float('nan')],[0,0,float('inf')]]:
            with self.assertRaises(ValueError):GripperAdapter(self.runtime(),grasp_center_local=center)
    def test_offset_is_readonly_alias_not_second_configuration(self):
        g=GripperAdapter(self.runtime())
        self.assertTrue(hasattr(g,'grasp_center_local'))
        with self.assertRaises(AttributeError):g.offset=(0,0,-.04)
        with self.assertRaises(AttributeError):g.grasp_center_local=(0,0,-.04)
