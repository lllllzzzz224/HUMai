import importlib.util
import unittest
from types import SimpleNamespace as N
import numpy as np
from treesim.rm65_official import PerceptionAdapter

class FixtureDiagnosticTests(unittest.TestCase):
    def strategy(self):
        self.assertIsNotNone(importlib.util.find_spec('treesim.rm65_fixture_diagnostic'))
        from treesim.rm65_fixture_diagnostic import FixtureRevalidateOnly
        s=FixtureRevalidateOnly.__new__(FixtureRevalidateOnly)
        s.state='DETECT';s.samples=0;s.done=False;s.offset=np.array([0.,0.,-.08])
        s.perception=PerceptionAdapter();s.association=N(index=27,accept=lambda sample:None)
        s.capture=lambda *a:None;s.event=lambda *a,**k:None;s.screenshot=lambda *a:None
        s.met=N(pick_start=lambda *a:None);s.r=N(frames=0,stopped=False)
        s.plans=[]
        s.robot=N(actual=lambda:np.eye(4),plan=lambda point,label:s.plans.append((point,label)))
        s.finish=lambda result:setattr(s,'result',result)
        return s
    def sample(self,i,points=((.8,.6,1.),)):
        return N(sample_id=i,simulation_time=i*.2,detections_world=[N(center_world=np.array(p)) for p in points])
    def test_pregrasp_uses_latest_natural_center_not_fixture_coordinates(self):
        s=self.strategy()
        for i in (1,2):s.after_camera_sample(s.r,self.sample(i))
        self.assertEqual(s.plans,[])
        s.after_camera_sample(s.r,self.sample(3,((.81,.6,1.),)))
        np.testing.assert_allclose(s.plans[0][0],[.81,.6,.92]);self.assertEqual(s.plans[0][1],'PREGRASP')
    def test_multiple_detections_are_not_id_filtered(self):
        s=self.strategy()
        with self.assertRaises(RuntimeError):s.after_camera_sample(s.r,self.sample(1,((.8,.6,1.),(.2,.4,.9))))
        self.assertEqual(s.plans,[]);self.assertEqual(s.state,'DETECT')
    def test_revalidation_stops_without_grasp_plan(self):
        s=self.strategy();s.state='REVALIDATE'
        for i in (1,2,3):s.after_camera_sample(s.r,self.sample(i))
        self.assertTrue(s.done);self.assertEqual(s.state,'REVALIDATE_ONLY_DONE');self.assertEqual(s.plans,[])

if __name__=='__main__':unittest.main()
