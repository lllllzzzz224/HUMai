import importlib.util
import unittest
from types import SimpleNamespace as N
import numpy as np

class SeedDiagnosticTests(unittest.TestCase):
    def api(self):
        self.assertIsNotNone(importlib.util.find_spec('treesim.rm65_seed_diagnostic'))
        from treesim import rm65_seed_diagnostic as m
        return m
    def program(self):
        m=self.api();s=m.RevalidateOnly.__new__(m.RevalidateOnly)
        from treesim.rm65_official import PerceptionAdapter
        s.state='REVALIDATE';s.samples=0;s.done=False;s.r=N(frames=100,stopped=False)
        s.perception=PerceptionAdapter();s.association=N(accept=lambda sample:None)
        s.event=lambda *a,**k:None;s.capture=lambda *a:None;s.screenshot=lambda *a:None
        s.finish=lambda result:setattr(s,'result',result)
        return s
    def sample(self,i,centers=((.8,.4,.6),)):
        return N(sample_id=i,simulation_time=i*.2,detections_world=[N(center_world=np.array(x)) for x in centers])
    def test_revalidation_stops_without_grasp_interface(self):
        s=self.program()
        for i in (1,2):
            s.after_camera_sample(s.r,self.sample(i));self.assertFalse(s.done)
        s.after_camera_sample(s.r,self.sample(3))
        self.assertEqual(s.state,'REVALIDATE_ONLY_DONE');self.assertTrue(s.r.stopped);self.assertTrue(s.done)
    def test_missing_and_multiple_do_not_finish(self):
        for centers in ((),((.8,.4,.6),(.81,.4,.6))):
            s=self.program()
            with self.assertRaises(RuntimeError):s.after_camera_sample(s.r,self.sample(1,centers))
            self.assertFalse(s.done);self.assertEqual(s.state,'REVALIDATE')
    def test_stale_and_drift_do_not_finish(self):
        s=self.program();s.after_camera_sample(s.r,self.sample(1))
        with self.assertRaises(RuntimeError):s.after_camera_sample(s.r,self.sample(1))
        with self.assertRaises(RuntimeError):s.after_camera_sample(s.r,self.sample(2,((1.,.4,.6),)))
        self.assertFalse(s.done)
    def test_all_downstream_states_are_rejected(self):
        s=self.program()
        for state in ('GRASP','CLOSE','HOLD_CONFIRM','PULL','TRANSPORT','SCORE','DONE'):
            with self.assertRaises(RuntimeError):s.goto(state)
        self.assertEqual(s.state,'REVALIDATE')
    def test_ray_association_selects_without_truth_positions(self):
        m=self.api();runtime=N(sim=N(model=N(shape_body=N(numpy=lambda:np.array([3,7])),body_label=['','','','apple2','','','','apple9'])),tree=N(apple_data={'apple_body':[3,7]}))
        a=m.NaturalAssociation(runtime)
        sample=N(detections_world=[N(px=(1,1))],shape_index=np.zeros((3,3),dtype=np.uint32))
        a.accept(sample);self.assertEqual(a.name,'apple2');self.assertEqual(a.index,0)
        sample.shape_index[:]=1
        with self.assertRaises(RuntimeError):a.accept(sample)
    def test_repeat_cannot_switch_to_different_target(self):
        m=self.api();runtime=N(sim=N(model=N(shape_body=N(numpy=lambda:np.array([3])),body_label=['','','','apple2'])),tree=N(apple_data={'apple_body':[3]}))
        a=m.NaturalAssociation(runtime,expected='apple9')
        sample=N(detections_world=[N(px=(1,1))],shape_index=np.zeros((3,3),dtype=np.uint32))
        with self.assertRaises(RuntimeError):a.accept(sample)
    def test_terminal_result_requires_no_error(self):
        for error,failure in ((None,None),('save failed',None),(None,'render failed')):
            s=self.program();del s.r
            s.done=True;s.state='REVALIDATE_ONLY_DONE';s.failure=failure
            s.blocked_calls={};s.captured=[];s.save=lambda *a:None
            result=s.diagnostic_result(error)
            self.assertEqual(result['status'],'FAIL' if error or failure else 'PASS')

if __name__=='__main__':unittest.main()
