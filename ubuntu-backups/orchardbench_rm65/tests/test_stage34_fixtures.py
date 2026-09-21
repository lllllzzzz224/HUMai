import importlib.util
import unittest
from pathlib import Path
from types import SimpleNamespace
import numpy as np

class Fixtures(unittest.TestCase):
    def api(self):
        name='scripts.validate_rm65_camera_fixtures'
        self.assertIsNotNone(importlib.util.find_spec(name), 'finite fixture validator missing')
        from scripts import validate_rm65_camera_fixtures as f
        return f
    def row(self, i=0, apple='apple1', x=0.):
        t=np.eye(4);t[0,3]=x
        return dict(candidate=i,apple=apple,eligible=True,T_world_camera=t.tolist())
    def test_fixed_grid(self):
        f=self.api();self.assertEqual(f.CANDIDATES,tuple((a,-.8,-.3,0.,b,0.) for a in [-.50,-.45,-.40,-.35,-.30] for b in [-.30,-.20,-.10]))
    def test_order_and_same_apple(self):
        f=self.api();rows=[self.row(0,'apple2'),self.row(0,'apple1'),self.row(1,'apple2',.02),self.row(2,'apple1',.02)]
        for data in [rows,rows[::-1]]:
            a,b=f.select_pair(data);self.assertEqual((a['apple'],a['candidate'],b['candidate']),('apple1',0,2))
        self.assertIsNone(f.select_pair([self.row(0,'apple1'),self.row(1,'apple2',.02)]))
    def test_first_qualified_apple_is_not_replaced_when_no_second_pose(self):
        f=self.api();self.assertIsNone(f.select_pair([self.row(0,'apple1'),self.row(0,'apple2'),self.row(1,'apple2',.02)]))
    def test_camera_change_required(self):
        f=self.api();self.assertIsNone(f.select_pair([self.row(),self.row(1,x=.009)]))
    def test_mixed_component_rejected(self):
        f=self.api();labels=np.array([[1,1,1],[0,0,0]]);hits=np.array([[366,366,365],[0,0,0]])
        self.assertFalse(f.component_purity(labels,hits,366)['pure'])
        self.assertTrue(f.component_purity(labels,np.where(hits==365,366,hits),366)['pure'])
    def test_truth_never_calls_detector(self):
        f=self.api()
        class Forbidden:
            def detect(self,*a,**k):raise AssertionError('scoring called detector')
        p=Forbidden();p.dirs=np.zeros((2,2,3));p.min_range=.12;p.max_range=1.35;p.edge_jump=.015
        s=SimpleNamespace(shape_index=np.zeros((2,2),int),depth=np.ones((2,2)),detection_depth=np.ones((2,2)),self_mask=np.zeros((2,2),bool),T_world_camera=np.eye(4),detections_world=())
        f.score_apple(s,p,np.array([0]),1,np.array([0,0,-1.]),.032,1,'apple1')
    def test_formal_provenance_and_unique_fresh_samples(self):
        f=self.api();rows=[dict(sample_id=i,simulation_time=float(i)) for i in [42,43,44]]
        f.require_formal_samples(rows,phase='formal',process_id=20,discovery_process_ids=[10])
        for change in [dict(phase='discovery',process_id=20),dict(phase='formal',process_id=10)]:
            with self.assertRaises(ValueError):f.require_formal_samples(rows,discovery_process_ids=[10],**change)
        for ids in [[42,42,43],[43,42,44]]:
            with self.assertRaises(ValueError):f.require_formal_samples([dict(sample_id=i,simulation_time=float(i)) for i in ids],phase='formal',process_id=20,discovery_process_ids=[10])
    def test_formal_failure_stops_without_fallback(self):
        f=self.api();calls=[]
        def run(p):calls.append(p);raise RuntimeError('FAIL')
        with self.assertRaises(RuntimeError):f.run_pair((self.row(),self.row(1,x=.02)),run)
        self.assertEqual(len(calls),1)
    def test_historical_miss_preserved(self):
        f=self.api();h=f.HISTORICAL_MISS
        self.assertEqual(h['apple'],'apple35');self.assertEqual(h['elongation_limit'],3.)
        self.assertEqual(h['elongation'],3.999615325038144)
        self.assertEqual(h['result'],'MISS');self.assertFalse(h['migration_error'])
        self.assertTrue(Path(h['sample']).is_file())

if __name__=='__main__':unittest.main()
