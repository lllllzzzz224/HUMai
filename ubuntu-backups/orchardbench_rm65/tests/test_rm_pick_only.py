import importlib.util
import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace as N
from unittest.mock import patch
import numpy as np

class PickOnlyTests(unittest.TestCase):
    def api(self):
        self.assertIsNotNone(importlib.util.find_spec('treesim.rm65_pick_only'))
        from treesim.rm65_pick_only import PickOnlyStrategy
        return PickOnlyStrategy
    def make(self,d):
        from treesim.rm65_run_config import load_config
        s=self.api()(d,config=load_config())
        s.r=N(frames=10,substeps=30,stopped=False,sim=N(apples=N(detached=[True],_held_host=[1]),frame_dt=1/60,substeps=3))
        s.association=N(index=0);s.screenshot=lambda name:None
        s.robot=N(settled=lambda:True,arm=lambda:(np.zeros(6),np.zeros(6)),check=lambda q:True,goal=np.eye(4))
        s.hold_samples=0
        return s
    def test_pick_only_entry_does_not_construct_bucket_or_sim(self):
        self.api()
        from treesim import rm65_run_entry as e
        from scripts import grow_tree
        with tempfile.TemporaryDirectory() as d,patch.object(e,'Bucket',side_effect=AssertionError('bucket forbidden')),patch.object(grow_tree,'main',side_effect=AssertionError('motion forbidden')):
            p=Path(d)/'preflight'
            self.assertEqual(e.main(['--pick-only','--config-only','--log-dir',str(p)],revalidate_only=False),0)
            r=json.loads((p/'config-preflight.json').read_text())
            self.assertEqual(r['task_mode'],'pick_only');self.assertIsNone(r['bucket_floor'])
            self.assertEqual((r['scene_seed'],r['strategy_target'],r['strategy_stem']),(35,'apple27','seg69'))
    def test_forbidden_states_and_release_fail_closed(self):
        with tempfile.TemporaryDirectory() as d:
            s=self.make(d)
            try:
                for state in ['TRANSPORT','DROP','SCORE','DONE']:
                    with self.assertRaises(RuntimeError):s.goto(state)
                with self.assertRaises(RuntimeError):s.forbid_release(0)
            finally:s.close_files()
    def test_detach_verify_requires_target_still_held(self):
        with tempfile.TemporaryDirectory() as d:
            s=self.make(d);s.state='DETACH_VERIFY';s.r.sim.apples._held_host=[0]
            try:
                with self.assertRaises(RuntimeError):s.after_frame(s.r)
                self.assertFalse(s.done)
            finally:s.close_files()
    def test_hold_requires_full_second_and_resets_on_instability(self):
        with tempfile.TemporaryDirectory() as d:
            s=self.make(d);s.state='SAFE_HOLD';s.met.pick_start(0,[1,2,3]);s.met.pick_event('grasp');s.met.pick_event('detach');s.finish=lambda result:None
            try:
                for _ in range(179):s.update_hold_window(True)
                s.after_frame(s.r);self.assertFalse(s.done)
                s.update_hold_window(False);self.assertEqual(s.hold_samples,0)
                for _ in range(180):s.update_hold_window(True)
                s.after_frame(s.r)
                self.assertTrue(s.done);self.assertTrue(s.r.stopped);self.assertEqual(s.state,'PICK_HOLD_DONE')
                m=s.met.summary();self.assertEqual((m['grasp_success'],m['pick_success'],m['place_success']),(1,1,0))
                self.assertIsNone(s.met.picks[0]['fail_reason'])
            finally:s.close_files()
    def test_bucket_shapes_rejected(self):
        with tempfile.TemporaryDirectory() as d:
            s=self.make(d)
            try:
                s.bucket_shapes={4}
                with self.assertRaises(RuntimeError):s.prepare_bucket(s.r)
            finally:s.close_files()
    def test_detach_finishes_existing_pull_before_hold(self):
        with tempfile.TemporaryDirectory() as d:
            s=self.make(d);s.state='PULL';s.r.sim.apples.pull_forces=lambda:np.array([7.])
            s.met.pick_start(0,[1,2,3]);s.met.pick_event('grasp')
            try:
                s.after_frame(s.r);self.assertEqual(s.state,'DETACH_VERIFY')
                s.robot.settled=lambda:False
                s.after_frame(s.r);self.assertEqual(s.state,'DETACH_VERIFY')
                s.robot.settled=lambda:True
                s.after_frame(s.r);self.assertEqual(s.state,'SAFE_HOLD');self.assertFalse(s.done)
            finally:s.close_files()
