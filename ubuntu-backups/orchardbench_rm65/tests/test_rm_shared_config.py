import importlib.util
import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace as N
from unittest.mock import patch
import numpy as np

class SharedConfigTests(unittest.TestCase):
    def api(self):
        self.assertIsNotNone(importlib.util.find_spec('treesim.rm65_run_config'))
        from treesim import rm65_run_config as m
        return m
    def test_locked_values_and_no_scoring_payload(self):
        c=self.api().load_config()
        self.assertEqual((c.seed,c.apple,c.stem,c.mode),(35,'apple27','seg69','official_assist'))
        np.testing.assert_allclose(c.mount,[.5816719824254781,.831022544457272,.3109927278320346,5.509892715650121],atol=1e-14)
        self.assertEqual(c.pregrasp_offset_tool,(0.,0.,-.08))
        np.testing.assert_allclose(c.tcp_quaternion_xyzw,[.5167925219767306,-.25325422296712896,.7342861612604015,.36001614013778593])
        self.assertFalse(hasattr(c,'natural_selection_center'))
        self.assertEqual(c.bucket_yaw,0.)
    def test_missing_and_changed_config_fail_closed(self):
        m=self.api()
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'missing.json'
            with self.assertRaises((ValueError,OSError)):m.load_config(p)
            original=json.loads(m.DEFAULT_FIXTURE.read_text())
            for key,value in [('seed',31),('apple','apple35'),('stem','seg294'),('pregrasp_offset_tool',[0,0,-.03]),('bucket_yaw',1.)]:
                modified=dict(original);modified[key]=value;p.write_text(json.dumps(modified))
                with self.assertRaises(ValueError):m.load_config(p)
            del original['mount'];p.write_text(json.dumps(original))
            with self.assertRaises(ValueError):m.load_config(p)
    def test_both_entrypoints_share_config_and_do_not_start_simulation(self):
        self.api()
        from scripts import run_rm65_official_assist as full,validate_rm65_fixture as check,grow_tree
        with tempfile.TemporaryDirectory() as d,patch.object(grow_tree,'main',side_effect=AssertionError('simulation forbidden')):
            snapshots=[]
            for name,entry in [('full',full),('check',check)]:
                out=Path(d)/name
                self.assertEqual(entry.main(['--log-dir',str(out),'--config-only']),0)
                snapshots.append(json.loads((out/'configuration.json').read_text()))
                pre=json.loads((out/'config-preflight.json').read_text())
                self.assertEqual(pre['physics_steps'],0);self.assertFalse(pre['viewer_created'])
                self.assertEqual(pre['strategy_target'],'apple27')
                self.assertEqual(pre['scene_seed'],35)
            self.assertEqual(snapshots[0]['runtime_config'],snapshots[1]['runtime_config'])
            self.assertEqual(snapshots[0]['fixture_sha256'],snapshots[1]['fixture_sha256'])
            self.assertEqual(snapshots[0]['argv'],snapshots[1]['argv'])
    def test_identity_association_contacts_hold_and_metrics_follow_config(self):
        c=self.api().load_config()
        from treesim.rm65_official import ObjectAssociation,GripperAdapter,expected_contact,PerceptionAdapter
        from treesim.rm65_official_strategy import LocalGraspStrategy
        calls=[]
        model=N(shape_body=N(numpy=lambda:np.array([3,7])),body_label=['','','','apple35','','','','apple27','seg69'])
        r=N(sim=N(model=model,apples=N(hold=lambda index,body:calls.append((index,body)))),tree=N(apple_data={'apple_body':[3,7],'parent_body':[8,8]}),audit=N(body_ids={'gripper_tcp':99}))
        a=ObjectAssociation(r,c)
        sample=N(sample_id=1,simulation_time=.2,detections_world=[N(center_world=np.array([.8,.6,1.]),px=(1,1))],shape_index=np.ones((3,3),dtype=np.uint32))
        a.accept(sample);self.assertEqual(a.index,1)
        g=GripperAdapter(r,a);g.hold(a.index,distance=.004,closed=True,stable=True)
        self.assertEqual(calls,[(1,99)])
        with self.assertRaises(RuntimeError):g.hold(0,distance=.004,closed=True,stable=True)
        self.assertTrue(expected_contact('apple27','gripper_left_finger',True,target=c.apple))
        self.assertFalse(expected_contact('apple35','gripper_left_finger',True,target=c.apple))
        self.assertFalse(expected_contact('seg69','gripper_left_finger',True,target=c.apple))
        with tempfile.TemporaryDirectory() as d:
            s=LocalGraspStrategy(d,__import__('treesim.rm65_official',fromlist=['Bucket']).Bucket(c.bucket_floor),config=c)
            try:
                s.state='DETECT';s.r=N(frames=0);s.association=a;s.screenshot=lambda *x:None
                plans=[];s.robot=N(actual=lambda:np.eye(4),plan=lambda point,label:plans.append((point,label)))
                for i in range(1,4):sample.sample_id=i;s.after_camera_sample(s.r,sample)
                np.testing.assert_allclose(plans[0][0],[.8,.6,.92])
                s.met.pick_event('grasp');s.met.pick_event('detach');s.finish_metrics(False)
                self.assertEqual(s.met.summary()['grasp_success'],1)
                self.assertEqual(s.met.summary()['pick_success'],1)
                self.assertEqual(s.met.summary()['place_success'],0)
                self.assertEqual(s.met.picks[0]['fruit_id'],1)
            finally:s.close_files()
        sample.shape_index[:]=0
        with self.assertRaises(RuntimeError):a.accept(sample)
    def test_full_strategy_refuses_missing_config(self):
        self.api()
        from treesim.rm65_official_strategy import LocalGraspStrategy
        from treesim.rm65_official import Bucket
        with tempfile.TemporaryDirectory() as d:
            with self.assertRaises((ValueError,TypeError)):LocalGraspStrategy(d,Bucket((0,0,0)))
    def test_missing_config_never_starts_either_entry(self):
        self.api()
        from scripts import run_rm65_official_assist as full,validate_rm65_fixture as check,grow_tree
        with tempfile.TemporaryDirectory() as d,patch.object(grow_tree,'main',side_effect=AssertionError('must not start')):
            for name,entry in [('full',full),('check',check)]:
                out=Path(d)/name
                with self.assertRaises(OSError):entry.main(['--fixture',str(Path(d)/'missing'),'--log-dir',str(out)])
                self.assertFalse(out.exists())
    def test_configured_fruit_with_wrong_stem_fails(self):
        c=self.api().load_config()
        from treesim.rm65_official import ObjectAssociation
        r=N(sim=N(model=N(shape_body=N(numpy=lambda:np.array([0])),body_label=['apple27','seg294'])),tree=N(apple_data={'apple_body':[0],'parent_body':[1]}))
        a=ObjectAssociation(r,c)
        sample=N(detections_world=[N(px=(1,1))],shape_index=np.zeros((3,3),dtype=np.uint32))
        with self.assertRaises(RuntimeError):a.accept(sample)
        self.assertIsNone(a.index)
    def test_original_pixi_entries_keep_franka_defaults(self):
        self.api()
        import shlex,tomllib
        from scripts import grow_tree
        root=Path(__file__).resolve().parents[1]
        tasks=tomllib.loads((root/'pyproject.toml').read_text())['tool']['pixi']['tasks']
        for name in ('bench','harvest'):
            tokens=shlex.split(tasks[name]);self.assertEqual(tokens[:2],['python','scripts/grow_tree.py'])
            args=grow_tree.parse_args(tokens[2:]);scene=grow_tree.make_config(args)
            self.assertEqual((scene.robot.model,scene.robot.mount),('franka','ridgeback'))
            self.assertIsNone(scene.robot.stage35_fixed_mount);self.assertFalse(scene.robot.rm_camera)
            self.assertEqual(args.auto,name=='harvest')

if __name__=='__main__':unittest.main()
