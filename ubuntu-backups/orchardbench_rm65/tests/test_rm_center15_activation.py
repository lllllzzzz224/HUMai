import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace as N
from unittest.mock import patch
import numpy as np
from treesim.rm65_run_config import load_config
from treesim.rm65_official import GripperAdapter, RobotAdapter
from treesim.rm65_official_strategy import LocalGraspStrategy

class CenterActivationTests(unittest.TestCase):
    def test_locked_config_reaches_actual_gripper_visual_goal_and_hold(self):
        c=load_config()
        self.assertEqual(getattr(c,'grasp_center_local',None),(0.,0.,-.015))
        calls=[]
        r=N(audit=N(body_ids={'gripper_tcp':42}),sim=N(apples=N(hold=lambda i,b:calls.append((i,b)))))
        with tempfile.TemporaryDirectory() as d:
            s=LocalGraspStrategy(d,None,config=c)
            try:
                g=s.create_gripper(r,N(index=27))
                self.assertEqual(g.grasp_center_local,(0.,0.,-.015))
                np.testing.assert_allclose(g.tcp_target_for_fruit([1,2,3],np.eye(4)),[1,2,3.015])
                g.hold(27,distance=.004,closed=True,stable=True)
                self.assertEqual(calls,[(27,42)])
                np.testing.assert_allclose(list(r.sim.apples.hold_off),[0,0,-.015],atol=1e-9)
                self.assertEqual(GripperAdapter(r).offset,(0.,0.,0.))
            finally:s.close_files()
    def test_both_preflights_resolve_adapter_center_and_hash(self):
        from treesim.rm65_run_entry import main
        with tempfile.TemporaryDirectory() as d:
            records=[]
            for mode in (False,True):
                out=Path(d)/str(mode)
                args=['--config-only','--log-dir',str(out)]+([] if mode else ['--pick-only'])
                self.assertEqual(main(args,revalidate_only=mode),0)
                record=json.loads((out/'config-preflight.json').read_text());records.append(record)
                self.assertEqual(record.get('grasp_center_local'),[0.,0.,-.015])
                self.assertEqual(record.get('resolved_hold_anchor'),[0.,0.,-.015])
            self.assertEqual(records[0]['fixture_sha256'],records[1]['fixture_sha256'])
    def test_carried_scratch_geometry_uses_adapter_not_tcp_origin(self):
        from treesim import rm65_official as m
        bodies=np.array([[1.,2.,3.,0,0,0,1],[9.,9.,9.,0,0,0,1]])
        class Array:
            def __init__(self,a):self.a=np.array(a)
            def numpy(self):return self.a.copy()
            def assign(self,a):self.a=np.array(a)
        state=N(body_q=Array(bodies))
        r=N(audit=N(body_ids={'gripper_tcp':0},allowed_touch_pairs=set()),
            tree=N(apple_data={'apple_body':[1]}),sim=N(state_0=state),
            controller=N(read_gripper_state=lambda s:(np.zeros(2),np.zeros(2))))
        robot=RobotAdapter.__new__(RobotAdapter);robot.r=r
        robot.carried_index=0;robot.carried_gripper=GripperAdapter(r,grasp_center_local=(0,0,-.015))
        robot.kin=N(soft_lower=np.full(6,-3),soft_upper=np.full(6,3));robot.contact_allowed=False
        robot.scratch=N(q=Array(np.zeros(8)),qd=None,snapshot_q=np.zeros(8),qs=np.arange(8),model=None,
            state=state,indices=None,audit=N(check_no_contact=lambda:None))
        robot.clear=N(check=lambda s,p:None)
        with patch.object(m.newton,'eval_fk'):
            robot.check(np.zeros(6))
        np.testing.assert_allclose(state.body_q.numpy()[1,:3],[1,2,2.985])
    def test_diagnostics_use_local_center_and_do_not_mutate_state(self):
        import importlib.util
        self.assertIsNotNone(importlib.util.find_spec('treesim.rm65_pick_diagnostics'))
        from treesim.rm65_pick_diagnostics import relative_geometry
        tcp=np.eye(4);tcp[:3,3]=[1,2,3]
        apple=np.array([1,2,3.005]);before=apple.copy()
        row=relative_geometry(tcp,apple,(0,0,-.015))
        np.testing.assert_allclose(row['apple_relative_grasp_center_local_m'],[0,0,.02])
        self.assertAlmostEqual(row['hold_anchor_error_m'],.02)
        np.testing.assert_array_equal(apple,before)
    def test_missing_or_tampered_center_is_rejected(self):
        from treesim.rm65_run_config import DEFAULT_FIXTURE
        original=json.loads(DEFAULT_FIXTURE.read_text())
        with tempfile.TemporaryDirectory() as d:
            path=Path(d)/'fixture.json'
            for value in (None,[0,0,0],[0,0,float('nan')],[0,0]):
                record=dict(original)
                if value is None:record.pop('grasp_center_local')
                else:record['grasp_center_local']=value
                path.write_text(json.dumps(record))
                with self.assertRaises(ValueError):load_config(path)
    def test_observer_records_contacts_force_and_frame_counters_without_writes(self):
        from treesim.rm65_pick_diagnostics import PickDiagnostics
        class ReadOnly:
            def __init__(self,a):self.a=np.array(a)
            def numpy(self):return self.a.copy()
        poses=[[0,0,0,0,0,0,1],[0,0,.005,0,0,0,1],
               [0,.026,-.08,0,0,0,1],[0,-.026,-.08,0,0,0,1]]
        state=N(body_q=ReadOnly(poses),body_qd=ReadOnly(np.zeros((4,6))))
        a=N(_held_host=[1],detached=[False],hold_k=500.,hold_c=17.888,
            _hold_body_host=[0],hold_off=(0,0,-.015),_pull=ReadOnly([10.]),_tension=ReadOnly([11.]),
            detach_force=[19.087],_over=[2],_over_t=[0],_hyst=6,_hyst_t=12,_tension_factor=1.2)
        r=N(tree=N(apple_data={'apple_body':[1]}),frames=1,substeps=3,
            audit=N(body_ids={'gripper_tcp':0,'gripper_left_finger':2,'gripper_right_finger':3}),
            sim=N(model=N(body_label=['gripper_tcp','apple27','gripper_left_finger','gripper_right_finger']),
                apples=a,frame_dt=1/60,substeps=3),
            controller=N(read_gripper_state=lambda s:(np.array([.04,.04]),np.zeros(2)),goal=np.r_[np.zeros(6),.06,.06],sent=np.r_[np.zeros(6),.06,.06]))
        with tempfile.TemporaryDirectory() as d:
            observer=PickDiagnostics(d,r,GripperAdapter(r,grasp_center_local=(0,0,-.015)),'apple27')
            contacts=[dict(step=3,pair=['apple27','gripper_left_finger'],force_N=2.)]
            observer.observe(state,'PULL',contacts)
            observer.observe(state,'PULL',contacts,phase='post_rupture_update')
            r.substeps=4;observer.observe(state,'PULL',[]);observer.close()
            rows=[json.loads(line) for line in (Path(d)/'pick-diagnostics.jsonl').read_text().splitlines()]
            self.assertAlmostEqual(rows[0]['inner_face_gap_m'],.04)
            np.testing.assert_allclose(rows[0]['derived_postsolver_hold_total_N'],[0,0,-10])
            self.assertEqual(rows[1]['finger_apple_contact_summary'][0]['substeps'],1)
            self.assertAlmostEqual(rows[-1]['finger_apple_contact_summary'][0]['duration_s'],1/180)
            self.assertEqual(rows[-1]['pull_counter'],2);self.assertFalse(rows[-1]['detached'])
            np.testing.assert_array_equal(state.body_q.numpy(),poses)
    def test_first_failing_contact_substep_is_still_recorded(self):
        from treesim.rm65_pick_only import PickOnlyStrategy
        with tempfile.TemporaryDirectory() as d:
            s=PickOnlyStrategy(d,config=load_config());seen=[]
            s.diagnostics=N(observe=lambda *args:seen.append(args),close=lambda:None)
            state=object()
            try:
                with patch.object(LocalGraspStrategy,'observe_contacts',side_effect=RuntimeError('FORBIDDEN_CONTACT')):
                    with self.assertRaisesRegex(RuntimeError,'FORBIDDEN_CONTACT'):s.observe_contacts(state)
                self.assertEqual(len(seen),1);self.assertIs(seen[0][0],state)
            finally:s.close_files()
