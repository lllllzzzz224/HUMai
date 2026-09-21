"""Stage 3.5 opt-in tracking policy and PREGRASP-only stopping contracts."""
import io
import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch

import numpy as np
from PIL import Image
from treesim import rm65_close_only as co
from treesim.rm65_runtime import GeometryAudit
from treesim.rm65_control import JointController


def array(x):
    return NS(numpy=lambda: np.asarray(x))


class TrackingPolicyTests(unittest.TestCase):
    def policy(self, enabled=True):
        self.assertTrue(hasattr(co, 'TrackingPolicy'), 'missing opt-in tracking policy')
        return co.TrackingPolicy(enabled=enabled)

    def test_pregrasp_5_to_8_mm_warns_and_8_is_allowed(self):
        p=self.policy()
        self.assertFalse(p.check('PREGRASP', .005, 0., 1))
        self.assertTrue(p.check('PREGRASP', .006, 0., 2))
        self.assertTrue(p.check('PREGRASP', .008, 0., 3))
        self.assertEqual(p.summary()['warning_events'], 1)

    def test_strictly_above_8_mm_fails_immediately(self):
        p=self.policy()
        with self.assertRaisesRegex(RuntimeError, 'TRACKING_ERROR'):
            p.check('PREGRASP', np.nextafter(.008, np.inf), 0., 1)

    def test_event_edges_duration_and_peak(self):
        p=self.policy()
        for i,e in enumerate([.005,.006,.007,.005,.0051,.005],1):
            p.check('PREGRASP',e,0.,i)
        s=p.summary()
        self.assertEqual(s['warning_events'],2)
        self.assertEqual(s['warning_substeps'],3)
        self.assertAlmostEqual(s['warning_sample_duration_s'],3/180)
        self.assertAlmostEqual(s['maximum_tracking_error_m'],.007)
        self.assertEqual([e['substeps'] for e in s['events']],[2,1])
        self.assertEqual([e['first_step'] for e in s['events']],[2,5])
        self.assertFalse(s['events'][-1]['open_at_stop'])

    def test_ready_opt_in_and_other_stages_keep_5_mm(self):
        self.assertTrue(self.policy().check('READY',.006,0.,1))
        for stage in ('DETECT','TARGET_CONFIRM','REVALIDATE','GRASP','CLOSE'):
            with self.subTest(stage=stage), self.assertRaisesRegex(RuntimeError,'TRACKING_ERROR'):
                self.policy().check(stage,.006,0.,1)

    def test_default_behavior_remains_5_mm(self):
        for stage in ('READY','PREGRASP'):
            with self.assertRaisesRegex(RuntimeError,'TRACKING_ERROR'):
                self.policy(False).check(stage,.0051,0.,1)
        p=co.CloseOnlyProgram(Path('/tmp'))
        self.assertFalse(p.pregrasp_only)
        with self.assertRaisesRegex(RuntimeError,'TRACKING_ERROR'):
            p.tracking.check('PREGRASP',.006,0.,1)

    def test_orientation_and_nonfinite_still_fail(self):
        for pos,angle in [(.006,np.deg2rad(1.001)),(np.nan,0.),(.006,np.nan)]:
            with self.assertRaises(RuntimeError): self.policy().check('PREGRASP',pos,angle,1)

    def test_close_distance_and_revalidation_not_relaxed(self):
        co.close_gate('GRASP',True,True,[.005,0,0],[0,0,0])
        for distance,fresh in [(.0051,True),(.007,True),(.004,False)]:
            with self.assertRaisesRegex(RuntimeError,'CLOSE_GATE'):
                co.close_gate('GRASP',True,fresh,[distance,0,0],[0,0,0])

    def test_pregrasp_terminal_sequence_cannot_revalidate(self):
        s=co.Sequence(pregrasp_only=True)
        for stage in ('READY','DETECT','TARGET_CONFIRM','PREGRASP','PREGRASP_ONLY_DONE'):
            s.advance(stage)
        for stage in ('REVALIDATE','GRASP','CLOSE'):
            with self.assertRaisesRegex(RuntimeError,'TRANSITION'): s.advance(stage)

    def make_program(self, *, collision=False, q=None):
        p=co.CloseOnlyProgram(Path('/tmp'),free_space_tracking=True,pregrasp_only=True)
        p.sequence.state='PREGRASP';p.base=np.eye(4);p.goal_pose=np.eye(4)
        q=np.zeros(6) if q is None else np.array(q)
        reference=np.array([.006,0,0,0,0,0])
        def fk(x):
            T=np.eye(4);T[0,3]=x[0];return T
        p.kin=NS(soft_lower=np.full(6,-3.),soft_upper=np.full(6,3.),fk=fk)
        p.actual_pose=lambda state: np.eye(4)
        c=NS(read_arm_state=lambda s:(q,np.zeros(6)),read_gripper_state=lambda s:(np.zeros(2),np.zeros(2)),
             sent=np.r_[reference,0.,0.],goal=np.r_[reference,0.,0.],ts=np.arange(8),effective_arm_target=reference)
        body=np.array([[0.,0,0,0,0,0,1.]])
        state=NS(body_q=array(body))
        audit=GeometryAudit.__new__(GeometryAudit)
        audit.audit_contacts=lambda **kw:None
        audit.last_contact_distances={'gripper_mount|seg301':-.001} if collision else {}
        p.clear=NS(check=lambda s,phase:audit.check_no_contact())
        p.gate=NS(update=lambda **kw:None,ready=False)
        p.r=NS(substeps=1,controller=c,base_initial=body[0],audit=NS(body_ids={'base_link':0}),
            camera=NS(binding=NS(world_pose=lambda b:np.eye(4),T_parent_camera=np.eye(4))),
            tree=NS(robot_data={'body_indices':{'gripper_mount':[0]}}),
            sim=NS(control=NS(joint_target_q=array(c.sent)),apples=NS(_held_host=np.zeros(1),detached=np.zeros(1),broken_count=0)))
        p.rows=io.StringIO();p.events_log=io.StringIO()
        return p,state

    def test_warning_does_not_skip_collision_guard(self):
        p,state=self.make_program(collision=True)
        with self.assertRaisesRegex(RuntimeError,'COLLISION'):p.after_substep(state)
        self.assertEqual(p.tracking.summary()['warning_substeps'],1)

    def test_warning_does_not_skip_joint_margin_guard(self):
        p,state=self.make_program(q=[3.1,0,0,0,0,0])
        with self.assertRaises(RuntimeError):p.after_substep(state)

    def test_measured_hard_limit_still_fails(self):
        c=JointController.__new__(JointController);c.qs=np.arange(8);c.ds=np.arange(8)
        c.limits={'lower':np.full(8,-1.),'upper':np.ones(8),'velocity':np.ones(8)}
        state=NS(joint_q=array([1.001,0,0,0,0,0,0,0]),joint_qd=array(np.zeros(8)),
                 body_q=array(np.zeros((1,7))),body_qd=array(np.zeros((1,6))))
        with self.assertRaisesRegex(RuntimeError,'joint limit'):c.check_state(state)

    def test_substep_warning_is_recorded_without_changing_reference(self):
        p,state=self.make_program();sent=p.r.controller.sent.copy();goal=p.r.controller.goal.copy()
        p.after_substep(state)
        self.assertTrue(json.loads(p.rows.getvalue())['tracking_soft_warning'])
        self.assertEqual(p.tracking.summary()['warning_substeps'],1)
        np.testing.assert_array_equal(p.r.controller.sent,sent)
        np.testing.assert_array_equal(p.r.controller.goal,goal)

    def test_stable_pregrasp_saves_error_and_stops_before_revalidation(self):
        with tempfile.TemporaryDirectory() as d:
            p=co.CloseOnlyProgram(Path(d),free_space_tracking=True,pregrasp_only=True)
            p.sequence.state='PREGRASP';p.motion=[NS()];p.motion_index=0
            p.gate=NS(ready=True);p.arrival_count=45
            p.boundary=lambda:None  # geometry/physics boundary tested separately
            p.events_log=io.StringIO()
            p.last_motion={'tcp_position_error_m':.0047,'tcp_rotation_error_rad':.002}
            p.r=NS(frames=12,substeps=36,max_base_error=0.,stopped=False,
                sim=NS(render=lambda:None,viewer=NS(get_frame=lambda:array(np.zeros((8,8,3),dtype=np.uint8)))))
            p.after_frame(p.r)
            self.assertTrue(p.done);self.assertTrue(p.r.stopped)
            self.assertEqual(p.state,'PREGRASP_ONLY_DONE')
            result=json.loads((Path(d)/'result.json').read_text())
            self.assertEqual(result['endpoint']['tcp_position_error_m'],.0047)
            self.assertEqual(result['result'],'PASS')
            self.assertTrue((Path(d)/'pregrasp-stable.png').exists())
            self.assertFalse(p.revalidated);self.assertFalse(p.close_authorized)

    def test_unstable_pregrasp_does_not_finish(self):
        p=co.CloseOnlyProgram(Path('/tmp'),free_space_tracking=True,pregrasp_only=True)
        p.sequence.state='PREGRASP';p.motion=[NS()];p.motion_index=0
        p.gate=NS(ready=True);p.arrival_count=44
        p.after_frame(NS())
        self.assertEqual(p.state,'PREGRASP');self.assertFalse(p.done)
