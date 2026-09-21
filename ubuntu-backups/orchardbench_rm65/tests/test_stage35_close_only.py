import ast
import inspect
import unittest
import io
from types import SimpleNamespace
from unittest.mock import Mock
import numpy as np

from treesim.rm65_close_only import Sequence, NaturalTrack, close_gate, permitted_pair, CloseOnlyProgram


class CloseOnlyContracts(unittest.TestCase):
    def test_order_and_terminal_states(self):
        s = Sequence()
        with self.assertRaisesRegex(RuntimeError, 'TRANSITION'): s.advance('GRASP')
        for name in Sequence.ORDER[1:]: s.advance(name)
        with self.assertRaisesRegex(RuntimeError, 'TRANSITION'): s.advance('READY')

    def test_failure_terminal_from_every_stage(self):
        for stage in Sequence.ORDER[:-1]:
            s = Sequence(); s.state = stage; s.advance('FAILED')
            with self.assertRaises(RuntimeError): s.advance('GRASP')

    def test_unique_fresh_association_without_truth(self):
        t = NaturalTrack()
        for i in range(3): t.add(i, (SimpleNamespace(center_world=np.array([.7, .4, .6])),))
        np.testing.assert_array_equal(t.center, [.7, .4, .6])
        with self.assertRaisesRegex(RuntimeError, 'FRESH'): t.add(2, ())
        for ds in [(), (SimpleNamespace(), SimpleNamespace())]:
            with self.assertRaisesRegex(RuntimeError, 'DETECTION'): t.add(3, ds)
        with self.assertRaisesRegex(RuntimeError, 'ASSOCIATION'):
            t.add(3, (SimpleNamespace(center_world=np.array([.8, .4, .6])),))

    def test_revalidation_failure_cannot_start_grasp(self):
        p = CloseOnlyProgram.__new__(CloseOnlyProgram)
        p.sequence = Sequence(); p.sequence.state = 'REVALIDATE'
        p.track = NaturalTrack(); p.track.center = np.zeros(3)
        p.accept_sample = Mock(side_effect=RuntimeError('DETECTION_AMBIGUITY'))
        p.plan_motion = Mock(); p.acquired = []; p.gate = SimpleNamespace(ready=True)
        p.r = SimpleNamespace(sim=SimpleNamespace(sim_time=1.))
        with self.assertRaises(RuntimeError): p.after_camera_sample(p.r, SimpleNamespace())
        p.plan_motion.assert_not_called()
        self.assertEqual(p.state, 'REVALIDATE')

    def test_close_requires_stable_tcp_and_fresh_revalidation(self):
        close_gate('GRASP', True, True, [.001, 0, 0], [0, 0, 0])
        for state, stable, revalidated, tcp in [('PREGRASP',True,True,[0,0,0]),
                ('GRASP',False,True,[0,0,0]),('GRASP',True,False,[0,0,0]),
                ('GRASP',True,True,[.0051,0,0]),('GRASP',True,True,[np.nan,0,0])]:
            with self.assertRaises(RuntimeError): close_gate(state,stable,revalidated,tcp,[0,0,0])

    def test_only_two_target_finger_pairs_during_close(self):
        for finger in ('gripper_left_finger','gripper_right_finger'):
            self.assertTrue(permitted_pair('CLOSE', [finger,'apple35']))
            for stage in ('INIT','PREGRASP','GRASP','FAILED'):
                self.assertFalse(permitted_pair(stage,[finger,'apple35']))
            for other in ('apple34','seg294','seg301','seg0','ground'):
                self.assertFalse(permitted_pair('CLOSE',[finger,other]))
        self.assertFalse(permitted_pair('CLOSE',['gripper_palm','apple35']))

    def test_no_hold_or_retract_commands_in_program(self):
        tree = ast.parse(inspect.getsource(CloseOnlyProgram))
        calls = [n.func.attr for n in ast.walk(tree) if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute)]
        self.assertNotIn('hold',calls)
        self.assertNotIn('retract',calls)
        self.assertNotIn('RETRACT',Sequence.ORDER)

    def test_hold_attempt_is_rejected_without_forwarding(self):
        p = CloseOnlyProgram.__new__(CloseOnlyProgram); p.hold_attempts = 0
        with self.assertRaisesRegex(RuntimeError,'HOLD_FORBIDDEN'): p.forbid_hold(0)
        self.assertEqual(p.hold_attempts,1)

    def test_non_target_solver_contact_fails_before_next_step(self):
        def array(x): return SimpleNamespace(numpy=lambda: np.array(x))
        p = CloseOnlyProgram.__new__(CloseOnlyProgram)
        p.sequence=Sequence(); p.sequence.state='CLOSE'
        p.shape_body=np.array([0,1]); p.robot_bodies={0}; p.contact_rows=[]
        p.contacts_log=io.StringIO()
        p.force_contacts=SimpleNamespace(rigid_contact_count=array([1]),
            rigid_contact_shape0=array([0]),rigid_contact_shape1=array([1]),
            force=array([[1.,0,0,0,0,0]]))
        p.r=SimpleNamespace(substeps=1,sim=SimpleNamespace(
            model=SimpleNamespace(body_label=['gripper_left_finger','seg294']),
            solver=SimpleNamespace(update_contacts=lambda *a: None,
                mjw_data=SimpleNamespace(contact=SimpleNamespace(dist=array([0.]))))))
        with self.assertRaisesRegex(RuntimeError,'FORBIDDEN_SOLVER_CONTACT'):
            p.observe_contacts(SimpleNamespace())
        self.assertFalse(p.contact_rows[0]['expected'])
        # The runtime's exception hook uses this same terminal transition.
        p.first_failure=None; p.last_state=None; p.events_log=None
        p.r.sim.apples=SimpleNamespace(_held_host=np.zeros(1),broken_count=0)
        p.last_motion=None; p.samples=[]; p.endpoints=[]; p.hold_attempts=0; p.save=Mock()
        p.fail(RuntimeError('FORBIDDEN_SOLVER_CONTACT'))
        self.assertEqual(p.state,'FAILED'); self.assertTrue(p.r.stopped)

    def test_fail_does_not_command_home_or_continue(self):
        p=CloseOnlyProgram.__new__(CloseOnlyProgram)
        p.sequence=Sequence(); p.sequence.state='PREGRASP'
        p.first_failure=None; p.last_state=None; p.events_log=None
        p.r=SimpleNamespace(stopped=False,controller=Mock(),sim=SimpleNamespace(
            apples=SimpleNamespace(_held_host=np.zeros(1),broken_count=0)))
        p.last_motion=None; p.samples=[]; p.endpoints=[]; p.contact_rows=[]
        p.hold_attempts=0; p.save=Mock()
        p.fail(RuntimeError('tracking')); p.fail(RuntimeError('later'))
        self.assertEqual(p.first_failure,'tracking'); self.assertTrue(p.r.stopped)
        self.assertEqual(p.r.controller.mock_calls,[]); p.save.assert_called_once()
