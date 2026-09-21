"""Stage 3.3 real-scene isolation and fail-stop regression checks (CUDA)."""
import contextlib
import io
import json
import unittest
from unittest.mock import patch

import newton
import numpy as np

from scripts import grow_tree
from treesim import builder, rm65_control as rc
from treesim.rm65_runtime import RMRuntime, ScratchScene, UnsafePose
from treesim.sim import Sim


def scene():
    args = grow_tree.parse_args([
        '--robot-model', 'rm65', '--mount', 'fixed', '--rm-control', 'joint',
        '--viewer', 'null', '--depth', '2', '--device', 'cuda:0', '--seed', '42',
    ])
    tree = builder.generate_and_build(grow_tree.make_config(args), num_envs=1)
    sim = Sim(tree, solver='mujoco', fps=60, substeps=3, collisions=True)
    return tree, sim


def buffers(sim):
    """Byte snapshots include both ping-pong states and the live model/targets."""
    arrays = {}
    for label, owner in [('model', sim.model), ('state0', sim.state_0), ('state1', sim.state_1)]:
        for name in ('joint_q', 'joint_qd', 'body_q', 'body_qd'):
            arrays[label + '.' + name] = getattr(owner, name).numpy().tobytes()
    arrays['target'] = sim.control.joint_target_q.numpy().tobytes()
    return arrays


class RuntimeSafetyTests(unittest.TestCase):
    def test_scratch_preserves_live_buffers_and_nonrest_tree_snapshot(self):
        tree, sim = scene()
        runtime = RMRuntime(tree, sim)
        tree_ids = np.array([i for i in range(sim.model.body_count)
                             if i not in runtime.audit.audit_body_ids])
        self.assertGreater(len(tree_ids), 0)
        # A snapshot must retain actual tree transforms, not regenerate rest FK.
        poses = sim.state_0.body_q.numpy()
        poses[tree_ids, 2] += .0123
        sim.state_0.body_q.assign(poses)
        before = buffers(sim)
        scratch = ScratchScene(runtime)
        self.assertIsNot(scratch.model, sim.model)
        self.assertIsNot(scratch.state, sim.state_0)
        self.assertNotEqual(scratch.state.body_q.ptr, sim.state_0.body_q.ptr)
        self.assertNotEqual(scratch.model.joint_q.ptr, sim.model.joint_q.ptr)
        for q in (np.zeros(6), np.array([.02, -.02, .01, -.01, .02, -.02])):
            self.assertTrue(scratch.check(q))
            np.testing.assert_array_equal(scratch.state.body_q.numpy()[tree_ids], poses[tree_ids])
            self.assertEqual(before, buffers(sim))
        self.assertEqual(scratch.samples, 2)

    def test_scratch_rejects_real_ground_collision_without_live_writes(self):
        tree, sim = scene()
        runtime = RMRuntime(tree, sim)
        before = buffers(sim)
        scratch = ScratchScene(runtime)
        q = np.zeros(6)
        q[1] = -2.258
        with self.assertRaises(UnsafePose) as caught:
            scratch.check(q)
        self.assertIn('ground', str(caught.exception))
        self.assertEqual(scratch.samples, 0)
        self.assertEqual(before, buffers(sim))

    def test_initial_actual_geometry_rejected_before_controller_write(self):
        tree, sim = scene()
        joints = rc.resolve_names(sim.model.joint_label)
        indices = sim.model.joint_q_start.numpy()[joints]
        q = sim.state_0.joint_q.numpy()
        q[indices[1]] = -2.258
        sim.state_0.joint_q.assign(q)
        newton.eval_fk(sim.model, sim.state_0.joint_q, sim.state_0.joint_qd, sim.state_0)
        before = buffers(sim)
        with patch.object(rc, 'JointController', side_effect=AssertionError('unsafe controller takeover')) as takeover:
            with self.assertRaises(UnsafePose) as caught:
                RMRuntime(tree, sim)
        takeover.assert_not_called()
        self.assertEqual(caught.exception.physics_steps, 0)
        self.assertIn('ground', str(caught.exception))
        self.assertEqual(before, buffers(sim))

    def test_initial_nonfinite_state_rejected_before_controller_write(self):
        tree, sim = scene()
        joints = rc.resolve_names(sim.model.joint_label)
        indices = sim.model.joint_q_start.numpy()[joints]
        q = sim.state_0.joint_q.numpy()
        q[indices[0]] = np.nan
        sim.state_0.joint_q.assign(q)
        before = buffers(sim)
        with patch.object(rc, 'JointController', side_effect=AssertionError('nonfinite controller takeover')) as takeover:
            with self.assertRaisesRegex(RuntimeError, 'NaN|Inf|nonfinite'):
                RMRuntime(tree, sim)
        takeover.assert_not_called()
        self.assertEqual(before, buffers(sim))

    def test_closed_finger_goal_rejected_before_target_update(self):
        tree, sim = scene()
        runtime = RMRuntime(tree, sim)
        before = buffers(sim)
        runtime.controller.set_gripper_open_fraction(0.)
        with self.assertRaisesRegex(RuntimeError, 'fingers must remain open'):
            runtime.before_frame()
        self.assertTrue(runtime.stopped)
        self.assertEqual((runtime.frames, runtime.substeps), (0, 0))
        self.assertEqual(runtime.controller.last_frame, -1)
        self.assertEqual(before, buffers(sim))

    def test_nonhome_start_rejected_before_controller_write(self):
        tree, sim = scene()
        js = rc.resolve_names(sim.model.joint_label)
        q = sim.state_0.joint_q.numpy()
        q[sim.model.joint_q_start.numpy()[js[0]]] = .02
        sim.state_0.joint_q.assign(q)
        newton.eval_fk(sim.model,sim.state_0.joint_q,sim.state_0.joint_qd,sim.state_0)
        before = buffers(sim)
        with patch.object(rc,'JointController',side_effect=AssertionError('nonhome takeover')):
            with self.assertRaisesRegex(RuntimeError,'home'):
                RMRuntime(tree,sim)
        self.assertEqual(before,buffers(sim))

    def test_second_real_substep_exception_stops_entire_sequence(self):
        tree, sim = scene()
        real_solver = sim.solver
        runtime = RMRuntime(tree, sim)
        runtime.trace = True
        original_check = runtime.controller.check_state
        checked = []
        def fail_second(state):
            result = original_check(state)
            checked.append(state.joint_q.numpy().copy())
            if len(checked) == 2:
                raise RuntimeError('injected second-substep safety failure')
            return result
        captured = io.StringIO()
        with patch.object(real_solver, 'step', wraps=real_solver.step) as steps:
            with patch.object(runtime.controller, 'check_state', side_effect=fail_second):
                with contextlib.redirect_stdout(captured):
                    runtime.before_frame()
                    target = sim.control.joint_target_q.numpy().tobytes()
                    with self.assertRaisesRegex(RuntimeError, 'injected second-substep'):
                        sim.step()
            self.assertEqual(steps.call_count, 2)
            self.assertEqual((runtime.frames, runtime.substeps), (0, 2))
            self.assertTrue(runtime.stopped)
            with self.assertRaisesRegex(RuntimeError, 'already stopped'):
                runtime.before_frame()
            with self.assertRaisesRegex(RuntimeError, 'already stopped'):
                sim.step()
            self.assertEqual(steps.call_count, 2)
        self.assertEqual(target, sim.control.joint_target_q.numpy().tobytes())
        self.assertEqual(sim.sim_time, 0.)
        rows = [json.loads(line) for line in captured.getvalue().splitlines() if line.startswith('{')]
        self.assertEqual([r['step'] for r in rows if r.get('event') == 'rm_substep'], [1, 2])
        self.assertEqual([r['step'] for r in rows if r.get('event') == 'rm_stop'], [2])


if __name__ == '__main__':
    unittest.main()
