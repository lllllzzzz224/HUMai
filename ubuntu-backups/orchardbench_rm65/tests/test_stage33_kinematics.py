"""Real CUDA kinematics checks; no physics or scene state is touched."""
import unittest
from unittest.mock import patch
import numpy as np
from treesim.rm65_kinematics import (RM65Kinematics, pose_error, pose_from_xyz_quat,
    target_pose_base_to_world, target_pose_world_to_base, validate_pose)

class KinematicsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.kin = RM65Kinematics(device='cuda:0')
        cls.q = np.array([.2, -.4, .6, .3, .7, -.2])

    def test_independent_fk_matches_newton(self):
        for q in (np.zeros(6), self.q, -.5*self.q):
            p, r = pose_error(self.kin.fk(q), self.kin.fk_newton(q, [.02,.04]))
            self.assertLessEqual(p, 1e-5)
            self.assertLessEqual(r, 1e-4)
        self.assertGreater(self.kin.singular_values(self.q)[-1], 1e-4)

    def test_frames_and_invalid_rotations(self):
        mount = pose_from_xyz_quat([1.8,0,0], [0,0,1,0])
        base = self.kin.fk(self.q)
        world = target_pose_base_to_world(base, mount)
        np.testing.assert_allclose(target_pose_world_to_base(world,mount), base, atol=1e-12)
        general_mount=pose_from_xyz_quat([1.2,-.4,.7],np.array([1.,2.,3.,4.])/np.sqrt(30.))
        general_world=target_pose_base_to_world(base,general_mount)
        np.testing.assert_allclose(general_world,general_mount@base,atol=1e-12)
        np.testing.assert_allclose(target_pose_world_to_base(general_world,general_mount),base,atol=1e-12)
        for bad in ([0,0,0,0], [0,0,0,2], [0,np.nan,0,1]):
            with self.assertRaises(ValueError): pose_from_xyz_quat([0,0,0], bad)
        bad = np.eye(4); bad[0,0] = -1
        with self.assertRaises(ValueError): validate_pose(bad)

    def test_ik_first_seed_and_closest_valid_branch(self):
        target = self.kin.fk(self.q)
        before = self.kin.model.joint_limit_lower.numpy().copy()
        result = self.kin.solve(target, self.q, q_fingers=[.02,.04], extra_seeds=[self.q+.02])
        self.assertEqual(result.status, 'success')
        np.testing.assert_array_equal(result.candidates[0]['seed'], self.q)
        self.assertEqual(result.q_arm.shape, (6,))
        valid = [c for c in result.candidates if c['accepted']]
        selected = min(valid, key=lambda c: c['distance'])
        np.testing.assert_array_equal(result.q_arm, selected['q_arm'])
        np.testing.assert_array_equal(before, self.kin.model.joint_limit_lower.numpy())
        p,r = pose_error(target,self.kin.fk(result.q_arm))
        self.assertLessEqual(p,.0005); self.assertLessEqual(r,np.deg2rad(.1))

    def test_real_ik_keeps_actual_joint6_winding_and_logs_unwrapped_distance(self):
        for winding in (6.,-6.):
            with self.subTest(winding=winding):
                current=self.q.copy();current[5]=winding
                equivalent=current.copy();equivalent[5]-=np.sign(winding)*2*np.pi
                self.assertTrue(np.all(equivalent>=self.kin.soft_lower))
                self.assertTrue(np.all(equivalent<=self.kin.soft_upper))
                target=self.kin.fk(current)
                result=self.kin.solve(target,current,extra_seeds=[equivalent])
                self.assertEqual(result.status,'success',result)
                self.assertEqual(len(result.candidates),2)
                first,distant=result.candidates
                np.testing.assert_array_equal(first['seed'],current)
                np.testing.assert_array_equal(distant['seed'],equivalent)
                self.assertTrue(first['accepted'],first)
                self.assertTrue(distant['accepted'],distant)
                self.assertLess(abs(result.q_arm[5]-winding),1e-4)
                self.assertLess(first['distance'],1e-4)
                self.assertAlmostEqual(distant['distance'],2*np.pi,places=4)
                self.assertAlmostEqual(distant['distance'],np.linalg.norm(distant['q_arm']-current),places=12)
                self.assertLess(first['distance'],distant['distance'])
                np.testing.assert_array_equal(result.q_arm,first['q_arm'])
                self.assertLess(abs(distant['q_arm'][5]-equivalent[5]),1e-4)
                print('winding regression:',{'measured_joint6':winding,
                    'selected_joint6':float(result.q_arm[5]),
                    'first_distance':first['distance'],'distant_distance':distant['distance'],
                    'distant_joint6':float(distant['q_arm'][5])})

    def test_solves_changed_target_and_numpy_collision_rejection(self):
        target=self.kin.fk(self.q + [.01,-.01,.015,.005,-.01,.01])
        result=self.kin.solve(target,self.q)
        self.assertEqual(result.status,'success',result)
        self.assertGreater(np.linalg.norm(result.q_arm-self.q),.005)
        rejected=self.kin.solve(target,self.q,collision_check=lambda q: np.bool_(False))
        self.assertIsNone(rejected.q_arm)

    def test_deadline_after_last_callback_and_during_construction(self):
        clock=[0.]
        def late(q):
            clock[0]=2.
            return True
        with patch('treesim.rm65_kinematics.time.monotonic',side_effect=lambda:clock[0]):
            result=self.kin.solve(self.kin.fk(self.q),self.q,max_iterations=0,
                                  timeout_s=1.,collision_check=late)
        self.assertEqual(result.status,'timeout')
        self.assertIsNone(result.q_arm)
        self.assertEqual(result.candidates[-1]['reason'],'timeout')
        self.assertFalse(result.candidates[-1]['accepted'])
        # Objective construction is part of the deadline and exceptions are structured.
        with patch('treesim.rm65_kinematics.ik.IKObjectivePosition',side_effect=RuntimeError('objective failed')):
            result=self.kin.solve(self.kin.fk(self.q),self.q)
        self.assertEqual(result.status,'solver_error')
        self.assertIsNone(result.q_arm)
        clock[0]=0.
        from treesim import rm65_kinematics as module
        original=module.ik.IKSolver
        def slow_constructor(*args,**kwargs):
            solver=original(*args,**kwargs);clock[0]=2.;return solver
        with patch('treesim.rm65_kinematics.time.monotonic',side_effect=lambda:clock[0]), patch('treesim.rm65_kinematics.ik.IKSolver',side_effect=slow_constructor):
            result=self.kin.solve(self.kin.fk(self.q),self.q,timeout_s=1.)
        self.assertEqual(result.status,'timeout')
        self.assertIsNone(result.q_arm)
        self.assertEqual(result.candidates[0]['reason'],'timeout')

    def test_technical_collision_callback_error_is_not_collision(self):
        def broken(q): raise RuntimeError('GPU evaluation failed')
        result=self.kin.solve(self.kin.fk(self.q),self.q,max_iterations=0,collision_check=broken)
        self.assertEqual(result.status,'collision_check_error')
        self.assertIsNone(result.q_arm)
        self.assertIn('GPU evaluation failed',result.message)

    def test_invalid_and_failure_no_output(self):
        for target in (np.full((4,4),np.nan), np.zeros((4,4))):
            result = self.kin.solve(target,self.q)
            self.assertEqual(result.status, 'invalid_input'); self.assertIsNone(result.q_arm)
        result = self.kin.solve(self.kin.fk(self.q), np.full(6,np.nan))
        self.assertIsNone(result.q_arm)
        result = self.kin.solve(self.kin.fk(self.q),self.q,collision_check=lambda q: False)
        self.assertIsNone(result.q_arm)
        self.assertTrue(all(not c['accepted'] for c in result.candidates))
        outside=self.kin.soft_upper + .02
        result=self.kin.solve(self.kin.fk(outside),outside,max_iterations=0)
        self.assertIsNone(result.q_arm)
        self.assertIn('soft_limit',result.candidates[0]['reason'])

if __name__ == '__main__': unittest.main()
