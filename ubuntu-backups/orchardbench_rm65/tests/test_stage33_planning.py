"""Atomic Cartesian planning: CUDA positive case and deterministic late failure."""
import unittest
import numpy as np
from treesim.rm65_kinematics import RM65Kinematics,IKResult,pose_error
from treesim.rm65_planning import solve_cartesian_segment,PlanningFailure

class PlanningTests(unittest.TestCase):
    def test_real_fixed_orientation_segment(self):
        kin=RM65Kinematics(device='cuda:0');q=np.array([.2,-.4,.6,.3,.7,-.2])
        start=kin.fk(q);end=start.copy();end[0,3]+=.002
        events=[];slews=[];before=kin.model.joint_q.numpy().copy()
        plan=solve_cartesian_segment(kin,start,end,q,audit=lambda **event:events.append(event),
            check_slew=lambda a,b:slews.append((a.copy(),b.copy())))
        self.assertGreaterEqual(len(plan),2)
        np.testing.assert_array_equal(events[0]['candidates'][0]['seed'],q)
        self.assertEqual(events[0]['seed_role'],'provided_start')
        for i,point in enumerate(plan):
            self.assertLessEqual(pose_error(kin.fk(point.q_arm),point.target_pose_base)[0],.0005)
            if i: np.testing.assert_array_equal([e for e in events if e['event']=='ik_candidates'][i]['candidates'][0]['seed'],plan[i-1].q_arm)
        self.assertEqual(len(slews),len(plan))
        continuity=[e for e in events if e['event']=='waypoint_continuity']
        self.assertEqual(len(continuity),len(plan))
        self.assertTrue(all(e['max_joint_step']<=.1 and e['accepted'] for e in continuity))
        np.testing.assert_array_equal(kin.model.joint_q.numpy(),before)

    def test_late_discontinuity_atomic_failure(self):
        class Fake:
            soft_lower=np.full(6,-3.);soft_upper=np.full(6,3.)
            calls=0
            def solve(self,target,current_q,**kwargs):
                self.pose=target.copy();self.calls+=1;q=current_q.copy();q[0]+=.01 if self.calls==1 else .2
                return IKResult('success',q,[])
            def fk(self,q): return self.pose.copy()
        start=np.eye(4);end=start.copy();end[0,3]=.002
        kin=Fake();returned=None
        with self.assertRaises(PlanningFailure) as caught:
            returned=solve_cartesian_segment(kin,start,end,np.zeros(6))
        self.assertIsNone(returned);self.assertEqual(caught.exception.category,'discontinuity')
        self.assertEqual(kin.calls,2)

    def test_limit_and_collision_classification(self):
        class Fake:
            soft_lower=np.full(6,-.1);soft_upper=np.full(6,.1)
            def solve(self,target,current_q,**kwargs):
                return IKResult('success',np.full(6,.11),[])
        start=np.eye(4);end=start.copy();end[0,3]=.001
        with self.assertRaises(PlanningFailure) as caught:
            solve_cartesian_segment(Fake(),start,end,np.full(6,.09))
        self.assertEqual(caught.exception.category,'soft_limit')
        with self.assertRaises(PlanningFailure) as caught:
            solve_cartesian_segment(Fake(),start,end,np.zeros(6),collision_check=lambda q:False)
        self.assertEqual(caught.exception.category,'collision')

if __name__=='__main__': unittest.main()
