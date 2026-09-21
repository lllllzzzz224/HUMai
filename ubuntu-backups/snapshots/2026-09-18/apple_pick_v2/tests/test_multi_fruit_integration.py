import importlib.util
import unittest
from test_multi_fruit_state import snapshot, detection
from multi_fruit_state import TargetManager

class Backend:
    """Only the external robot boundary is replaced; scheduler and runner are real."""
    def __init__(self): self.motion=[]; self.planned=[]; self.guard=True; self.fail=None
    def prepare(self,target,budget,record):
        budget.remaining()
        self.planned.append(target.center[0])
        if self.fail: raise self.fail
        return target
    def revalidate(self,prepared,budget): return True
    def safe(self): return self.guard
    def execute(self,prepared,record): self.motion.append(prepared.center[0])
    def cancel(self): pass

class IntegrationTests(unittest.TestCase):
    def setUp(self):
        self.assertIsNotNone(importlib.util.find_spec('multi_fruit_runtime'), 'persistent runtime is missing')
        from multi_fruit_runtime import Runner
        self.backend=Backend(); self.manager=TargetManager()
        self.now=10.
        self.runner=Runner(self.manager,self.backend,ros_now=lambda:self.now)
        self.fruits=[detection(.2,.75),detection(.4,.95),detection(.6,.85)]
        self.feed()
    def feed(self,fruits=None):
        for _ in range(2):
            self.now+=.1
            self.manager.update(snapshot(self.now,self.fruits if fruits is None else fruits),self.now)
    def test_default_plans_three_confidence_ranked_targets_without_motion_or_gripper(self):
        for _ in range(3):
            self.assertEqual(self.runner.step(),'PLANNED_ONLY')
            self.feed()
        self.assertEqual(self.backend.planned,[.4,.6,.2])
        self.assertEqual(self.backend.motion,[])
        self.assertEqual(self.runner.step(),'ALL_ATTEMPTED')
    def test_execute_requires_operator_and_fresh_interlock(self):
        self.runner.execute_requested=True
        self.assertEqual(self.runner.step(),'WAITING_INTERLOCK')
        self.runner.operator_confirmed=True; self.backend.guard=False
        self.assertEqual(self.runner.step(),'WAITING_INTERLOCK')
        self.assertEqual(self.backend.motion,[])
    def test_post_plan_disappearance_prevents_execution(self):
        self.runner.execute_requested=self.runner.operator_confirmed=True
        prepare=self.backend.prepare
        def disappearing(target,budget,record):
            result=prepare(target,budget,record)
            self.feed([self.fruits[0],self.fruits[2]])
            return result
        self.backend.prepare=disappearing
        self.assertEqual(self.runner.step(),'TARGET_CHANGED')
        self.assertEqual(self.backend.motion,[])
    def test_three_fruit_execute_order_and_human_removal(self):
        self.runner.execute_requested=self.runner.operator_confirmed=True
        remaining=list(self.fruits)
        for x in [.4,.6,.2]:
            self.assertEqual(self.runner.step(),'COMMANDED_RETURN_COMPLETE')
            remaining=[d for d in remaining if d['center_xyz_m'][0]!=x]
            self.feed(remaining)
        self.assertEqual(self.backend.motion,[.4,.6,.2])
        self.assertEqual(self.runner.step(),'SCENE_EMPTY')
    def test_budget_exhaustion_skips_candidate_without_moving(self):
        from multi_fruit_runtime import BudgetExpired
        self.backend.fail=BudgetExpired('expired')
        self.assertEqual(self.runner.step(),'PLANNING_TIMEOUT')
        self.assertEqual(self.backend.motion,[])
        self.assertEqual(self.manager.lock(self.now).center[0],.6)
    def test_fault_latches_and_does_not_retry(self):
        from multi_fruit_runtime import HardwareFault
        self.backend.fail=HardwareFault('service completion unknown')
        self.assertEqual(self.runner.step(),'FAULT_LATCHED_STOP_UNCONFIRMED')
        self.backend.fail=None
        self.assertEqual(self.runner.step(),'FAULT_LATCHED_STOP_UNCONFIRMED')
        self.assertEqual(len(self.backend.planned),1)

    def test_settled_execution_timeout_latches_instead_of_rescheduling(self):
        from rclpy.task import Future
        from multi_fruit_runtime import PhaseLog
        from multi_fruit_transport import GuardedTransport
        from types import SimpleNamespace
        clock=[0.]
        transport=GuardedTransport(pump=lambda seconds:clock.__setitem__(0,clock[0]+seconds),
            guard=lambda:None,log=PhaseLog(),clock=lambda:clock[0])
        result=Future()
        class Handle:
            accepted=True
            cancelled=False
            def get_result_async(self): return result
            def cancel_goal_async(self):
                self.cancelled=True
                result.set_result(SimpleNamespace(status=5))
                return Future()
        handle=Handle()
        class Client:
            def send_goal_async(self,goal):
                accepted=Future(); accepted.set_result(handle); return accepted
        def execute(prepared,record):
            action=transport.action(Client(),'execution',timeout_s=.05)
            action.send_goal_async(object()).result().get_result_async()
        self.backend.execute=execute
        self.backend.cancel=lambda:setattr(transport,'fault','session fault; stop unconfirmed')
        self.runner.execute_requested=self.runner.operator_confirmed=True
        self.assertEqual(self.runner.step(),'FAULT_LATCHED_STOP_UNCONFIRMED')
        self.assertTrue(handle.cancelled)
        self.assertIsNotNone(self.runner.fault)
        self.assertIsNotNone(transport.fault)
        self.feed()
        self.assertEqual(self.runner.step(),'FAULT_LATCHED_STOP_UNCONFIRMED')
        self.assertEqual(len(self.backend.planned),1)

    def test_idle_wait_ends_before_real_preflight_and_terminal_wait_is_closed(self):
        clock=[1.]
        self.runner.log.clock=lambda:clock[0]
        self.runner.execute_requested=True
        self.assertEqual(self.runner.step(),'WAITING_INTERLOCK')
        clock[0]=4.
        self.runner.execute_requested=False
        prepare=self.backend.prepare
        def slow_prepare(*args):
            clock[0]=9.
            return prepare(*args)
        self.backend.prepare=slow_prepare
        self.assertEqual(self.runner.step(),'PLANNED_ONLY')
        waits=[p for p in self.runner.log.spans if p['phase']=='state_wait']
        self.assertEqual(len(waits),1)
        self.assertEqual(waits[0]['duration_s'],3.)
        self.assertEqual(waits[0]['end_monotonic_s'],4.)
        self.runner.execute_requested=True
        clock[0]=10.
        self.assertEqual(self.runner.step(),'WAITING_INTERLOCK')
        clock[0]=12.
        self.runner.wait_timer.close()
        waits=[p for p in self.runner.log.spans if p['phase']=='state_wait']
        self.assertEqual(waits[-1]['duration_s'],2.)

if __name__=='__main__': unittest.main()

class ROSBoundaryTests(unittest.TestCase):
    def setUp(self):
        self.assertIsNotNone(importlib.util.find_spec('multi_fruit_grasp'),'ROS entrypoint missing')
        import multi_fruit_grasp
        self.module=multi_fruit_grasp

    def test_cli_defaults_no_motion_and_replay_cannot_execute(self):
        args=self.module.parse_args([])
        self.assertFalse(args.execute)
        self.assertAlmostEqual(args.planning_budget_s,2.)
        from contextlib import redirect_stderr
        import io
        with self.assertRaises(SystemExit),redirect_stderr(io.StringIO()):
            self.module.parse_args(['--execute','--replay'])

    def test_start_state_mismatch_rejects_cached_trajectory(self):
        from trajectory_msgs.msg import JointTrajectoryPoint
        from moveit_msgs.msg import RobotTrajectory
        from types import SimpleNamespace
        import numpy as np
        node=self.module.MultiFruitNode.__new__(self.module.MultiFruitNode)
        node.JOINT_NAMES=['joint1']; node.joints={'joint1':.2}
        node.guard=lambda:None
        trajectory=RobotTrajectory()
        trajectory.joint_trajectory.joint_names=['joint1']
        trajectory.joint_trajectory.points=[JointTrajectoryPoint(positions=[.1])]
        with self.assertRaises(RuntimeError): node.check_start(trajectory)

    def test_parent_joint_delta_validation_is_retained(self):
        from trajectory_msgs.msg import JointTrajectoryPoint
        from moveit_msgs.msg import RobotTrajectory
        node=self.module.MultiFruitNode.__new__(self.module.MultiFruitNode)
        node.JOINT_NAMES=['joint1']; node.joints={'joint1':0.}
        node.cfg={'safety':{'max_joint_delta_rad':.5}}
        trajectory=RobotTrajectory()
        trajectory.joint_trajectory.joint_names=['joint1']
        trajectory.joint_trajectory.points=[JointTrajectoryPoint(positions=[1.])]
        with self.assertRaisesRegex(RuntimeError,'关节运动量'):
            node.validate_trajectory(trajectory,'test')

    def test_mid_motion_snapshots_are_not_authoritative(self):
        from types import SimpleNamespace
        from std_msgs.msg import String
        import json
        node=self.module.MultiFruitNode.__new__(self.module.MultiFruitNode)
        node.active=True
        node.manager=TargetManager()
        node.manager.update(snapshot(10.,[detection(.2)]),10.)
        node.on_candidates(String(data=json.dumps(snapshot(10.1,[]))))
        self.assertEqual(node.manager.capture,10.)

    def test_identical_scene_update_does_not_fault_but_changed_object_does(self):
        from moveit_msgs.msg import PlanningScene,CollisionObject
        from geometry_msgs.msg import Pose
        from shape_msgs.msg import SolidPrimitive
        from types import SimpleNamespace
        node=self.module.MultiFruitNode.__new__(self.module.MultiFruitNode)
        node.active=True; node.scene_changed=False
        node.transport=SimpleNamespace(fault=None,cancel=lambda:None)
        scene=PlanningScene(); obj=CollisionObject(); obj.id='table'
        obj.primitives=[SolidPrimitive(type=SolidPrimitive.SPHERE,dimensions=[.1])]
        obj.primitive_poses=[Pose()]; scene.world.collision_objects=[obj]
        node.scene_reference=self.module.geometry_signature(scene)
        node.on_scene(scene)
        self.assertFalse(node.scene_changed)
        obj.primitive_poses[0].position.x=.1
        scene.world.collision_objects=[obj]
        node.on_scene(scene)
        self.assertTrue(node.scene_changed)

    def test_changed_neighbor_rejects_snapshot_even_when_selected_fruit_is_unchanged(self):
        self.assertTrue(self.module.same_detections([detection(.2),detection(.4)], [detection(.201),detection(.401)]))
        self.assertFalse(self.module.same_detections([detection(.2),detection(.4)], [detection(.201),detection(.43)]))

    def test_real_prepare_round_and_validators_with_only_external_ros_calls_doubled(self):
        from contextlib import redirect_stdout
        from copy import deepcopy
        import io
        import time
        from unittest.mock import patch
        from types import SimpleNamespace
        from builtin_interfaces.msg import Time
        from rclpy.task import Future
        from moveit_msgs.msg import RobotTrajectory,PlanningScene,RobotState
        from trajectory_msgs.msg import JointTrajectoryPoint
        from multi_fruit_runtime import Budget,PhaseLog
        from multi_fruit_transport import GuardedTransport
        m=self.module
        node=m.MultiFruitNode.__new__(m.MultiFruitNode)
        node.cfg=m.load_config(m.DEFAULT_CONFIG)
        node.args=m.parse_args(['--disable-visual-xy-correction'])
        node.base=node.cfg['frames']['base']; node.tcp=node.cfg['frames']['tcp']
        node.joints=dict(node.cfg['scan']['joints']); node.joint_rx_monotonic=time.monotonic()
        node.joint_source_stamp=10.1
        node.active=node.arming=False; node.gripper=None; node.events=[]; node.neighbor_ids=set()
        node.cached=[]; node.log=PhaseLog(); node.snapshot_detections=[detection(.3)]
        node.manager=TargetManager()
        node.manager.update(snapshot(10.,node.snapshot_detections),10.)
        node.manager.update(snapshot(10.1,node.snapshot_detections),10.1)
        node.get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(to_msg=lambda:Time(),nanoseconds=10100000000))
        node.get_logger=lambda:SimpleNamespace(info=lambda *a,**k:None)
        node.transport=GuardedTransport(pump=lambda s:None,guard=node.guard,log=node.log)
        calls=[]
        scene=PlanningScene()
        def complete(value): f=Future(); f.set_result(value); return f
        def trajectory():
            t=RobotTrajectory(); t.joint_trajectory.joint_names=node.JOINT_NAMES
            point=JointTrajectoryPoint(positions=node.current_joint_vector().tolist())
            point.time_from_start.sec=1; t.joint_trajectory.points=[point]; return t
        class Service:
            def __init__(self,kind): self.kind=kind
            def call_async(self,request):
                calls.append((self.kind,request))
                if self.kind=='ik':
                    state=RobotState(); state.joint_state.name=node.JOINT_NAMES
                    state.joint_state.position=node.current_joint_vector().tolist()
                    response=SimpleNamespace(error_code=SimpleNamespace(val=1),solution=state)
                elif self.kind=='cartesian': response=SimpleNamespace(error_code=SimpleNamespace(val=1),solution=trajectory(),fraction=1.)
                elif self.kind=='validity': response=SimpleNamespace(valid=True)
                elif self.kind=='scene': response=SimpleNamespace(success=True)
                else: response=SimpleNamespace(scene=scene)
                return complete(response)
        class Handle:
            accepted=True
            def get_result_async(self):
                return complete(SimpleNamespace(result=SimpleNamespace(error_code=SimpleNamespace(val=1),planned_trajectory=trajectory())))
        class Action:
            def send_goal_async(self,goal): calls.append(('plan',goal)); return complete(Handle())
        for attr,kind in [('ik_client','ik'),('cartesian_client','cartesian'),('validity_client','validity'),('scene_client','scene'),('get_scene_client','get_scene')]:
            setattr(node,attr,node.transport.service(Service(kind),kind))
        node.move_client=node.transport.action(Action(),'plan')
        with patch('rclpy.ok',return_value=True),patch('rclpy.spin_until_future_complete',lambda *a,**k:None),redirect_stdout(io.StringIO()):
            bundle=node.prepare(node.manager.lock(10.1),Budget(2.),{})
            self.assertEqual(len(bundle.trajectories),7)
            self.assertTrue(node.revalidate(bundle,Budget(2.)))
        self.assertIsNone(node.gripper)
        self.assertGreater(sum(kind=='validity' for kind,_ in calls),3)
        self.assertEqual(sum(kind=='cartesian' for kind,_ in calls),5)
        self.assertTrue(all(request.avoid_collisions for kind,request in calls if kind=='cartesian'))
        self.assertTrue(all(goal.planning_options.plan_only for kind,goal in calls if kind=='plan'))


    def test_actuation_guard_rejects_stale_future_or_nonfinite_joint_source_stamp(self):
        from unittest.mock import patch
        import time
        from multi_fruit_runtime import HardwareFault
        node=self.module.MultiFruitNode.__new__(self.module.MultiFruitNode)
        node.active=True; node.arming=False; node.scene_changed=False
        node.safe=lambda:True; node.ros_now=lambda:100.
        node.joints={name:0. for name in node.JOINT_NAMES}
        node.joint_rx_monotonic=time.monotonic()
        with patch('rclpy.ok',return_value=True):
            for stamp in (1.,101.,float('nan'),float('inf')):
                node.joint_source_stamp=stamp
                with self.subTest(stamp=stamp),self.assertRaises(HardwareFault): node.guard()
            node.joint_source_stamp=99.9
            node.guard()
