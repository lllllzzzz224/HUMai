import copy
import importlib.util
import os
import sys
import unittest
from types import SimpleNamespace
from pathlib import Path
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from moveit_msgs.msg import PlanningScene
from obstacle_geometry import build_snapshot
from test_obstacle_geometry import record
from obstacle_scene_ros import (box_object, publish_snapshot, scene_digest,
                                make_plan_goal, verify_trajectory, observation_coverage, RosPlanTransport)


class MemoryScene:
    def __init__(self):
        self.scene = PlanningScene()
        self.scene.world.collision_objects = [box_object('foreign_table', [0, 0, -.2], [.8, .8, .1])]
        self.reject = False
        self.drop_writes = False
        self.calls = 0

    def execution_disabled(self):
        return True

    def read(self):
        return copy.deepcopy(self.scene)

    def apply(self, diff):
        self.calls += 1
        if self.reject:
            return False
        if not self.drop_writes:
            self.assert_diff = (diff.is_diff, diff.robot_state.is_diff)
            objs = {o.id:o for o in self.scene.world.collision_objects}
            for o in diff.world.collision_objects:
                if o.operation == o.REMOVE:
                    objs.pop(o.id, None)
                else:
                    objs[o.id] = o
            self.scene.world.collision_objects = list(objs.values())
        return True


class SceneTests(unittest.TestCase):
    def snapshot(self):
        return build_snapshot(np.full((3, 3), 1000, np.uint16), record(), {})

    @patch.dict(os.environ, {'ROS_DOMAIN_ID':'87', 'ROS_AUTOMATIC_DISCOVERY_RANGE':'LOCALHOST'})
    def test_readback_matches_and_foreign_geometry_survives(self):
        t = MemoryScene(); foreign = scene_digest(t.read())
        result = publish_snapshot(self.snapshot(), t, replay=True, now_s=200.)
        self.assertEqual(result['status'], 'SCENE_PREVIEW')
        self.assertFalse(result['executable'])
        self.assertEqual(t.assert_diff, (True, True))
        table = [o for o in t.scene.world.collision_objects if o.id == 'foreign_table']
        check = PlanningScene(); check.world.collision_objects = table
        self.assertEqual(scene_digest(check), foreign)
        # A new view that sees nothing cannot clear the old scene.
        empty = self.snapshot(); empty['occupied_cells'] = {}
        with self.assertRaises(ValueError):
            publish_snapshot(empty, t, replay=True, now_s=200.)

    @patch.dict(os.environ, {'ROS_DOMAIN_ID':'87', 'ROS_AUTOMATIC_DISCOVERY_RANGE':'LOCALHOST'})
    def test_rejected_write_and_lost_readback_are_failures(self):
        for field in ['reject', 'drop_writes']:
            t = MemoryScene(); setattr(t, field, True)
            with self.subTest(field=field), self.assertRaises(RuntimeError):
                publish_snapshot(self.snapshot(), t, replay=True, now_s=200.)

    @patch.dict(os.environ, {'ROS_DOMAIN_ID':'87', 'ROS_AUTOMATIC_DISCOVERY_RANGE':'LOCALHOST'})
    def test_changed_grid_or_padding_cannot_erase_old_occupied_volume(self):
        depth = np.full((3, 3), 1000, np.uint16)
        for change in ({'voxel_size_m':.1, 'padding_m':0.}, {'voxel_size_m':.11, 'padding_m':.1}):
            t = MemoryScene()
            first = build_snapshot(depth, record(), {'voxel_size_m':.1, 'padding_m':.1})
            publish_snapshot(first, t, replay=True, now_s=200.)
            digest = scene_digest(t.read())
            second = build_snapshot(depth, record(), change)
            # Force an index overlap as happens when only one spatial axis changes cell.
            if change['voxel_size_m'] != .1:
                ident = next(iter(first['occupied_cells']))
                second['occupied_cells'][ident] = [0., 0., 1.1]
            with self.subTest(change=change), self.assertRaises(ValueError):
                publish_snapshot(second, t, replay=True, now_s=200.)
            self.assertEqual(t.calls, 1)
            self.assertEqual(scene_digest(t.read()), digest)

    def test_fk_normalizes_joint_names_before_assigning_trajectory_samples(self):
        from moveit_msgs.msg import RobotState
        from geometry_msgs.msg import PoseStamped
        transport = RosPlanTransport.__new__(RosPlanTransport)
        transport.node = SimpleNamespace(create_client=lambda *a: object(), destroy_client=lambda c: None)
        captured = []
        def call(client, req):
            captured.append(dict(zip(req.robot_state.joint_state.name, req.robot_state.joint_state.position)))
            return SimpleNamespace(error_code=SimpleNamespace(val=1), pose_stamped=[PoseStamped() for _ in range(7)])
        transport.call = call
        start = RobotState(); start.joint_state.name = [f'joint{i}' for i in range(6, 0, -1)]
        start.joint_state.position = [0.]*6
        transport.fk_link_origins(np.array([[.1, .2, .3, .4, .5, .6]]), start)
        self.assertEqual(captured[0], dict(joint1=.1, joint2=.2, joint3=.3, joint4=.4, joint5=.5, joint6=.6))

    def test_stale_or_unverified_scene_never_writes_online(self):
        t = MemoryScene()
        for now in (100., 200.):
            with self.assertRaises(ValueError):
                publish_snapshot(self.snapshot(), t, replay=False, now_s=now)
        self.assertEqual(t.calls, 0)

    @patch.dict(os.environ, {'ROS_DOMAIN_ID':'0', 'ROS_AUTOMATIC_DISCOVERY_RANGE':'LOCALHOST'})
    def test_replay_refuses_default_live_domain(self):
        t = MemoryScene()
        with self.assertRaises(ValueError):
            publish_snapshot(self.snapshot(), t, replay=True, now_s=200.)
        self.assertEqual(t.calls, 0)

    def test_goal_is_plan_only_and_explicit_start(self):
        q = [.1, .2, .3, .4, .5, .6]
        scene = PlanningScene()
        goal = make_plan_goal(scene, q, [v+.01 for v in q])
        self.assertTrue(goal.planning_options.plan_only)
        self.assertFalse(goal.planning_options.look_around)
        self.assertEqual(list(goal.request.start_state.joint_state.position), q)

    def test_coverage_never_turns_occluded_path_into_known_free(self):
        result = observation_coverage([[0, 0, .5], [0, 0, 1.1]],
                                      np.full((3, 3), 1000, np.uint16), record(), {})
        self.assertEqual(result['free'], 1)
        self.assertEqual(result['unknown'], 1)
        self.assertFalse(result['full_robot_swept_volume_certified'])

    def test_waypoints_alone_cannot_hide_collision_between_them(self):
        from moveit_msgs.msg import RobotTrajectory, RobotState
        from trajectory_msgs.msg import JointTrajectoryPoint
        traj = RobotTrajectory(); traj.joint_trajectory.joint_names = [f'joint{i}' for i in range(1, 7)]
        traj.joint_trajectory.points = [JointTrajectoryPoint(positions=[0.]*6), JointTrajectoryPoint(positions=[.1]*6)]
        def valid(state):
            return not (.04 < state.joint_state.position[0] < .06)
        with self.assertRaises(RuntimeError):
            verify_trajectory(traj, RobotState(), valid, step_rad=.01)

    @patch.dict(os.environ, {'ROS_DOMAIN_ID':'87', 'ROS_AUTOMATIC_DISCOVERY_RANGE':'LOCALHOST'})
    def test_launch_only_constructs_display_and_planner_with_execution_disabled(self):
        path = Path(__file__).resolve().parents[1]/'obstacle_preview.launch.py'
        spec = importlib.util.spec_from_file_location('obstacle_launch', path)
        module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
        calls = []
        from launch.actions import LogInfo
        def collect(**kwargs):
            calls.append(kwargs)
            return LogInfo(msg='test does not start processes')
        with patch.object(module, 'Node', collect):
            module.generate_launch_description()
        self.assertEqual({c['executable'] for c in calls}, {'robot_state_publisher', 'move_group', 'rviz2'})
        move = next(c for c in calls if c['executable'] == 'move_group')
        params = {k:v for d in move['parameters'] for k,v in d.items()}
        self.assertIs(params['allow_trajectory_execution'], False)
        self.assertIs(params['moveit_manage_controllers'], False)
        self.assertIn('move_group/MoveGroupExecuteTrajectoryAction', params['disable_capabilities'])


if __name__ == '__main__':
    unittest.main()
