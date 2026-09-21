#!/usr/bin/env python3
"""Isolated recorded-scene preview. No trajectory execution or gripper client.

RosSceneTransport only owns scene services. The optional planner creates a
MoveGroup action client whose goals are always plan_only. None of these outputs
authorize physical motion.
"""
import argparse
import copy
import hashlib
import json
import math
import os
import time
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import Pose
from moveit_msgs.msg import (PlanningScene, CollisionObject, AttachedCollisionObject,
                            Constraints, JointConstraint, DisplayTrajectory)
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene, GetStateValidity, GetPositionFK
from moveit_msgs.action import MoveGroup
from shape_msgs.msg import SolidPrimitive
from rosidl_runtime_py.convert import message_to_ordereddict

from obstacle_geometry import PREFIX, canonical_scene, validate_observation, classify_points

JOINTS = [f'joint{i}' for i in range(1, 7)]
PREVIEW_DOMAIN = '87'


def raw_message(msg):
    return json.loads(json.dumps(message_to_ordereddict(msg)))


def _clean(value):
    if isinstance(value, dict):
        return {k: _clean(v) for k, v in value.items() if k != 'stamp'}
    if isinstance(value, list):
        return [_clean(v) for v in value]
    if isinstance(value, float):
        if not math.isfinite(value):
            raise ValueError('nonfinite_scene')
        return round(value, 8)
    return value


def _matrix(pose):
    q = pose.orientation
    quat = np.array([q.x, q.y, q.z, q.w], float)
    # A default empty scene may have zero poses; actual objects must not.
    if not np.isfinite(quat).all() or not np.isclose(np.linalg.norm(quat), 1., atol=1e-4):
        raise ValueError('invalid_object_quaternion')
    t = np.eye(4); t[:3, :3] = Rotation.from_quat(quat).as_matrix()
    t[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
    return t


def object_geometry(obj):
    root = _matrix(obj.pose)
    parts = []
    for field in ('primitive', 'mesh', 'plane'):
        shapes, poses = getattr(obj, 'meshes' if field == 'mesh' else field+'s'), getattr(obj, field+'_poses')
        if len(shapes) != len(poses):
            raise ValueError('shape_pose_count_mismatch')
        for shape, pose in zip(shapes, poses):
            parts.append(dict(kind=field, shape=raw_message(shape), pose=(root @ _matrix(pose)).tolist()))
    if len(obj.subframe_names) != len(obj.subframe_poses):
        raise ValueError('invalid_subframes')
    return _clean(dict(id=obj.id, frame=obj.header.frame_id, type=raw_message(obj.type),
                       parts=sorted(parts, key=lambda p: json.dumps(_clean(p), sort_keys=True)),
                       subframes={n:(root @ _matrix(p)).tolist() for n, p in zip(obj.subframe_names, obj.subframe_poses)}))


def scene_components(scene):
    acm = scene.allowed_collision_matrix
    n = len(acm.entry_names)
    if len(acm.entry_values) != n or any(len(r.enabled) != n for r in acm.entry_values):
        raise ValueError('invalid_acm_shape')
    entries = {a:{b:bool(acm.entry_values[i].enabled[j]) for j, b in enumerate(acm.entry_names)}
               for i, a in enumerate(acm.entry_names)}
    if any(entries[a][b] != entries[b][a] for a in entries for b in entries):
        raise ValueError('asymmetric_acm')
    if len(acm.default_entry_names) != len(acm.default_entry_values):
        raise ValueError('invalid_acm_defaults')
    octo = raw_message(scene.world.octomap)
    data = octo['octomap'].pop('data')
    octo['octomap']['data_sha256'] = hashlib.sha256(bytes(v & 255 for v in data)).hexdigest()
    return _clean(dict(
        world=sorted([object_geometry(o) for o in scene.world.collision_objects], key=lambda o:o['id']),
        attached=sorted([dict(object=object_geometry(o.object), link=o.link_name,
                              touch_links=sorted(o.touch_links), weight=o.weight)
                         for o in scene.robot_state.attached_collision_objects], key=lambda o:o['object']['id']),
        acm=dict(entries=entries, defaults=dict(zip(acm.default_entry_names, acm.default_entry_values))),
        octomap=octo, transforms=sorted([raw_message(t) for t in scene.fixed_frame_transforms], key=lambda t:t['child_frame_id']),
        padding=sorted([raw_message(p) for p in scene.link_padding], key=lambda p:p['link_name']),
        scale=sorted([raw_message(p) for p in scene.link_scale], key=lambda p:p['link_name']),
        colors=sorted([raw_message(c) for c in scene.object_colors], key=lambda c:c['id'])))


def scene_digest(scene):
    return canonical_scene([scene_components(scene)])


def box_object(ident, center, dimensions, frame='base_link'):
    xyz, size = np.asarray(center, float), np.asarray(dimensions, float)
    if xyz.shape != (3,) or size.shape != (3,) or not np.isfinite([xyz, size]).all() or np.any(size <= 0):
        raise ValueError('invalid_box')
    obj = CollisionObject(); obj.id = ident; obj.header.frame_id = frame
    obj.pose.orientation.w = 1.; obj.operation = CollisionObject.ADD
    pose = Pose(); pose.position.x, pose.position.y, pose.position.z = xyz.tolist(); pose.orientation.w = 1.
    obj.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=size.tolist())]
    obj.primitive_poses = [pose]
    return obj


def recorded_state(scene, q):
    q = np.asarray(q, float)
    if q.shape != (6,) or not np.isfinite(q).all():
        raise ValueError('invalid_six_joint_state')
    state = copy.deepcopy(scene.robot_state)
    state.is_diff = False
    state.joint_state.name = JOINTS
    state.joint_state.position = q.tolist()
    state.joint_state.velocity = []; state.joint_state.effort = []
    return state


def ensure_isolated(transport):
    if (os.environ.get('ROS_DOMAIN_ID') != PREVIEW_DOMAIN
            or os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE') != 'LOCALHOST'
            or os.environ.get('ROS_STATIC_PEERS')):
        raise ValueError('requires_isolated_preview_domain_87_localhost')
    if not transport.execution_disabled():
        raise ValueError('trajectory_execution_not_confirmed_disabled')


def publish_snapshot(snapshot, transport, replay=False, now_s=None):
    now_s = time.time() if now_s is None else now_s
    rec = snapshot['observation']
    if snapshot.get('executable') is not False or snapshot.get('frame_id') != 'base_link' or rec.get('frame_id') != 'base_link':
        raise ValueError('requires_nonexecutable_base_snapshot')
    validate_observation(rec, rec['capture_stamp_s'] if replay else now_s,
                         0. if replay else min(3., snapshot.get('config', {}).get('max_capture_age_s', 3.)))
    if not replay and snapshot['quality']['limitations']:
        raise ValueError('unverified_geometry_cannot_write_online_scene')
    if not snapshot['occupied_cells']:
        raise ValueError('empty_geometry')
    ensure_isolated(transport)
    before = transport.read()
    expected = copy.deepcopy(before)
    old = {o.id:o for o in before.world.collision_objects}
    side = float(snapshot['box_size_m'])
    changes = []
    for ident, center in snapshot['occupied_cells'].items():
        if not ident.startswith(PREFIX):
            raise ValueError('foreign_object_id')
        obj = box_object(ident, center, [side]*3)
        if ident in old and object_geometry(old[ident]) != object_geometry(obj):
            raise ValueError('existing_occupied_geometry_changed_without_free_evidence')
        changes.append(obj); old[ident] = obj
    # Retain every previously occupied object. Deletion is intentionally absent
    # from this one-shot writer; cleared_cells/scene_delta are for reviewed future
    # updates with an actual positive observation of the entire old volume.
    diff = PlanningScene(); diff.is_diff = True; diff.robot_state.is_diff = True
    diff.world.collision_objects = changes
    diff.robot_state.joint_state = recorded_state(before, rec['joint_positions_rad']).joint_state
    expected.world.collision_objects = list(old.values())
    if not transport.apply(diff):
        raise RuntimeError('planning_scene_write_rejected')
    after = transport.read()
    if scene_digest(after) != scene_digest(expected):
        raise RuntimeError('planning_scene_readback_mismatch')
    names = {o.object.id for o in after.robot_state.attached_collision_objects}
    return dict(status='SCENE_PREVIEW', scene_digest=scene_digest(after),
                capture_stamp_s=rec['capture_stamp_s'], age_at_replay_s=now_s-rec['capture_stamp_s'],
                state_source='recorded_controller_bracket', mode='isolated_recorded_replay' if replay else 'isolated_online_preview',
                observed_cells=len(changes), retained_previous_objects=len(old)-len(changes),
                camera_guard_present=bool(names & {'auto_obstacle_camera_guard', 'realsense_camera_guard'}),
                support_model='observed_surface_voxels_only_no_certified_plane',
                limitations=snapshot['quality']['limitations'], executable=False)


def add_recorded_camera_guard(transport, snapshot):
    """Explicit approximate body from existing baseline, never certified envelope."""
    hand = np.asarray(snapshot['observation']['transform_provenance']['hand_eye'], float)
    obj = box_object('auto_obstacle_camera_guard', [0., 0., 0.], [.110, .050, .050], 'Link6')
    obj.pose.position.x, obj.pose.position.y, obj.pose.position.z = hand[:3, 3].tolist()
    q = Rotation.from_matrix(hand[:3, :3]).as_quat()
    obj.pose.orientation.x, obj.pose.orientation.y, obj.pose.orientation.z, obj.pose.orientation.w = q.tolist()
    attach = AttachedCollisionObject(link_name='Link6', object=obj,
                                     touch_links=['Link6', 'gripper_body_approx', 'tcp_link'])
    before = transport.read(); expected = copy.deepcopy(before)
    attachments = {a.object.id:a for a in expected.robot_state.attached_collision_objects}
    attachments[obj.id] = attach
    expected.robot_state.attached_collision_objects = list(attachments.values())
    diff = PlanningScene(); diff.is_diff = True; diff.robot_state.is_diff = True
    diff.robot_state.attached_collision_objects = [attach]
    if not transport.apply(diff) or scene_digest(transport.read()) != scene_digest(expected):
        raise RuntimeError('camera_guard_readback_mismatch')


def make_plan_goal(scene, q_start, q_goal):
    q_goal = np.asarray(q_goal, float)
    if q_goal.shape != (6,) or not np.isfinite(q_goal).all():
        raise ValueError('invalid_goal_joints')
    goal = MoveGroup.Goal(); req = goal.request
    req.group_name = 'rm_group'; req.pipeline_id = 'ompl'; req.planner_id = 'RRTConnect'
    req.start_state = recorded_state(scene, q_start)
    req.num_planning_attempts = 1; req.allowed_planning_time = 10.
    req.max_velocity_scaling_factor = .1; req.max_acceleration_scaling_factor = .1
    req.workspace_parameters.header.frame_id = 'base_link'
    req.workspace_parameters.min_corner.x = req.workspace_parameters.min_corner.y = req.workspace_parameters.min_corner.z = -2.
    req.workspace_parameters.max_corner.x = req.workspace_parameters.max_corner.y = req.workspace_parameters.max_corner.z = 2.
    req.goal_constraints = [Constraints(joint_constraints=[
        JointConstraint(joint_name=n, position=float(q), tolerance_above=.001, tolerance_below=.001, weight=1.)
        for n, q in zip(JOINTS, q_goal)])]
    goal.planning_options.plan_only = True
    goal.planning_options.look_around = False; goal.planning_options.replan = False
    goal.planning_options.planning_scene_diff.is_diff = True
    goal.planning_options.planning_scene_diff.robot_state.is_diff = True
    return goal


def verify_trajectory(trajectory, start, validity, step_rad=.01):
    jt = trajectory.joint_trajectory
    if list(jt.joint_names) != JOINTS or not jt.points:
        raise RuntimeError('empty_or_unexpected_joint_trajectory')
    if not math.isfinite(step_rad) or step_rad <= 0:
        raise ValueError('invalid_interpolation_step')
    values = np.array([p.positions for p in jt.points], float)
    if values.ndim != 2 or values.shape[1] != 6 or not np.isfinite(values).all():
        raise RuntimeError('invalid_trajectory_values')
    samples = [values[0]]
    for a, b in zip(values, values[1:]):
        n = max(1, int(np.ceil(np.max(np.abs(b-a))/step_rad)))
        samples.extend(a+(b-a)*(i/n) for i in range(1, n+1))
    for q in samples:
        state = copy.deepcopy(start); state.joint_state.name = JOINTS; state.joint_state.position = q.tolist()
        if not validity(state):
            raise RuntimeError('trajectory_sample_in_collision')
    return np.asarray(samples)


def plan_preview(snapshot, candidate, transport):
    ensure_isolated(transport)
    if (candidate.get('kind') != 'pregrasp' or candidate.get('validated') is not True
            or candidate.get('scene_version') != snapshot.get('scene_version')
            or not candidate.get('source')):
        raise ValueError('missing_validated_pregrasp_candidate_for_this_scene')
    before = transport.read(); digest = scene_digest(before)
    observed = {o.id:object_geometry(o) for o in before.world.collision_objects}
    for ident, center in snapshot['occupied_cells'].items():
        wanted = object_geometry(box_object(ident, center, [snapshot['box_size_m']]*3))
        if observed.get(ident) != wanted:
            raise ValueError('snapshot_geometry_not_present_in_planning_scene')
    q_start = snapshot['observation']['joint_positions_rad']
    goal = make_plan_goal(before, q_start, candidate['joint_positions_rad'])
    if not transport.valid(goal.request.start_state):
        return {'status':'START_STATE_INVALID', 'executable':False}
    result = transport.plan(goal)
    if result.error_code.val != 1:
        return {'status':'PLANNING_FAILED', 'moveit_error':result.error_code.val, 'executable':False}
    # Prevent MoveIt's start-state repair adapters silently changing the bracketed start.
    actual = dict(zip(result.trajectory_start.joint_state.name, result.trajectory_start.joint_state.position))
    if any(n not in actual or abs(actual[n]-q) > .001 for n, q in zip(JOINTS, q_start)):
        raise RuntimeError('planner_changed_recorded_start')
    samples = verify_trajectory(result.planned_trajectory, result.trajectory_start, transport.valid)
    if not np.allclose(samples[0], q_start, atol=.001, rtol=0):
        raise RuntimeError('trajectory_does_not_start_at_recorded_state')
    if not np.allclose(samples[-1], candidate['joint_positions_rad'], atol=.002, rtol=0):
        raise RuntimeError('trajectory_does_not_reach_candidate')
    rec = snapshot['observation']
    depth_file = Path(rec['depth_file'])
    if hashlib.sha256(depth_file.read_bytes()).hexdigest() != rec['depth_sha256']:
        raise ValueError('coverage_depth_evidence_changed')
    points = transport.fk_link_origins(samples, result.trajectory_start)
    coverage = observation_coverage(points, np.load(depth_file, allow_pickle=False), rec, snapshot['config'])
    if scene_digest(transport.read()) != digest:
        raise RuntimeError('scene_changed_during_planning')
    transport.display(result)
    return dict(status='PLAN_PREVIEW', scene_digest=digest, checked_state_count=len(samples),
                unknown_coverage=coverage,
                limitations=snapshot['quality']['limitations'], executable=False,
                trajectory=raw_message(result.planned_trajectory),
                trajectory_start=raw_message(result.trajectory_start))


def observation_coverage(points, depth, record, config):
    states = classify_points(points, depth, record, config)
    return dict(free=int(np.sum(states == 'free')), occupied=int(np.sum(states == 'occupied')),
                unknown=int(np.sum(states == 'unknown')), sample_count=len(states),
                scope='Link1..Link6_and_tcp_origins_only_not_link_surfaces',
                full_robot_swept_volume_certified=False)


class RosSceneTransport:
    def __init__(self, node):
        self.node = node
        self.apply_client = node.create_client(ApplyPlanningScene, '/apply_planning_scene')
        self.get_client = node.create_client(GetPlanningScene, '/get_planning_scene')
        self.valid_client = node.create_client(GetStateValidity, '/check_state_validity')

    def wait(self, future, timeout=20.):
        import rclpy
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)
        if not future.done():
            future.cancel()
            raise TimeoutError('preview_service_timeout')
        return future.result()

    def call(self, client, request):
        if not client.wait_for_service(timeout_sec=10.):
            raise TimeoutError('preview_service_unavailable')
        return self.wait(client.call_async(request))

    def execution_disabled(self):
        from rcl_interfaces.srv import GetParameters
        from rcl_interfaces.msg import ParameterType
        client = self.node.create_client(GetParameters, '/move_group/get_parameters')
        try:
            response = self.call(client, GetParameters.Request(names=['allow_trajectory_execution']))
            return (len(response.values) == 1 and response.values[0].type == ParameterType.PARAMETER_BOOL
                    and response.values[0].bool_value is False)
        finally:
            self.node.destroy_client(client)

    def read(self):
        req = GetPlanningScene.Request(); req.components.components = 1023
        return self.call(self.get_client, req).scene

    def apply(self, diff):
        return self.call(self.apply_client, ApplyPlanningScene.Request(scene=diff)).success

    def valid(self, state):
        return self.call(self.valid_client, GetStateValidity.Request(robot_state=state, group_name='rm_group')).valid


class RosPlanTransport(RosSceneTransport):
    def fk_link_origins(self, samples, start):
        client = self.node.create_client(GetPositionFK, '/compute_fk')
        points = []
        try:
            for q in samples:
                req = GetPositionFK.Request(); req.header.frame_id = 'base_link'
                req.fk_link_names = [f'Link{i}' for i in range(1, 7)]+['tcp_link']
                req.robot_state = recorded_state(PlanningScene(robot_state=start), q)
                result = self.call(client, req)
                if result.error_code.val != 1 or len(result.pose_stamped) != 7:
                    raise RuntimeError('coverage_fk_failed')
                points.extend([p.pose.position.x, p.pose.position.y, p.pose.position.z] for p in result.pose_stamped)
            return points
        finally:
            self.node.destroy_client(client)

    def plan(self, goal):
        from rclpy.action import ActionClient
        if goal.planning_options.plan_only is not True:
            raise ValueError('plan_only_required')
        ensure_isolated(self)
        client = ActionClient(self.node, MoveGroup, '/move_action')
        try:
            if not client.wait_for_server(timeout_sec=10.):
                raise TimeoutError('planner_unavailable')
            handle = self.wait(client.send_goal_async(goal))
            if not handle.accepted:
                raise RuntimeError('planning_goal_rejected')
            outcome = self.wait(handle.get_result_async(), timeout=30.)
            if outcome.status != 4:
                raise RuntimeError(f'planning_action_status_{outcome.status}')
            return outcome.result
        finally:
            client.destroy()

    def display(self, result):
        pub = self.node.create_publisher(DisplayTrajectory, '/display_planned_path', 1)
        msg = DisplayTrajectory(model_id='rm_65_description', trajectory_start=result.trajectory_start,
                                trajectory=[result.planned_trajectory])
        pub.publish(msg)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--recorded-replay', action='store_true', required=True)
    parser.add_argument('--candidate', type=Path)
    parser.add_argument('--hold-seconds', type=float, default=0., help='Publish explicitly labeled recorded-state display only')
    args = parser.parse_args()
    if args.output.exists():
        raise ValueError('refusing_to_overwrite_result')
    snapshot = json.loads(args.scene.read_text())
    from ament_index_python.packages import get_package_share_directory
    urdf = Path(get_package_share_directory('rm_description'))/'urdf/rm_65.urdf'
    if hashlib.sha256(urdf.read_bytes()).hexdigest() != snapshot['robot_model_sha256']:
        raise ValueError('robot_model_hash_mismatch')
    import rclpy
    rclpy.init(); node = rclpy.create_node('recorded_obstacle_preview')
    report = {'executable':False, 'status':'NOT_STARTED'}
    try:
        transport = RosPlanTransport(node) if args.candidate else RosSceneTransport(node)
        ensure_isolated(transport)
        report = publish_snapshot(snapshot, transport, replay=True)
        add_recorded_camera_guard(transport, snapshot)
        report['scene_digest'] = scene_digest(transport.read())
        report['camera_guard_present'] = True
        state = recorded_state(transport.read(), snapshot['observation']['joint_positions_rad'])
        report['start_state_valid_in_observed_model'] = transport.valid(state)
        report['planning'] = (plan_preview(snapshot, json.loads(args.candidate.read_text()), transport)
                              if args.candidate else {'status':'NO_VALIDATED_PREGRASP_CANDIDATE', 'executable':False})
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.with_suffix('.readback.json').write_text(json.dumps(raw_message(transport.read()), indent=2))
        if args.hold_seconds > 0:
            publish_recorded_display(node, snapshot, args.scene, args.hold_seconds)
    except Exception as exc:
        report.update(status='PREVIEW_FAILED', error=str(exc))
        raise
    finally:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2))
        node.destroy_node(); rclpy.shutdown()
    print(json.dumps({k:v for k,v in report.items() if k not in ('trajectory', 'trajectory_start')}, ensure_ascii=False))


def publish_recorded_display(node, snapshot, scene_path, duration):
    import rclpy
    from sensor_msgs.msg import JointState
    from sensor_msgs_py import point_cloud2
    from std_msgs.msg import Header
    from visualization_msgs.msg import Marker
    from rclpy.qos import QoSProfile, DurabilityPolicy
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    joint_pub = node.create_publisher(JointState, '/offline_preview/joint_states', 1)
    cloud_pub = node.create_publisher(point_cloud2.PointCloud2, '/offline_preview/observed_points', qos)
    label_pub = node.create_publisher(Marker, '/offline_preview/status', qos)
    cloud = np.load(scene_path.parent/'observation.npz')['xyz_m']
    label = Marker(); label.header.frame_id = 'base_link'; label.ns = 'recorded_snapshot'; label.id = 0
    label.type = Marker.TEXT_VIEW_FACING; label.action = Marker.ADD; label.pose.orientation.w = 1.
    label.pose.position.x = .3; label.pose.position.y = -.15
    label.pose.position.z = .65; label.scale.z = .025
    label.color.r = 1.; label.color.g = .7; label.color.a = 1.
    label.text = f'RECORDED PREVIEW / NO EXECUTION\nCapture epoch {snapshot["capture_stamp_s"]:.3f}\nUnknown space & robot envelope UNVERIFIED'
    label_pub.publish(label)
    header = Header(frame_id='base_link', stamp=node.get_clock().now().to_msg())
    cloud_pub.publish(point_cloud2.create_cloud_xyz32(header, cloud[::4].astype(np.float32)))
    until = time.monotonic()+min(duration, 3600.)
    while rclpy.ok() and time.monotonic() < until:
        joints = JointState(); joints.header.stamp = node.get_clock().now().to_msg()
        joints.header.frame_id = 'RECORDED_SNAPSHOT_NOT_LIVE'
        joints.name = JOINTS; joints.position = snapshot['observation']['joint_positions_rad']
        joint_pub.publish(joints)
        rclpy.spin_once(node, timeout_sec=.1)


if __name__ == '__main__':
    main()
