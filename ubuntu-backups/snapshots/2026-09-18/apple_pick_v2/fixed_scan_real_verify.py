#!/usr/bin/env python3
"""Guarded fixed-SCAN -> PREGRASP -> VERIFY real-robot test.

The node never publishes a gripper command.  PREGRASP is planned by MoveIt;
VERIFY is reached with a collision-checked, time-parameterized Cartesian path.
The default mode is preflight-only.  Real execution additionally requires the
operator to type the phrase configured in fixed_scan_real_verify.yaml.
"""

from __future__ import annotations

import argparse
from collections import deque
import json
import math
from pathlib import Path
import sys
import time

from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (
    AttachedCollisionObject,
    BoundingVolume,
    CollisionObject,
    Constraints,
    JointConstraint,
    OrientationConstraint,
    PlanningScene,
    PositionConstraint,
)
from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath, GetPositionIK, GetStateValidity
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String
import tf2_ros
import yaml


ROOT = Path(__file__).resolve().parent
DEFAULT_CONFIG = ROOT / "fixed_scan_real_verify.yaml"
SUCCESS = 1


def load_config(path: Path) -> dict:
    with path.expanduser().open("r", encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def scan_arrival_tolerance(config: dict) -> float:
    goal = float(config["scan"]["tolerance_rad"])
    arrival = float(config["scan"].get("arrival_tolerance_rad", goal))
    if arrival < goal:
        raise ValueError(
            "scan.arrival_tolerance_rad 不能小于规划 tolerance_rad"
        )
    return arrival

def duration_seconds(point) -> float:
    return float(point.time_from_start.sec) + float(point.time_from_start.nanosec) * 1e-9


def vertical_pose(position, yaw: float, pitch: float = 0.0) -> Pose:
    """Downward tool pose with an optional bounded pitch offset."""
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = map(float, position)
    # Quaternion for xyz RPY=(pi, pitch, yaw).  pitch=0 preserves the
    # original exactly vertical orientation and quaternion.
    cp = math.cos(0.5 * pitch)
    sp = math.sin(0.5 * pitch)
    cy = math.cos(0.5 * yaw)
    sy = math.sin(0.5 * yaw)
    pose.orientation.x = cp * cy
    pose.orientation.y = cp * sy
    pose.orientation.z = -sp * cy
    pose.orientation.w = sp * sy
    return pose


class FixedScanVerify(Node):
    JOINT_NAMES = [f"joint{i}" for i in range(1, 7)]
    SUPPORT_GUARD_ID = "apple_test_table_guard"

    def __init__(self, config: dict) -> None:
        super().__init__("fixed_scan_real_verify")
        self.cfg = config
        self.base = config["frames"]["base"]
        self.tcp = config["frames"]["tcp"]
        self.joints: dict[str, float] = {}
        self.joint_rx_monotonic = 0.0
        self.collecting = False
        self.apple_samples: deque[np.ndarray] = deque(maxlen=int(config["vision"]["sample_count"]))
        self.diagnostics: deque[dict] = deque(maxlen=30)

        self.create_subscription(JointState, "/joint_states", self.on_joint, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, config["vision"]["center_topic"], self.on_apple, 10)
        self.create_subscription(String, config["vision"]["diagnostics_topic"], self.on_diagnostics, 10)

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.move_client = ActionClient(self, MoveGroup, "/move_action")
        self.execute_client = ActionClient(self, ExecuteTrajectory, "/execute_trajectory")
        self.cartesian_client = self.create_client(GetCartesianPath, "/compute_cartesian_path")
        self.ik_client = self.create_client(GetPositionIK, "/compute_ik")
        self.validity_client = self.create_client(GetStateValidity, "/check_state_validity")
        self.scene_client = self.create_client(ApplyPlanningScene, "/apply_planning_scene")

    def on_joint(self, message: JointState) -> None:
        self.joints = {name: float(value) for name, value in zip(message.name, message.position)}
        self.joint_rx_monotonic = time.monotonic()

    def on_apple(self, message: PoseStamped) -> None:
        if not self.collecting or message.header.frame_id != self.base:
            return
        p = message.pose.position
        xyz = np.asarray([p.x, p.y, p.z], dtype=np.float64)
        if np.all(np.isfinite(xyz)):
            self.apple_samples.append(xyz)

    def on_diagnostics(self, message: String) -> None:
        try:
            value = json.loads(message.data)
        except (ValueError, TypeError):
            return
        if value.get("frame") == self.base:
            value["_received_monotonic"] = time.monotonic()
            self.diagnostics.append(value)

    def spin_until(self, predicate, timeout: float, label: str) -> bool:
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate():
                return True
        self.get_logger().error(f"超时：{label}")
        return False

    def wait_interfaces(self) -> None:
        timeout = float(self.cfg["safety"]["service_timeout_s"])
        checks = [
            (self.move_client.wait_for_server, "/move_action"),
            (self.execute_client.wait_for_server, "/execute_trajectory"),
            (self.cartesian_client.wait_for_service, "/compute_cartesian_path"),
            (self.ik_client.wait_for_service, "/compute_ik"),
            (self.validity_client.wait_for_service, "/check_state_validity"),
            (self.scene_client.wait_for_service, "/apply_planning_scene"),
        ]
        for wait, name in checks:
            if not wait(timeout_sec=timeout):
                raise RuntimeError(f"必要 MoveIt 接口不可用：{name}")
        if not self.spin_until(
            lambda: all(name in self.joints for name in self.JOINT_NAMES)
            and time.monotonic() - self.joint_rx_monotonic < 1.0,
            timeout,
            "等待新鲜 /joint_states",
        ):
            raise RuntimeError("没有收到完整的新鲜关节状态")
        if not self.spin_until(
            lambda: self.tf_buffer.can_transform(self.base, self.tcp, rclpy.time.Time()),
            timeout,
            f"等待 {self.base} <- {self.tcp} TF",
        ):
            raise RuntimeError(f"TF 不连通：{self.base} <- {self.tcp}")

    def current_joint_vector(self, names=None) -> np.ndarray:
        names = names or self.JOINT_NAMES
        return np.asarray([self.joints[name] for name in names], dtype=np.float64)

    def scan_vector(self) -> np.ndarray:
        values = self.cfg["scan"]["joints"]
        return np.asarray([float(values[name]) for name in self.JOINT_NAMES], dtype=np.float64)

    def make_joint_goal(self, target: np.ndarray) -> Constraints:
        constraints = []
        tolerance = float(self.cfg["scan"]["tolerance_rad"])
        for name, position in zip(self.JOINT_NAMES, target):
            item = JointConstraint()
            item.joint_name = name
            item.position = float(position)
            item.tolerance_above = tolerance
            item.tolerance_below = tolerance
            item.weight = 1.0
            constraints.append(item)
        return Constraints(name="fixed_scan", joint_constraints=constraints)

    def make_pose_goal(self, pose: Pose, label: str) -> Constraints:
        tolerance = float(self.cfg["targets"]["position_tolerance_m"])
        sphere = SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[tolerance])
        centre = Pose()
        centre.position = pose.position
        centre.orientation.w = 1.0

        position = PositionConstraint()
        position.header.frame_id = self.base
        position.link_name = self.tcp
        position.constraint_region = BoundingVolume(primitives=[sphere], primitive_poses=[centre])
        position.weight = 1.0

        angle = float(self.cfg["targets"]["orientation_tolerance_rad"])
        orientation = OrientationConstraint()
        orientation.header.frame_id = self.base
        orientation.link_name = self.tcp
        orientation.orientation = pose.orientation
        orientation.absolute_x_axis_tolerance = angle
        orientation.absolute_y_axis_tolerance = angle
        orientation.absolute_z_axis_tolerance = angle
        orientation.weight = 1.0
        return Constraints(
            name=label,
            position_constraints=[position],
            orientation_constraints=[orientation],
        )

    def plan(
        self,
        goal_constraint: Constraints,
        label: str,
        *,
        planning_time_s: float | None = None,
        planning_attempts: int | None = None,
    ):
        move = self.cfg["moveit"]
        goal = MoveGroup.Goal()
        goal.request.group_name = str(move["group"])
        goal.request.pipeline_id = str(move["pipeline"])
        goal.request.planner_id = str(move["planner"])
        goal.request.start_state.is_diff = True
        goal.request.goal_constraints = [goal_constraint]
        goal.request.num_planning_attempts = int(
            move["planning_attempts"]
            if planning_attempts is None
            else planning_attempts
        )
        goal.request.allowed_planning_time = float(
            move["planning_time_s"]
            if planning_time_s is None
            else planning_time_s
        )
        goal.request.max_velocity_scaling_factor = float(move["velocity_scaling"])
        goal.request.max_acceleration_scaling_factor = float(move["acceleration_scaling"])
        goal.planning_options.plan_only = True
        goal.planning_options.look_around = False
        goal.planning_options.replan = False

        future = self.move_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        handle = future.result()
        if handle is None or not handle.accepted:
            raise RuntimeError(f"{label}：MoveIt 拒绝规划请求")
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(
            self, result_future, timeout_sec=float(self.cfg["safety"]["motion_timeout_s"])
        )
        wrapped = result_future.result()
        if wrapped is None:
            raise RuntimeError(f"{label}：MoveIt 规划超时")
        result = wrapped.result
        if result.error_code.val != SUCCESS:
            raise RuntimeError(f"{label}：MoveIt 规划失败 error_code={result.error_code.val}")
        self.validate_trajectory(result.planned_trajectory, label)
        return result.planned_trajectory

    def validate_trajectory(self, trajectory, label: str) -> None:
        jt = trajectory.joint_trajectory
        if not jt.joint_names or not jt.points:
            raise RuntimeError(f"{label}：轨迹为空")
        positions = np.asarray([point.positions for point in jt.points], dtype=np.float64)
        if positions.ndim != 2 or not np.all(np.isfinite(positions)):
            raise RuntimeError(f"{label}：轨迹包含无效关节值")
        current = self.current_joint_vector(list(jt.joint_names))
        maximum = np.max(np.abs(positions - current), axis=0)
        limit = float(self.cfg["safety"]["max_joint_delta_rad"])
        if np.any(maximum > limit):
            report = dict(zip(jt.joint_names, np.round(maximum, 3).tolist()))
            raise RuntimeError(f"{label}：关节运动量超过 {limit:.2f} rad：{report}")
        times = np.asarray([duration_seconds(point) for point in jt.points])
        if times[-1] <= 0.0 or np.any(np.diff(times) < -1e-9):
            raise RuntimeError(f"{label}：轨迹时间戳无效")
        report = dict(zip(jt.joint_names, np.round(maximum, 3).tolist()))
        self.get_logger().info(
            f"{label} 规划有效：{len(jt.points)} 点，时长 {times[-1]:.2f}s，最大关节变化 {report}"
        )

    def execute(self, trajectory, label: str) -> None:
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory
        future = self.execute_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        handle = future.result()
        if handle is None or not handle.accepted:
            raise RuntimeError(f"{label}：轨迹执行请求被拒绝")
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(
            self, result_future, timeout_sec=float(self.cfg["safety"]["motion_timeout_s"])
        )
        wrapped = result_future.result()
        if wrapped is None or wrapped.result.error_code.val != SUCCESS:
            code = None if wrapped is None else wrapped.result.error_code.val
            raise RuntimeError(f"{label}：真实执行失败 error_code={code}")
        self.get_logger().info(f"{label}：真实执行完成")

    def wait_scan(self) -> None:
        target = self.scan_vector()
        tolerance = scan_arrival_tolerance(self.cfg)
        if not self.spin_until(
            lambda: np.max(np.abs(self.current_joint_vector() - target)) <= tolerance,
            10.0,
            "等待到达固定鸟瞰位",
        ):
            raise RuntimeError("未到达固定鸟瞰位")
        self.get_logger().info(
            f"固定鸟瞰位已确认，最大关节误差 {np.max(np.abs(self.current_joint_vector() - target)):.5f} rad"
        )

    def collect_apple(self) -> tuple[np.ndarray, float, dict]:
        count = int(self.cfg["vision"]["sample_count"])
        self.apple_samples.clear()
        self.diagnostics.clear()
        self.collecting = True
        try:
            ok = self.spin_until(
                lambda: len(self.apple_samples) >= count and len(self.diagnostics) >= count,
                float(self.cfg["vision"]["timeout_s"]),
                f"采集 {count} 个苹果中心样本",
            )
        finally:
            self.collecting = False
        if not ok:
            raise RuntimeError(
                f"苹果有效样本不足：center={len(self.apple_samples)}/{count}, diagnostics={len(self.diagnostics)}/{count}"
            )
        samples = np.asarray(self.apple_samples, dtype=np.float64)
        median = np.median(samples, axis=0)
        span = np.ptp(samples, axis=0)
        max_span = float(self.cfg["vision"]["max_axis_span_m"])
        if np.any(span > max_span):
            raise RuntimeError(f"苹果坐标不稳定，XYZ span={np.round(span, 6).tolist()} m")

        recent = list(self.diagnostics)[-count:]
        confidence = float(np.median([float(item["confidence"]) for item in recent]))
        radius = float(np.median([float(item["radius_m"]) for item in recent]))
        vision = self.cfg["vision"]
        if confidence < float(vision["min_confidence"]):
            raise RuntimeError(f"苹果检测置信度过低：{confidence:.3f}")
        if not float(vision["min_radius_m"]) <= radius <= float(vision["max_radius_m"]):
            raise RuntimeError(f"苹果半径异常：{radius:.4f} m")
        self.get_logger().info(
            f"苹果 base_link 中心={np.round(median, 6).tolist()} m，"
            f"XYZ span={np.round(span, 6).tolist()} m，confidence={confidence:.3f}，radius={radius:.4f}m"
        )
        return median, radius, recent[-1]

    def apply_scene(self, apple: np.ndarray, radius: float) -> float:
        scene_cfg = self.cfg["scene"]
        raw_table_top = float(apple[2] - radius)
        table_top = raw_table_top + float(scene_cfg["table_safety_inflation_m"])
        legacy_size = list(map(float, scene_cfg["table_size_xyz_m"]))
        if len(legacy_size) != 3 or any(value <= 0.0 for value in legacy_size):
            raise RuntimeError("支撑面碰撞体尺寸必须是三个正数")

        footprint_diameters = float(
            scene_cfg.get("support_guard_footprint_diameters", 0.0)
        )
        if footprint_diameters > 0.0:
            apple_diameter = 2.0 * float(radius)
            footprint = footprint_diameters * apple_diameter
            size = [footprint, footprint, legacy_size[2]]
            centre_xy = [float(apple[0]), float(apple[1])]
            guard_description = (
                f"局部支撑保护={footprint_diameters:.1f}倍苹果直径，"
                f"范围={footprint:.3f}x{footprint:.3f}m，"
                f"中心=({centre_xy[0]:.3f},{centre_xy[1]:.3f})m"
            )
        else:
            size = legacy_size
            centre_xy = list(map(float, scene_cfg["table_center_xy_m"]))
            guard_description = (
                f"固定支撑保护范围={size[0]:.3f}x{size[1]:.3f}m，"
                f"中心=({centre_xy[0]:.3f},{centre_xy[1]:.3f})m"
            )

        table_box = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=size)
        table_pose = Pose()
        table_pose.position.x, table_pose.position.y = centre_xy
        table_pose.position.z = table_top - 0.5 * size[2]
        table_pose.orientation.w = 1.0
        table = CollisionObject()
        table.header.frame_id = self.base
        table.id = self.SUPPORT_GUARD_ID
        table.primitives = [table_box]
        table.primitive_poses = [table_pose]
        table.operation = CollisionObject.ADD

        camera_box = SolidPrimitive(
            type=SolidPrimitive.BOX,
            dimensions=list(map(float, scene_cfg["camera_box_xyz_m"])),
        )
        camera_pose = Pose()
        camera_pose.position.x, camera_pose.position.y, camera_pose.position.z = map(
            float, scene_cfg["camera_translation_link6_m"]
        )
        (
            camera_pose.orientation.x,
            camera_pose.orientation.y,
            camera_pose.orientation.z,
            camera_pose.orientation.w,
        ) = map(float, scene_cfg["camera_quaternion_link6_xyzw"])
        camera = CollisionObject()
        camera.header.frame_id = "Link6"
        camera.id = "realsense_camera_guard"
        camera.primitives = [camera_box]
        camera.primitive_poses = [camera_pose]
        camera.operation = CollisionObject.ADD
        attached = AttachedCollisionObject()
        attached.link_name = "Link6"
        attached.object = camera
        attached.touch_links = ["Link6", "gripper_body_approx", self.tcp]

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = [table]
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects = [attached]
        request = ApplyPlanningScene.Request(scene=scene)
        future = self.scene_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        response = future.result()
        if response is None or not response.success:
            raise RuntimeError("MoveIt 未接受桌面/相机安全碰撞体")
        self.get_logger().info(
            f"规划场景已加入：支撑面原始顶面 z={raw_table_top:.4f}m，"
            f"安全顶面 z={table_top:.4f}m；{guard_description}；"
            "RealSense 使用临时保守包络"
        )
        return table_top

    def remove_support_guard(self) -> bool:
        """Remove the per-run synthetic support guard from MoveIt's world."""
        if not rclpy.ok():
            return False
        if not self.scene_client.service_is_ready() and not self.scene_client.wait_for_service(
            timeout_sec=1.0
        ):
            self.get_logger().warning(
                "无法清理虚拟支撑面：/apply_planning_scene 不可用"
            )
            return False

        support = CollisionObject()
        support.header.frame_id = self.base
        support.id = self.SUPPORT_GUARD_ID
        support.operation = CollisionObject.REMOVE
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = [support]
        request = ApplyPlanningScene.Request(scene=scene)
        future = self.scene_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        response = future.result()
        if response is None:
            self.get_logger().warning("删除虚拟支撑面请求没有收到 MoveIt 响应")
            return False
        if not response.success:
            # MoveIt returns success=false when the requested id does not
            # exist.  That already is the desired clean state.
            self.get_logger().info("虚拟支撑面不存在或已经删除，无需重复清理")
            return True
        self.get_logger().info("已从 MoveIt PlanningScene 删除本轮虚拟支撑面")
        return True

    def state_is_valid(self, state, label: str) -> None:
        request = GetStateValidity.Request()
        request.robot_state = state
        request.group_name = str(self.cfg["moveit"]["group"])
        future = self.validity_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        response = future.result()
        if response is None or not response.valid:
            contacts = [] if response is None else [
                f"{item.contact_body_1}<->{item.contact_body_2}" for item in response.contacts[:5]
            ]
            raise RuntimeError(f"{label} 碰撞检查失败：{contacts}")

    def current_state_is_valid(self) -> None:
        from moveit_msgs.msg import RobotState

        state = RobotState()
        state.joint_state.name = list(self.JOINT_NAMES)
        state.joint_state.position = self.current_joint_vector().tolist()
        self.state_is_valid(state, "当前姿态")

    def solve_ik(
        self,
        pose: Pose,
        label: str,
        *,
        attempts: int | None = None,
        timeout_s: float | None = None,
        quiet: bool = False,
    ):
        from builtin_interfaces.msg import Duration as DurationMsg
        from moveit_msgs.msg import RobotState

        attempts = int(
            self.cfg["moveit"].get("ik_attempts", 12)
            if attempts is None
            else attempts
        )
        timeout_s = float(
            self.cfg["moveit"].get("ik_timeout_s", 1.0)
            if timeout_s is None
            else timeout_s
        )
        if attempts < 1 or timeout_s <= 0.0:
            raise RuntimeError("IK attempts 必须 >= 1，timeout_s 必须 > 0")
        current_names = list(self.JOINT_NAMES)
        current_values = self.current_joint_vector(current_names)
        best_state = None
        best_max_delta = float("inf")
        error_codes = []

        for _ in range(attempts):
            seed = RobotState()
            seed.joint_state.name = current_names
            seed.joint_state.position = current_values.tolist()
            seed.is_diff = True

            request = GetPositionIK.Request()
            request.ik_request.group_name = str(self.cfg["moveit"]["group"])
            request.ik_request.robot_state = seed
            request.ik_request.ik_link_name = self.tcp
            request.ik_request.pose_stamped = PoseStamped()
            request.ik_request.pose_stamped.header.frame_id = self.base
            request.ik_request.pose_stamped.pose = pose
            request.ik_request.avoid_collisions = True
            timeout_whole = int(timeout_s)
            timeout_nanos = int(round((timeout_s - timeout_whole) * 1e9))
            request.ik_request.timeout = DurationMsg(
                sec=timeout_whole, nanosec=timeout_nanos
            )
            future = self.ik_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_s + 2.0)
            response = future.result()
            code = None if response is None else response.error_code.val
            error_codes.append(code)
            if response is None or code != SUCCESS:
                continue

            solution = response.solution
            values_by_name = dict(
                zip(solution.joint_state.name, solution.joint_state.position)
            )
            if not all(name in values_by_name for name in current_names):
                continue
            solution_values = np.asarray(
                [values_by_name[name] for name in current_names], dtype=np.float64
            )
            max_delta = float(np.max(np.abs(solution_values - current_values)))
            if max_delta < best_max_delta:
                best_state = solution
                best_max_delta = max_delta

        if best_state is None:
            raise RuntimeError(
                f"{label} IK 在 {attempts} 次当前关节种子尝试后仍失败："
                f"error_codes={error_codes}"
            )
        self.state_is_valid(best_state, f"{label} IK 解")
        if not quiet:
            self.get_logger().info(
                f"{label} IK 与碰撞检查通过：{attempts} 次尝试，"
                f"所选解最大关节变化 {best_max_delta:.3f} rad"
            )
        return best_state

    def cartesian(self, verify_pose: Pose):
        cfg = self.cfg["cartesian"]
        request = GetCartesianPath.Request()
        request.header.frame_id = self.base
        request.header.stamp = self.get_clock().now().to_msg()
        request.start_state.is_diff = True
        request.group_name = str(self.cfg["moveit"]["group"])
        request.link_name = self.tcp
        request.waypoints = [verify_pose]
        request.max_step = float(cfg["max_step_m"])
        request.jump_threshold = float(cfg["jump_threshold"])
        request.prismatic_jump_threshold = 0.0
        request.revolute_jump_threshold = float(cfg["revolute_jump_threshold_rad"])
        request.avoid_collisions = True
        request.max_velocity_scaling_factor = float(self.cfg["moveit"]["velocity_scaling"])
        request.max_acceleration_scaling_factor = float(self.cfg["moveit"]["acceleration_scaling"])
        request.cartesian_speed_limited_link = self.tcp
        request.max_cartesian_speed = float(cfg["max_tcp_speed_m_s"])
        future = self.cartesian_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
        response = future.result()
        if response is None or response.error_code.val != SUCCESS:
            code = None if response is None else response.error_code.val
            raise RuntimeError(f"PREGRASP -> VERIFY 笛卡尔规划失败 error_code={code}")
        if float(response.fraction) < float(cfg["required_fraction"]):
            raise RuntimeError(f"笛卡尔路径比例不足：fraction={response.fraction:.4f}")
        self.validate_trajectory(response.solution, "PREGRASP -> VERIFY 笛卡尔")
        self.get_logger().info(f"笛卡尔路径 fraction={response.fraction:.4f}，TCP 限速 {cfg['max_tcp_speed_m_s']}m/s")
        return response.solution

    def tcp_position(self) -> np.ndarray:
        transform = self.tf_buffer.lookup_transform(self.base, self.tcp, rclpy.time.Time())
        t = transform.transform.translation
        return np.asarray([t.x, t.y, t.z], dtype=np.float64)

    def wait_tcp_arrival(self, target: Pose, label: str) -> np.ndarray:
        expected = np.asarray([target.position.x, target.position.y, target.position.z])
        tolerance = float(self.cfg["safety"]["max_tcp_final_error_m"])
        stable_time = float(self.cfg["safety"]["tcp_stable_time_s"])
        deadline = time.monotonic() + 15.0
        stable_since = None
        actual = self.tcp_position()
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            actual = self.tcp_position()
            error = float(np.linalg.norm(actual - expected))
            if error <= tolerance:
                if stable_since is None:
                    stable_since = time.monotonic()
                if time.monotonic() - stable_since >= stable_time:
                    self.get_logger().info(
                        f"{label} 已稳定到位：误差 {error * 1000.0:.2f} mm，持续 {stable_time:.1f}s"
                    )
                    return actual
            else:
                stable_since = None
        raise RuntimeError(
            f"{label} 到位或稳定性检查失败：actual={np.round(actual, 6).tolist()}, "
            f"target={np.round(expected, 6).tolist()}"
        )


def main() -> int:
    parser = argparse.ArgumentParser(description="固定鸟瞰位真机 PREGRASP/VERIFY 测试")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--execute", action="store_true", help="通过预检后执行真机运动")
    parser.add_argument(
        "--operator-confirmed",
        action="store_true",
        help="确认夹爪已打开、工作区无人无障碍、急停可用；仍需输入执行口令",
    )
    args, ros_args = parser.parse_known_args()
    config = load_config(args.config)
    rclpy.init(args=ros_args)
    node = FixedScanVerify(config)
    try:
        node.wait_interfaces()
        node.remove_support_guard()
        target_scan = node.scan_vector()
        scan_error = float(np.max(np.abs(node.current_joint_vector() - target_scan)))
        if scan_error <= float(config["scan"]["tolerance_rad"]):
            node.get_logger().info("机械臂已经位于预先定义的固定鸟瞰位，无需制造多余运动")
        else:
            scan_trajectory = node.plan(node.make_joint_goal(target_scan), "当前位置 -> SCAN")
            if not args.execute:
                node.get_logger().info("预检模式：SCAN 轨迹已生成但未执行")
                return 0
            if not args.operator_confirmed:
                raise RuntimeError("真实执行被锁定：缺少 --operator-confirmed")
            node.execute(scan_trajectory, "当前位置 -> SCAN")
        node.wait_scan()

        apple, radius, diagnostics = node.collect_apple()
        yaw = float(config["targets"]["vertical_yaw_rad"])
        pre_xyz = apple + np.asarray([0.0, 0.0, float(config["targets"]["pregrasp_above_apple_m"])])
        verify_xyz = apple + np.asarray([0.0, 0.0, float(config["targets"]["verify_above_apple_m"])])
        pre_pose = vertical_pose(pre_xyz, yaw)
        verify_pose = vertical_pose(verify_xyz, yaw)
        node.get_logger().info(f"PREGRASP tcp_link={np.round(pre_xyz, 6).tolist()} m")
        node.get_logger().info(f"VERIFY   tcp_link={np.round(verify_xyz, 6).tolist()} m")

        table_top = node.apply_scene(apple, radius)
        node.current_state_is_valid()
        node.solve_ik(pre_pose, "PREGRASP")
        node.solve_ik(verify_pose, "VERIFY")
        pre_trajectory = node.plan(node.make_pose_goal(pre_pose, "pregrasp"), "SCAN -> PREGRASP")

        if not args.execute:
            node.get_logger().info("预检完成：未执行真机运动。使用 --execute --operator-confirmed 才会执行")
            return 0
        if not args.operator_confirmed:
            raise RuntimeError("真实执行被锁定：缺少 --operator-confirmed")
        phrase = str(config["safety"]["require_operator_phrase"])
        typed = input(
            f"\n确认：夹爪已打开、路径及桌面上无人无障碍、急停可用；输入 {phrase} 开始真机运动："
        ).strip()
        if typed != phrase:
            raise RuntimeError("执行口令不匹配，真机运动已取消")

        # SCAN -> PREGRASP uses the already collision-checked MoveIt trajectory.
        node.execute(pre_trajectory, "SCAN -> PREGRASP")
        node.wait_tcp_arrival(pre_pose, "PREGRASP")

        # Recompute from the actual PREGRASP state, require a complete Cartesian path,
        # then execute it at 0.05 scaling and 0.01 m/s TCP cap.
        cartesian_trajectory = node.cartesian(verify_pose)
        node.execute(cartesian_trajectory, "PREGRASP -> VERIFY")
        actual = node.wait_tcp_arrival(verify_pose, "VERIFY")
        record = {
            "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
            "apple_center_base_m": np.round(apple, 7).tolist(),
            "apple_radius_m": radius,
            "vision_diagnostics": diagnostics,
            "pregrasp_base_m": np.round(pre_xyz, 7).tolist(),
            "verify_base_m": np.round(verify_xyz, 7).tolist(),
            "actual_tcp_base_m": np.round(actual, 7).tolist(),
            "table_guard_top_z_m": table_top,
            "velocity_scaling": 0.05,
            "acceleration_scaling": 0.05,
            "gripper_commanded": False,
        }
        result_dir = ROOT / "results"
        result_dir.mkdir(exist_ok=True)
        output = result_dir / f"fixed_scan_verify_{time.strftime('%Y%m%d_%H%M%S')}.json"
        output.write_text(json.dumps(record, indent=2, ensure_ascii=False), encoding="utf-8")
        print("\n已到验证点，等待人工确认", flush=True)
        print(f"执行记录：{output}", flush=True)
        return 0
    except (KeyboardInterrupt, RuntimeError) as error:
        node.get_logger().error(str(error))
        return 2
    finally:
        node.remove_support_guard()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
