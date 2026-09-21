#!/usr/bin/env python3
"""Measure visual apple-centre error against a manually aligned TCP.

The default mode is read-only.  With the explicit ``--auto-scan`` option, a
separate guarded MoveIt controller moves to the configured fixed SCAN pose and,
opens the gripper for uncontaminated manual alignment.  After measurement it
retreats upward before returning to SCAN.  It never initializes/zeros or closes
the gripper, and it never changes the grasp configuration.
"""

from __future__ import annotations

import argparse
import csv
from datetime import datetime
import json
import math
from pathlib import Path
import sys
import time

from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import Constraints, JointConstraint, RobotState
from moveit_msgs.srv import GetCartesianPath
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import tf2_ros
import yaml

from fixed_scan_real_verify import FixedScanVerify, SUCCESS
from project_paths import RESULTS_DIR


ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT.parent))
from modbus_gripper_ros import ToolRs485ModbusGripper  # noqa: E402

DEFAULT_CONFIG = ROOT / "single_apple_full_grasp.yaml"
DEFAULT_RESULT_ROOT = RESULTS_DIR / "visual_tcp_offset"


def point_statistics(points: list[list[float]]) -> dict:
    array = np.asarray(points, dtype=np.float64)
    median = np.median(array, axis=0)
    mean = np.mean(array, axis=0)
    return {
        "sample_count": int(len(array)),
        "median_xyz_m": median.tolist(),
        "mean_xyz_m": mean.tolist(),
        "std_xyz_mm": (np.std(array, axis=0) * 1000.0).tolist(),
        "axis_span_mm": (np.ptp(array, axis=0) * 1000.0).tolist(),
        "max_distance_from_median_mm": float(
            np.max(np.linalg.norm(array - median, axis=1)) * 1000.0
        ),
    }


def optional_median(items: list[dict], key: str) -> float | None:
    values = []
    for item in items:
        try:
            value = float(item[key])
        except (KeyError, TypeError, ValueError):
            continue
        if math.isfinite(value):
            values.append(value)
    return None if not values else float(np.median(values))


def quaternion_angle_deg(first: list[float], second: list[float]) -> float:
    a = np.asarray(first, dtype=np.float64)
    b = np.asarray(second, dtype=np.float64)
    a /= np.linalg.norm(a)
    b /= np.linalg.norm(b)
    # q and -q represent the same rotation.
    dot = float(np.clip(abs(np.dot(a, b)), 0.0, 1.0))
    return math.degrees(2.0 * math.acos(dot))


def tool_z_tilt_from_base_down_deg(quaternion_xyzw: list[float]) -> float:
    """Angle between the TCP +Z axis and base_link -Z; yaw is irrelevant."""
    q = np.asarray(quaternion_xyzw, dtype=np.float64)
    q /= np.linalg.norm(q)
    x, y, _z, _w = q
    # Third component of R(q) * [0, 0, 1].  A perfectly downward tool has
    # tool_z_base.z == -1.
    tool_z_base_z = 1.0 - 2.0 * (x * x + y * y)
    return math.degrees(math.acos(float(np.clip(-tool_z_base_z, -1.0, 1.0))))


class VisualTcpOffsetRecorder(Node):
    def __init__(self, base_frame: str, tcp_frame: str) -> None:
        super().__init__("measure_visual_tcp_offset")
        self.base_frame = base_frame
        self.tcp_frame = tcp_frame
        self.collecting_vision = False
        self.vision_samples: list[dict] = []
        self.diagnostics: list[dict] = []
        self.latest_joint_names: list[str] = []
        self.latest_joint_positions: list[float] = []
        self.latest_joint_received_monotonic = 0.0

        self.create_subscription(
            PoseStamped, "/apple_pick_v2/apple_center", self.on_apple, 30
        )
        self.create_subscription(
            String, "/apple_pick_v2/apple_diagnostics", self.on_diagnostics, 30
        )
        self.create_subscription(
            JointState, "/joint_states", self.on_joint_state, qos_profile_sensor_data
        )
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def current_joint_map(self) -> dict[str, float]:
        return dict(zip(self.latest_joint_names, self.latest_joint_positions))

    def on_joint_state(self, message: JointState) -> None:
        self.latest_joint_names = list(message.name)
        self.latest_joint_positions = [float(value) for value in message.position]
        self.latest_joint_received_monotonic = time.monotonic()

    def on_apple(self, message: PoseStamped) -> None:
        if not self.collecting_vision or message.header.frame_id != self.base_frame:
            return
        p = message.pose.position
        xyz = [float(p.x), float(p.y), float(p.z)]
        if not np.all(np.isfinite(xyz)):
            return
        self.vision_samples.append(
            {
                "received_monotonic_s": time.monotonic(),
                "stamp_ns": int(
                    message.header.stamp.sec * 1_000_000_000
                    + message.header.stamp.nanosec
                ),
                "xyz_m": xyz,
            }
        )

    def on_diagnostics(self, message: String) -> None:
        if not self.collecting_vision:
            return
        try:
            value = json.loads(message.data)
        except (TypeError, ValueError):
            return
        if value.get("frame") != self.base_frame:
            return
        value["received_monotonic_s"] = time.monotonic()
        self.diagnostics.append(value)

    def spin_for(self, duration_s: float) -> None:
        deadline = time.monotonic() + duration_s
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=min(0.05, deadline - time.monotonic()))

    def collect_vision(self, count: int, timeout_s: float) -> tuple[list[dict], list[dict]]:
        # Drain queued DDS samples first, then accept only callbacks received in
        # this capture window.
        self.collecting_vision = False
        self.spin_for(0.5)
        self.vision_samples = []
        self.diagnostics = []
        self.collecting_vision = True
        deadline = time.monotonic() + timeout_s
        try:
            while rclpy.ok() and time.monotonic() < deadline:
                if len(self.vision_samples) >= count and len(self.diagnostics) >= count:
                    break
                rclpy.spin_once(self, timeout_sec=0.1)
        finally:
            self.collecting_vision = False
        return self.vision_samples[:count], self.diagnostics[:count]

    def collect_tcp(self, count: int, interval_s: float, timeout_s: float) -> list[dict]:
        self.spin_for(0.5)
        samples: list[dict] = []
        deadline = time.monotonic() + timeout_s
        next_sample = time.monotonic()
        while rclpy.ok() and time.monotonic() < deadline and len(samples) < count:
            rclpy.spin_once(self, timeout_sec=0.02)
            now = time.monotonic()
            if now < next_sample:
                continue
            next_sample = now + interval_s
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.base_frame, self.tcp_frame, rclpy.time.Time()
                )
            except Exception as exc:  # tf2 exception types vary across ROS releases
                self.get_logger().warn(
                    f"等待 {self.base_frame} <- {self.tcp_frame}：{exc}",
                    throttle_duration_sec=2.0,
                )
                continue
            t = transform.transform.translation
            q = transform.transform.rotation
            xyz = [float(t.x), float(t.y), float(t.z)]
            if not np.all(np.isfinite(xyz)):
                continue
            samples.append(
                {
                    "received_monotonic_s": now,
                    "stamp_ns": int(
                        transform.header.stamp.sec * 1_000_000_000
                        + transform.header.stamp.nanosec
                    ),
                    "xyz_m": xyz,
                    "quaternion_xyzw": [float(q.x), float(q.y), float(q.z), float(q.w)],
                }
            )
        return samples


class ScanMotionController(FixedScanVerify):
    """Small adapter around the already validated MoveIt execution path."""

    def __init__(self, config: dict) -> None:
        super().__init__(config)
        self.gripper = ToolRs485ModbusGripper(self)
        self.gripper_configured = False

    def open_gripper_for_measurement(self) -> float:
        cfg = self.cfg["modbus_gripper"]
        if not self.gripper_configured:
            # Deliberately do not call initialize(): mechanical zeroing can
            # first close the jaws and disturb the fixed apple.
            self.gripper.configure(cfg)
            self.gripper_configured = True
        opening_mm = float(cfg["open_mm"])
        self.gripper.set_opening_mm(cfg, opening_mm)
        settle_s = float(cfg.get("close_settle_s", 1.0))
        deadline = time.monotonic() + settle_s
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        self.get_logger().info(
            f"测量夹爪已打开到 {opening_mm:.1f} mm；未执行机械归零，整轮不会闭合"
        )
        return opening_mm

    def measurement_scan_tolerance(self) -> float:
        return min(float(self.cfg["scan"]["tolerance_rad"]), 0.005)

    def make_measurement_scan_goal(self, target: np.ndarray) -> Constraints:
        # The normal grasp workflow accepts 0.015 rad around SCAN.  Offset
        # calibration needs a more repeatable camera pose, so use 0.003 rad
        # goal constraints and independently verify within 0.005 rad.
        tolerance = min(self.measurement_scan_tolerance(), 0.003)
        joints = []
        for name, position in zip(self.JOINT_NAMES, target):
            joint = JointConstraint()
            joint.joint_name = name
            joint.position = float(position)
            joint.tolerance_above = tolerance
            joint.tolerance_below = tolerance
            joint.weight = 1.0
            joints.append(joint)
        return Constraints(name="measurement_fixed_scan", joint_constraints=joints)

    def cartesian_to(
        self,
        target: Pose,
        label: str,
        *,
        velocity_scaling: float,
        acceleration_scaling: float,
        max_tcp_speed_m_s: float,
    ):
        cfg = self.cfg["cartesian"]
        request = GetCartesianPath.Request()
        request.header.frame_id = self.base
        request.header.stamp = self.get_clock().now().to_msg()
        request.start_state = RobotState(is_diff=True)
        request.group_name = str(self.cfg["moveit"]["group"])
        request.link_name = self.tcp
        request.waypoints = [target]
        request.max_step = float(cfg["max_step_m"])
        request.jump_threshold = float(cfg["jump_threshold"])
        request.prismatic_jump_threshold = 0.0
        request.revolute_jump_threshold = float(cfg["revolute_jump_threshold_rad"])
        request.avoid_collisions = True
        request.max_velocity_scaling_factor = float(velocity_scaling)
        request.max_acceleration_scaling_factor = float(acceleration_scaling)
        request.cartesian_speed_limited_link = self.tcp
        request.max_cartesian_speed = float(max_tcp_speed_m_s)
        future = self.cartesian_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
        response = future.result()
        if response is None or response.error_code.val != SUCCESS:
            code = None if response is None else response.error_code.val
            raise RuntimeError(f"{label} 笛卡尔规划失败 error_code={code}")
        fraction = float(response.fraction)
        if fraction < float(cfg["required_fraction"]):
            raise RuntimeError(f"{label} 路径比例不足：fraction={fraction:.4f}")
        self.validate_trajectory(response.solution, f"{label} 笛卡尔")
        self.get_logger().info(
            f"{label}：fraction={fraction:.4f}，速度/加速度缩放="
            f"{velocity_scaling:.2f}/{acceleration_scaling:.2f}，"
            f"TCP 限速={max_tcp_speed_m_s:.3f}m/s"
        )
        return response.solution

    def move_to_fixed_scan(self, label: str) -> None:
        self.wait_interfaces()
        target = self.scan_vector()
        error = float(np.max(np.abs(self.current_joint_vector() - target)))
        tolerance = self.measurement_scan_tolerance()
        if error > tolerance:
            trajectory = self.plan(self.make_measurement_scan_goal(target), label)
            self.execute(trajectory, label)
        deadline = time.monotonic() + 10.0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            error = float(np.max(np.abs(self.current_joint_vector() - target)))
            if error <= tolerance:
                self.get_logger().info(
                    f"测量固定 SCAN 已确认，最大关节误差 {error:.5f} rad"
                )
                return
        raise RuntimeError(
            f"未到达测量固定 SCAN：最大关节误差 {error:.5f} rad > {tolerance:.5f} rad"
        )

    def is_at_fixed_scan(self) -> bool:
        self.wait_interfaces()
        error = float(np.max(np.abs(self.current_joint_vector() - self.scan_vector())))
        return error <= self.measurement_scan_tolerance()

    def ensure_fixed_scan_safely(self, retreat_m: float) -> float:
        """Reach SCAN; when starting elsewhere, retreat before joint motion."""
        if self.is_at_fixed_scan():
            self.move_to_fixed_scan("确认测量固定 SCAN")
            return 0.0
        target = self.scan_vector()
        joint_error = float(np.max(np.abs(self.current_joint_vector() - target)))
        transform = self.tf_buffer.lookup_transform(
            self.base, self.tcp, rclpy.time.Time()
        )
        tcp_z = float(transform.transform.translation.z)
        # A pose just outside the tight calibration gate is not a low workspace
        # pose.  Likewise, a TCP already above 0.10 m is safe for the small or
        # normal joint-space return; asking it to retreat farther upward can
        # unnecessarily hit the workspace boundary.
        if joint_error <= 0.05 or tcp_z >= 0.10:
            self.get_logger().info(
                f"当前位置可直接精确回 SCAN：最大关节误差 {joint_error:.5f} rad，"
                f"TCP z={tcp_z:.3f}m"
            )
            self.move_to_fixed_scan("当前位置 -> 测量固定 SCAN")
            return 0.0
        self.get_logger().info(
            f"当前位置明显远离 SCAN 且 TCP 较低（z={tcp_z:.3f}m）："
            "先做完整笛卡尔上撤，再进行关节空间回位"
        )
        return self.retreat_up_then_scan(retreat_m)

    def retreat_up_then_scan(self, retreat_m: float) -> float:
        self.wait_interfaces()
        # TF and /joint_states can already look stable while MoveIt's internal
        # CurrentStateMonitor/PlanningScene is still catching up after manual
        # teach-pendant motion.  Give it a deterministic synchronization window.
        sync_deadline = time.monotonic() + 2.0
        while rclpy.ok() and time.monotonic() < sync_deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

        speed = float(self.cfg["lift_cartesian"]["max_tcp_speed_m_s"])
        candidates = []
        for distance in [retreat_m, 0.08, 0.06, 0.05]:
            bounded = min(float(retreat_m), float(distance))
            if bounded >= 0.05 and all(abs(bounded - old) > 1e-9 for old in candidates):
                candidates.append(bounded)

        chosen_distance = None
        trajectory = None
        retreat = None
        failures: list[str] = []
        for distance in candidates:
            # Retry each complete target after a short state-synchronization
            # interval.  Partial Cartesian trajectories are never executed.
            for attempt in range(1, 4):
                transform = self.tf_buffer.lookup_transform(
                    self.base, self.tcp, rclpy.time.Time()
                )
                t = transform.transform.translation
                q = transform.transform.rotation
                candidate = Pose()
                candidate.position.x = float(t.x)
                candidate.position.y = float(t.y)
                candidate.position.z = float(t.z) + distance
                candidate.orientation.x = float(q.x)
                candidate.orientation.y = float(q.y)
                candidate.orientation.z = float(q.z)
                candidate.orientation.w = float(q.w)
                label = f"安全上撤 {distance:.3f}m（尝试 {attempt}/3）"
                try:
                    planned = self.cartesian_to(
                        candidate,
                        label,
                        velocity_scaling=0.03,
                        acceleration_scaling=0.03,
                        max_tcp_speed_m_s=speed,
                    )
                except RuntimeError as exc:
                    failures.append(str(exc))
                    retry_deadline = time.monotonic() + 0.5
                    while rclpy.ok() and time.monotonic() < retry_deadline:
                        rclpy.spin_once(self, timeout_sec=0.05)
                    continue
                chosen_distance = distance
                trajectory = planned
                retreat = candidate
                break
            if trajectory is not None:
                break

        if trajectory is None or retreat is None or chosen_distance is None:
            raise RuntimeError(
                "安全上撤所有完整路径候选均失败；未执行部分轨迹。最后原因："
                + (failures[-1] if failures else "unknown")
            )
        if chosen_distance < retreat_m - 1e-9:
            self.get_logger().warn(
                f"100% 完整路径改用上撤 {chosen_distance:.3f}m；"
                f"原请求 {retreat_m:.3f}m 未执行"
            )
        self.execute(trajectory, "人工对准点 -> 安全上撤")
        self.wait_tcp_arrival(retreat, "安全上撤点")
        self.move_to_fixed_scan("安全上撤点 -> SCAN")
        return chosen_distance


def prompt(message: str) -> None:
    choice = input(message).strip().lower()
    if choice in {"q", "quit", "cancel", "取消"}:
        raise KeyboardInterrupt


def load_config(path: Path) -> dict:
    with path.expanduser().open("r", encoding="utf-8") as handle:
        value = yaml.safe_load(handle)
    if not isinstance(value, dict) or "scan" not in value:
        raise ValueError(f"配置缺少 scan：{path}")
    return value


def require_manual_scan(
    node: VisualTcpOffsetRecorder,
    scan_joints: dict[str, float],
    tolerance_rad: float,
    stage: str,
) -> dict[str, float]:
    """Require, but never command, the configured fixed SCAN pose."""
    while rclpy.ok():
        node.spin_for(0.5)
        current = node.current_joint_map()
        missing = [name for name in scan_joints if name not in current]
        stale = time.monotonic() - node.latest_joint_received_monotonic > 2.0
        if not missing and not stale:
            errors = {
                name: current[name] - float(target)
                for name, target in scan_joints.items()
            }
            max_error = max(abs(value) for value in errors.values())
            if max_error <= tolerance_rad:
                print(
                    f"{stage}：固定 SCAN 已确认，最大关节误差 "
                    f"{max_error:.5f} rad（容差 {tolerance_rad:.5f} rad）"
                )
                return {name: current[name] for name in scan_joints}
            print(
                f"{stage}：当前不在固定 SCAN，最大关节误差 "
                f"{max_error:.4f} rad。"
            )
            print(
                "  当前-目标关节误差(rad)："
                + str({name: round(value, 4) for name, value in errors.items()})
            )
        else:
            reason = f"缺少关节 {missing}" if missing else "/joint_states 已过期"
            print(f"{stage}：无法确认固定 SCAN（{reason}）。")
        prompt("  请用示教器安全回到固定鸟瞰位，稳定后按 Enter 重新检查（q 取消）：")
    raise KeyboardInterrupt


def append_history(path: Path, row: dict) -> None:
    columns = [
        "created_at",
        "label",
        "visual_x_m",
        "visual_y_m",
        "visual_z_m",
        "tcp_x_m",
        "tcp_y_m",
        "tcp_z_m",
        "delta_x_mm",
        "delta_y_mm",
        "delta_z_mm",
        "delta_norm_mm",
        "confidence_median",
        "radius_median_m",
        "vision_span_x_mm",
        "vision_span_y_mm",
        "vision_span_z_mm",
    ]
    new_file = not path.exists()
    with path.open("a", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=columns)
        if new_file:
            writer.writeheader()
        writer.writerow({key: row.get(key) for key in columns})


def save_result(
    output_root: Path,
    label: str,
    base_frame: str,
    tcp_frame: str,
    vision_samples: list[dict],
    diagnostics: list[dict],
    tcp_samples: list[dict],
    scan_expected: dict[str, float],
    scan_observed: dict[str, float],
    auto_scan_enabled: bool,
    retreat_before_return_m: float,
    gripper_opened_for_measurement: bool,
    gripper_opening_mm: float | None,
    reference_orientation_xyzw: list[float],
    orientation_error_deg: float,
    orientation_tolerance_deg: float,
    joint_names: list[str],
    joint_positions: list[float],
) -> tuple[Path, dict]:
    created_at = datetime.now().astimezone().isoformat()
    vision_stats = point_statistics([sample["xyz_m"] for sample in vision_samples])
    tcp_stats = point_statistics([sample["xyz_m"] for sample in tcp_samples])
    visual = np.asarray(vision_stats["median_xyz_m"], dtype=np.float64)
    tcp = np.asarray(tcp_stats["median_xyz_m"], dtype=np.float64)
    delta = tcp - visual
    confidence = optional_median(diagnostics, "confidence")
    radius = optional_median(diagnostics, "radius_m")
    sync_error_ms = optional_median(diagnostics, "sync_error_ms")

    result = {
        "created_at": created_at,
        "label": label,
        "read_only_measurement": not auto_scan_enabled,
        "formula": "delta_base = manually_aligned_tcp - frozen_visual_apple_center",
        "interpretation": "Do not apply automatically. If repeated trials prove a stable fixed bias, candidate correction is apple_corrected = apple_visual + delta_base.",
        "frames": {"base": base_frame, "tcp": tcp_frame},
        "fixed_scan_gate": {
            "required_for_every_visual_capture": True,
            "auto_scan_enabled": auto_scan_enabled,
            "retreat_before_return_m": retreat_before_return_m,
            "expected_joint_positions_rad": scan_expected,
            "observed_joint_positions_rad": scan_observed,
        },
        "measurement_gripper": {
            "opened_before_visual_capture": gripper_opened_for_measurement,
            "opening_mm": gripper_opening_mm,
            "mechanical_zeroing_called": False,
            "close_command_sent": False,
        },
        "topics": {
            "apple_center": "/apple_pick_v2/apple_center",
            "diagnostics": "/apple_pick_v2/apple_diagnostics",
            "joint_states": "/joint_states",
        },
        "vision": {
            "statistics": vision_stats,
            "samples": vision_samples,
            "diagnostics": diagnostics,
            "confidence_median": confidence,
            "radius_median_m": radius,
            "sync_error_median_ms": sync_error_ms,
        },
        "manual_alignment": {
            "tcp_statistics": tcp_stats,
            "tcp_samples": tcp_samples,
            "latest_tcp_quaternion_xyzw": tcp_samples[-1]["quaternion_xyzw"],
            "reference_vertical_quaternion_xyzw": reference_orientation_xyzw,
            "orientation_error_deg": orientation_error_deg,
            "orientation_tolerance_deg": orientation_tolerance_deg,
            "orientation_gate_passed": orientation_error_deg <= orientation_tolerance_deg,
            "orientation_gate_definition": "angle(TCP +Z, base_link -Z); yaw ignored",
            "joint_names": joint_names,
            "joint_positions_rad": joint_positions,
        },
        "offset": {
            "delta_base_m": delta.tolist(),
            "delta_base_mm": (delta * 1000.0).tolist(),
            "norm_mm": float(np.linalg.norm(delta) * 1000.0),
        },
    }

    output_root.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir = output_root / f"offset_{stamp}"
    suffix = 1
    while run_dir.exists():
        run_dir = output_root / f"offset_{stamp}_{suffix}"
        suffix += 1
    run_dir.mkdir()
    (run_dir / "result.json").write_text(
        json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8"
    )

    with (run_dir / "samples.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["source", "index", "x_m", "y_m", "z_m", "stamp_ns"])
        for index, sample in enumerate(vision_samples, start=1):
            writer.writerow(["visual_apple_center", index, *sample["xyz_m"], sample["stamp_ns"]])
        for index, sample in enumerate(tcp_samples, start=1):
            writer.writerow(["manual_aligned_tcp", index, *sample["xyz_m"], sample["stamp_ns"]])

    span = vision_stats["axis_span_mm"]
    append_history(
        output_root / "history.csv",
        {
            "created_at": created_at,
            "label": label,
            "visual_x_m": visual[0],
            "visual_y_m": visual[1],
            "visual_z_m": visual[2],
            "tcp_x_m": tcp[0],
            "tcp_y_m": tcp[1],
            "tcp_z_m": tcp[2],
            "delta_x_mm": delta[0] * 1000.0,
            "delta_y_mm": delta[1] * 1000.0,
            "delta_z_mm": delta[2] * 1000.0,
            "delta_norm_mm": np.linalg.norm(delta) * 1000.0,
            "confidence_median": confidence,
            "radius_median_m": radius,
            "vision_span_x_mm": span[0],
            "vision_span_y_mm": span[1],
            "vision_span_z_mm": span[2],
        },
    )
    return run_dir, result


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Visual-to-TCP offset measurement; read-only by default, guarded real motion with --auto-scan"
    )
    parser.add_argument("--label", default="apple_alignment", help="trial label")
    parser.add_argument("--samples", type=int, default=10, help="fresh visual samples")
    parser.add_argument("--vision-timeout", type=float, default=30.0, help="visual timeout in seconds")
    parser.add_argument("--tcp-samples", type=int, default=20, help="TF samples after manual alignment")
    parser.add_argument("--tcp-timeout", type=float, default=5.0, help="TCP TF timeout in seconds")
    parser.add_argument(
        "--max-vision-span-mm",
        type=float,
        default=10.0,
        help="reject measurement when any visual axis span exceeds this value",
    )
    parser.add_argument("--base-frame", default="base_link")
    parser.add_argument("--tcp-frame", default="tcp_link")
    parser.add_argument(
        "--auto-scan",
        action="store_true",
        help="execute real motion to SCAN, open gripper, then retreat upward and return to SCAN",
    )
    parser.add_argument(
        "--return-retreat-m",
        type=float,
        default=0.10,
        help="base +Z Cartesian retreat before automatic return to SCAN",
    )
    parser.add_argument(
        "--orientation-tolerance-deg",
        type=float,
        default=6.0,
        help="maximum tilt of TCP +Z from base_link -Z; yaw is ignored",
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=DEFAULT_CONFIG,
        help="configuration containing the verified fixed SCAN joints",
    )
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_RESULT_ROOT)
    args, ros_args = parser.parse_known_args()
    if args.samples < 5:
        parser.error("--samples must be at least 5")
    if args.tcp_samples < 5:
        parser.error("--tcp-samples must be at least 5")
    if not 0.05 <= args.return_retreat_m <= 0.20:
        parser.error("--return-retreat-m must be in [0.05, 0.20] m")
    if not 0.5 <= args.orientation_tolerance_deg <= 10.0:
        parser.error("--orientation-tolerance-deg must be in [0.5, 10.0] degrees")

    try:
        config = load_config(args.config)
        scan_joints = {
            str(name): float(value) for name, value in config["scan"]["joints"].items()
        }
        scan_tolerance = min(float(config["scan"]["tolerance_rad"]), 0.005)
        vertical_yaw = float(config["targets"]["vertical_yaw_rad"])
    except (OSError, TypeError, ValueError, KeyError) as exc:
        parser.error(f"cannot load fixed SCAN from {args.config}: {exc}")
    reference_orientation = [
        math.cos(0.5 * vertical_yaw),
        math.sin(0.5 * vertical_yaw),
        0.0,
        0.0,
    ]

    rclpy.init(args=ros_args)
    node = VisualTcpOffsetRecorder(args.base_frame, args.tcp_frame)
    motion = ScanMotionController(config) if args.auto_scan else None
    gripper_opening_mm: float | None = None
    try:
        print("\n视觉 apple_center ↔ 人工对准 TCP 偏差测量")
        if motion is None:
            print("当前为只读模式：不会移动机械臂、不会控制夹爪、不会调用 MoveIt。")
        else:
            print("AUTO-SCAN 已启用：将真实执行到固定 SCAN，并把夹爪打开到测量宽度。")
            print("不会调用机械归零，不会发送夹爪闭合命令。")
        print("门禁规则：每一轮视觉采样都必须先回到配置中的固定 SCAN 鸟瞰位。")
        print("任意提示处输入 q 可取消。\n")
        node.spin_for(1.0)

        if motion is not None:
            print("正在安全回固定 SCAN；若当前不在 SCAN，将先完整笛卡尔上撤……")
            motion.ensure_fixed_scan_safely(args.return_retreat_m)
            print("正在配置工具端 RS485，并打开夹爪以避免人工对准时推动苹果……")
            gripper_opening_mm = motion.open_gripper_for_measurement()
            node.spin_for(0.5)
        scan_observed = require_manual_scan(node, scan_joints, scan_tolerance, "视觉采样前")

        prompt(
            "[1/2] 已在固定 SCAN：固定苹果、保持相机能看见苹果；按 Enter 冻结视觉坐标："
        )
        print(f"正在采集 {args.samples} 个新的 base_link 苹果中心样本……")
        vision_samples, diagnostics = node.collect_vision(args.samples, args.vision_timeout)
        if len(vision_samples) < args.samples:
            raise RuntimeError(
                f"视觉样本不足：{len(vision_samples)}/{args.samples}；确认 localizer 正在发布 base_link 坐标"
            )
        if len(diagnostics) < args.samples:
            raise RuntimeError(
                f"诊断样本不足：{len(diagnostics)}/{args.samples}；确认 /apple_pick_v2/apple_diagnostics 正在发布"
            )
        vision_stats = point_statistics([sample["xyz_m"] for sample in vision_samples])
        visual = np.asarray(vision_stats["median_xyz_m"])
        span = np.asarray(vision_stats["axis_span_mm"])
        print(
            f"已冻结视觉 apple_center = {np.round(visual, 7).tolist()} m；"
            f"XYZ span = {np.round(span, 2).tolist()} mm"
        )
        if np.any(span > args.max_vision_span_mm):
            raise RuntimeError(
                f"视觉坐标不稳定：某轴 span 超过 {args.max_vision_span_mm:.1f} mm；本轮不计算固定偏差"
            )

        print("\n现在可以用示教器低速人工移动；苹果绝对不要移动。")
        print("把实际两指夹持中心对准真实苹果中心，示教器只调整 XYZ。")
        print(
            "姿态门禁只要求 TCP 正 Z 轴指向 base_link 正下方，"
            f"倾斜不超过 {args.orientation_tolerance_deg:.2f}°；绕自身 Z 轴旋转不受限制。"
        )
        print("脚本不会在人工对准阶段发送机械臂或夹爪命令。")
        while True:
            prompt("[2/2] 垂直姿态人工对准并稳定后按 Enter 读取实际 tcp_link：")
            tcp_samples = node.collect_tcp(args.tcp_samples, 0.05, args.tcp_timeout)
            if len(tcp_samples) < args.tcp_samples:
                raise RuntimeError(
                    f"TCP 样本不足：{len(tcp_samples)}/{args.tcp_samples}；检查 TF {args.base_frame} -> {args.tcp_frame}"
                )
            orientation_error_deg = tool_z_tilt_from_base_down_deg(
                tcp_samples[-1]["quaternion_xyzw"]
            )
            if orientation_error_deg <= args.orientation_tolerance_deg:
                print(
                    f"垂直姿态门禁：PASS，工具 Z 轴倾斜 {orientation_error_deg:.3f}° "
                    f"<= {args.orientation_tolerance_deg:.3f}°"
                )
                break
            print(
                f"垂直姿态门禁：REJECT，工具 Z 轴倾斜 {orientation_error_deg:.3f}° "
                f"> {args.orientation_tolerance_deg:.3f}°；本批 TCP 不记录。"
            )
            print("请保持苹果不动，在示教器笛卡尔模式减小夹爪倾斜后重新对准；yaw 不用调整。")
        if not node.latest_joint_names or time.monotonic() - node.latest_joint_received_monotonic > 2.0:
            raise RuntimeError("/joint_states 缺失或已过期；不保存无法复现的人工对准结果")

        run_dir, result = save_result(
            args.output_dir,
            args.label,
            args.base_frame,
            args.tcp_frame,
            vision_samples,
            diagnostics,
            tcp_samples,
            scan_joints,
            scan_observed,
            args.auto_scan,
            args.return_retreat_m,
            gripper_opening_mm is not None,
            gripper_opening_mm,
            reference_orientation,
            orientation_error_deg,
            args.orientation_tolerance_deg,
            node.latest_joint_names,
            node.latest_joint_positions,
        )
        tcp = np.asarray(result["manual_alignment"]["tcp_statistics"]["median_xyz_m"])
        delta_mm = np.asarray(result["offset"]["delta_base_mm"])
        print("\n测量完成（base_link 坐标系）：")
        print(f"  冻结视觉 apple_center : {np.round(visual, 7).tolist()} m")
        print(f"  人工对准实际 tcp_link : {np.round(tcp, 7).tolist()} m")
        print(f"  Δ = TCP - 视觉中心    : {np.round(delta_mm, 2).tolist()} mm")
        print(f"  |Δ|                   : {result['offset']['norm_mm']:.2f} mm")
        print("  当前仅记录，不会把 Δ 写入抓取代码或配置。")
        print(f"  JSON/CSV 已保存：{run_dir}")
        print("\n本轮测量结束。下一轮采样前仍必须回到同一个固定 SCAN。")
        if motion is not None:
            print(
                f"正在低速沿 base_link +Z 上撤 {args.return_retreat_m:.3f} m，"
                "随后规划并真实执行回固定 SCAN……"
            )
            motion.retreat_up_then_scan(args.return_retreat_m)
            node.spin_for(0.5)
        else:
            print("请先把 TCP 从苹果附近安全上撤，再用示教器回鸟瞰位。")
        require_manual_scan(node, scan_joints, scan_tolerance, "本轮结束回位")
        print("固定 SCAN 回位已确认，可以结束或重新运行下一轮。")
    except (KeyboardInterrupt, EOFError):
        if motion is None:
            print("\n测量已取消；没有发送任何运动或夹爪命令。")
        else:
            print("\n测量已取消；已停止后续机械臂动作，不会发送夹爪闭合命令。")
    except RuntimeError as exc:
        print(f"\n测量失败：{exc}")
        if motion is None:
            print("没有发送任何运动或夹爪命令，也没有修改抓取配置。")
        else:
            print("失败即停：不会继续运动或闭合夹爪，也没有修改抓取配置。")
        raise SystemExit(1) from exc
    finally:
        if motion is not None:
            motion.destroy_node()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
