#!/usr/bin/env python3
"""Execute exactly one guarded apple grasp, lift 0.20 m, then stop."""

from __future__ import annotations

from builtin_interfaces.msg import Duration as DurationMsg
from dataclasses import dataclass
import json
from pathlib import Path
import sys
import time

from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, RobotState
from moveit_msgs.srv import GetCartesianPath, GetPositionIK
import numpy as np
import rclpy

from continuous_grasp_state import (
    AppleStabilityGate,
    ContinuousSession,
    SessionState,
)
from grasp_cli import parse_cli, resolve_run_mode
from fixed_scan_real_verify import (
    FixedScanVerify,
    ROOT,
    SUCCESS,
    load_config,
    scan_arrival_tolerance,
    vertical_pose,
)

sys.path.insert(0, str(ROOT.parent))
from modbus_gripper_ros import ToolRs485ModbusGripper  # noqa: E402


DEFAULT_CONFIG = ROOT / "single_apple_full_grasp.yaml"


def apply_visual_xy_correction(
    apple_xyz: np.ndarray,
    config: dict,
    *,
    disabled_by_cli: bool = False,
) -> tuple[np.ndarray, dict]:
    """Apply one bounded experimental 2-D similarity transform in base_link."""
    raw = np.asarray(apple_xyz, dtype=np.float64)
    if raw.shape != (3,) or not np.all(np.isfinite(raw)):
        raise RuntimeError(f"苹果坐标无效，无法执行视觉 XY 校正：{raw}")
    model = config.get("visual_xy_correction", {})
    configured = bool(model.get("enabled", False))
    enabled = configured and not disabled_by_cli
    details = {
        "configured_enabled": configured,
        "disabled_by_cli": bool(disabled_by_cli),
        "applied": enabled,
        "model": str(model.get("model", "none")),
        "raw_xyz_m": raw.tolist(),
    }
    if not enabled:
        details["corrected_xyz_m"] = raw.tolist()
        details["delta_xyz_m"] = [0.0, 0.0, 0.0]
        return raw.copy(), details

    if details["model"] != "similarity_2d_base_link":
        raise RuntimeError(f"不支持的视觉 XY 校正模型：{details['model']}")
    scale = float(model["scale"])
    rotation_deg = float(model["rotation_deg"])
    translation = np.asarray(model["translation_xy_m"], dtype=np.float64)
    bounds_x = np.asarray(model["valid_visual_x_m"], dtype=np.float64)
    bounds_y = np.asarray(model["valid_visual_y_m"], dtype=np.float64)
    if not 0.80 <= scale <= 1.10:
        raise RuntimeError(f"视觉 XY 尺度超出保护范围：{scale}")
    if abs(rotation_deg) > 10.0:
        raise RuntimeError(f"视觉 XY 旋转超出保护范围：{rotation_deg}°")
    if translation.shape != (2,) or not np.all(np.isfinite(translation)):
        raise RuntimeError("视觉 XY 平移配置无效")
    if bounds_x.shape != (2,) or bounds_y.shape != (2,):
        raise RuntimeError("视觉 XY 有效区域配置无效")
    if not bounds_x[0] <= raw[0] <= bounds_x[1]:
        raise RuntimeError(
            f"苹果视觉 X={raw[0]:.4f}m 超出尺度模型有效范围 "
            f"[{bounds_x[0]:.3f}, {bounds_x[1]:.3f}]m"
        )
    if not bounds_y[0] <= raw[1] <= bounds_y[1]:
        raise RuntimeError(
            f"苹果视觉 Y={raw[1]:.4f}m 超出尺度模型有效范围 "
            f"[{bounds_y[0]:.3f}, {bounds_y[1]:.3f}]m"
        )

    theta = np.deg2rad(rotation_deg)
    cosine = float(np.cos(theta))
    sine = float(np.sin(theta))
    corrected = raw.copy()
    corrected[0] = scale * (cosine * raw[0] - sine * raw[1]) + translation[0]
    corrected[1] = scale * (sine * raw[0] + cosine * raw[1]) + translation[1]
    # Z deliberately remains governed by targets.grasp_above_apple_m.
    corrected[2] = raw[2]
    delta = corrected - raw
    maximum = float(model["max_correction_m"])
    correction_norm = float(np.linalg.norm(delta[:2]))
    if correction_norm > maximum:
        raise RuntimeError(
            f"视觉 XY 校正量 {correction_norm * 1000.0:.1f}mm 超过保护上限 "
            f"{maximum * 1000.0:.1f}mm"
        )
    details.update({
        "scale": scale,
        "rotation_deg": rotation_deg,
        "translation_xy_m": translation.tolist(),
        "corrected_xyz_m": corrected.tolist(),
        "delta_xyz_m": delta.tolist(),
        "correction_norm_mm": correction_norm * 1000.0,
        "fit_point_count": int(model.get("fit_point_count", 0)),
        "fit_rmse_mm": float(model.get("fit_rmse_mm", float("nan"))),
        "fit_max_residual_mm": float(
            model.get("fit_max_residual_mm", float("nan"))
        ),
    })
    return corrected, details


class SingleAppleFullGrasp(FixedScanVerify):
    def __init__(self, config: dict) -> None:
        super().__init__(config)
        self.gripper = ToolRs485ModbusGripper(self)
        self.events: list[dict] = []

    def event(self, step: str, result: str, **details) -> None:
        item = {
            "time": time.strftime("%Y-%m-%d %H:%M:%S"),
            "step": step,
            "result": result,
            **details,
        }
        self.events.append(item)
        self.get_logger().info(f"{step}: {result}")

    def cartesian_to(
        self,
        target: Pose,
        label: str,
        *,
        start_state: RobotState | None = None,
        velocity_scaling: float,
        acceleration_scaling: float,
        max_tcp_speed_m_s: float,
    ):
        cfg = self.cfg["cartesian"]
        request = GetCartesianPath.Request()
        request.header.frame_id = self.base
        request.header.stamp = self.get_clock().now().to_msg()
        if start_state is None:
            request.start_state.is_diff = True
        else:
            request.start_state = start_state
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

    @staticmethod
    def trajectory_end_state(trajectory) -> RobotState:
        joint_trajectory = trajectory.joint_trajectory
        if not joint_trajectory.joint_names or not joint_trajectory.points:
            raise RuntimeError("无法从空轨迹构造连续规划起点")
        state = RobotState()
        state.joint_state.name = list(joint_trajectory.joint_names)
        state.joint_state.position = list(joint_trajectory.points[-1].positions)
        state.is_diff = True
        return state

    def plan(
        self,
        goal_constraint: Constraints,
        label: str,
        *,
        start_state: RobotState | None = None,
        planning_time_s: float | None = None,
        planning_attempts: int | None = None,
    ):
        move = self.cfg["moveit"]
        goal = MoveGroup.Goal()
        goal.request.group_name = str(move["group"])
        goal.request.pipeline_id = str(move["pipeline"])
        goal.request.planner_id = str(move["planner"])
        if start_state is None:
            goal.request.start_state.is_diff = True
        else:
            goal.request.start_state = start_state
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

    def settle(self, seconds: float) -> None:
        deadline = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    def fast_multi_seed_ik(
        self,
        pose: Pose,
        seeds: list[np.ndarray],
        *,
        total_solver_budget_s: float,
    ) -> tuple[list[RobotState], list[dict]]:
        """Return unique collision-free IK states and per-seed diagnostics.

        IK is deliberately requested without collision checking first.  Every
        returned solution is then checked independently against the active
        planning scene so JSON logs can distinguish geometry/limits from
        collision rejection.
        """
        if not seeds or total_solver_budget_s <= 0.0:
            raise RuntimeError("快速 IK 种子不能为空且总预算必须大于 0")
        per_seed_timeout_s = total_solver_budget_s / len(seeds)
        solutions: list[RobotState] = []
        solution_vectors: list[np.ndarray] = []
        diagnostics: list[dict] = []

        for seed_index, seed_values in enumerate(seeds):
            seed = RobotState()
            seed.joint_state.name = list(self.JOINT_NAMES)
            seed.joint_state.position = np.asarray(
                seed_values, dtype=np.float64
            ).tolist()
            seed.is_diff = True

            request = GetPositionIK.Request()
            request.ik_request.group_name = str(self.cfg["moveit"]["group"])
            request.ik_request.robot_state = seed
            request.ik_request.ik_link_name = self.tcp
            request.ik_request.pose_stamped = PoseStamped()
            request.ik_request.pose_stamped.header.frame_id = self.base
            request.ik_request.pose_stamped.pose = pose
            request.ik_request.avoid_collisions = False
            whole = int(per_seed_timeout_s)
            nanos = int(round((per_seed_timeout_s - whole) * 1e9))
            request.ik_request.timeout = DurationMsg(sec=whole, nanosec=nanos)

            future = self.ik_client.call_async(request)
            rclpy.spin_until_future_complete(
                self, future, timeout_sec=per_seed_timeout_s + 0.05
            )
            if not future.done():
                future.cancel()
                diagnostics.append({
                    "seed_index": seed_index,
                    "solver_timeout_s": per_seed_timeout_s,
                    "ik_error_code": None,
                    "result": "SERVICE_TIMEOUT",
                })
                continue
            response = future.result()
            error_code = None if response is None else response.error_code.val
            diagnostic = {
                "seed_index": seed_index,
                "solver_timeout_s": per_seed_timeout_s,
                "ik_error_code": error_code,
            }
            diagnostics.append(diagnostic)
            if response is None or error_code != SUCCESS:
                diagnostic["result"] = "NO_IK"
                continue

            solution = response.solution
            by_name = dict(
                zip(solution.joint_state.name, solution.joint_state.position)
            )
            if not all(name in by_name for name in self.JOINT_NAMES):
                diagnostic["result"] = "INCOMPLETE_SOLUTION"
                continue
            values = np.asarray(
                [by_name[name] for name in self.JOINT_NAMES], dtype=np.float64
            )
            if any(np.max(np.abs(values - prior)) < 1e-4 for prior in solution_vectors):
                diagnostic["result"] = "DUPLICATE_IK"
                continue

            try:
                self.state_is_valid(solution, f"快速 IK seed {seed_index}")
            except RuntimeError as error:
                diagnostic["result"] = "COLLISION"
                diagnostic["collision_reason"] = str(error)
                continue

            diagnostic["result"] = "PASS"
            solution_vectors.append(values)
            solutions.append(solution)

        return solutions, diagnostics


def pose_xyz(pose: Pose) -> list[float]:
    return [float(pose.position.x), float(pose.position.y), float(pose.position.z)]


def approach_candidate_stages(cfg: dict) -> list[list[dict]]:
    """Build staged yaw/pitch candidates relative to the validated base yaw."""
    search = cfg["candidate_search"]
    yaw_offsets = [float(value) for value in search["yaw_offsets_deg"]]
    pitch_stages = [
        [float(value) for value in stage]
        for stage in search["pitch_stages_deg"]
    ]
    max_pitch = float(search["max_abs_pitch_deg"])
    if not yaw_offsets or not pitch_stages or any(not stage for stage in pitch_stages):
        raise RuntimeError("候选 yaw 和 pitch stage 均不能为空")
    if any(abs(pitch) > max_pitch + 1e-9 for stage in pitch_stages for pitch in stage):
        raise RuntimeError(f"候选倾角超过允许的 +/-{max_pitch:.1f} 度")

    base_yaw = float(cfg["targets"]["vertical_yaw_rad"])
    stages: list[list[dict]] = []
    for stage_index, pitches in enumerate(pitch_stages):
        stage_candidates = []
        for pitch_deg in pitches:
            for yaw_offset_deg in yaw_offsets:
                stage_candidates.append({
                    "stage": stage_index,
                    "yaw_offset_deg": yaw_offset_deg,
                    "yaw_rad": base_yaw + float(np.deg2rad(yaw_offset_deg)),
                    "pitch_deg": pitch_deg,
                    "pitch_rad": float(np.deg2rad(pitch_deg)),
                })
        stages.append(stage_candidates)
    return stages


def tool_approach_vector(yaw: float, pitch: float) -> np.ndarray:
    """Unit vector in base frame pointing along the tool's downward approach axis.

    For pitch=0, this is exactly [0, 0, -1] (straight down in base_link).
    For pitch != 0, it tilts in the vertical plane defined by yaw.
    """
    ux = -math.sin(pitch) * math.cos(yaw)
    uy = -math.sin(pitch) * math.sin(yaw)
    uz = -math.cos(pitch)
    return np.array([ux, uy, uz], dtype=np.float64)


def candidate_waypoints(
    apple_center: np.ndarray, yaw: float, pitch: float, cfg: dict
) -> dict[str, np.ndarray]:
    """Calculate 3D waypoint positions aligned with the candidate approach angle.

    The tool approach vector points from the tool toward the target along the tool axis.
    Retracting along -u_approach moves away from the target along the tool's entry corridor.
    """
    u_app = tool_approach_vector(yaw, pitch)
    pre_dist = float(cfg["targets"]["pregrasp_above_apple_m"])
    verify_dist = float(cfg["targets"]["verify_above_apple_m"])
    grasp_dist = float(cfg["targets"]["grasp_above_apple_m"])
    lift_dist = float(cfg["targets"]["lift_m"])

    grasp_xyz = apple_center - grasp_dist * u_app
    verify_xyz = apple_center - verify_dist * u_app
    pre_xyz = apple_center - pre_dist * u_app
    lift_xyz = grasp_xyz - lift_dist * u_app

    return {
        "PREGRASP": pre_xyz,
        "VERIFY": verify_xyz,
        "GRASP": grasp_xyz,
        "LIFT": lift_xyz,
    }


def fast_ik_seeds(current: np.ndarray, cfg: dict) -> list[np.ndarray]:
    """Generate bounded deterministic seeds around the current SCAN state."""
    search = cfg["candidate_search"]
    offsets = np.asarray(search["ik_seed_offsets_rad"], dtype=np.float64)
    lower = np.asarray(search["joint_lower_limits_rad"], dtype=np.float64)
    upper = np.asarray(search["joint_upper_limits_rad"], dtype=np.float64)
    if offsets.ndim != 2 or offsets.shape[1] != len(current):
        raise RuntimeError("ik_seed_offsets_rad 必须是 N x 6")
    if lower.shape != current.shape or upper.shape != current.shape:
        raise RuntimeError("快速 IK 关节上下限必须各包含 6 个值")
    if np.any(lower >= upper):
        raise RuntimeError("快速 IK 关节上下限无效")
    margin = 1e-4
    return [
        np.clip(current + offset, lower + margin, upper - margin)
        for offset in offsets
    ]


def solution_vector(state: RobotState, joint_names: list[str]) -> np.ndarray:
    by_name = dict(zip(state.joint_state.name, state.joint_state.position))
    if not all(name in by_name for name in joint_names):
        raise RuntimeError("IK 解缺少规划组关节")
    return np.asarray([by_name[name] for name in joint_names], dtype=np.float64)


def candidate_score(
    solution: np.ndarray,
    current: np.ndarray,
    candidate: dict,
    cfg: dict,
) -> tuple[float, dict]:
    """Rank collision-free IK solutions without running a full planner."""
    search = cfg["candidate_search"]
    weights = search["score_weights"]
    lower = np.asarray(search["joint_lower_limits_rad"], dtype=np.float64)
    upper = np.asarray(search["joint_upper_limits_rad"], dtype=np.float64)
    delta = np.abs(solution - current)
    span = upper - lower
    normalized_margin = np.minimum(solution - lower, upper - solution) / span
    min_limit_margin = float(np.min(normalized_margin))
    limit_penalty = 1.0 / max(min_limit_margin, 1e-3)
    yaw_offset_deg = float(candidate["yaw_offset_deg"])
    canonical_yaw_offset_deg = abs((yaw_offset_deg + 180.0) % 360.0 - 180.0)
    score = (
        float(weights["max_joint_delta"]) * float(np.max(delta))
        + float(weights["sum_joint_delta"]) * float(np.sum(delta))
        + float(weights["joint_limit_penalty"]) * limit_penalty
        + float(weights["yaw_offset_deg"]) * canonical_yaw_offset_deg
        + float(weights["pitch_deg"]) * abs(float(candidate["pitch_deg"]))
    )
    details = {
        "score": score,
        "max_joint_delta_rad": float(np.max(delta)),
        "sum_joint_delta_rad": float(np.sum(delta)),
        "min_normalized_joint_limit_margin": min_limit_margin,
        "joint_limit_penalty": limit_penalty,
        "canonical_yaw_offset_deg": canonical_yaw_offset_deg,
    }
    return score, details


@dataclass(frozen=True)
class AppleTarget:
    raw_center_m: np.ndarray
    corrected_center_m: np.ndarray
    radius_m: float
    diagnostics: dict
    correction: dict


@dataclass
class PreparedRound:
    target: AppleTarget
    pre_pose: Pose
    verify_pose: Pose
    grasp_pose: Pose
    lift_pose: Pose
    pre_trajectory: object
    grasp_state: RobotState | None
    record: dict


@dataclass(frozen=True)
class RoundOutcome:
    status: str
    release_center_m: tuple[float, float, float] | None
    motion_started: bool


def mark_motion_started(record: dict) -> None:
    record["motion_started"] = True


def new_round_record(args, *, round_index: int, continuous: bool) -> dict:
    return {
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        "status": "STARTED",
        "single_round": not continuous,
        "continuous": continuous,
        "round_index": int(round_index),
        "autonomous_requested": bool(args.autonomous or continuous),
        "fully_autonomous_requested": bool(
            args.fully_autonomous or continuous
        ),
        "preflight_only": bool(args.preflight_only),
        "verify_only": bool(args.verify_only),
        "return_after_lift_requested": bool(
            args.return_after_lift or continuous
        ),
        "visual_xy_correction_disabled_by_cli": bool(
            args.disable_visual_xy_correction
        ),
        "single_apple_scene_operator_confirmed": bool(
            args.operator_confirmed
        ),
        "motion_started": False,
        "gripper_close_commanded": False,
        "lift_completed": False,
        "gripper_reopened": False,
        "returned_to_scan": False,
    }


def collect_target(node, cfg, *, disabled_by_cli: bool) -> AppleTarget:
    raw, radius, diagnostics = node.collect_apple()
    corrected, correction = apply_visual_xy_correction(
        raw,
        cfg,
        disabled_by_cli=disabled_by_cli,
    )
    return AppleTarget(
        raw_center_m=np.asarray(raw, dtype=np.float64),
        corrected_center_m=np.asarray(corrected, dtype=np.float64),
        radius_m=float(radius),
        diagnostics=dict(diagnostics),
        correction=dict(correction),
    )


def write_round_log(path: Path, node, record: dict, event_start: int) -> None:
    record["events"] = list(node.events[event_start:])
    record["finished_at"] = time.strftime("%Y-%m-%d %H:%M:%S")
    path.write_text(
        json.dumps(record, indent=2, ensure_ascii=False),
        encoding="utf-8",
    )


def prepare_round(
    node, cfg, args, mode, target: AppleTarget, record: dict
) -> PreparedRound:
    """Build and validate trajectories without moving or commanding the gripper."""
    raw_apple = target.raw_center_m
    radius = target.radius_m
    diagnostics = target.diagnostics
    apple = target.corrected_center_m
    correction = target.correction
    correction_delta_mm = np.asarray(correction["delta_xyz_m"]) * 1000.0
    if correction["applied"]:
        node.event(
            "实验性视觉 XY 尺度模型",
            "APPLIED",
            raw_xyz_m=np.round(raw_apple, 7).tolist(),
            corrected_xyz_m=np.round(apple, 7).tolist(),
            delta_xy_mm=np.round(correction_delta_mm[:2], 2).tolist(),
            scale=correction["scale"],
            rotation_deg=correction["rotation_deg"],
        )
    else:
        node.event(
            "实验性视觉 XY 尺度模型",
            "DISABLED",
            disabled_by_cli=bool(args.disable_visual_xy_correction),
        )
    node.event("单苹果现场", "PASS", verification="operator-confirmed: one visible apple")
    grasp_state = None
    candidate_stages = approach_candidate_stages(cfg)
    pre_xyz = apple + [0.0, 0.0, float(cfg["targets"]["pregrasp_above_apple_m"])]
    verify_xyz = apple + [0.0, 0.0, float(cfg["targets"]["verify_above_apple_m"])]
    grasp_xyz = apple + [0.0, 0.0, float(cfg["targets"]["grasp_above_apple_m"])]
    lift_xyz = grasp_xyz + [0.0, 0.0, float(cfg["targets"]["lift_m"])]
    record.update({
        "apple_center_raw_base_m": np.round(raw_apple, 7).tolist(),
        "apple_center_corrected_base_m": np.round(apple, 7).tolist(),
        "apple_center_base_m": np.round(apple, 7).tolist(),
        "visual_xy_correction": correction,
        "apple_radius_m": float(radius),
        "vision_diagnostics": diagnostics,
        "pregrasp_base_m": np.round(pre_xyz, 7).tolist(),
        "verify_base_m": np.round(verify_xyz, 7).tolist(),
        "grasp_base_m": np.round(grasp_xyz, 7).tolist(),
        "grasp_offset_above_center_m": float(cfg["targets"]["grasp_above_apple_m"]),
        "lift_base_m": np.round(lift_xyz, 7).tolist(),
        "gripper_open_mm": float(cfg["modbus_gripper"]["open_mm"]),
        "gripper_close_mm": float(cfg["modbus_gripper"]["close_mm"]),
        "candidate_yaw_offsets_deg": [
            float(value)
            for value in cfg["candidate_search"]["yaw_offsets_deg"]
        ],
        "candidate_pitch_stages_deg": [
            [float(value) for value in stage]
            for stage in cfg["candidate_search"]["pitch_stages_deg"]
        ],
    })
    print(f"\n苹果原始 apple_center: {record['apple_center_raw_base_m']} m")
    if correction["applied"]:
        print(
            f"实验 XY 校正后中心  : {record['apple_center_corrected_base_m']} m"
        )
        print(
            f"本轮 XY 校正量      : "
            f"{np.round(correction_delta_mm[:2], 2).tolist()} mm "
            f"（scale={correction['scale']:.6f}, "
            f"rotation={correction['rotation_deg']:+.3f}°）"
        )
    else:
        print("实验 XY 校正        : DISABLED（使用原始中心）")
    print(f"PREGRASP            : {record['pregrasp_base_m']} m")
    print(f"VERIFY              : {record['verify_base_m']} m")
    print(
        f"GRASP（中心上方{record['grasp_offset_above_center_m'] * 1000.0:.0f}mm）"
        f": {record['grasp_base_m']} m"
    )
    print(f"LIFT                : {record['lift_base_m']} m", flush=True)
    print(
        f"候选搜索             : yaw 相对偏移 "
        f"{record['candidate_yaw_offsets_deg']}°；pitch 分阶段 "
        f"{record['candidate_pitch_stages_deg']}°",
        flush=True,
    )
    print(
        f"夹爪命令             : 打开 {record['gripper_open_mm']:.1f} mm，"
        f"闭合目标 {record['gripper_close_mm']:.1f} mm（位置命令）",
        flush=True,
    )

    table_top = node.apply_scene(apple, radius)
    record["table_guard_top_z_m"] = table_top
    node.current_state_is_valid()

    current_joints = node.current_joint_vector()
    seeds = fast_ik_seeds(current_joints, cfg)
    solver_budget_s = float(
        cfg["candidate_search"]["ik_total_solver_budget_s"]
    )
    screen_records: list[dict] = []
    ranked_candidates: list[dict] = []
    selected_stage: int | None = None
    screen_waypoints = [
        ("PREGRASP", pre_xyz),
        ("VERIFY", verify_xyz),
    ]
    if not mode.verify_only:
        screen_waypoints.extend(
            [
                ("GRASP", grasp_xyz),
            ]
        )
    waypoint_solver_budget_s = solver_budget_s / len(screen_waypoints)

    for stage_index, stage in enumerate(candidate_stages):
        stage_valid: list[dict] = []
        for candidate in stage:
            screen_item = {
                "stage": stage_index,
                "yaw_offset_deg": candidate["yaw_offset_deg"],
                "yaw_rad": candidate["yaw_rad"],
                "pitch_deg": candidate["pitch_deg"],
                "waypoints": {},
            }
            waypoint_best: dict[str, dict] = {}
            rejected_waypoint: str | None = None
            reject_reason: str | None = None

            c_wps = candidate_waypoints(
                apple, candidate["yaw_rad"], candidate["pitch_rad"], cfg
            )
            c_screen_waypoints = [
                ("PREGRASP", c_wps["PREGRASP"]),
                ("VERIFY", c_wps["VERIFY"]),
            ]
            if not mode.verify_only:
                c_screen_waypoints.append(("GRASP", c_wps["GRASP"]))

            for waypoint_name, waypoint_xyz in c_screen_waypoints:
                waypoint_pose = vertical_pose(
                    waypoint_xyz,
                    candidate["yaw_rad"],
                    candidate["pitch_rad"],
                )
                solutions, ik_diagnostics = node.fast_multi_seed_ik(
                    waypoint_pose,
                    seeds,
                    total_solver_budget_s=waypoint_solver_budget_s,
                )
                waypoint_item = {
                    "ik_diagnostics": ik_diagnostics,
                    "collision_free_solution_count": len(solutions),
                }
                screen_item["waypoints"][waypoint_name] = waypoint_item
                if not solutions:
                    results = {
                        item.get("result", "UNKNOWN")
                        for item in ik_diagnostics
                    }
                    reject_reason = (
                        "COLLISION" if "COLLISION" in results else "NO_IK"
                    )
                    waypoint_item["result"] = "REJECT"
                    waypoint_item["reason"] = reject_reason
                    rejected_waypoint = waypoint_name
                    break

                scored_solutions = []
                for solution in solutions:
                    values = solution_vector(solution, node.JOINT_NAMES)
                    score, score_details = candidate_score(
                        values, current_joints, candidate, cfg
                    )
                    scored_solutions.append({
                        "score": score,
                        "score_details": score_details,
                        "joint_positions_rad": values.tolist(),
                    })
                best_solution = min(
                    scored_solutions, key=lambda item: item["score"]
                )
                waypoint_item["result"] = "PASS"
                waypoint_item["best_solution"] = best_solution
                waypoint_best[waypoint_name] = best_solution

            if rejected_waypoint is not None:
                screen_item["result"] = "REJECT"
                screen_item["reason"] = reject_reason
                screen_item["rejected_waypoint"] = rejected_waypoint
                screen_records.append(screen_item)
                node.event(
                    "抓取候选快速筛选",
                    "REJECT",
                    stage=stage_index,
                    yaw_offset_deg=candidate["yaw_offset_deg"],
                    pitch_deg=candidate["pitch_deg"],
                    waypoint=rejected_waypoint,
                    reason=reject_reason,
                )
                continue

            # Rank by the worst of the four waypoint costs.  This avoids
            # selecting a PREGRASP-friendly branch which becomes marginal
            # at GRASP or LIFT.
            ranking_score = max(
                item["score"] for item in waypoint_best.values()
            )
            screen_item["result"] = "PASS"
            screen_item["ranking_score"] = ranking_score
            screen_records.append(screen_item)
            stage_valid.append({
                **candidate,
                "score": ranking_score,
                "score_details": {
                    "ranking_basis": "max_waypoint_score",
                    "waypoint_scores": {
                        name: item["score"]
                        for name, item in waypoint_best.items()
                    },
                },
                "joint_positions_rad": waypoint_best[
                    "PREGRASP"
                ]["joint_positions_rad"],
                "waypoint_best_solutions": waypoint_best,
            })

        # Preserve a vertical top-down grasp whenever any candidate in the
        # current stage works.  Only expand to tilted stages if the whole
        # earlier stage has no collision-free IK solution.
        if stage_valid:
            ranked_candidates = sorted(
                stage_valid, key=lambda item: item["score"]
            )
            selected_stage = stage_index
            break

    record["fast_candidate_screen"] = {
        "total_solver_budget_s_per_candidate": solver_budget_s,
        "screen_waypoints": [name for name, _ in screen_waypoints],
        "solver_budget_s_per_waypoint": waypoint_solver_budget_s,
        "seed_count": len(seeds),
        "ik_avoid_collisions": False,
        "independent_state_collision_check": True,
        "records": screen_records,
    }
    if not ranked_candidates or selected_stage is None:
        raise RuntimeError(
            "所有分阶段 yaw/pitch 候选均未通过快速 IK/碰撞筛选；"
            "详细原因见 JSON 日志"
        )

    selected = ranked_candidates[0]
    selected_yaw = float(selected["yaw_rad"])
    selected_pitch = float(selected["pitch_rad"])
    selected_yaw_offset_deg = float(selected["yaw_offset_deg"])
    selected_pitch_deg = float(selected["pitch_deg"])
    record["ranked_candidates_in_selected_stage"] = ranked_candidates
    record["selected_candidate"] = {
        key: value for key, value in selected.items()
        if key != "pitch_rad"
    }
    record["selected_candidate"]["pitch_rad"] = selected_pitch
    selected_wps = candidate_waypoints(
        apple, selected_yaw, selected_pitch, cfg
    )
    pre_xyz = selected_wps["PREGRASP"]
    verify_xyz = selected_wps["VERIFY"]
    grasp_xyz = selected_wps["GRASP"]
    lift_xyz = selected_wps["LIFT"]
    record.update({
        "pregrasp_base_m": np.round(pre_xyz, 7).tolist(),
        "verify_base_m": np.round(verify_xyz, 7).tolist(),
        "grasp_base_m": np.round(grasp_xyz, 7).tolist(),
        "lift_base_m": np.round(lift_xyz, 7).tolist(),
    })
    pre_pose = vertical_pose(pre_xyz, selected_yaw, selected_pitch)
    verify_pose = vertical_pose(verify_xyz, selected_yaw, selected_pitch)
    grasp_pose = vertical_pose(grasp_xyz, selected_yaw, selected_pitch)
    lift_pose = vertical_pose(lift_xyz, selected_yaw, selected_pitch)
    label = (
        f"yaw偏移 {selected_yaw_offset_deg:+.0f}°/"
        f"pitch {selected_pitch_deg:+.0f}°"
    )
    print(
        f"已选择抓取姿态       : yaw 偏移 {selected_yaw_offset_deg:+.0f}°，"
        f"pitch {selected_pitch_deg:+.0f}°，stage={selected_stage}",
        flush=True,
    )
    print(f"姿态自适应 PREGRASP  : {record['pregrasp_base_m']} m")
    print(f"姿态自适应 VERIFY    : {record['verify_base_m']} m")
    print(f"姿态自适应 GRASP     : {record['grasp_base_m']} m")
    print(f"姿态自适应 LIFT      : {record['lift_base_m']} m", flush=True)

    full_plan_time_s = float(
        cfg["candidate_search"]["full_planning_time_s"]
    )
    full_plan_attempts = int(
        cfg["candidate_search"]["full_planning_attempts"]
    )
    try:
        # Exactly one ranked candidate receives full OMPL/path preflight.
        best_wp = selected.get("waypoints", {}).get("PREGRASP", {})
        best_joints = best_wp.get("best_solution", {}).get("joint_positions_rad")
        if best_joints is not None:
            pre_goal = node.make_joint_goal(np.asarray(best_joints))
        else:
            pre_goal = node.make_pose_goal(
                pre_pose,
                f"pregrasp_yaw_{selected_yaw_offset_deg:+.0f}"
                f"_pitch_{selected_pitch_deg:+.0f}",
            )
        pre_traj = node.plan(
            pre_goal,
            f"{label} SCAN -> PREGRASP",
            planning_time_s=full_plan_time_s,
            planning_attempts=full_plan_attempts,
        )
        pre_state = node.trajectory_end_state(pre_traj)
        verify_preflight = node.cartesian_to(
            verify_pose,
            f"{label} PREGRASP -> VERIFY 预检",
            start_state=pre_state,
            velocity_scaling=0.05,
            acceleration_scaling=0.05,
            max_tcp_speed_m_s=float(cfg["cartesian"]["max_tcp_speed_m_s"]),
        )
        verify_state = node.trajectory_end_state(verify_preflight)
        if not mode.verify_only:
            grasp_preflight = node.cartesian_to(
                grasp_pose,
                f"{label} VERIFY -> GRASP 预检",
                start_state=verify_state,
                velocity_scaling=float(cfg["grasp_cartesian"]["velocity_scaling"]),
                acceleration_scaling=float(cfg["grasp_cartesian"]["acceleration_scaling"]),
                max_tcp_speed_m_s=float(cfg["grasp_cartesian"]["max_tcp_speed_m_s"]),
            )
            grasp_state = node.trajectory_end_state(grasp_preflight)
            lift_preflight = node.cartesian_to(
                lift_pose,
                f"{label} GRASP -> LIFT 预检",
                start_state=grasp_state,
                velocity_scaling=float(cfg["lift_cartesian"]["velocity_scaling"]),
                acceleration_scaling=float(cfg["lift_cartesian"]["acceleration_scaling"]),
                max_tcp_speed_m_s=float(cfg["lift_cartesian"]["max_tcp_speed_m_s"]),
            )
            lift_state = node.trajectory_end_state(lift_preflight)
            if mode.auto_return:
                if getattr(mode, "place_target", "basin") == "basin":
                    basin_cfg = cfg.get("place_target", {})
                    app_xyz = basin_cfg.get("approach_xyz_m", [0.464, 0.006, 0.080])
                    place_xyz = basin_cfg.get("place_xyz_m", [0.464, 0.006, -0.020])
                    app_joints = basin_cfg.get("target_joints_rad", [0.0484, -1.6343, 0.7521, -0.0675, -2.0939, -2.8371])
                    basin_yaw = float(basin_cfg.get("yaw_rad", 2.847))
                    basin_pitch = float(basin_cfg.get("pitch_rad", 0.1745))

                    app_pose = vertical_pose(app_xyz, basin_yaw, basin_pitch)
                    place_pose = vertical_pose(place_xyz, basin_yaw, basin_pitch)

                    basin_app_traj = node.cartesian_to(
                        app_pose,
                        f"{label} LIFT -> BASIN_APPROACH 预检",
                        start_state=lift_state,
                        velocity_scaling=0.03,
                        acceleration_scaling=0.02,
                        max_tcp_speed_m_s=float(cfg.get("lift_cartesian", {}).get("max_tcp_speed_m_s", 0.015)),
                    )
                    basin_app_state = node.trajectory_end_state(basin_app_traj)

                    basin_place_traj = node.cartesian_to(
                        place_pose,
                        f"{label} BASIN_APPROACH -> BASIN_PLACE 预检",
                        start_state=basin_app_state,
                        velocity_scaling=0.03,
                        acceleration_scaling=0.03,
                        max_tcp_speed_m_s=float(cfg["lift_cartesian"]["max_tcp_speed_m_s"]),
                    )
                    basin_place_state = node.trajectory_end_state(basin_place_traj)

                    basin_retreat_traj = node.cartesian_to(
                        app_pose,
                        f"{label} BASIN_PLACE -> BASIN_APPROACH 预检",
                        start_state=basin_place_state,
                        velocity_scaling=0.03,
                        acceleration_scaling=0.03,
                        max_tcp_speed_m_s=float(cfg["lift_cartesian"]["max_tcp_speed_m_s"]),
                    )
                    basin_retreat_state = node.trajectory_end_state(basin_retreat_traj)

                    node.plan(
                        node.make_joint_goal(node.scan_vector()),
                        f"{label} BASIN_APPROACH -> SCAN 预检",
                        start_state=basin_retreat_state,
                        planning_time_s=full_plan_time_s,
                        planning_attempts=full_plan_attempts,
                    )
                else:
                    release_preflight = node.cartesian_to(
                        grasp_pose,
                        f"{label} LIFT -> RELEASE 预检",
                        start_state=lift_state,
                        velocity_scaling=float(cfg["lift_cartesian"]["velocity_scaling"]),
                        acceleration_scaling=float(cfg["lift_cartesian"]["acceleration_scaling"]),
                        max_tcp_speed_m_s=float(cfg["lift_cartesian"]["max_tcp_speed_m_s"]),
                    )
                    release_state = node.trajectory_end_state(release_preflight)
                    node.cartesian_to(
                        pre_pose,
                        f"{label} RELEASE -> PREGRASP 预检",
                        start_state=release_state,
                        velocity_scaling=float(cfg["lift_cartesian"]["velocity_scaling"]),
                        acceleration_scaling=float(cfg["lift_cartesian"]["acceleration_scaling"]),
                        max_tcp_speed_m_s=float(cfg["lift_cartesian"]["max_tcp_speed_m_s"]),
                    )
    except RuntimeError as error:
        record["selected_candidate_full_preflight"] = {
            "result": "REJECT",
            "reason": str(error),
            "planning_time_s": full_plan_time_s,
            "planning_attempts": full_plan_attempts,
        }
        raise RuntimeError(
            "排名第一候选的完整规划/路径预检失败；未尝试其他候选，"
            "详细原因见 JSON 日志"
        ) from error

    record["selected_candidate_full_preflight"] = {
        "result": "PASS",
        "planning_time_s": full_plan_time_s,
        "planning_attempts": full_plan_attempts,
    }
    if mode.auto_return:
        node.event(
            "放回与撤离路径预检",
            "PASS",
            yaw_offset_deg=selected_yaw_offset_deg,
            pitch_deg=selected_pitch_deg,
        )
    node.event(
        "全部路径预检",
        "PASS",
        yaw_offset_deg=selected_yaw_offset_deg,
        pitch_deg=selected_pitch_deg,
    )
    if mode.auto_return and grasp_state is None:
        raise RuntimeError("自动放回模式缺少 GRASP 链式预检终态")
    return PreparedRound(
        target=target,
        pre_pose=pre_pose,
        verify_pose=verify_pose,
        grasp_pose=grasp_pose,
        lift_pose=lift_pose,
        pre_trajectory=pre_traj,
        grasp_state=grasp_state,
        record=record,
    )


def execute_prepared_round(
    node,
    cfg,
    args,
    mode,
    prepared: PreparedRound,
    motion_start_callback=None,
) -> RoundOutcome:
    record = prepared.record
    pre_pose = prepared.pre_pose
    verify_pose = prepared.verify_pose
    grasp_pose = prepared.grasp_pose
    lift_pose = prepared.lift_pose
    pre_traj = prepared.pre_trajectory
    grasp_state = prepared.grasp_state
    # --execute --operator-confirmed is the explicit real-robot gate.
    # Once every IK/collision/path check above passes, move automatically
    # to VERIFY.  The final Enter is skipped only by explicit --autonomous.
    node.event("自动开始 SCAN -> PREGRASP -> VERIFY", "PASS")
    verify_gate = (
        "到达 VERIFY 后按 --autonomous 自动继续。"
        if mode.auto_grasp
        else "到达 VERIFY 后等待人工 Enter。"
    )
    print(
        "\n全部预检通过，自动执行 SCAN → PREGRASP → VERIFY；"
        + verify_gate,
        flush=True,
    )

    if motion_start_callback is not None:
        motion_start_callback()
    mark_motion_started(record)
    node.execute(pre_traj, "SCAN -> PREGRASP")
    node.wait_tcp_arrival(pre_pose, "PREGRASP")
    verify_traj = node.cartesian_to(
        verify_pose,
        "PREGRASP -> VERIFY",
        velocity_scaling=0.05,
        acceleration_scaling=0.05,
        max_tcp_speed_m_s=float(cfg["cartesian"]["max_tcp_speed_m_s"]),
    )
    node.execute(verify_traj, "PREGRASP -> VERIFY")
    node.wait_tcp_arrival(verify_pose, "VERIFY")
    node.event(
        "到达 VERIFY", "PASS", actual_tcp_base_m=np.round(node.tcp_position(), 7).tolist()
    )

    if mode.verify_only:
        record["status"] = "PASS_VERIFY_ONLY"
        node.event(
            "VERIFY_ONLY 到位即停",
            "PASS",
            no_grasp_descent=True,
            no_gripper_close=True,
        )
        print(
            "\nPASS：真机已到 VERIFY；按 --verify-only 要求立即停止，"
            "未下降到 GRASP，夹爪保持打开。",
            flush=True,
        )
        return RoundOutcome("PASS_VERIFY_ONLY", None, True)

    if mode.auto_grasp:
        node.event("VERIFY 后自动继续", "PASS", explicit_flag="--autonomous")
        print("--autonomous 已启用：VERIFY 到位后自动继续 GRASP。", flush=True)
    else:
        confirmed = input(
            f"\n下一步闭合目标为 {cfg['modbus_gripper']['close_mm']:.1f} mm（位置命令）。"
            "\n请人工确认 TCP 位于两指中间正上方；确认后按 Enter"
            "（输入其他内容或 Ctrl-C 将停止且不闭爪）："
        )
        if confirmed.strip():
            raise RuntimeError("人工未确认；停止在 VERIFY，夹爪保持打开")

    grasp_traj = node.cartesian_to(
        grasp_pose,
        "VERIFY -> GRASP",
        velocity_scaling=0.03,
        acceleration_scaling=0.03,
        max_tcp_speed_m_s=float(cfg["grasp_cartesian"]["max_tcp_speed_m_s"]),
    )
    node.execute(grasp_traj, "VERIFY -> GRASP")
    node.wait_tcp_arrival(grasp_pose, "GRASP")
    node.event(
        "到达 GRASP", "PASS", actual_tcp_base_m=np.round(node.tcp_position(), 7).tolist()
    )

    node.gripper.set_opening_mm(
        cfg["modbus_gripper"], cfg["modbus_gripper"]["close_mm"]
    )
    record["gripper_close_commanded"] = True
    node.event("保守闭爪", "PASS", opening_mm=cfg["modbus_gripper"]["close_mm"])
    node.settle(float(cfg["modbus_gripper"]["close_settle_s"]))

    lift_traj = node.cartesian_to(
        lift_pose,
        "GRASP -> LIFT",
        velocity_scaling=0.03,
        acceleration_scaling=0.03,
        max_tcp_speed_m_s=float(cfg["lift_cartesian"]["max_tcp_speed_m_s"]),
    )
    node.execute(lift_traj, "GRASP -> LIFT")
    actual_lift = node.wait_tcp_arrival(lift_pose, "LIFT")
    record["lift_completed"] = True
    record["actual_lift_tcp_base_m"] = np.round(actual_lift, 7).tolist()
    node.event(
        "上抬完成",
        "PASS",
        distance_m=float(cfg["targets"]["lift_m"]),
    )
    if mode.auto_return:
        if getattr(mode, "place_target", "basin") == "basin":
            basin_cfg = cfg.get("place_target", {})
            transit_z = float(basin_cfg.get("transit_z_safe_m", 0.120))
            selected_yaw = float(record["selected_candidate"]["yaw_rad"])
            selected_pitch = float(record["selected_candidate"]["pitch_rad"])

            transit_xyz = [grasp_pose.position.x, grasp_pose.position.y, max(lift_pose.position.z + 0.05, transit_z)]
            transit_pose = vertical_pose(transit_xyz, selected_yaw, selected_pitch)

            app_xyz = basin_cfg.get("approach_xyz_m", [0.464, 0.006, 0.080])
            place_xyz = basin_cfg.get("place_xyz_m", [0.464, 0.006, -0.020])
            app_joints = basin_cfg.get("target_joints_rad", [0.0484, -1.6343, 0.7521, -0.0675, -2.0939, -2.8371])
            basin_yaw = float(basin_cfg.get("yaw_rad", 2.847))
            basin_pitch = float(basin_cfg.get("pitch_rad", 0.1745))

            app_pose = vertical_pose(app_xyz, basin_yaw, basin_pitch)
            place_pose = vertical_pose(place_xyz, basin_yaw, basin_pitch)

            # LIFT is already complete, will plan directly to BASIN_APPROACH

            if mode.continuous or args.fully_autonomous:
                node.event(
                    "LIFT 后自动转运入盆",
                    "PASS",
                    explicit_flag="--fully-autonomous",
                )
                print(
                    "--fully-autonomous 已启用：LIFT 到位后自动转运入盆、"
                    "开爪释放、撤离并返回 SCAN。",
                    flush=True,
                )
            else:
                return_confirmed = input(
                    "\n果实已抓取并上抬。请确认果实夹持稳定、盆子已放置到位；"
                    "确认现在转运入盆请按 Enter"
                    "（输入其他内容将停在 LIFT 并保持夹持）："
                )
                if return_confirmed.strip():
                    record["status"] = "PASS_GRASP_HELD_AT_LIFT"
                    print("未确认入盆：程序停止在 LIFT，夹爪保持闭合。", flush=True)
                    return RoundOutcome("PASS_GRASP_HELD_AT_LIFT", None, True)

            # 1. LIFT -> BASIN_APPROACH (Gentle Cartesian Translation)
            approach_traj = node.cartesian_to(
                app_pose,
                "LIFT -> BASIN_APPROACH",
                velocity_scaling=0.03,
                acceleration_scaling=0.02,
                max_tcp_speed_m_s=float(cfg.get("lift_cartesian", {}).get("max_tcp_speed_m_s", 0.015)),
            )
            node.execute(approach_traj, "LIFT -> BASIN_APPROACH")
            node.event("到达盆口上方", "PASS")

            # 3. BASIN_APPROACH -> BASIN_PLACE
            place_traj = node.cartesian_to(
                place_pose,
                "BASIN_APPROACH -> BASIN_PLACE",
                velocity_scaling=0.03,
                acceleration_scaling=0.03,
                max_tcp_speed_m_s=float(cfg["lift_cartesian"]["max_tcp_speed_m_s"]),
            )
            node.execute(place_traj, "BASIN_APPROACH -> BASIN_PLACE")
            actual_release = node.wait_tcp_arrival(place_pose, "BASIN_PLACE")
            record["actual_release_tcp_base_m"] = np.round(actual_release, 7).tolist()
            node.event("到达放料位", "PASS")

            # 4. Open gripper
            node.gripper.set_opening_mm(
                cfg["modbus_gripper"], cfg["modbus_gripper"]["open_mm"]
            )
            record["gripper_reopened"] = True
            node.event("释放果实入盆", "PASS", opening_mm=cfg["modbus_gripper"]["open_mm"])
            node.settle(float(basin_cfg.get("release_settle_s", 1.0)))

            # 5. Vertical retreat
            retreat_traj = node.cartesian_to(
                app_pose,
                "BASIN_PLACE -> BASIN_APPROACH",
                velocity_scaling=0.03,
                acceleration_scaling=0.03,
                max_tcp_speed_m_s=float(cfg["lift_cartesian"]["max_tcp_speed_m_s"]),
            )
            node.execute(retreat_traj, "BASIN_PLACE -> BASIN_APPROACH")
            node.wait_tcp_arrival(app_pose, "BASIN_APPROACH")
            node.event("放料后垂直回抽", "PASS")

            # 6. BASIN_APPROACH -> SCAN
            scan_trajectory = node.plan(
                node.make_joint_goal(node.scan_vector()), "BASIN_APPROACH -> SCAN"
            )
            node.execute(scan_trajectory, "BASIN_APPROACH -> SCAN")
            node.wait_scan()
            record["returned_to_scan"] = True
            node.event("返回固定鸟瞰位", "PASS")
            record["status"] = "PASS_PICK_PLACE_BASIN_COMPLETE"
            print(
                "\nPASS：果实抓取、上抬、转运入盆、释放、回抽和返回 SCAN 已全部完成；"
                "程序立即停止，不循环。",
                flush=True,
            )
            return RoundOutcome(
                "PASS_PICK_PLACE_BASIN_COMPLETE",
                tuple(np.asarray(app_xyz, dtype=float).tolist()),
                True,
            )
        else:
            # Recompute from the actual LIFT before asking for the return action.
            release_traj = node.cartesian_to(
                grasp_pose,
                "LIFT -> RELEASE",
                velocity_scaling=0.03,
                acceleration_scaling=0.03,
                max_tcp_speed_m_s=float(cfg["lift_cartesian"]["max_tcp_speed_m_s"]),
            )
            node.cartesian_to(
                pre_pose,
                "RELEASE -> PREGRASP 再预检",
                start_state=grasp_state,
                velocity_scaling=0.03,
                acceleration_scaling=0.03,
                max_tcp_speed_m_s=float(cfg["lift_cartesian"]["max_tcp_speed_m_s"]),
            )
            if mode.continuous or args.fully_autonomous:
                node.event(
                    "LIFT 后自动放回",
                    "PASS",
                    explicit_flag="--fully-autonomous",
                )
                print(
                    "--fully-autonomous 已启用：LIFT 到位后自动沿预检路径放回、"
                    "开爪、撤离并返回 SCAN。",
                    flush=True,
                )
            else:
                return_confirmed = input(
                    "\n苹果已抓取并上抬。请确认苹果夹持稳定、原抓取位置可以放回；"
                    "确认现在需要放回后按 Enter"
                    "（输入其他内容将停在 LIFT 并保持夹持）："
                )
                if return_confirmed.strip():
                    record["status"] = "PASS_GRASP_HELD_AT_LIFT"
                    print("未确认放回：程序停止在 LIFT，夹爪保持闭合。", flush=True)
                    return RoundOutcome("PASS_GRASP_HELD_AT_LIFT", None, True)

            node.execute(release_traj, "LIFT -> RELEASE")
            actual_release = node.wait_tcp_arrival(grasp_pose, "RELEASE")
            record["actual_release_tcp_base_m"] = np.round(actual_release, 7).tolist()
            node.event("到达 RELEASE", "PASS")

            node.gripper.set_opening_mm(
                cfg["modbus_gripper"], cfg["modbus_gripper"]["open_mm"]
            )
            record["gripper_reopened"] = True
            node.event("释放苹果", "PASS", opening_mm=cfg["modbus_gripper"]["open_mm"])
            node.settle(1.0)

            retreat_traj = node.cartesian_to(
                pre_pose,
                "RELEASE -> PREGRASP",
                velocity_scaling=0.03,
                acceleration_scaling=0.03,
                max_tcp_speed_m_s=float(cfg["lift_cartesian"]["max_tcp_speed_m_s"]),
            )
            node.execute(retreat_traj, "RELEASE -> PREGRASP")
            node.wait_tcp_arrival(pre_pose, "PREGRASP")
            node.event("释放后直线撤离", "PASS")

            scan_trajectory = node.plan(
                node.make_joint_goal(node.scan_vector()), "PREGRASP -> SCAN"
            )
            node.execute(scan_trajectory, "PREGRASP -> SCAN")
            node.wait_scan()
            record["returned_to_scan"] = True
            node.event("返回固定鸟瞰位", "PASS")
            record["status"] = "PASS_PICK_RETURN_COMPLETE"
            print(
                "\nPASS：单苹果抓取、上抬、放回、开爪和返回 SCAN 已全部完成；"
                "程序立即停止，不循环。",
                flush=True,
            )
            return RoundOutcome(
                "PASS_PICK_RETURN_COMPLETE",
                tuple(prepared.target.corrected_center_m.tolist()),
                True,
            )

    record["status"] = "PASS_SINGLE_ROUND_COMPLETE"
    print(
        f"\nPASS：单苹果一轮抓取完成，已上抬 "
        f"{float(cfg['targets']['lift_m']):.2f} m；"
        "程序立即停止，不放置、不循环。",
        flush=True,
    )
    return RoundOutcome("PASS_SINGLE_ROUND_COMPLETE", None, True)


def run_single_main(node, cfg, args, mode, record: dict) -> int:
    target = collect_target(
        node, cfg, disabled_by_cli=args.disable_visual_xy_correction
    )
    prepared = prepare_round(node, cfg, args, mode, target, record)
    if mode.preflight_only:
        record["status"] = "PASS_PREFLIGHT_ONLY"
        node.event(
            "实时尺度模型抓取预检",
            "PASS_PREFLIGHT_ONLY",
            no_trajectory_execution=True,
            no_gripper_command=True,
        )
        print(
            "\nPASS：实时苹果的尺度模型校正、IK、碰撞和全部抓取路径预检通过；"
            "未配置/控制夹爪，未发送任何机械臂执行轨迹。",
            flush=True,
        )
        return 0
    outcome = execute_prepared_round(node, cfg, args, mode, prepared)
    record["status"] = outcome.status
    return 0


def wait_for_ready_target(node, cfg, args, gate, session_record):
    last_state = None
    while rclpy.ok():
        try:
            target = collect_target(
                node,
                cfg,
                disabled_by_cli=args.disable_visual_xy_correction,
            )
        except RuntimeError as error:
            gate.missing()
            session_record["vision_wait_errors"] += 1
            node.get_logger().warning(f"等待稳定苹果：{error}")
            node.settle(float(gate.settings.retry_delay_s))
            continue
        snapshot = gate.observe(
            target.corrected_center_m,
            time.monotonic(),
        )
        if snapshot.state != last_state:
            details = (
                ""
                if snapshot.distance_from_reference_m is None
                else (
                    f"，距上次位置 "
                    f"{snapshot.distance_from_reference_m * 1000.0:.1f}mm"
                )
            )
            node.get_logger().info(
                f"连续抓取门禁：{snapshot.state}，"
                f"稳定 {snapshot.stable_elapsed_s:.1f}s{details}"
            )
            last_state = snapshot.state
        if snapshot.ready:
            return target, snapshot
    raise KeyboardInterrupt


def validate_frozen_target(node, cfg, args, frozen: AppleTarget) -> AppleTarget:
    latest = collect_target(
        node,
        cfg,
        disabled_by_cli=args.disable_visual_xy_correction,
    )
    distance = float(
        np.linalg.norm(
            latest.corrected_center_m - frozen.corrected_center_m
        )
    )
    limit = float(cfg["vision"]["max_axis_span_m"])
    if distance > limit:
        raise RuntimeError(
            f"规划后苹果位置变化 {distance * 1000.0:.1f}mm 超过 "
            f"{limit * 1000.0:.1f}mm"
        )
    return latest


def continuous_main(node, cfg, args, mode, result_dir: Path) -> int:
    settings = mode.settings
    if settings is None:
        raise RuntimeError("连续模式缺少 continuous 配置")
    session = ContinuousSession(max_rounds=settings.max_rounds)
    gate = AppleStabilityGate(settings)
    started = time.strftime("%Y%m%d_%H%M%S")
    session_path = (
        result_dir / f"continuous_apple_grasp_{started}.json"
    )
    session_record = {
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        "status": "STARTED",
        "settings": dict(settings.__dict__),
        "completed_rounds": 0,
        "pre_motion_rejections": 0,
        "vision_wait_errors": 0,
        "rounds": [],
    }
    node.event(
        "连续抓取授权",
        "PASS",
        explicit_flags=[
            "--continuous",
            "--execute",
            "--operator-confirmed",
        ],
        max_rounds=settings.max_rounds,
    )
    print(
        "\n连续模式已启动：每轮自动抓取、上抬、原位放回并返回 SCAN。"
        "\n仅在苹果相对上次放回位置移动至少 "
        f"{settings.min_position_change_m * 1000.0:.0f}mm，"
        f"稳定 {settings.stable_time_s:.1f}s，再完成 "
        f"{settings.safety_countdown_s:.1f}s 安全倒计时后开始下一轮。"
        "\nCtrl+C 可停止等待；机械臂运动期间请使用示教器急停处理危险。",
        flush=True,
    )
    try:
        while rclpy.ok() and not session.terminal:
            session.set_state(SessionState.WAIT_NEW_APPLE)
            target, snapshot = wait_for_ready_target(
                node, cfg, args, gate, session_record
            )
            session_record["last_gate_snapshot"] = {
                "state": snapshot.state,
                "stable_elapsed_s": snapshot.stable_elapsed_s,
                "distance_from_reference_m": (
                    snapshot.distance_from_reference_m
                ),
                "center_m": snapshot.center_m,
            }
            attempts = 0
            completed = False
            while attempts < settings.max_pre_motion_retries:
                attempts += 1
                event_start = len(node.events)
                round_index = session.completed_rounds + 1
                round_record = new_round_record(
                    args,
                    round_index=round_index,
                    continuous=True,
                )
                round_record["pre_motion_attempt"] = attempts
                round_path = result_dir / (
                    f"single_apple_grasp_{time.strftime('%Y%m%d_%H%M%S')}_"
                    f"round{round_index:03d}_attempt{attempts:02d}.json"
                )
                try:
                    session.set_state(SessionState.PLAN_ROUND)
                    prepared = prepare_round(
                        node, cfg, args, mode, target, round_record
                    )
                    latest = validate_frozen_target(
                        node, cfg, args, target
                    )
                    round_record["final_target_validation_m"] = float(
                        np.linalg.norm(
                            latest.corrected_center_m
                            - target.corrected_center_m
                        )
                    )
                    outcome = execute_prepared_round(
                        node, cfg, args, mode, prepared,
                        motion_start_callback=session.motion_started,
                    )
                    if outcome.release_center_m is None:
                        raise RuntimeError(
                            "连续模式完成一轮后缺少原位释放坐标"
                        )
                    round_record["status"] = outcome.status
                    round_record["release_center_m"] = list(
                        outcome.release_center_m
                    )
                    write_round_log(
                        round_path, node, round_record, event_start
                    )
                    session_record["rounds"].append({
                        "log": str(round_path),
                        "status": outcome.status,
                        "round_index": round_index,
                    })
                    node.remove_support_guard()
                    gate.after_release(outcome.release_center_m)
                    session.round_completed()
                    session_record["completed_rounds"] = (
                        session.completed_rounds
                    )
                    completed = True
                    if session.terminal:
                        print(
                            f"\n第 {round_index} 轮完成，已达到轮数上限。",
                            flush=True,
                        )
                    else:
                        print(
                            f"\n第 {round_index} 轮完成。请把苹果移动至少 "
                            f"{settings.min_position_change_m * 1000.0:.0f}mm；"
                            "检测稳定并完成安全倒计时后自动开始下一轮。",
                            flush=True,
                        )
                    break
                except RuntimeError as error:
                    round_record["status"] = "FAILED_STOPPED"
                    round_record["error"] = str(error)
                    write_round_log(
                        round_path, node, round_record, event_start
                    )
                    session.fail(error)
                    session_record["rounds"].append({
                        "log": str(round_path),
                        "status": round_record["status"],
                        "round_index": round_index,
                        "attempt": attempts,
                        "motion_started": bool(
                            round_record["motion_started"]
                        ),
                        "error": str(error),
                    })
                    node.remove_support_guard()
                    if session.state == SessionState.FAULT_LATCHED:
                        raise
                    session_record["pre_motion_rejections"] += 1
                    node.get_logger().warning(
                        f"运动前规划拒绝 {attempts}/"
                        f"{settings.max_pre_motion_retries}：{error}"
                    )
                    node.settle(settings.retry_delay_s)
            if not completed:
                gate.reject(target.corrected_center_m)
                node.get_logger().warning(
                    "同一苹果位置连续规划失败，等待苹果移动后再试"
                )
        session_record["status"] = session.state.value
        return 0
    except KeyboardInterrupt:
        session.stop("Ctrl+C")
        session_record["status"] = session.state.value
        return 130
    except RuntimeError as error:
        session_record["status"] = SessionState.FAULT_LATCHED.value
        session_record["error"] = str(error)
        node.get_logger().error(
            f"连续流程运动后故障锁停：{error}"
        )
        return 2
    finally:
        node.remove_support_guard()
        session_record["completed_rounds"] = session.completed_rounds
        session_record["finished_at"] = time.strftime(
            "%Y-%m-%d %H:%M:%S"
        )
        session_path.write_text(
            json.dumps(
                session_record,
                indent=2,
                ensure_ascii=False,
            ),
            encoding="utf-8",
        )
        print(f"连续执行汇总：{session_path}", flush=True)

def main(argv=None) -> int:
    args, ros_args = parse_cli(argv, DEFAULT_CONFIG)
    cfg = load_config(args.config)
    mode = resolve_run_mode(args, cfg)
    if args.fully_autonomous:
        args.autonomous = True
        args.return_after_lift = True
    result_dir = ROOT / "results"
    result_dir.mkdir(exist_ok=True)
    output = None
    record = None
    if not mode.continuous:
        output = result_dir / (
            f"single_apple_grasp_{time.strftime('%Y%m%d_%H%M%S')}.json"
        )
        record = new_round_record(args, round_index=1, continuous=False)

    rclpy.init(args=ros_args)
    node = SingleAppleFullGrasp(cfg)
    try:
        if args.verify_only and (
            args.preflight_only
            or args.autonomous
            or args.fully_autonomous
            or args.return_after_lift
        ):
            raise RuntimeError("--verify-only 不能与预检/自动抓取/放回参数组合使用")
        if args.preflight_only:
            if args.execute:
                raise RuntimeError("--preflight-only 与 --execute 不能同时使用")
            if args.fully_autonomous:
                raise RuntimeError("--preflight-only 与 --fully-autonomous 不能同时使用")
            if not args.operator_confirmed:
                raise RuntimeError("实时单苹果预检仍需提供 --operator-confirmed")
        elif not args.execute or not args.operator_confirmed:
            raise RuntimeError("真机单轮抓取必须同时提供 --execute --operator-confirmed")
        if args.fully_autonomous:
            node.event(
                "单轮全自动授权",
                "PASS",
                explicit_flag="--fully-autonomous",
                auto_verify_to_grasp=True,
                auto_lift_to_release_and_scan=True,
                loop=False,
            )
        node.wait_interfaces()
        stale_guard_removed = node.remove_support_guard()
        if record is not None:
            record["stale_support_guard_removed_on_start"] = stale_guard_removed

        if args.preflight_only:
            node.event(
                "夹爪通信与命令",
                "SKIPPED_PREFLIGHT_ONLY",
                no_publish=True,
            )
        else:
            # Never call initialize() here: it performs a mechanical zeroing
            # stroke which can close the jaws before the open command.  The
            # calibrated gripper has already been zeroed; only restore
            # communication and 24 V.
            node.gripper.configure(cfg["modbus_gripper"])
            node.gripper.set_opening_mm(
                cfg["modbus_gripper"], cfg["modbus_gripper"]["open_mm"]
            )
            node.event(
                "夹爪通信配置并保持打开",
                "PASS",
                opening_mm=cfg["modbus_gripper"]["open_mm"],
                mechanical_zeroing=False,
            )

        scan_target = node.scan_vector()
        scan_error = float(np.max(np.abs(node.current_joint_vector() - scan_target)))
        if scan_error > scan_arrival_tolerance(cfg):
            if args.preflight_only:
                raise RuntimeError(
                    f"plan-only 要求机械臂已在固定 SCAN；当前最大关节误差 "
                    f"{scan_error:.4f} rad"
                )
            scan_traj = node.plan(node.make_joint_goal(scan_target), "当前位置 -> SCAN")
            node.execute(scan_traj, "当前位置 -> SCAN")
        node.wait_scan()
        node.event("固定鸟瞰位", "PASS")

        if mode.continuous:
            return continuous_main(node, cfg, args, mode, result_dir)
        if record is None:
            raise RuntimeError("单轮模式日志记录未初始化")
        return run_single_main(node, cfg, args, mode, record)
    except KeyboardInterrupt:
        if record is not None:
            record["status"] = "STOPPED_BY_OPERATOR"
        node.event("流程", "STOPPED_BY_OPERATOR")
        return 130
    except RuntimeError as error:
        if record is not None:
            record["status"] = "FAILED_STOPPED"
            record["error"] = str(error)
        node.get_logger().error(f"失败即停：{error}")
        return 2
    finally:
        support_guard_removed = node.remove_support_guard()
        if record is not None and output is not None:
            record["support_guard_removed_on_exit"] = (
                support_guard_removed
            )
            record["events"] = node.events
            record["finished_at"] = time.strftime(
                "%Y-%m-%d %H:%M:%S"
            )
            output.write_text(
                json.dumps(record, indent=2, ensure_ascii=False),
                encoding="utf-8",
            )
            print(f"完整执行日志：{output}", flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
