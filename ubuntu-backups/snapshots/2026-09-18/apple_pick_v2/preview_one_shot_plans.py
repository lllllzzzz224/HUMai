#!/usr/bin/env python3
"""Publish plan-only SCAN->VERIFY and SCAN->GRASP trajectories to RViz."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import time

from moveit_msgs.msg import DisplayTrajectory, RobotState
import numpy as np
import rclpy

from fixed_scan_real_verify import FixedScanVerify, ROOT, load_config, vertical_pose


DEFAULT_CONFIG = ROOT / "single_apple_full_grasp.yaml"
DEFAULT_LOG = ROOT / "results" / "single_apple_grasp_20260808_164129.json"


def trajectory_score(node: FixedScanVerify, trajectory) -> float:
    joint_trajectory = trajectory.joint_trajectory
    positions = np.asarray(
        [point.positions for point in joint_trajectory.points], dtype=np.float64
    )
    current = node.current_joint_vector(list(joint_trajectory.joint_names))
    return float(np.max(np.abs(positions - current)))


def choose_plan(node, pose, label: str, attempts: int):
    best = None
    best_score = float("inf")
    failures = []
    for attempt in range(1, attempts + 1):
        try:
            trajectory = node.plan(
                node.make_pose_goal(pose, f"{label}_{attempt}"),
                f"{label} 只规划候选 {attempt}",
            )
        except RuntimeError as exc:
            failures.append(str(exc))
            continue
        score = trajectory_score(node, trajectory)
        if score < best_score:
            best = trajectory
            best_score = score
    if best is None:
        raise RuntimeError(f"{label} 的 {attempts} 个只规划候选全部失败：{failures}")
    node.get_logger().info(
        f"{label} 选择最小关节变化候选：max_delta={best_score:.3f} rad"
    )
    return best, best_score


def current_state(node) -> RobotState:
    state = RobotState()
    state.joint_state.name = list(node.JOINT_NAMES)
    state.joint_state.position = node.current_joint_vector().tolist()
    state.is_diff = False
    return state


def publish_preview(node, publisher, trajectory, label: str) -> None:
    message = DisplayTrajectory()
    message.model_id = "rm_65"
    message.trajectory_start = current_state(node)
    message.trajectory = [trajectory]
    for _ in range(5):
        publisher.publish(message)
        rclpy.spin_once(node, timeout_sec=0.15)
    print(
        f"\n已发布 {label} 到 RViz /display_planned_path。"
        "这是 plan-only，不会执行真机。",
        flush=True,
    )


def main() -> int:
    parser = argparse.ArgumentParser(description="RViz 一次性规划预览，绝不执行")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--run-log", type=Path, default=DEFAULT_LOG)
    parser.add_argument("--attempts", type=int, default=8)
    args, ros_args = parser.parse_known_args()

    cfg = load_config(args.config)
    run = json.loads(args.run_log.expanduser().read_text(encoding="utf-8"))
    apple = np.asarray(run["apple_center_base_m"], dtype=np.float64)
    radius = float(run["apple_radius_m"])
    yaw = float(cfg["targets"]["vertical_yaw_rad"])
    verify_pose = vertical_pose(run["verify_base_m"], yaw)
    grasp_pose = vertical_pose(run["grasp_base_m"], yaw)

    rclpy.init(args=ros_args)
    node = FixedScanVerify(cfg)
    publisher = node.create_publisher(
        DisplayTrajectory, "/display_planned_path", 10
    )
    try:
        node.wait_interfaces()
        scan_error = float(
            np.max(np.abs(node.current_joint_vector() - node.scan_vector()))
        )
        if scan_error > float(cfg["scan"]["tolerance_rad"]):
            raise RuntimeError(
                f"机械臂不在固定 SCAN：最大关节误差 {scan_error:.4f} rad；"
                "拒绝用旧日志做预览"
            )
        node.apply_scene(apple, radius)
        node.current_state_is_valid()
        print("PLAN-ONLY：不会发送 /execute_trajectory。", flush=True)
        print(f"苹果中心={np.round(apple, 7).tolist()} m", flush=True)

        verify_trajectory, verify_score = choose_plan(
            node, verify_pose, "SCAN -> VERIFY", max(1, args.attempts)
        )
        grasp_trajectory, grasp_score = choose_plan(
            node, grasp_pose, "SCAN -> GRASP", max(1, args.attempts)
        )
        print(
            f"VERIFY max_delta={verify_score:.3f} rad；"
            f"GRASP max_delta={grasp_score:.3f} rad",
            flush=True,
        )

        publish_preview(node, publisher, verify_trajectory, "SCAN → VERIFY")
        input("请在 RViz 查看 VERIFY 轨迹；看完后按 Enter 切换到 GRASP 预览：")
        publish_preview(node, publisher, grasp_trajectory, "SCAN → GRASP")
        input("请在 RViz 查看 GRASP 轨迹；看完后按 Enter 结束只规划预览：")
        print("RViz 只规划预览结束；机械臂从未执行。", flush=True)
        return 0
    except (KeyboardInterrupt, RuntimeError) as exc:
        node.get_logger().error(str(exc))
        return 2
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
