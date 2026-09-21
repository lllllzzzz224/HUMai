#!/usr/bin/env python3
"""Execute exactly one collision-checked SCAN->VERIFY trajectory at low speed.

This diagnostic deliberately never commands the gripper and never descends to
GRASP.  The exact selected trajectory is published to RViz before execution.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

import numpy as np
import rclpy
from moveit_msgs.msg import DisplayTrajectory

from fixed_scan_real_verify import FixedScanVerify, ROOT, load_config, vertical_pose
from preview_one_shot_plans import choose_plan, current_state


DEFAULT_CONFIG = ROOT / "single_apple_full_grasp.yaml"
DEFAULT_LOG = ROOT / "results" / "single_apple_grasp_20260808_164129.json"


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Low-speed one-shot SCAN->VERIFY execution; never commands gripper"
    )
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--run-log", type=Path, default=DEFAULT_LOG)
    parser.add_argument("--attempts", type=int, default=8)
    parser.add_argument("--execute", action="store_true")
    args, ros_args = parser.parse_known_args()

    cfg = load_config(args.config)
    run = json.loads(args.run_log.expanduser().read_text(encoding="utf-8"))
    apple = np.asarray(run["apple_center_base_m"], dtype=np.float64)
    radius = float(run["apple_radius_m"])
    verify_pose = vertical_pose(
        run["verify_base_m"], float(cfg["targets"]["vertical_yaw_rad"])
    )

    rclpy.init(args=ros_args)
    node = FixedScanVerify(cfg)
    display_pub = node.create_publisher(DisplayTrajectory, "/display_planned_path", 10)
    try:
        node.wait_interfaces()
        scan_error = float(
            np.max(np.abs(node.current_joint_vector() - node.scan_vector()))
        )
        if scan_error > float(cfg["scan"]["tolerance_rad"]):
            raise RuntimeError(
                f"机械臂不在固定 SCAN：最大关节误差 {scan_error:.4f} rad"
            )
        if float(cfg["moveit"]["velocity_scaling"]) > 0.05 or float(
            cfg["moveit"]["acceleration_scaling"]
        ) > 0.05:
            raise RuntimeError("拒绝执行：速度或加速度缩放超过 0.05")

        node.apply_scene(apple, radius)
        node.current_state_is_valid()
        trajectory, score = choose_plan(
            node, verify_pose, "SCAN -> VERIFY", max(1, args.attempts)
        )
        # This diagnostic is intended to follow the already-inspected smooth
        # branch, not a wrist-flip solution permitted by the general 5 rad cap.
        if score > 1.50:
            raise RuntimeError(
                f"拒绝执行：所选轨迹最大关节变化 {score:.3f} rad > 1.50 rad"
            )

        display = DisplayTrajectory()
        display.model_id = "rm_65"
        display.trajectory_start = current_state(node)
        display.trajectory = [trajectory]
        for _ in range(8):
            display_pub.publish(display)
            rclpy.spin_once(node, timeout_sec=0.15)

        target = np.asarray(run["verify_base_m"], dtype=np.float64)
        print(f"苹果中心 : {np.round(apple, 7).tolist()} m", flush=True)
        print(f"VERIFY   : {np.round(target, 7).tolist()} m", flush=True)
        print(
            f"即将执行同一条 RViz 轨迹：速度/加速度缩放 0.05，"
            f"max_delta={score:.3f} rad；不发送夹爪命令。",
            flush=True,
        )
        if not args.execute:
            print("PLAN-ONLY：缺少 --execute，真机未运动。", flush=True)
            return 0

        node.execute(trajectory, "SCAN -> VERIFY 一次性低速测试")
        actual = node.wait_tcp_arrival(verify_pose, "VERIFY")
        error_mm = float(np.linalg.norm(actual - target) * 1000.0)
        print("已到 VERIFY，停止；未下降到 GRASP，未闭合夹爪。", flush=True)
        print(
            f"实际 TCP={np.round(actual, 7).tolist()} m，位置误差={error_mm:.2f} mm",
            flush=True,
        )
        return 0
    except (KeyboardInterrupt, RuntimeError) as exc:
        node.get_logger().error(f"失败即停：{exc}")
        return 2
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
