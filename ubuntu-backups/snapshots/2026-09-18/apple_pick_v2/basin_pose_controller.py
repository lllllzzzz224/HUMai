#!/usr/bin/env python3
"""Modular controller to move the arm to basin place point and return."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys
import time
import numpy as np
import rclpy

from fixed_scan_real_verify import ROOT, load_config, vertical_pose
from single_apple_full_grasp import SingleAppleFullGrasp

DEFAULT_CONFIG = ROOT / "single_apple_full_grasp.yaml"


def main() -> int:
    parser = argparse.ArgumentParser(description="落料点悬停与回位控制")
    parser.add_argument("action", choices=["go", "return", "check"])
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    args, ros_args = parser.parse_known_args()

    cfg = load_config(args.config)
    basin_cfg = cfg.get("place_target", {})

    app_xyz = basin_cfg.get("approach_xyz_m", [0.464, 0.006, 0.080])
    place_xyz = basin_cfg.get("place_xyz_m", [0.464, 0.006, -0.020])
    app_joints = basin_cfg.get("target_joints_rad", [0.0484, -1.6343, 0.7521, -0.0675, -2.0939, -2.8371])
    basin_yaw = float(basin_cfg.get("yaw_rad", 2.847))
    basin_pitch = float(basin_cfg.get("pitch_rad", 0.1745))

    app_pose = vertical_pose(app_xyz, basin_yaw, basin_pitch)
    place_pose = vertical_pose(place_xyz, basin_yaw, basin_pitch)

    rclpy.init(args=ros_args)
    node = SingleAppleFullGrasp(cfg)

    try:
        node.wait_interfaces()
        current_tcp = node.tcp_position()

        if args.action == "check":
            print(f"Current TCP: {current_tcp.round(4).tolist()}")
            print(f"Target Basin Place: {place_xyz}")
            print(f"Target Basin Approach: {app_xyz}")
            return 0

        elif args.action == "go":
            print("[1/3] 正在确保夹爪开启到 80mm...")
            node.gripper.configure(cfg["modbus_gripper"])
            node.gripper.set_opening_mm(cfg["modbus_gripper"], float(cfg["modbus_gripper"]["open_mm"]))
            time.sleep(0.5)

            print("[2/3] 正在移动至盆口上方 (BASIN_APPROACH)... (最长约 10 秒)")
            app_plan = node.plan(
                node.make_joint_goal(app_joints),
                "MOVE -> BASIN_APPROACH",
                planning_time_s=5.0,
                planning_attempts=5,
            )
            if app_plan is None:
                raise RuntimeError("移动至盆口上方规划失败！")
            node.execute(app_plan, "MOVE -> BASIN_APPROACH")
            time.sleep(0.5)  # 到达盆口上方，准备下潜

            print("[3/3] 正在轻柔下潜至落料点 (BASIN_PLACE)... (限速 0.015 m/s)")
            place_plan = node.cartesian_to(
                place_pose,
                "BASIN_APPROACH -> BASIN_PLACE",
                velocity_scaling=0.03,
                acceleration_scaling=0.03,
                max_tcp_speed_m_s=float(cfg["lift_cartesian"]["max_tcp_speed_m_s"]),
            )
            node.execute(place_plan, "BASIN_APPROACH -> BASIN_PLACE")
            node.wait_tcp_arrival(place_pose, "BASIN_PLACE")
            print("SUCCESS_REACHED_BASIN_PLACE")
            return 0

        elif args.action == "return":
            print("[1/2] 正在垂直回抽至盆口上方 (BASIN_APPROACH)... (限速 0.015 m/s)")
            retreat_plan = node.cartesian_to(
                app_pose,
                "BASIN_PLACE -> BASIN_APPROACH",
                velocity_scaling=0.03,
                acceleration_scaling=0.03,
                max_tcp_speed_m_s=float(cfg["lift_cartesian"]["max_tcp_speed_m_s"]),
            )
            node.execute(retreat_plan, "BASIN_PLACE -> BASIN_APPROACH")
            time.sleep(0.5)  # 到达盆口上方，准备下潜

            print("[2/2] 正在平滑返回空中固定鸟瞰工位 (SCAN)... (最长约 10 秒)")
            scan_plan = node.plan(
                node.make_joint_goal(node.scan_vector()),
                "BASIN_APPROACH -> SCAN",
                planning_time_s=5.0,
                planning_attempts=5,
            )
            if scan_plan is None:
                raise RuntimeError("返回 SCAN 规划失败！")
            node.execute(scan_plan, "BASIN_APPROACH -> SCAN")
            node.wait_scan()
            print("SUCCESS_RETURNED_TO_SCAN")
            return 0

    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
