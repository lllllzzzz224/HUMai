#!/usr/bin/env python3
"""Safely reposition the arm from current hover point to new basin location (Y = -0.080)."""

from __future__ import annotations
import math
import sys
import time
from pathlib import Path
import numpy as np
import rclpy
from geometry_msgs.msg import Pose

from fixed_scan_real_verify import ROOT, load_config, vertical_pose
from single_apple_full_grasp import SingleAppleFullGrasp

NEW_APP_XYZ = [0.380, -0.080, 0.080]
NEW_PLACE_XYZ = [0.380, -0.080, -0.020]
NEW_JOINTS_RAD = [2.934090, 0.811061, 1.023813, 0.000000, 1.306711, 0.087093]
BASIN_YAW = 2.847
BASIN_PITCH = 0.0


def main() -> int:
    rclpy.init()
    cfg = load_config(ROOT / "single_apple_full_grasp.yaml")
    node = SingleAppleFullGrasp(cfg)

    try:
        node.wait_interfaces()
        current_tcp = node.tcp_position()
        print(f"[0/4] 当前机械臂 TCP 坐标: {np.round(current_tcp, 4).tolist()}", flush=True)

        # 1. 垂直安全抬升（如果当前在落料深位）
        if current_tcp[2] < 0.05:
            print("[1/4] 正在从当前落料位垂直抬起 10cm 以避开桌面障碍...", flush=True)
            lift_target_pose = vertical_pose(
                [float(current_tcp[0]), float(current_tcp[1]), float(current_tcp[2]) + 0.10],
                BASIN_YAW,
                BASIN_PITCH,
            )
            lift_traj = node.cartesian_to(
                lift_target_pose,
                "OLD_PLACE -> OLD_APPROACH",
                velocity_scaling=0.03,
                acceleration_scaling=0.03,
                max_tcp_speed_m_s=float(cfg["lift_cartesian"]["max_tcp_speed_m_s"]),
            )
            node.execute(lift_traj, "OLD_PLACE -> OLD_APPROACH")
            time.sleep(0.5)

        # 2. 关节空间平滑横移至新落料口上方（Joint 1 微动仅 ~9°，Joint 5 保持正向朝下）
        print(f"[2/4] 正在端水平移至新落料口上方 {NEW_APP_XYZ}... (距橘子 24cm)", flush=True)
        app_pose = vertical_pose(NEW_APP_XYZ, BASIN_YAW, BASIN_PITCH)
        app_plan = node.plan(
            node.make_joint_goal(NEW_JOINTS_RAD),
            "TRANSIT -> NEW_BASIN_APPROACH",
            planning_time_s=5.0,
            planning_attempts=5,
        )
        if app_plan is None:
            raise RuntimeError("平移至新落料口规划失败！")
        node.execute(app_plan, "TRANSIT -> NEW_BASIN_APPROACH")
        node.wait_tcp_arrival(app_pose, "NEW_BASIN_APPROACH")

        # 3. 笛卡尔匀速垂直下潜至新落料深度
        print(f"[3/4] 正在垂直匀速下潜至新落料位 {NEW_PLACE_XYZ}... (限速 0.015 m/s)", flush=True)
        place_pose = vertical_pose(NEW_PLACE_XYZ, BASIN_YAW, BASIN_PITCH)
        place_traj = node.cartesian_to(
            place_pose,
            "NEW_APPROACH -> NEW_PLACE",
            velocity_scaling=0.03,
            acceleration_scaling=0.03,
            max_tcp_speed_m_s=float(cfg["lift_cartesian"]["max_tcp_speed_m_s"]),
        )
        node.execute(place_traj, "NEW_APPROACH -> NEW_PLACE")
        actual_place = node.wait_tcp_arrival(place_pose, "NEW_PLACE")

        # 4. 确保夹爪张开至 80mm
        print("[4/4] 正在确认夹爪开启至 80mm 供现场对齐...", flush=True)
        node.gripper.configure(cfg["modbus_gripper"])
        node.gripper.set_opening_mm(cfg["modbus_gripper"], 80.0)

        print(f"SUCCESS_MOVED_TO_NEW_BASIN_PLACE actual={np.round(actual_place, 4).tolist()}", flush=True)
        return 0

    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
