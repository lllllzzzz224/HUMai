#!/usr/bin/env python3
"""Guide operator to place the collection basin by hovering the arm at the basin place point."""

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
    parser = argparse.ArgumentParser(description="机械臂盆子落料点对位示教")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--execute", action="store_true", help="真机物理执行")
    parser.add_argument("--dry-run", action="store_true", help="仅规划不动作")
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
        print("\n=======================================================")
        print("       🤖 机械臂入盆落料点对位示教 (Align Basin Guide)   ")
        print("=======================================================")
        print(f" 预设进刀点 (BASIN_APPROACH): {app_xyz} m")
        print(f" 预设落料点 (BASIN_PLACE)   : {place_xyz} m")
        print("-------------------------------------------------------")

        # 1. 确保夹爪通信与张开
        if args.execute:
            node.gripper.configure(cfg["modbus_gripper"])
            node.gripper.set_opening_mm(cfg["modbus_gripper"], float(cfg["modbus_gripper"]["open_mm"]))
            time.sleep(0.5)

        # 2. 规划 SCAN -> BASIN_APPROACH
        print("\n[1/4] 规划 SCAN -> BASIN_APPROACH...")
        approach_plan = node.plan(
            node.make_joint_goal(app_joints),
            "SCAN -> BASIN_APPROACH",
            planning_time_s=5.0,
            planning_attempts=5,
        )
        if approach_plan is None:
            raise RuntimeError("SCAN 到盆口上方规划失败！")
        print("   ✔ 进刀路径规划成功！")

        # 3. 规划 BASIN_APPROACH -> BASIN_PLACE
        print("[2/4] 规划 BASIN_APPROACH -> BASIN_PLACE (笛卡尔下潜)...")
        app_end_state = node.trajectory_end_state(approach_plan)
        place_plan = node.cartesian_to(
            place_pose,
            "BASIN_APPROACH -> BASIN_PLACE",
            start_state=app_end_state,
            velocity_scaling=0.03,
            acceleration_scaling=0.03,
            max_tcp_speed_m_s=float(cfg["lift_cartesian"]["max_tcp_speed_m_s"]),
        )
        print("   ✔ 下潜路径规划成功！")

        if args.dry_run or not args.execute:
            print("\n[DRY RUN] 预检全部通过，未执行任何真机动作。加 --execute 参数以真机运动。")
            return 0

        # 真机执行
        print("\n▶ [真机运动] 正在平缓移动至盆口上方 (BASIN_APPROACH)...")
        node.execute(approach_plan, "SCAN -> BASIN_APPROACH")
        node.wait_joints_arrival(app_joints, "BASIN_APPROACH")

        print("▶ [真机运动] 正在轻柔下潜至落料点 (BASIN_PLACE)...")
        # 重新根据实际到达状态进行笛卡尔规划执行
        actual_place_plan = node.cartesian_to(
            place_pose,
            "BASIN_APPROACH -> BASIN_PLACE",
            velocity_scaling=0.03,
            acceleration_scaling=0.03,
            max_tcp_speed_m_s=float(cfg["lift_cartesian"]["max_tcp_speed_m_s"]),
        )
        node.execute(actual_place_plan, "BASIN_APPROACH -> BASIN_PLACE")
        node.wait_tcp_arrival(place_pose, "BASIN_PLACE")

        print("\n=======================================================")
        print(" 🎯 机械臂已悬停在入盆落料点！")
        print("    - TCP 坐标: [X=0.464, Y=+0.006, Z=-0.020] m")
        print("    - 夹爪已张开至 80mm。")
        print("\n 👉 请将【浅绿色塑料盆】推到夹爪正下方，对齐两指中央。")
        print("=======================================================")

        input("\n对位完成后，请按 Enter 键让机械臂安全返回 SCAN 工位...")

        # 4. 回抽与返回 SCAN
        print("\n[3/4] 正在垂直回抽至盆口上方...")
        retreat_plan = node.cartesian_to(
            app_pose,
            "BASIN_PLACE -> BASIN_APPROACH",
            velocity_scaling=0.03,
            acceleration_scaling=0.03,
            max_tcp_speed_m_s=float(cfg["lift_cartesian"]["max_tcp_speed_m_s"]),
        )
        node.execute(retreat_plan, "BASIN_PLACE -> BASIN_APPROACH")
        node.wait_tcp_arrival(app_pose, "BASIN_APPROACH")

        print("[4/4] 正在平滑返回空中固定鸟瞰工位 (SCAN)...")
        scan_plan = node.plan(
            node.make_joint_goal(node.scan_vector()),
            "BASIN_APPROACH -> SCAN",
            planning_time_s=5.0,
            planning_attempts=5,
        )
        node.execute(scan_plan, "BASIN_APPROACH -> SCAN")
        node.wait_scan()

        print("\n✔ 机械臂已安全返回空中 SCAN 鸟瞰工位！盆子对位完成。\n")
        return 0

    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
