#!/usr/bin/env python3
import sys

path = "/home/li/hand_eye_calibration/apple_pick_v2/single_apple_full_grasp.py"
with open(path, "r", encoding="utf-8") as f:
    code = f.read()

# Replace preflight block
old_pf_pattern = """            if mode.auto_return:
                if getattr(mode, "place_target", "basin") == "basin":
                    basin_cfg = cfg.get("place_target", {})
                    transit_z = float(basin_cfg.get("transit_z_safe_m", 0.120))
                    transit_xyz = [grasp_xyz[0], grasp_xyz[1], max(lift_xyz[2] + 0.05, transit_z)]
                    transit_pose = vertical_pose(transit_xyz, selected_yaw, selected_pitch)
                    transit_preflight = node.cartesian_to(
                        transit_pose,
                        f"{label} LIFT -> TRANSIT 预检",
                        start_state=lift_state,
                        velocity_scaling=float(cfg["lift_cartesian"]["velocity_scaling"]),
                        acceleration_scaling=float(cfg["lift_cartesian"]["acceleration_scaling"]),
                        max_tcp_speed_m_s=float(cfg["lift_cartesian"]["max_tcp_speed_m_s"]),
                    )
                    transit_state = node.trajectory_end_state(transit_preflight)

                    app_xyz = basin_cfg.get("approach_xyz_m", [0.464, 0.006, 0.080])
                    place_xyz = basin_cfg.get("place_xyz_m", [0.464, 0.006, -0.020])
                    app_joints = basin_cfg.get("target_joints_rad", [0.0484, -1.6343, 0.7521, -0.0675, -2.0939, -2.8371])
                    basin_yaw = float(basin_cfg.get("yaw_rad", 2.847))
                    basin_pitch = float(basin_cfg.get("pitch_rad", 0.1745))

                    app_pose = vertical_pose(app_xyz, basin_yaw, basin_pitch)
                    place_pose = vertical_pose(place_xyz, basin_yaw, basin_pitch)

                    basin_app_traj = node.plan(
                        node.make_joint_goal(app_joints),
                        f"{label} TRANSIT -> BASIN_APPROACH 预检",
                        start_state=transit_state,
                        planning_time_s=full_plan_time_s,
                        planning_attempts=full_plan_attempts,
                    )"""

new_pf_pattern = """            if mode.auto_return:
                if getattr(mode, "place_target", "basin") == "basin":
                    basin_cfg = cfg.get("place_target", {})
                    app_xyz = basin_cfg.get("approach_xyz_m", [0.464, 0.006, 0.080])
                    place_xyz = basin_cfg.get("place_xyz_m", [0.464, 0.006, -0.020])
                    app_joints = basin_cfg.get("target_joints_rad", [0.0484, -1.6343, 0.7521, -0.0675, -2.0939, -2.8371])
                    basin_yaw = float(basin_cfg.get("yaw_rad", 2.847))
                    basin_pitch = float(basin_cfg.get("pitch_rad", 0.1745))

                    app_pose = vertical_pose(app_xyz, basin_yaw, basin_pitch)
                    place_pose = vertical_pose(place_xyz, basin_yaw, basin_pitch)

                    basin_app_traj = node.plan(
                        node.make_joint_goal(app_joints),
                        f"{label} LIFT -> BASIN_APPROACH 预检",
                        start_state=lift_state,
                        planning_time_s=full_plan_time_s,
                        planning_attempts=full_plan_attempts,
                    )"""

assert old_pf_pattern in code, "old_pf_pattern not found"
code = code.replace(old_pf_pattern, new_pf_pattern, 1)

# Replace execute block
old_ex_pattern = """            # 1. LIFT -> TRANSIT
            node.execute(transit_traj, "LIFT -> TRANSIT")
            node.wait_tcp_arrival(transit_pose, "TRANSIT")
            node.event("到达安全转场高度", "PASS")

            # 2. TRANSIT -> BASIN_APPROACH
            approach_traj = node.plan(
                node.make_joint_goal(app_joints),
                "TRANSIT -> BASIN_APPROACH",
                planning_time_s=float(cfg["candidate_search"]["full_planning_time_s"]),
                planning_attempts=int(cfg["candidate_search"]["full_planning_attempts"]),
            )
            node.execute(approach_traj, "TRANSIT -> BASIN_APPROACH")
            node.event("到达盆口上方", "PASS")"""

new_ex_pattern = """            # 1. LIFT -> BASIN_APPROACH
            approach_traj = node.plan(
                node.make_joint_goal(app_joints),
                "LIFT -> BASIN_APPROACH",
                planning_time_s=float(cfg["candidate_search"]["full_planning_time_s"]),
                planning_attempts=int(cfg["candidate_search"]["full_planning_attempts"]),
            )
            node.execute(approach_traj, "LIFT -> BASIN_APPROACH")
            node.event("到达盆口上方", "PASS")"""

# Also remove transit_traj definition in execute
old_transit_def = """            transit_traj = node.cartesian_to(
                transit_pose,
                "LIFT -> TRANSIT",
                velocity_scaling=0.03,
                acceleration_scaling=0.03,
                max_tcp_speed_m_s=float(cfg["lift_cartesian"]["max_tcp_speed_m_s"]),
            )"""

new_transit_def = """            # LIFT is already complete, will plan directly to BASIN_APPROACH"""

assert old_transit_def in code, "old_transit_def not found"
code = code.replace(old_transit_def, new_transit_def, 1)

assert old_ex_pattern in code, "old_ex_pattern not found"
code = code.replace(old_ex_pattern, new_ex_pattern, 1)

with open(path, "w", encoding="utf-8") as f:
    f.write(code)

print("Direct LIFT -> BASIN_APPROACH applied successfully!")
