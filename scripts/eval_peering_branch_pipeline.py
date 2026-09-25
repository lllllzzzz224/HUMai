#!/usr/bin/env python3
"""
Evaluate Peering & Branch Cylinder Avoidance Pipeline (Scheme 2 + Scheme 3)
----------------------------------------------------------------------------
1. Runs FoliageBranchExtractor on registered point cloud to extract rigid branch cylinders.
2. Injects extracted branch cylinders into MoveIt 2 PlanningScene.
3. Tests IK reachability and collision validity across candidate approach vectors:
   - Baseline: Vertical approach (0° pitch) -> expected to collide with rigid branch
   - Angled approaches (Pitch 15°~35°, Yaw ±30°) -> expected to bypass branch
4. Validates that MoveIt correctly flags collisions with branch cylinders and confirms clear paths.
"""

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Any, Dict, List

import numpy as np
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import (
    CollisionObject,
    PlanningScene,
    RobotState,
)
from moveit_msgs.srv import ApplyPlanningScene, GetPositionIK, GetStateValidity
from shape_msgs.msg import SolidPrimitive

from foliage_branch_extractor import BranchCylinder, FoliageBranchExtractor

JOINTS = [f"joint{i}" for i in range(1, 7)]
SCAN_JOINTS = [2.510152891, 0.354521179, 0.647129746, 0.061472860, 2.045541343, -0.071433319]


def vertical_pose(position, yaw: float, pitch: float = 0.0) -> Pose:
    """Downward tool pose with an optional bounded pitch offset."""
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = map(float, position)
    cp = math.cos(0.5 * pitch)
    sp = math.sin(0.5 * pitch)
    cy = math.cos(0.5 * yaw)
    sy = math.sin(0.5 * yaw)
    pose.orientation.x = cp * cy
    pose.orientation.y = cp * sy
    pose.orientation.z = -sp * cy
    pose.orientation.w = sp * sy
    return pose


class MoveItBranchEvaluator:
    def __init__(self, node: Node):
        self.node = node
        self.apply_scene_cli = node.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self.valid_cli = node.create_client(GetStateValidity, "/check_state_validity")
        self.ik_cli = node.create_client(GetPositionIK, "/compute_ik")

        # Wait for MoveIt services
        timeout_sec = 10.0
        assert self.apply_scene_cli.wait_for_service(timeout_sec=timeout_sec), "ApplyPlanningScene unavailable"
        assert self.valid_cli.wait_for_service(timeout_sec=timeout_sec), "GetStateValidity unavailable"
        assert self.ik_cli.wait_for_service(timeout_sec=timeout_sec), "GetPositionIK unavailable"
        self.node.get_logger().info("MoveIt 2 planning scene services connected.")

    def inject_cylinders(self, cylinders: List[BranchCylinder]) -> bool:
        """Injects branch cylinders into MoveIt planning scene."""
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.robot_state.joint_state.name = JOINTS
        scene.robot_state.joint_state.position = [0.0, 0.5, -1.0, 0.0, 0.5, 0.0]

        changes = []
        for c in cylinders:
            obj = CollisionObject()
            obj.header.frame_id = "base_link"
            obj.id = c.id
            obj.pose.orientation.w = 1.0
            obj.operation = CollisionObject.ADD

            prim = SolidPrimitive()
            prim.type = SolidPrimitive.CYLINDER
            # In ROS 2 shape_msgs SolidPrimitive.CYLINDER dimensions: [CYLINDER_HEIGHT, CYLINDER_RADIUS]
            prim.dimensions = [float(c.length_m), float(c.radius_inflated_m)]
            obj.primitives = [prim]

            pose = Pose()
            pose.position.x = float(c.center[0])
            pose.position.y = float(c.center[1])
            pose.position.z = float(c.center[2])
            pose.orientation.x = float(c.quaternion[0])
            pose.orientation.y = float(c.quaternion[1])
            pose.orientation.z = float(c.quaternion[2])
            pose.orientation.w = float(c.quaternion[3])
            obj.primitive_poses = [pose]

            changes.append(obj)

        scene.world.collision_objects = changes
        req = ApplyPlanningScene.Request(scene=scene)
        fut = self.apply_scene_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut)
        res = fut.result()
        return res.success if res else False

    def evaluate_approach_pose(
        self, pos: np.ndarray, yaw_rad: float, pitch_rad: float
    ) -> Dict[str, Any]:
        """Evaluates IK solution and collision status for a given TCP pose."""
        pose = vertical_pose(pos, yaw=yaw_rad, pitch=pitch_rad)

        ps = PoseStamped()
        ps.header.frame_id = "base_link"
        ps.pose = pose

        seed = RobotState()
        seed.joint_state.name = JOINTS
        seed.joint_state.position = SCAN_JOINTS
        seed.is_diff = True

        req = GetPositionIK.Request()
        req.ik_request.group_name = "rm_group"
        req.ik_request.ik_link_name = "tcp_link"
        req.ik_request.pose_stamped = ps
        req.ik_request.robot_state = seed
        req.ik_request.timeout.sec = 1
        req.ik_request.avoid_collisions = False  # Let IK solve, check collision separately

        fut = self.ik_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut)
        res = fut.result()

        if res.error_code.val != 1:
            return {"ik": "NO_IK", "error_code": res.error_code.val, "valid": False}

        sol_joints = list(res.solution.joint_state.position)

        st = RobotState()
        st.joint_state.name = JOINTS
        st.joint_state.position = sol_joints

        v_req = GetStateValidity.Request(robot_state=st, group_name="rm_group")
        v_fut = self.valid_cli.call_async(v_req)
        rclpy.spin_until_future_complete(self.node, v_fut)
        v_res = v_fut.result()

        contacts = [f"{c.contact_body_1} <-> {c.contact_body_2}" for c in v_res.contacts]

        return {
            "ik": "IK_OK",
            "valid": v_res.valid,
            "joints_deg": [round(math.degrees(v), 2) for v in sol_joints],
            "contacts_count": len(contacts),
            "contacts": contacts[:5],
        }


def main():
    parser = argparse.ArgumentParser(description="Evaluate peering branch pipeline in MoveIt 2.")
    parser.add_argument("--npz", type=str, required=True, help="Point cloud npz path")
    parser.add_argument(
        "--fruit-center",
        type=float,
        nargs=3,
        default=[0.3534, -0.2419, -0.0681],
        help="Target fruit center in base_link",
    )
    parser.add_argument("--output", type=str, default="/tmp/eval_peering_results.json")
    args = parser.parse_args()

    # Step 1: Extract Cylinders
    npz_data = np.load(args.npz)
    xyz_pts = npz_data["xyz_m"]
    rgb_pts = npz_data["rgb"]
    fruit_center = np.array(args.fruit_center)

    extractor = FoliageBranchExtractor(margin_inflation_m=0.012)
    cylinders, stats = extractor.extract(xyz_pts, rgb_pts, fruit_center)

    print("==================================================")
    print("      Scheme 2 + Scheme 3 Pipeline Evaluation     ")
    print("==================================================")
    print(f"Target fruit: {list(fruit_center)}")
    print(f"Fitted cylinders count: {len(cylinders)}")
    for c in cylinders:
        print(f"  [{c.id}] radius={c.radius_inflated_m*1000:.1f}mm, length={c.length_m*1000:.1f}mm, center={c.center}")

    # Step 2: Initialize ROS 2 & MoveIt
    rclpy.init()
    node = rclpy.create_node("eval_peering_pipeline")
    evaluator = MoveItBranchEvaluator(node)

    # Inject into scene
    ok = evaluator.inject_cylinders(cylinders)
    print(f"\nMoveIt PlanningScene Injection: {'SUCCESS' if ok else 'FAILED'}")

    # Step 3: Test Approach Trajectories
    # Azimuth yaw towards fruit
    yaw_base = math.atan2(fruit_center[1], fruit_center[0])

    test_angles = [
        ("垂直直下 (Pitch 0°)", yaw_base, 0.0),
        ("适度前倾 (Pitch +10°)", yaw_base, math.radians(10.0)),
        ("大角度前倾 (Pitch +20°)", yaw_base, math.radians(20.0)),
        ("极度前倾 (Pitch +30°)", yaw_base, math.radians(30.0)),
        ("适度后倾 (Pitch -15°)", yaw_base, math.radians(-15.0)),
        ("偏航向右 (Yaw +20°, Pitch 15°)", yaw_base + math.radians(20.0), math.radians(15.0)),
        ("偏航向左 (Yaw -20°, Pitch 15°)", yaw_base - math.radians(20.0), math.radians(15.0)),
    ]

    results = []
    print("\n--- 进刀姿态避障与碰撞测试结果 ---")
    for name, yaw, pitch in test_angles:
        # Pregrasp at +12cm along Z, Near-grasp at +3cm
        pos_pre = fruit_center + np.array([0.0, 0.0, 0.12])
        pos_near = fruit_center + np.array([0.0, 0.0, 0.03])

        res_pre = evaluator.evaluate_approach_pose(pos_pre, yaw, pitch)
        res_near = evaluator.evaluate_approach_pose(pos_near, yaw, pitch)

        entry = {
            "name": name,
            "yaw_deg": math.degrees(yaw),
            "pitch_deg": math.degrees(pitch),
            "pregrasp_12cm": res_pre,
            "near_3cm": res_near,
        }
        results.append(entry)

        status_pre = "✔ 无碰撞" if res_pre.get("valid") else f"✘ 碰撞 ({res_pre.get('contacts_count')} 处)"
        status_near = "✔ 无碰撞" if res_near.get("valid") else f"✘ 碰撞 ({res_near.get('contacts_count')} 处)"
        print(f"[{name}]")
        print(f"  预抓取位 (+12cm): IK={res_pre['ik']} | {status_pre}")
        if not res_pre.get("valid") and res_pre.get("contacts"):
            print(f"    碰撞实体: {res_pre['contacts']}")
        print(f"  贴近果实 (+3cm) : IK={res_near['ik']} | {status_near}")
        if not res_near.get("valid") and res_near.get("contacts"):
            print(f"    碰撞实体: {res_near['contacts']}")

    out_file = Path(args.output)
    report = {
        "fruit_center": list(fruit_center),
        "cylinders": [c.to_dict() for c in cylinders],
        "evaluations": results,
    }
    out_file.write_text(json.dumps(report, indent=2, ensure_ascii=False))
    print(f"\n完整评估数据已保存至: {out_file}")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
