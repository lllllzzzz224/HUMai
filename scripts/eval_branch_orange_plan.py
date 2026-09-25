#!/usr/bin/env python3
"""Evaluate vertical vs angled approach to the branch-occluded orange in MoveIt 2."""
import json
import math
import sys
import numpy as np
import rclpy
from pathlib import Path
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import PlanningScene, RobotState
from moveit_msgs.srv import GetPositionIK, GetStateValidity

sys.path.insert(0, '/home/humai/桌面/9月机械臂实验（算法）/development/hand_eye_calibration/apple_pick_v2')
from obstacle_scene_ros import RosPlanTransport, JOINTS

rclpy.init()
node = rclpy.create_node('eval_branch_plan')
transport = RosPlanTransport(node)

# True orange center in base_link (from leaf-filtered RANSAC sphere fit)
ORANGE_CENTER = np.array([0.2958, -0.2056, -0.0838])
print(f"=== 真实目标橘子坐标 (base_link): {ORANGE_CENTER} ===")

scene = transport.read()
print(f"当前 MoveIt 碰撞场景对象数: {len(scene.world.collision_objects)}")

ik_cli = node.create_client(GetPositionIK, '/compute_ik')
assert ik_cli.wait_for_service(timeout_sec=5.0), "IK service unavailable"

def check_pose_ik_and_collision(pos, rpy_deg):
    rot = Rotation.from_euler('xyz', np.deg2rad(rpy_deg)).as_matrix()
    q = Rotation.from_matrix(rot).as_quat()  # [x, y, z, w]
    
    ps = PoseStamped()
    ps.header.frame_id = 'base_link'
    ps.pose.position.x = float(pos[0])
    ps.pose.position.y = float(pos[1])
    ps.pose.position.z = float(pos[2])
    ps.pose.orientation.x = float(q[0])
    ps.pose.orientation.y = float(q[1])
    ps.pose.orientation.z = float(q[2])
    ps.pose.orientation.w = float(q[3])
    
    req = GetPositionIK.Request()
    req.ik_request.group_name = 'rm_group'
    req.ik_request.ik_link_name = 'tcp_link'
    req.ik_request.pose_stamped = ps
    req.ik_request.timeout.sec = 1
    req.ik_request.avoid_collisions = False
    
    fut = ik_cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut)
    res = fut.result()
    
    if res.error_code.val != 1:
        return {"reachability": "NO_IK", "error_code": res.error_code.val}
    
    sol_joints = list(res.solution.joint_state.position)
    
    st = RobotState()
    st.joint_state.name = JOINTS
    st.joint_state.position = sol_joints
    
    v_req = GetStateValidity.Request(robot_state=st, group_name='rm_group')
    v_fut = transport.valid_client.call_async(v_req)
    rclpy.spin_until_future_complete(node, v_fut)
    v_res = v_fut.result()
    
    contacts = [
        f"{c.contact_body_1} <-> {c.contact_body_2}"
        for c in v_res.contacts
    ]
    
    return {
        "reachability": "IK_OK",
        "valid": v_res.valid,
        "joints": [round(math.degrees(v), 2) for v in sol_joints],
        "joints_rad": sol_joints,
        "contacts_count": len(contacts),
        "contacts": contacts[:5]
    }

yaw_angle = math.atan2(ORANGE_CENTER[1], ORANGE_CENTER[0])
print(f"视线方位角 Yaw: {math.degrees(yaw_angle):.1f}°")

test_angles = [
    ("垂直正下 (0°倾角)", [180.0, 0.0, math.degrees(yaw_angle)]),
    ("右侧斜下 15° (Pitch +15°)", [180.0, 15.0, math.degrees(yaw_angle)]),
    ("右侧斜下 25° (Pitch +25°)", [180.0, 25.0, math.degrees(yaw_angle)]),
    ("右侧斜下 35° (Pitch +35°)", [180.0, 35.0, math.degrees(yaw_angle)]),
    ("左侧斜下 15° (Pitch -15°)", [180.0, -15.0, math.degrees(yaw_angle)]),
    ("偏航斜入 30° (Yaw +30°, Pitch 20°)", [180.0, 20.0, math.degrees(yaw_angle) + 30.0]),
    ("偏航斜入 -30° (Yaw -30°, Pitch 20°)", [180.0, 20.0, math.degrees(yaw_angle) - 30.0]),
]

results = []
for name, rpy in test_angles:
    pos_pre = ORANGE_CENTER + np.array([0, 0, 0.12])
    pos_near = ORANGE_CENTER + np.array([0, 0, 0.03])
    
    res_pre = check_pose_ik_and_collision(pos_pre, rpy)
    res_near = check_pose_ik_and_collision(pos_near, rpy)
    
    res = {
        "name": name,
        "rpy": rpy,
        "pregrasp_12cm": res_pre,
        "near_3cm": res_near
    }
    results.append(res)
    print(f"\n--- {name} ---")
    print(f"  预抓取位 (+12cm): IK={res_pre['reachability']}, 无碰撞={res_pre.get('valid')}, 碰撞数={res_pre.get('contacts_count', 0)}")
    if res_pre.get('contacts'):
        print(f"    主要碰撞: {res_pre['contacts']}")
    print(f"  贴近果实 (+3cm) : IK={res_near['reachability']}, 无碰撞={res_near.get('valid')}, 碰撞数={res_near.get('contacts_count', 0)}")
    if res_near.get('contacts'):
        print(f"    主要碰撞: {res_near['contacts']}")

out_file = Path('/home/humai/桌面/9月机械臂实验（算法）/reports/branch_preview_20260922_221500/eval_results.json')
out_file.write_text(json.dumps(results, indent=2, ensure_ascii=False))
print(f"\n评估报告已保存至: {out_file}")
