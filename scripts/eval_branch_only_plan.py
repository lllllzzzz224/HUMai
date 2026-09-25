#!/usr/bin/env python3
"""Evaluate approach to the orange in MoveIt 2 with BRANCH-ONLY obstacles (leaves filtered out)."""
import json
import math
import sys
import numpy as np
import rclpy
from pathlib import Path
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import PlanningScene, RobotState, CollisionObject
from moveit_msgs.srv import GetPositionIK, GetStateValidity, ApplyPlanningScene

sys.path.insert(0, '/home/humai/桌面/9月机械臂实验（算法）/development/hand_eye_calibration/apple_pick_v2')
from obstacle_scene_ros import RosPlanTransport, JOINTS, box_object

rclpy.init()
node = rclpy.create_node('eval_branch_only')
transport = RosPlanTransport(node)

# True orange center in base_link (from leaf-filtered RANSAC sphere fit)
ORANGE_CENTER = np.array([0.2958, -0.2056, -0.0838])
print(f"=== 真实目标橘子坐标 (base_link): {ORANGE_CENTER} ===")

# Load scene_branch_only.json
scene_file = Path('/home/humai/桌面/9月机械臂实验（算法）/reports/branch_preview_20260922_221500/scene_branch_only.json')
with open(scene_file) as f:
    branch_scene_data = json.load(f)

# Clear existing auto_obstacle_* from MoveIt
current_scene = transport.read()
removes = []
for obj in current_scene.world.collision_objects:
    if obj.id.startswith('auto_obstacle_'):
        rm = CollisionObject(id=obj.id, operation=CollisionObject.REMOVE)
        rm.header.frame_id = 'base_link'
        rm.pose.orientation.w = 1.0
        removes.append(rm)

# Add branch-only obstacles
adds = []
box_size = branch_scene_data['box_size_m']
for ident, center in branch_scene_data['occupied_cells'].items():
    adds.append(box_object(ident, center, [box_size]*3))

diff = PlanningScene()
diff.is_diff = True
diff.robot_state.is_diff = True
diff.world.collision_objects = removes + adds
assert transport.apply(diff), "Failed to apply branch-only diff"

readback = transport.read()
print(f"MoveIt 场景已更新为【纯枝干障碍物】：当前碰撞对象数 = {len(readback.world.collision_objects)} (原2442个全量体素 -> 现{len(adds)}个硬枝体素)")

ik_cli = node.create_client(GetPositionIK, '/compute_ik')
assert ik_cli.wait_for_service(timeout_sec=5.0), "IK service unavailable"

def check_pose_ik_and_collision(pos, rpy_deg):
    rot = Rotation.from_euler('xyz', np.deg2rad(rpy_deg)).as_matrix()
    q = Rotation.from_matrix(rot).as_quat()
    
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
    ("左侧斜下 15° (Pitch -15°)", [180.0, -15.0, math.degrees(yaw_angle)]),
    ("偏航斜入 30° (Yaw +30°, Pitch 20°)", [180.0, 20.0, math.degrees(yaw_angle) + 30.0]),
    ("偏航斜入 -30° (Yaw -30°, Pitch 20°)", [180.0, 20.0, math.degrees(yaw_angle) - 30.0]),
]

results = []
print("\n" + "="*50)
print("=== 纯枝干环境下的多姿态可达性与碰撞评估 ===")
print("="*50)

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

out_file = Path('/home/humai/桌面/9月机械臂实验（算法）/reports/branch_preview_20260922_221500/eval_branch_only_results.json')
out_file.write_text(json.dumps(results, indent=2, ensure_ascii=False))
print(f"\n纯枝干评估报告已保存至: {out_file}")
