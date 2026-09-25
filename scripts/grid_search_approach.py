#!/usr/bin/env python3
"""Sweep orientations to find collision-free approach to the orange in MoveIt 2."""
import json
import math
import sys
import numpy as np
import rclpy
from pathlib import Path
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetPositionIK, GetStateValidity

sys.path.insert(0, '/home/humai/桌面/9月机械臂实验（算法）/development/hand_eye_calibration/apple_pick_v2')
from obstacle_scene_ros import RosPlanTransport, JOINTS

rclpy.init()
node = rclpy.create_node('grid_search')
transport = RosPlanTransport(node)

ORANGE_CENTER = np.array([0.2958, -0.2056, -0.0838])
ik_cli = node.create_client(GetPositionIK, '/compute_ik')
assert ik_cli.wait_for_service(timeout_sec=5.0)

def test_approach(pos, yaw_deg, pitch_deg, roll_deg=180.0):
    rot = Rotation.from_euler('xyz', np.deg2rad([roll_deg, pitch_deg, yaw_deg])).as_matrix()
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
        return 'NO_IK', 0, []
    
    st = RobotState()
    st.joint_state.name = JOINTS
    st.joint_state.position = list(res.solution.joint_state.position)
    
    v_req = GetStateValidity.Request(robot_state=st, group_name='rm_group')
    v_fut = transport.valid_client.call_async(v_req)
    rclpy.spin_until_future_complete(node, v_fut)
    v_res = v_fut.result()
    
    return ('VALID' if v_res.valid else 'COLLISION'), len(v_res.contacts), [f'{c.contact_body_1}<->{c.contact_body_2}' for c in v_res.contacts]

print("=== 空间全向网格搜索：寻找无碰撞采摘进近姿态 ===")
found = []
for yaw in range(-90, 91, 15):
    for pitch in range(0, 61, 10):
        pos_grasp = ORANGE_CENTER + np.array([0, 0, 0.02])
        pos_pre = ORANGE_CENTER + np.array([0, 0, 0.12])
        
        st_pre, c_pre, _ = test_approach(pos_pre, yaw, pitch)
        if st_pre == 'VALID':
            st_g, c_g, _ = test_approach(pos_grasp, yaw, pitch)
            print(f"[*] 发现无碰撞预抓取位! Yaw={yaw:+.0f}deg, Pitch={pitch:+.0f}deg: Pregrasp={st_pre}, Grasp={st_g} (碰撞数={c_g})")
            found.append((yaw, pitch, st_pre, st_g, c_g))

if not found:
    print("447个硬树枝体素下，预抓取位仍有接触。")
else:
    print(f"共搜索到 {len(found)} 个候选通道！")
