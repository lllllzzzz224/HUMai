#!/usr/bin/env python3
"""One guarded VERIFY->PREGRASP->SCAN retreat; no gripper object or command."""
import copy
import argparse
import json
import math
from pathlib import Path
import sys
import time
import numpy as np
import rclpy
from moveit_msgs.action import MoveGroup
from moveit_msgs.srv import GetPositionFK
from moveit_msgs.msg import RobotState, Constraints
sys.path.insert(0, '/home/li/hand_eye_calibration/apple_pick_v2')
from fixed_scan_real_verify import FixedScanVerify, load_config, vertical_pose, SUCCESS

class Retreat(FixedScanVerify):
    def plan(
        self,
        goal_constraint: Constraints,
        label: str,
        *,
        start_state: RobotState | None = None,
        planning_time_s: float | None = None,
        planning_attempts: int | None = None,
    ):
        move = self.cfg["moveit"]
        goal = MoveGroup.Goal()
        goal.request.group_name = str(move["group"])
        goal.request.pipeline_id = str(move["pipeline"])
        goal.request.planner_id = str(move["planner"])
        if start_state is None:
            goal.request.start_state.is_diff = True
        else:
            goal.request.start_state = start_state
        goal.request.goal_constraints = [goal_constraint]
        goal.request.num_planning_attempts = int(
            move["planning_attempts"]
            if planning_attempts is None
            else planning_attempts
        )
        goal.request.allowed_planning_time = float(
            move["planning_time_s"]
            if planning_time_s is None
            else planning_time_s
        )
        goal.request.max_velocity_scaling_factor = float(move["velocity_scaling"])
        goal.request.max_acceleration_scaling_factor = float(move["acceleration_scaling"])
        goal.planning_options.plan_only = True
        goal.planning_options.look_around = False
        goal.planning_options.replan = False

        future = self.move_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        handle = future.result()
        if handle is None or not handle.accepted:
            raise RuntimeError(f"{label}：MoveIt 拒绝规划请求")
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(
            self, result_future, timeout_sec=float(self.cfg["safety"]["motion_timeout_s"])
        )
        wrapped = result_future.result()
        if wrapped is None:
            raise RuntimeError(f"{label}：MoveIt 规划超时")
        result = wrapped.result
        if result.error_code.val != SUCCESS:
            raise RuntimeError(f"{label}：MoveIt 规划失败 error_code={result.error_code.val}")
        self.validate_trajectory(result.planned_trajectory, label)
        return result.planned_trajectory


def inspect_pose(node, expected):
    # Refresh subscriptions, require recent joint data and transform stamp.
    end = time.monotonic() + 0.5
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    if time.monotonic() - node.joint_rx_monotonic > 0.5:
        raise RuntimeError('Joint state stale')
    tf = node.tf_buffer.lookup_transform(node.base, node.tcp, rclpy.time.Time())
    age = (node.get_clock().now().nanoseconds - rclpy.time.Time.from_msg(tf.header.stamp).nanoseconds) / 1e9
    if not -0.1 <= age <= 0.5:
        raise RuntimeError(f'TCP TF stale: {age}s')
    t, q = tf.transform.translation, tf.transform.rotation
    actual = np.array([t.x,t.y,t.z])
    wanted = np.array([expected.position.x,expected.position.y,expected.position.z])
    err = float(np.linalg.norm(actual-wanted))
    eq = expected.orientation
    qa=np.array([q.x,q.y,q.z,q.w]); qb=np.array([eq.x,eq.y,eq.z,eq.w])
    angle=2*math.acos(float(np.clip(abs(np.dot(qa,qb)/np.linalg.norm(qa)/np.linalg.norm(qb)),0,1)))
    if err > 0.003 or angle > 0.03:
        raise RuntimeError(f'Pose mismatch: position={err}m orientation={angle}rad')
    return dict(tcp_m=actual.tolist(), position_error_m=err,orientation_error_rad=angle,tf_age_s=age,joints_rad=node.current_joint_vector().tolist())

def endpoint(trajectory):
    state=RobotState()
    state.joint_state.name=list(trajectory.joint_trajectory.joint_names)
    state.joint_state.position=list(trajectory.joint_trajectory.points[-1].positions)
    state.is_diff=True
    return state

def retime_up(node, trajectory):
    client=node.create_client(GetPositionFK,'/compute_fk')
    if not client.wait_for_service(timeout_sec=10.0):
        raise RuntimeError('FK service unavailable')
    xyz=[]
    jt=trajectory.joint_trajectory
    for point in jt.points:
        req=GetPositionFK.Request()
        req.header.frame_id=node.base
        req.fk_link_names=[node.tcp]
        req.robot_state.joint_state.name=list(jt.joint_names)
        req.robot_state.joint_state.position=list(point.positions)
        req.robot_state.is_diff=True
        future=client.call_async(req)
        rclpy.spin_until_future_complete(node,future,timeout_sec=3.0)
        result=future.result()
        if result is None or result.error_code.val!=SUCCESS or len(result.pose_stamped)!=1 or result.pose_stamped[0].header.frame_id!=node.base:
            raise RuntimeError('FK failed or returned wrong frame')
        pp=result.pose_stamped[0].pose.position
        xyz.append([pp.x,pp.y,pp.z])
    times=np.array([p.time_from_start.sec+p.time_from_start.nanosec*1e-9 for p in jt.points])
    dt=np.diff(times)
    if len(dt)==0 or not np.all(dt>0):raise RuntimeError('Non-increasing trajectory timestamps')
    positions=np.array(xyz)
    if not np.all(np.isfinite(positions)):raise RuntimeError('Non-finite FK position')
    distances=np.linalg.norm(np.diff(positions,axis=0),axis=1)
    before=float(np.max(distances/dt))
    stretch=max(1.0,before/0.008)*1.1
    scaled=copy.deepcopy(trajectory)
    for point,t in zip(scaled.joint_trajectory.points,times):
        ns=round(float(t)*stretch*1e9)
        point.time_from_start.sec=ns//1000000000
        point.time_from_start.nanosec=ns%1000000000
        point.velocities=[float(v)/stretch for v in point.velocities]
        point.accelerations=[float(a)/(stretch*stretch) for a in point.accelerations]
    newtimes=np.array([p.time_from_start.sec+p.time_from_start.nanosec*1e-9 for p in scaled.joint_trajectory.points])
    if not np.all(np.diff(newtimes)>0):raise RuntimeError('Retime produced non-increasing timestamps')
    after=float(np.max(distances/np.diff(newtimes)))
    if after>0.008+1e-9:raise RuntimeError('Sampled segment speed exceeds 0.008m/s')
    node.validate_trajectory(scaled,'VERIFY->PREGRASP FK retimed')
    return scaled,dict(original_duration_s=float(times[-1]),retimed_duration_s=float(newtimes[-1]),original_max_sampled_segment_speed_m_s=before,retimed_max_sampled_segment_speed_m_s=after,stretch=stretch,fk_points=len(xyz),scope='sampled segment mean speed; continuous instantaneous speed not guaranteed')

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--execute',action='store_true')
    parser.add_argument('--run-log',type=Path,default=Path('/home/li/hand_eye_calibration/apple_pick_v2/results/single_apple_grasp_20260914_203004.json'))
    parser.add_argument('--config',type=Path,default=Path('/home/li/桌面/9月机械臂实验（算法）/reports/live/single_orange_control/open_at_grasp/config.yaml'))
    args=parser.parse_args()
    source=json.loads(args.run_log.read_text())
    if source.get('status')!='PASS_VERIFY_ONLY' or source.get('gripper_close_commanded') is not False or source.get('returned_to_scan') is not False:
        raise RuntimeError('Requires untouched VERIFY_ONLY source log')
    candidate=source['selected_candidate']
    if candidate['pitch_deg']!=0 or candidate['yaw_offset_deg']!=90:
        raise RuntimeError('Unexpected source orientation')
    verify=np.array(source['verify_base_m']); pre=np.array(source['pregrasp_base_m'])
    if not np.allclose(pre-verify,[0,0,0.09],atol=1e-6):
        raise RuntimeError('Source is not a 90mm vertical retreat')
    cfg=load_config(args.config)
    if not cfg['visual_xy_correction']['enabled'] or cfg['candidate_search']['yaw_offsets_deg']!=[90] or cfg['candidate_search']['pitch_stages_deg']!=[[0]]:
        raise RuntimeError('Configuration changed from yaw90/pitch0/correction-enabled')
    # Local in-memory conservative speed limits; production YAML is unchanged.
    cfg['moveit']['velocity_scaling']=0.03
    cfg['moveit']['acceleration_scaling']=0.03
    cfg['cartesian']['max_tcp_speed_m_s']=0.008
    yaw=float(candidate['yaw_rad'])
    verify_pose=vertical_pose(verify,yaw)
    pre_pose=vertical_pose(pre,yaw)
    record={'source':str(args.run_log),'execute_requested':args.execute,'gripper_commands':False,'status':'STARTED'}
    output=Path(__file__).parent / ('retreat_'+time.strftime('%Y%m%d_%H%M%S')+'.json')
    rclpy.init();node=Retreat(cfg)
    try:
        node.wait_interfaces()
        record['start']=inspect_pose(node,verify_pose)
        node.current_state_is_valid()
        up=node.cartesian(pre_pose)
        up,record['up_retime']=retime_up(node,up)
        # This inherited method logs PREGRASP->VERIFY; actual target is PREGRASP.
        record['up_target_m']=pre.tolist()
        end_state=endpoint(up)
        node.state_is_valid(end_state,'PREGRASP endpoint')
        node.plan(node.make_joint_goal(node.scan_vector()),'PREGRASP->SCAN chained preflight',start_state=end_state)
        record['preflight']='PASS'
        if not args.execute:
            record['status']='PASS_PLAN_ONLY'
            return 0
        inspect_pose(node,verify_pose)
        node.execute(up,'VERIFY->PREGRASP vertical retreat 90mm')
        node.wait_tcp_arrival(pre_pose,'PREGRASP')
        record['pregrasp']=inspect_pose(node,pre_pose)
        node.current_state_is_valid()
        scan=node.plan(node.make_joint_goal(node.scan_vector()),'PREGRASP->SCAN actual-state replan')
        inspect_pose(node,pre_pose)
        node.execute(scan,'PREGRASP->SCAN')
        node.wait_scan()
        record['final_joints_rad']=node.current_joint_vector().tolist()
        record['status']='PASS_RETURNED_TO_SCAN'
        return 0
    except Exception as exc:
        record['status']='FAILED_STOPPED';record['error']=str(exc)
        print(str(exc),flush=True)
        return 2
    finally:
        record['finished_at']=time.strftime('%Y-%m-%d %H:%M:%S')
        output.write_text(json.dumps(record,ensure_ascii=False,indent=2))
        print(json.dumps(record,ensure_ascii=False),flush=True)
        print('RESULT_LOG='+str(output),flush=True)
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':sys.exit(main())
