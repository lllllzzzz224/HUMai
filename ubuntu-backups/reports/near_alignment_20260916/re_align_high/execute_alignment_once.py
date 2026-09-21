#!/usr/bin/env python3
"""Reviewed single high alignment. Requires explicit execute flag; never closes."""
import argparse,hashlib,json,time,sys,copy
from pathlib import Path
import numpy as np
import rclpy
from scipy.spatial.transform import Rotation
from rclpy.serialization import deserialize_message
from moveit_msgs.msg import RobotTrajectory,RobotState,PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene
from moveit_msgs.action import ExecuteTrajectory
from std_msgs.msg import Empty,Bool
from preflight_alignment import ReadOnly,P,load_config
sys.path.insert(0,'/home/li/hand_eye_calibration')

def main():
 ap=argparse.ArgumentParser();ap.add_argument('--execute',action='store_true',required=True);a=ap.parse_args()
 record={'status':'STARTED','closed':False,'return':False};out=P/('execution_'+time.strftime('%Y%m%d_%H%M%S')+'.json')
 lock=P/'execution_once.lock';fd=lock.open('x');fd.write(str(out));fd.close()
 cfg=load_config(P/'config.yaml');approved=json.loads((P/'approved_plan.json').read_text());data=(P/'trajectory.cdr').read_bytes()
 if hashlib.sha256(data).hexdigest()!=approved['trajectory_sha256']:raise RuntimeError('trajectory changed')
 traj=deserialize_message(data,RobotTrajectory);rclpy.init();n=ReadOnly(cfg);handle=None;possibly_active=False;stop_results=[]
 pub=n.create_publisher(Empty,'/rm_driver/move_stop_cmd',10);sub=n.create_subscription(Bool,'/rm_driver/move_stop_result',lambda msg:stop_results.append((time.monotonic(),msg.data)),10)
 def save():out.write_text(json.dumps(record,indent=2))
 def stop():
  ts=time.monotonic();pub.publish(Empty());history=[];ok=False
  deadline=time.monotonic()+4.0
  while time.monotonic()<deadline:
   rclpy.spin_once(n,timeout_sec=.05)
   if time.monotonic()-n.joint_rx_monotonic<.2:
    history.append((time.monotonic(),n.current_joint_vector().copy()))
    history=[x for x in history if x[0]>time.monotonic()-1.0]
   if history and history[-1][0]-history[0][0]>.7 and np.max(np.ptp(np.array([x[1] for x in history]),axis=0))<.001 and any(t>=ts and v for t,v in stop_results):ok=True;break
  record['stop_confirmed']=ok
  if handle is not None:
   f=handle.cancel_goal_async();rclpy.spin_until_future_complete(n,f,timeout_sec=3);record['cancel_reply']=f.result() is not None
   terminal=handle.get_result_async();rclpy.spin_until_future_complete(n,terminal,timeout_sec=3);record['action_terminal_confirmed']=terminal.done() and terminal.result() is not None
  save()
  if not ok:print('STOP NOT CONFIRMED — physical operator attention required',flush=True)
 try:
  for c in [n.scene_client,n.validity_client]:
   if not c.wait_for_service(timeout_sec=10):raise RuntimeError('service absent')
  if not n.spin_until(lambda:len(n.joints)==6,10,'real state'):raise RuntimeError('no joints')
  for _ in range(20):rclpy.spin_once(n,timeout_sec=.05)
  if time.monotonic()-n.joint_rx_monotonic>.5:raise RuntimeError('stale joints')
  start=n.current_joint_vector(list(traj.joint_trajectory.joint_names));delta=float(np.max(np.abs(start-np.array(traj.joint_trajectory.points[0].positions))));record['start_joint_error_rad']=delta
  if delta>.01 or np.linalg.norm(n.tcp_position()-np.array(approved['start_m']))>.003:raise RuntimeError('start changed')
  n.apply_protection()
  from verify_current_scene import verify_scene
  record['scene_readback']=verify_scene(n,cfg,approved['observed_voxels'])
  n.current_state_is_valid()
  for pt in traj.joint_trajectory.points:
   st=RobotState();st.joint_state.name=traj.joint_trajectory.joint_names;st.joint_state.position=pt.positions;n.state_is_valid(st,'execution recheck')
  if not n.execute_client.wait_for_server(timeout_sec=5) or pub.get_subscription_count()<2:raise RuntimeError('execution/stop interfaces absent')
  record['gripper_commands']=False;save()
  if time.monotonic()-n.joint_rx_monotonic>.5 or np.linalg.norm(n.tcp_position()-np.array(approved['start_m']))>.003 or np.max(np.abs(n.current_joint_vector(list(traj.joint_trajectory.joint_names))-start))>.01:raise RuntimeError('start changed/stale before movement')
  goal=ExecuteTrajectory.Goal();goal.trajectory=traj;possibly_active=True;f=n.execute_client.send_goal_async(goal);rclpy.spin_until_future_complete(n,f,timeout_sec=5);handle=f.result()
  if handle is None or not handle.accepted:raise RuntimeError('execution goal uncertain/rejected')
  print('HIGH HORIZONTAL REALIGNMENT trajectory started',flush=True);f=handle.get_result_async();duration=approved['retime']['retimed_duration_s'];deadline=time.monotonic()+duration+10
  while not f.done() and time.monotonic()<deadline:
   rclpy.spin_once(n,timeout_sec=.05)
   if time.monotonic()-n.joint_rx_monotonic>.5:raise RuntimeError('feedback stale during motion')
  result=f.result() if f.done() else None
  if result is None or result.result.error_code.val!=1:raise RuntimeError('execution timeout/failure')
  possibly_active=False
  from geometry_msgs.msg import Pose
  pose=Pose();pose.position.x,pose.position.y,pose.position.z=approved['target_m'];n.wait_tcp_arrival(pose,'HIGH ALIGNMENT')
  record['final_tcp_m']=n.tcp_position().tolist();record['status']='PASS_HIGH_ALIGNMENT_ONLY';save();print(json.dumps(record),flush=True)
 except BaseException as e:
  record['status']='FAILED';record['error']=repr(e)
  if possibly_active:stop()
  save();raise
 finally:n.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
