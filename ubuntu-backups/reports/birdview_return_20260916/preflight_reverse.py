#!/usr/bin/env python3
"""Read-only two-waypoint return preflight, no scene removal or execution."""
import sys,json,time,copy,importlib.util
from pathlib import Path
import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetPositionFK,GetCartesianPath
from rclpy.serialization import serialize_message,deserialize_message
from moveit_msgs.msg import RobotTrajectory
import hashlib
P=Path(__file__).resolve().parent
sys.path.insert(0,'/home/li/hand_eye_calibration/apple_pick_v2')
from fixed_scan_real_verify import load_config
spec=importlib.util.spec_from_file_location('prior_readonly','/home/li/桌面/9月机械臂实验（算法）/reports/near_alignment_20260916/re_align_high/preflight_alignment.py');prior=importlib.util.module_from_spec(spec);spec.loader.exec_module(prior)
mod=prior.mod
class ReadOnly(prior.ReadOnly):
 def apply_protection(self):
  # Never reconstruct or remove existing geometry during this return.
  from verify_current_scene import verify_scene
  verify_scene(self,self.cfg,96)
  return 96
def main():
 cfg=load_config(P/'config.yaml');cfg['cartesian']['max_step_m']=.002;cfg['cartesian']['max_tcp_speed_m_s']=.008
 out={'motion_commands':False,'gripper_commands':False,'status':'STARTED','table_top_m':-.055,'tool_assumption_m':[.16,.16,.22]}
 rclpy.init();n=ReadOnly(cfg)
 try:
  for client in [n.scene_client,n.validity_client,n.ik_client,n.cartesian_client]:
   if not client.wait_for_service(timeout_sec=10):raise RuntimeError('Missing service')
  if not n.spin_until(lambda:len(n.joints)==6,10,'real joints'):raise RuntimeError('No joints')
  if not n.spin_until(lambda:n.tf_buffer.can_transform(n.base,n.tcp,rclpy.time.Time()),5,'TCP TF available'):raise RuntimeError('TCP TF missing')
  tf=n.tf_buffer.lookup_transform(n.base,n.tcp,rclpy.time.Time());p=tf.transform.translation;q=tf.transform.rotation
  pose=Pose();ref=json.loads(Path('/home/li/桌面/9月机械臂实验（算法）/reports/alignment_20260916/approved_plan.json').read_text());pose.position.x,pose.position.y,pose.position.z=map(float,ref['start_m']);pose.orientation=copy.deepcopy(q)
  start=np.array([p.x,p.y,p.z]);expected=np.array(json.loads((P/'geometry.json').read_text())['records'][0]['tcp_m']);out['start_m']=start.tolist();out['start_error_m']=float(np.linalg.norm(start-expected));out['target_m']=[pose.position.x,pose.position.y,pose.position.z]
  if out['start_error_m']>.003 or time.monotonic()-n.joint_rx_monotonic>.5:raise RuntimeError('Start moved/stale')
  out['observed_voxels']=n.apply_protection();
  from verify_current_scene import verify_scene
  out['scene_readback']=verify_scene(n,cfg,out['observed_voxels'])
  n.current_state_is_valid();n.solve_ik(pose,'high alignment',attempts=3)
  sources=[Path('/home/li/桌面/9月机械臂实验（算法）/reports/near_alignment_20260916/re_align_high/trajectory.cdr'),Path('/home/li/桌面/9月机械臂实验（算法）/reports/alignment_20260916/trajectory.cdr')]
  expected_hashes=['2b6530857eb37eba44e4eefe28f0093ace012e399baf9bae04a8f94def26c4e9','b7e758e579535293b3b03ccf78aa0f3ffc88fcd2bb3b85e72ac87586b25954c8']
  traj=RobotTrajectory();elapsed=0.;joins=[]
  for source,expected_hash in zip(sources,expected_hashes):
   data=source.read_bytes()
   if hashlib.sha256(data).hexdigest()!=expected_hash:raise RuntimeError('source hash changed')
   old=deserialize_message(data,RobotTrajectory).joint_trajectory
   if traj.joint_trajectory.joint_names and traj.joint_trajectory.joint_names!=old.joint_names:raise RuntimeError('source joint names differ')
   traj.joint_trajectory.joint_names=old.joint_names;total=old.points[-1].time_from_start.sec+old.points[-1].time_from_start.nanosec*1e-9
   if traj.joint_trajectory.points:
    joins.append(float(np.max(np.abs(np.array(traj.joint_trajectory.points[-1].positions)-np.array(old.points[-1].positions)))));elapsed+=.5
   for point in reversed(old.points):
    pp=copy.deepcopy(point);t=elapsed+total-(point.time_from_start.sec+point.time_from_start.nanosec*1e-9);ns=round(t*1e9);pp.time_from_start.sec=ns//1000000000;pp.time_from_start.nanosec=ns%1000000000;pp.velocities=[-v for v in pp.velocities];traj.joint_trajectory.points.append(pp)
   elapsed+=total
  out['source_hashes']=expected_hashes;out['join_joint_delta_rad']=joins
  positions=np.array([pt.positions for pt in traj.joint_trajectory.points]);out['max_joint_step_rad']=float(np.max(np.abs(np.diff(positions,axis=0))))
  if out['max_joint_step_rad']>float(cfg['cartesian']['revolute_jump_threshold_rad']):raise RuntimeError('joint jump threshold exceeded')
  if np.max(np.abs(positions[0]-n.current_joint_vector(traj.joint_trajectory.joint_names)))>.01:raise RuntimeError('new start mismatch')
  n.validate_trajectory(traj,'reversed verified paths');traj,out['retime']=mod.retime_up(n,traj)
  extra=0
  for aa,bb in zip(positions[:-1],positions[1:]):
   count=int(np.ceil(np.max(np.abs(bb-aa))/.01))
   for alpha in np.linspace(0,1,max(count,1)+1)[1:-1]:
    state=RobotState();state.joint_state.name=traj.joint_trajectory.joint_names;state.joint_state.position=((1-alpha)*aa+alpha*bb).tolist();n.state_is_valid(state,'interpolated return sample');extra+=1
  out['extra_interpolated_collision_samples']=extra
  fk=n.create_client(GetPositionFK,'/compute_fk');mins=[];camera_mins=[];tcpz=[];tcp_xyz=[];rotations=[]
  for pt in traj.joint_trajectory.points:
   req=GetPositionFK.Request();req.header.frame_id=n.base;req.fk_link_names=[n.tcp,'Link6'];req.robot_state.joint_state.name=traj.joint_trajectory.joint_names;req.robot_state.joint_state.position=pt.positions
   f=fk.call_async(req);rclpy.spin_until_future_complete(n,f,timeout_sec=3);res=f.result()
   if res is None or res.error_code.val!=1:raise RuntimeError('FK failed')
   tp,lp=[x.pose for x in res.pose_stamped];tcpz.append(tp.position.z);tcp_xyz.append([tp.position.x,tp.position.y,tp.position.z])
   from scipy.spatial.transform import Rotation
   rot=Rotation.from_quat([lp.orientation.x,lp.orientation.y,lp.orientation.z,lp.orientation.w]).as_matrix();rotations.append(rot)
   corners=np.array([[x,y,z] for x in [-.08,.08] for y in [-.08,.08] for z in [0,.22]])
   mins.append(float((corners@rot.T+np.array([lp.position.x,lp.position.y,lp.position.z]))[:,2].min()))
   sc=cfg['scene'];dim=np.array(sc['camera_box_xyz_m'])/2;cr=Rotation.from_quat(sc['camera_quaternion_link6_xyzw']).as_matrix();cc=np.array([[x,y,z] for x in [-dim[0],dim[0]] for y in [-dim[1],dim[1]] for z in [-dim[2],dim[2]]]);cc=cc@cr.T+np.array(sc['camera_translation_link6_m']);camera_mins.append(float((cc@rot.T+np.array([lp.position.x,lp.position.y,lp.position.z]))[:,2].min()))
   state=RobotState();state.joint_state=req.robot_state.joint_state;n.state_is_valid(state,'path sample')
  reference=np.array(json.loads(Path('/home/li/桌面/9月机械臂实验（算法）/reports/alignment_20260916/capture.json').read_text())['records'][0]['base_to_Link6'])[:3,:3];out['final_orientation_vs_reference_rad']=float(Rotation.from_matrix(rotations[-1]@reference.T).magnitude());out['final_fk_tcp_m']=tcp_xyz[-1];out['target_m']=tcp_xyz[-1];out['tcp_samples_m']=tcp_xyz;out['min_camera_box_z_m']=min(camera_mins);out['min_tcp_z_m']=min(tcpz);out['min_tool_box_z_m']=min(mins);out['tool_table_clearance_m']=min(mins)+.055;out['status']='PASS_PREFLIGHT_ONLY';(P/'trajectory.cdr').write_bytes(serialize_message(traj))
 except Exception as e:out['status']='FAILED';out['error']=str(e)
 finally:(P/'preflight.json').write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2));n.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
