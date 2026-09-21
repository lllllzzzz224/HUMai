#!/usr/bin/env python3
"""Read-only path feasibility; no movement or gripper entry point."""
import sys,json,time,copy,importlib.util
from pathlib import Path
import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import PlanningScene,CollisionObject,AttachedCollisionObject,RobotState
from moveit_msgs.srv import ApplyPlanningScene,GetPositionFK
from shape_msgs.msg import SolidPrimitive
from rclpy.serialization import serialize_message
P=Path(__file__).resolve().parent
sys.path.insert(0,'/home/li/hand_eye_calibration/apple_pick_v2')
from fixed_scan_real_verify import FixedScanVerify,load_config
spec=importlib.util.spec_from_file_location('retreat','/home/li/桌面/9月机械臂实验（算法）/reports/live/single_orange_control/retreat_verify_to_scan.py');mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
def box(id,frame,xyz,size):
 o=CollisionObject();o.id=id;o.header.frame_id=frame;o.operation=o.ADD
 p=Pose();p.position.x,p.position.y,p.position.z=map(float,xyz);p.orientation.w=1.
 o.primitives=[SolidPrimitive(type=SolidPrimitive.BOX,dimensions=list(map(float,size)))];o.primitive_poses=[p];return o
class ReadOnly(FixedScanVerify):
 def state_is_valid(self,state,label):
  state=copy.deepcopy(state);state.is_diff=True
  return super().state_is_valid(state,label)
 def execute(self,*args):raise RuntimeError('Execution forbidden in this file')
 def remove_support_guard(self):raise RuntimeError('Protection removal forbidden')
 def apply_protection(self):
  s=PlanningScene();s.is_diff=True;s.robot_state.is_diff=True
  s.world.collision_objects=[box('alignment_table_guard',self.base,[.40,-.20,-.105],[1.,1.,.10])]
  cfg=self.cfg['scene'];cam=box('realsense_camera_guard','Link6',cfg['camera_translation_link6_m'],cfg['camera_box_xyz_m']);q=cam.primitive_poses[0].orientation;q.x,q.y,q.z,q.w=map(float,cfg['camera_quaternion_link6_xyzw'])
  tool=box('alignment_open_tool_guard','Link6',[0,0,.11],[.16,.16,.22])
  for obj in [cam,tool]:
   a=AttachedCollisionObject();a.link_name='Link6';a.object=obj;a.touch_links=['Link6','gripper_body_approx',self.tcp];s.robot_state.attached_collision_objects.append(a)
  # Observed objects above tabletop, voxel padding includes sparse edge uncertainty.
  c=json.loads((P/'capture.json').read_text());r=c['records'][0];d=np.load(P/'depth_00.npy');k=r['K'];yy,xx=np.mgrid[0:d.shape[0]:8,0:d.shape[1]:8];z=d[::8,::8]*.001
  q=np.c_[((xx-k[2])*z/k[0]).ravel(),((yy-k[5])*z/k[4]).ravel(),z.ravel()];T=np.array(r['camera_to_base']);q=q@T[:3,:3].T+T[:3,3];q=q[(z.ravel()>0)&(q[:,2]>-.055)&(q[:,2]<.15)&(q[:,0]>.15)&(q[:,0]<.7)&(q[:,1]>-.5)&(q[:,1]<.10)]
  vox=np.unique(np.floor(q/.02).astype(int),axis=0)
  for i,v in enumerate(vox):s.world.collision_objects.append(box(f'birdalign_observed_{i}',self.base,(v+.5)*.02,[.04,.04,.04]))
  f=self.scene_client.call_async(ApplyPlanningScene.Request(scene=s));rclpy.spin_until_future_complete(self,f,timeout_sec=10);res=f.result()
  if res is None or not res.success:raise RuntimeError('Scene rejected')
  return len(vox)
def main():
 cfg=load_config(Path('/home/li/hand_eye_calibration/apple_pick_v2/single_apple_full_grasp.yaml'));cfg['cartesian']['max_step_m']=.002;cfg['cartesian']['max_tcp_speed_m_s']=.008
 out={'motion_commands':False,'gripper_commands':False,'status':'STARTED','table_top_m':-.055,'tool_assumption_m':[.16,.16,.22]}
 rclpy.init();n=ReadOnly(cfg)
 try:
  for client in [n.scene_client,n.validity_client,n.ik_client,n.cartesian_client]:
   if not client.wait_for_service(timeout_sec=10):raise RuntimeError('Missing service')
  if not n.spin_until(lambda:len(n.joints)==6,10,'real joints'):raise RuntimeError('No joints')
  if not n.spin_until(lambda:n.tf_buffer.can_transform(n.base,n.tcp,rclpy.time.Time()),5,'TCP TF available'):raise RuntimeError('TCP TF missing')
  tf=n.tf_buffer.lookup_transform(n.base,n.tcp,rclpy.time.Time());p=tf.transform.translation;q=tf.transform.rotation
  pose=Pose();g=json.loads((P/'geometry.json').read_text());pose.position.x,pose.position.y=map(float,g['right']['surface_median_m'][:2]);pose.position.z=.22;pose.orientation=copy.deepcopy(q)
  start=np.array([p.x,p.y,p.z]);expected=np.array(json.loads((P/'geometry.json').read_text())['records'][0]['tcp_m']);out['start_m']=start.tolist();out['start_error_m']=float(np.linalg.norm(start-expected));out['target_m']=[pose.position.x,pose.position.y,pose.position.z]
  if out['start_error_m']>.003 or time.monotonic()-n.joint_rx_monotonic>.5:raise RuntimeError('Start moved/stale')
  out['observed_voxels']=n.apply_protection();
  from verify_current_scene import verify_scene
  out['scene_readback']=verify_scene(n,cfg,out['observed_voxels'])
  n.current_state_is_valid();n.solve_ik(pose,'high alignment',attempts=3)
  traj=n.cartesian(pose);traj,out['retime']=mod.retime_up(n,traj)
  fk=n.create_client(GetPositionFK,'/compute_fk');mins=[];camera_mins=[];tcpz=[]
  for pt in traj.joint_trajectory.points:
   req=GetPositionFK.Request();req.header.frame_id=n.base;req.fk_link_names=[n.tcp,'Link6'];req.robot_state.joint_state.name=traj.joint_trajectory.joint_names;req.robot_state.joint_state.position=pt.positions
   f=fk.call_async(req);rclpy.spin_until_future_complete(n,f,timeout_sec=3);res=f.result()
   if res is None or res.error_code.val!=1:raise RuntimeError('FK failed')
   tp,lp=[x.pose for x in res.pose_stamped];tcpz.append(tp.position.z)
   from scipy.spatial.transform import Rotation
   rot=Rotation.from_quat([lp.orientation.x,lp.orientation.y,lp.orientation.z,lp.orientation.w]).as_matrix()
   corners=np.array([[x,y,z] for x in [-.08,.08] for y in [-.08,.08] for z in [0,.22]])
   mins.append(float((corners@rot.T+np.array([lp.position.x,lp.position.y,lp.position.z]))[:,2].min()))
   sc=cfg['scene'];dim=np.array(sc['camera_box_xyz_m'])/2;cr=Rotation.from_quat(sc['camera_quaternion_link6_xyzw']).as_matrix();cc=np.array([[x,y,z] for x in [-dim[0],dim[0]] for y in [-dim[1],dim[1]] for z in [-dim[2],dim[2]]]);cc=cc@cr.T+np.array(sc['camera_translation_link6_m']);camera_mins.append(float((cc@rot.T+np.array([lp.position.x,lp.position.y,lp.position.z]))[:,2].min()))
   state=RobotState();state.joint_state=req.robot_state.joint_state;n.state_is_valid(state,'path sample')
  out['target_surface_z_m']=g['right']['surface_median_m'][2];out['tool_target_surface_vertical_clearance_m']=min(mins)-out['target_surface_z_m'];out['min_camera_box_z_m']=min(camera_mins);out['min_tcp_z_m']=min(tcpz);out['min_tool_box_z_m']=min(mins);out['tool_table_clearance_m']=min(mins)+.055;out['status']='PASS_PREFLIGHT_ONLY';(P/'trajectory.cdr').write_bytes(serialize_message(traj))
 except Exception as e:out['status']='FAILED';out['error']=str(e)
 finally:(P/'preflight.json').write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2));n.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
