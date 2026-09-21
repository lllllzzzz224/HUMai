import json,time,copy,hashlib
from pathlib import Path
import numpy as np
import rclpy
from rclpy.serialization import deserialize_message,serialize_message
from moveit_msgs.msg import RobotTrajectory
from scipy.spatial.transform import Rotation
from preflight_alignment import ReadOnly,load_config,P
from local_fk import fk,HAND
ROOT=P.parent
sources=[ROOT/'vertical_diagnostic20_20260916/four_step_plan/step03',ROOT/'vertical_diagnostic20_20260916/four_step_plan/step02/retry_discovery',ROOT/'vertical_diagnostic20_20260916/four_step_plan/step01',ROOT/'vertical_diagnostic20_20260916',ROOT/'birdview_alignment_20260916']
out={'status':'STARTED','purpose':'RETURN_TO_RECORDED_OBSERVATION_ONLY','motion_commands':False,'gripper_commands':False,'observed_voxels':81,'sources':[]};cfg=load_config(P/'config.yaml');rclpy.init();n=ReadOnly(cfg)
try:
 for c in [n.scene_client,n.validity_client]:
  if not c.wait_for_service(timeout_sec=10):raise RuntimeError('Service absent')
 if not n.spin_until(lambda:len(n.joints)==6,10,'real state'):raise RuntimeError('No actual joints')
 if not n.spin_until(lambda:n.tf_buffer.can_transform(n.base,n.tcp,rclpy.time.Time()),5,'TF'):raise RuntimeError('No TCP TF')
 if time.monotonic()-n.joint_rx_monotonic>.5:raise RuntimeError('Stale joints')
 out['start_m']=n.tcp_position().tolist();out['start_joints_rad']=n.current_joint_vector().tolist()
 from verify_current_scene import verify_scene
 out['scene_readback']=verify_scene(n,cfg,81);n.current_state_is_valid()
 traj=RobotTrajectory();elapsed=0.;joins=[]
 for source in sources:
  data=(source/'trajectory.cdr').read_bytes();approved=json.loads((source/'approved_plan.json').read_text());sha=hashlib.sha256(data).hexdigest()
  if sha!=approved['trajectory_sha256']:raise RuntimeError('Source changed')
  old=deserialize_message(data,RobotTrajectory).joint_trajectory
  if traj.joint_trajectory.joint_names and old.joint_names!=traj.joint_trajectory.joint_names:raise RuntimeError('Joint names changed')
  traj.joint_trajectory.joint_names=old.joint_names;total=old.points[-1].time_from_start.sec+old.points[-1].time_from_start.nanosec*1e-9
  if traj.joint_trajectory.points:
   joins.append(float(np.max(np.abs(np.array(traj.joint_trajectory.points[-1].positions)-old.points[-1].positions))));elapsed+=.5
  for pt in reversed(old.points):
   pp=copy.deepcopy(pt);t=elapsed+total-(pt.time_from_start.sec+pt.time_from_start.nanosec*1e-9);ns=round(t*1e9);pp.time_from_start.sec=ns//10**9;pp.time_from_start.nanosec=ns%10**9;pp.velocities=[-v for v in pp.velocities];traj.joint_trajectory.points.append(pp)
  elapsed+=total;out['sources'].append({'directory':str(source),'sha256':sha})
 q=np.array([p.positions for p in traj.joint_trajectory.points]);out['join_joint_delta_rad']=joins;out['max_joint_step_rad']=float(np.abs(np.diff(q,axis=0)).max());out['start_joint_error_rad']=float(np.abs(q[0]-n.current_joint_vector(traj.joint_trajectory.joint_names)).max())
 if out['max_joint_step_rad']>cfg['cartesian']['revolute_jump_threshold_rad'] or out['start_joint_error_rad']>.01:raise RuntimeError('Jump/start mismatch')
 n.validate_trajectory(traj,'reverse completed descents and alignment')
 T=fk(np.rad2deg(q[-1])*1000);out['target_m']=(T@np.array([0,0,.138,1]))[:3].tolist();out['target_joints_rad']=q[-1].tolist();out['tool_z_down_deg']=float(np.rad2deg(np.arccos(np.clip(-T[2,2],-1,1))))
 optical=np.array(json.loads((ROOT/'alignment_20260916/capture.json').read_text())['records'][0]['camera_link_to_optical']);C=T@HAND@optical;out['camera_optical_z_down_deg_conditional']=float(np.rad2deg(np.arccos(np.clip(-C[2,2],-1,1))))
 (P/'trajectory.cdr').write_bytes(serialize_message(traj));out['status']='PASS_KNOT_PREFLIGHT_REQUIRES_SPLINE';out['point_count']=len(q)
except Exception as e:out['status']='FAILED';out['error']=repr(e)
finally:(P/'preflight.json').write_text(json.dumps(out,indent=2));print(json.dumps(out));n.destroy_node();rclpy.shutdown()
