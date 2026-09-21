import json,hashlib
import numpy as np,rclpy
from moveit_msgs.msg import RobotState,RobotTrajectory
from rclpy.serialization import deserialize_message
from preflight_alignment import ReadOnly,load_config,P
from verify_current_scene import verify_scene
rclpy.init();n=ReadOnly(load_config(P/'config.yaml'));out={'motion_commands':False,'status':'STARTED'}
try:
 assert n.validity_client.wait_for_service(timeout_sec=5);out['scene_readback']=verify_scene(n,n.cfg,81)
 out['trajectory_sha256']=hashlib.sha256((P/'trajectory.cdr').read_bytes()).hexdigest();traj=deserialize_message((P/'trajectory.cdr').read_bytes(),RobotTrajectory);samples=np.load(P/'spline_samples.npz')['joints_rad'];out['samples']=len(samples)
 for q in samples:
  state=RobotState();state.joint_state.name=traj.joint_trajectory.joint_names;state.joint_state.position=q.tolist();n.state_is_valid(state,'latest scene cubic sample')
 out['status']='PASS_NEW_SCENE_SPLINE_COLLISION'
finally:(P/'current_scene_spline_validation.json').write_text(json.dumps(out,indent=2));print(json.dumps(out));n.destroy_node();rclpy.shutdown()
