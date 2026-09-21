import json,copy
import numpy as np
import rclpy
from rclpy.serialization import deserialize_message
from moveit_msgs.msg import RobotTrajectory,RobotState
from moveit_msgs.srv import GetPositionIK
from geometry_msgs.msg import Pose
from builtin_interfaces.msg import Duration
from preflight_alignment import ReadOnly,load_config,P
rclpy.init();n=ReadOnly(load_config(P/'config.yaml'));out=[]
try:
 assert n.ik_client.wait_for_service(timeout_sec=5);assert n.spin_until(lambda:len(n.joints)==6 and n.tf_buffer.can_transform(n.base,n.tcp,rclpy.time.Time()),5,'real state')
 n.apply_protection();tf=n.tf_buffer.lookup_transform(n.base,n.tcp,rclpy.time.Time());g=json.loads((P/'geometry.json').read_text());pose=Pose();pose.position.x,pose.position.y=g['right']['surface_median_m'][:2];pose.position.z=.22;pose.orientation=copy.deepcopy(tf.transform.rotation)
 source='/home/li/桌面/9月机械臂实验（算法）/reports/near_alignment_20260916/re_align_high/trajectory.cdr';traj=deserialize_message(open(source,'rb').read(),RobotTrajectory).joint_trajectory
 req=GetPositionIK.Request();ik=req.ik_request;ik.group_name=n.cfg['moveit']['group'];ik.robot_state.joint_state.name=traj.joint_names;ik.robot_state.joint_state.position=traj.points[-1].positions;ik.robot_state.is_diff=True;ik.ik_link_name=n.tcp;ik.pose_stamped.header.frame_id=n.base;ik.pose_stamped.pose=pose;ik.avoid_collisions=True;ik.timeout=Duration(sec=1)
 f=n.ik_client.call_async(req);rclpy.spin_until_future_complete(n,f,timeout_sec=3);res=f.result();row={'seed':'previous actually achieved right high endpoint','target_z_m':.22,'error_code':None if res is None else res.error_code.val}
 if res and res.error_code.val==1:
  n.state_is_valid(res.solution,'known-seed candidate');row['solution_names']=res.solution.joint_state.name;row['solution_rad']=res.solution.joint_state.position.tolist() if hasattr(res.solution.joint_state.position,'tolist') else list(res.solution.joint_state.position)
 out.append(row)
finally:(P/'known_seed_diagnosis.json').write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2));n.destroy_node();rclpy.shutdown()
