"""Bounded four-step planning only. No action goal, no gripper command."""
import sys,json,copy,time,shutil
from pathlib import Path
import numpy as np,rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.srv import GetCartesianPath,GetStateValidity,GetPositionIK
from moveit_msgs.msg import RobotState
from builtin_interfaces.msg import Duration
from rclpy.serialization import serialize_message
ROOT=Path(__file__).resolve().parent;BASE=ROOT.parent;sys.path.insert(0,str(BASE))
from preflight_alignment import ReadOnly,load_config,mod
from verify_current_scene import verify_scene
rclpy.init();n=ReadOnly(load_config(BASE/'config.yaml'));out={'requested_new_steps':4,'previous_completed_20mm_excluded':True,'step_size_m':.02,'steps':[],'motion_commands':False}
try:
 for cli in [n.cartesian_client,n.validity_client,n.ik_client]:assert cli.wait_for_service(timeout_sec=5)
 assert n.spin_until(lambda:len(n.joints)==6 and n.tf_buffer.can_transform(n.base,n.tcp,rclpy.time.Time()),5,'fresh pose')
 out['scene']=verify_scene(n,n.cfg,81);n.current_state_is_valid();tf=n.tf_buffer.lookup_transform(n.base,n.tcp,rclpy.time.Time());t=tf.transform.translation;start=np.array([t.x,t.y,t.z]);out['start_m']=start.tolist();q=n.current_joint_vector().tolist();names=n.JOINT_NAMES
 for step in range(1,5):
  dest=start-np.array([0,0,step*.02]);pose=Pose();pose.position.x,pose.position.y,pose.position.z=dest.tolist();pose.orientation=copy.deepcopy(tf.transform.rotation)
  req=GetCartesianPath.Request();req.header.frame_id=n.base;req.group_name=n.cfg['moveit']['group'];req.link_name=n.tcp;req.start_state.joint_state.name=names;req.start_state.joint_state.position=q;req.start_state.is_diff=True;req.waypoints=[pose];req.max_step=.002;req.jump_threshold=float(n.cfg['cartesian']['jump_threshold']);req.revolute_jump_threshold=float(n.cfg['cartesian']['revolute_jump_threshold_rad']);req.avoid_collisions=True;req.max_velocity_scaling_factor=.03;req.max_acceleration_scaling_factor=.03
  f=n.cartesian_client.call_async(req);rclpy.spin_until_future_complete(n,f,timeout_sec=30);res=f.result();row={'new_step_number':step,'target_m':dest.tolist(),'start_joints_rad':q,'code':None if res is None else res.error_code.val,'fraction':None if res is None else res.fraction};out['steps'].append(row)
  if res is None or res.error_code.val!=1 or res.fraction<.999999:
   row['status']='FAILED_CARTESIAN_NO_EXECUTABLE_PATH';out['safe_planning_prefix_before_spline']=step-1;break
  traj=res.solution;n.validate_trajectory(traj,f'new step {step}');traj,row['retime']=mod.retime_up(n,traj)
  for pt in traj.joint_trajectory.points:
   st=RobotState();st.joint_state.name=traj.joint_trajectory.joint_names;st.joint_state.position=pt.positions;n.state_is_valid(st,'step point')
  p=ROOT/f'step{step:02d}';p.mkdir(exist_ok=True)
  for file in ['config.yaml','verify_current_scene.py','current_obstacles.json','local_fk.py','verify_driver_spline.py','execute_alignment_once.py']:shutil.copy(BASE/file,p/file)
  # Local path binds copied validation/runner artifacts to this independent step.
  (p/'preflight_alignment.py').write_text("from pathlib import Path\nimport sys\nsys.path.insert(1,"+repr(str(BASE))+ ")\nfrom importlib.util import spec_from_file_location,module_from_spec\nspec=spec_from_file_location('readonly_parent',"+repr(str(BASE/'preflight_alignment.py'))+");m=module_from_spec(spec);spec.loader.exec_module(m)\nReadOnly=m.ReadOnly;load_config=m.load_config;P=Path(__file__).resolve().parent\n")
  row['status']='PASS_KNOTS_ONLY_PENDING_DRIVER_SPLINE';row['points']=len(traj.joint_trajectory.points);row['start_m']=(start-np.array([0,0,(step-1)*.02])).tolist();row['observed_voxels']=81
  (p/'trajectory.cdr').write_bytes(serialize_message(traj));(p/'preflight.json').write_text(json.dumps(row,indent=2));q=list(traj.joint_trajectory.points[-1].positions)
except Exception as e:out['error']=str(e)
finally:(ROOT/'four_step_feasibility.json').write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2));n.destroy_node();rclpy.shutdown()
