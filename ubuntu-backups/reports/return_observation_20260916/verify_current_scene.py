import json
from pathlib import Path
import numpy as np
import rclpy
from scipy.spatial.transform import Rotation
from moveit_msgs.srv import GetPlanningScene
from moveit_msgs.msg import PlanningSceneComponents
def verify_scene(n,cfg,count):
  client=n.create_client(GetPlanningScene,'/get_planning_scene')
  if not client.wait_for_service(timeout_sec=5):raise RuntimeError('Scene unavailable')
  req=GetPlanningScene.Request();req.components.components=PlanningSceneComponents.WORLD_OBJECT_GEOMETRY|PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
  f=client.call_async(req);rclpy.spin_until_future_complete(n,f,timeout_sec=5);res=f.result()
  if res is None:raise RuntimeError('Scene readback failed')
  world={x.id:x for x in res.scene.world.collision_objects};attached={x.object.id:x.object for x in res.scene.robot_state.attached_collision_objects};record={}
  if len([x for x in world if x.startswith('alignment_observed_')])!=115 or len([x for x in world if x.startswith('realign_observed_')])!=96 or len([x for x in world if x.startswith('birdalign_observed_')])!=count:raise RuntimeError('Observation count changed')
  if set(['alignment_open_tool_guard','realsense_camera_guard'])-attached.keys():raise RuntimeError('Attachment missing')
  if not np.allclose(attached['alignment_open_tool_guard'].primitives[0].dimensions,[.16,.16,.22]):raise RuntimeError('Tool dimensions changed')
  def composed_box(obj):
   def mat(p):
    M=np.eye(4);q=p.orientation;M[:3,:3]=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix();M[:3,3]=[p.position.x,p.position.y,p.position.z];return M
   M=mat(obj.pose)@mat(obj.primitive_poses[0]);dim=np.array(obj.primitives[0].dimensions)/2
   corners=np.array([[x,y,z,1.] for x in [-dim[0],dim[0]] for y in [-dim[1],dim[1]] for z in [-dim[2],dim[2]]]);return M,(corners@M.T)[:,:3]
  table=world['alignment_table_guard'];tm,tc=composed_box(table);top=float(tc[:,2].max())
  if table.header.frame_id!=n.base or not np.allclose(tm[:3,3],[.4,-.2,-.105],atol=1e-6) or not np.allclose(tm[:3,:3],np.eye(3),atol=1e-6) or not np.allclose(table.primitives[0].dimensions,[1.,1.,.1]) or abs(top+.055)>1e-6:raise RuntimeError('table mismatch')
  tool=attached['alignment_open_tool_guard'];mt,_=composed_box(tool)
  if tool.header.frame_id!='Link6' or not np.allclose(mt[:3,3],[0,0,.11],atol=1e-6) or not np.allclose(mt[:3,:3],np.eye(3),atol=1e-6):raise RuntimeError('tool pose mismatch')
  cam=attached['realsense_camera_guard'];mc,_=composed_box(cam);sc=cfg['scene']
  if cam.header.frame_id!='Link6' or not np.allclose(cam.primitives[0].dimensions,sc['camera_box_xyz_m']) or not np.allclose(mc[:3,3],sc['camera_translation_link6_m'],atol=1e-6) or not np.allclose(mc[:3,:3],Rotation.from_quat(sc['camera_quaternion_link6_xyzw']).as_matrix(),atol=1e-6):raise RuntimeError('camera pose mismatch')
  record['scene_readback_table_top_m']=top
  latest=json.loads((Path(__file__).parent/'current_obstacles.json').read_text());actual=[x for x in world if x.startswith('diag20_observed_')]
  if len(actual)!=latest['count']:raise RuntimeError('Current diag20 obstacles missing')
  for i,xyz in enumerate(latest['voxel_centres_m']):
   obj=world[f'diag20_observed_{i}'];matrix,_=composed_box(obj)
   if obj.header.frame_id!=n.base or not np.allclose(obj.primitives[0].dimensions,[.04,.04,.04]) or not np.allclose(matrix[:3,3],xyz,atol=1e-6):raise RuntimeError('Current diag20 geometry changed')
  added=json.loads((Path(__file__).parent/'return_obstacles.json').read_text());now=[x for x in world if x.startswith('return_observed_')]
  if len(now)!=added['count']:raise RuntimeError('Return observation count changed')
  for i,xyz in enumerate(added['voxel_centres_m']):
   obj=world[f'return_observed_{i}'];matrix,_=composed_box(obj)
   if obj.header.frame_id!=n.base or not np.allclose(obj.primitives[0].dimensions,[.04,.04,.04]) or not np.allclose(matrix[:3,3],xyz,atol=1e-6):raise RuntimeError('Return observation geometry changed')
  record['return_observation_count']=len(now)
  record['current_diag20_obstacles']=len(actual)
  record['previous_obstacles_preserved']={'alignment_observed':115,'realign_observed':96}
  record['new_obstacles']=count
  return record
