"""Restore only recorded collision geometry; no hardware or gripper commands."""
import importlib.util,json
from pathlib import Path
import rclpy
P=Path(__file__).resolve().parent;cfgpath=P/'config.yaml';counts=[]
rclpy.init()
try:
 for label,path,expected in [('alignment','reports/alignment_20260916/preflight_alignment.py',115),('realign','reports/near_alignment_20260916/re_align_high/preflight_alignment.py',96),('birdalign','reports/birdview_alignment_20260916/preflight_alignment.py',81)]:
  spec=importlib.util.spec_from_file_location('restore_'+label,path);module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module);n=module.ReadOnly(module.load_config(cfgpath))
  try:
   if not n.scene_client.wait_for_service(timeout_sec=10):raise RuntimeError('Scene service absent')
   count=n.apply_protection()
   if count!=expected:raise RuntimeError('Saved geometry count changed')
   counts.append({'source':str(Path(path).resolve()),'expected':expected,'restored':count})
  finally:n.destroy_node()
 # Restore the newly frozen current observation geometry before final validation.
 current=P/'current_obstacles.json'
 if current.exists():
  from moveit_msgs.msg import PlanningScene
  from moveit_msgs.srv import ApplyPlanningScene
  latest=json.loads(current.read_text());n=module.ReadOnly(module.load_config(cfgpath))
  try:
   assert n.scene_client.wait_for_service(timeout_sec=5)
   scene=PlanningScene();scene.is_diff=True;scene.robot_state.is_diff=True
   for i,xyz in enumerate(latest['voxel_centres_m']):scene.world.collision_objects.append(module.box(f'diag20_observed_{i}',n.base,xyz,[.04,.04,.04]))
   f=n.scene_client.call_async(ApplyPlanningScene.Request(scene=scene));rclpy.spin_until_future_complete(n,f,timeout_sec=5);assert f.result() is not None and f.result().success
   counts.append({'source':str(current),'expected':latest['count'],'restored':latest['count']})
  finally:n.destroy_node()
 from preflight_alignment import ReadOnly,load_config
 from verify_current_scene import verify_scene
 n=ReadOnly(load_config(cfgpath))
 try:validation=verify_scene(n,n.cfg,81)
 finally:n.destroy_node()
 (P/'restored_scene.json').write_text(json.dumps({'hardware_motion':False,'geometry_sources':counts,'scene_readback':validation},indent=2));print(counts)
finally:rclpy.shutdown()
