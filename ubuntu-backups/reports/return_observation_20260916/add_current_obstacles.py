"""Append current observed geometry, preserving all previous guards; no motion."""
import json,sys
from pathlib import Path
import numpy as np,rclpy
from moveit_msgs.msg import PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from preflight_alignment import ReadOnly,load_config,P
import importlib.util
spec=importlib.util.spec_from_file_location("box_source",str(P.parent/"birdview_alignment_20260916/preflight_alignment.py"));prior=importlib.util.module_from_spec(spec);spec.loader.exec_module(prior)
source=Path('/home/li/桌面/9月机械臂实验（算法）/reports/return_birdview_review_20260916/before');c=json.loads((source/'capture.json').read_text());r=c['records'][0];d=np.load(source/'depth_00.npy');k=r['K'];yy,xx=np.mgrid[0:d.shape[0]:8,0:d.shape[1]:8];z=d[::8,::8]*.001;q=np.c_[((xx-k[2])*z/k[0]).ravel(),((yy-k[5])*z/k[4]).ravel(),z.ravel()];T=np.array(r['camera_to_base']);q=q@T[:3,:3].T+T[:3,3];q=q[(z.ravel()>0)&(q[:,2]>-.055)&(q[:,2]<.15)&(q[:,0]>.15)&(q[:,0]<.7)&(q[:,1]>-.5)&(q[:,1]<.10)];vox=np.unique(np.floor(q/.02).astype(int),axis=0)
rclpy.init();n=ReadOnly(load_config(P/'config.yaml'))
try:
 assert n.scene_client.wait_for_service(timeout_sec=5);scene=PlanningScene();scene.is_diff=True;scene.robot_state.is_diff=True
 for i,v in enumerate(vox):scene.world.collision_objects.append(prior.box(f'return_observed_{i}',n.base,(v+.5)*.02,[.04,.04,.04]))
 f=n.scene_client.call_async(ApplyPlanningScene.Request(scene=scene));rclpy.spin_until_future_complete(n,f,timeout_sec=5);assert f.result() is not None and f.result().success
 (P/'return_obstacles.json').write_text(json.dumps({'source':str(source),'prefix':'return_observed_','count':len(vox),'voxel_centres_m':((vox+.5)*.02).tolist(),'all_previous_obstacles_retained':True},indent=2));print('added',len(vox))
finally:n.destroy_node();rclpy.shutdown()
