#!/usr/bin/env python3
import json,textwrap
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from moveit_msgs.srv import GetPlanningScene
from moveit_msgs.msg import PlanningSceneComponents
from preflight_alignment import P,load_config
rclpy.init();node=rclpy.create_node('alignment_geometry_gate_readonly');client=node.create_client(GetPlanningScene,'/get_planning_scene');assert client.wait_for_service(timeout_sec=5)
req=GetPlanningScene.Request();req.components.components=PlanningSceneComponents.WORLD_OBJECT_GEOMETRY|PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
f=client.call_async(req);rclpy.spin_until_future_complete(node,f,timeout_sec=5);assert f.result() is not None
scene=f.result().scene;world={x.id:x for x in scene.world.collision_objects};attached={x.object.id:x.object for x in scene.robot_state.attached_collision_objects}
n=SimpleNamespace(base='base_link');cfg=load_config(P/'config.yaml');record={}
s=(P/'execute_alignment_once.py').read_text();s=s[s.index('  def composed_box'):s.index('  n.current_state_is_valid()')];exec(textwrap.dedent(s))
record.update(status='PASS_READONLY_SCENE_GATE',observed_voxels=len([x for x in world if x.startswith('alignment_observed_')]),motion_commands=False)
(P/'scene_gate_validation.json').write_text(json.dumps(record,indent=2));print(json.dumps(record));node.destroy_node();rclpy.shutdown()
