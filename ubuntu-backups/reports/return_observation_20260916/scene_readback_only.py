import json,rclpy
from preflight_alignment import ReadOnly,load_config,P
from verify_current_scene import verify_scene
rclpy.init();n=ReadOnly(load_config(P/'config.yaml'))
try:
 result=verify_scene(n,n.cfg,81);(P/'scene_readback.json').write_text(json.dumps(result,indent=2));print(json.dumps(result))
finally:n.destroy_node();rclpy.shutdown()
