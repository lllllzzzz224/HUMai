import json,time
import rclpy
from std_msgs.msg import Empty
from preflight_alignment import ReadOnly,load_config,P
rclpy.init();n=ReadOnly(load_config(P/'config.yaml'));pub=n.create_publisher(Empty,'/rm_driver/move_stop_cmd',10)
start=time.monotonic();rows=[]
while time.monotonic()-start<5:
 endpoints=n.get_subscriptions_info_by_topic('/rm_driver/move_stop_cmd')
 rows.append({'elapsed_s':time.monotonic()-start,'action_ready':n.execute_client.server_is_ready(),'stop_count':pub.get_subscription_count(),'stop_endpoints':[{'node':x.node_name,'namespace':x.node_namespace} for x in endpoints]})
 rclpy.spin_once(n,timeout_sec=.05)
ready=[x for x in rows if x['action_ready'] and x['stop_count']>=2 and {'rm_driver','rm_control'}<={e['node'] for e in x['stop_endpoints']}]
out={'read_only':True,'no_goal_or_stop_publish':True,'node':n.get_name(),'samples':rows,'first_ready_s':ready[0]['elapsed_s'] if ready else None}
(P/'discovery_readonly.json').write_text(json.dumps(out,indent=2));print(json.dumps({'first':rows[0],'first_ready_s':out['first_ready_s'],'last':rows[-1]}));n.destroy_node();rclpy.shutdown()
