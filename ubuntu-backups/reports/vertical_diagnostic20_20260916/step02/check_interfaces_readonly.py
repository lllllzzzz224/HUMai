import json,time
import rclpy
from action_msgs.msg import GoalStatusArray
from std_msgs.msg import Empty
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
from preflight_alignment import ReadOnly,load_config,P
n=None;rclpy.init();n=ReadOnly(load_config(P/'config.yaml'));pub=n.create_publisher(Empty,'/rm_driver/move_stop_cmd',10);statuses={};q=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
subs=[n.create_subscription(GoalStatusArray,topic,lambda m,t=topic:statuses.update({t:[int(x.status) for x in m.status_list]}),q) for topic in ['/execute_trajectory/_action/status','/rm_group_controller/follow_joint_trajectory/_action/status']]
end=time.monotonic()+3
while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.05)
out={'no_goals_or_stop_commands_sent':True,'execute_action_available':n.execute_client.wait_for_server(timeout_sec=2),'hard_stop_subscribers':pub.get_subscription_count(),'observed_action_status_arrays':statuses,'active_status_seen':any(s in [1,2,3] for arr in statuses.values() for s in arr),'joint_feedback_age_s':time.monotonic()-n.joint_rx_monotonic,'joint_values_rad':n.current_joint_vector().tolist(),'note':'freshly restarted control and move_group; absent status publication is reported as absent, not fabricated empty state'}
(P/'execution_interface_readiness.json').write_text(json.dumps(out,indent=2));print(json.dumps(out));n.destroy_node();rclpy.shutdown()
