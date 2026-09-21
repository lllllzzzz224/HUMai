import time,json
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
rclpy.init(); n=Node('orange_read_scene'); got={}; bridge=CvBridge()
def pic(m):
 if 'image' not in got:
  cv2.imwrite('/home/li/桌面/9月机械臂实验（算法）/reports/live/single_orange_control/measured_table/after.jpg',bridge.imgmsg_to_cv2(m,desired_encoding='bgr8'));got['image']=True
def diag(m):got['diagnostics']=m.data
subs=[n.create_subscription(Image,'/camera/camera/color/image_raw',pic,10),n.create_subscription(String,'/apple_pick_v2/apple_diagnostics',diag,10)]
end=time.monotonic()+15
while len(got)<2 and time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.2)
print(json.dumps(got,ensure_ascii=False));print('center_publishers',n.count_publishers('/apple_pick_v2/apple_center'))
n.destroy_node();rclpy.shutdown()
