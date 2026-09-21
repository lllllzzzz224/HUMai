import rclpy,time,json,cv2,numpy as np
from pathlib import Path
from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
out=Path(__file__).resolve().parent
rclpy.init();n=rclpy.create_node('environment_readonly');b=CvBridge();counts={'color':0,'depth':0};frames={}
def cb(k,m):counts[k]+=1;frames[k]=m
subs=[n.create_subscription(Image,'/camera/camera/color/image_raw',lambda m:cb('color',m),qos_profile_sensor_data),n.create_subscription(Image,'/camera/camera/aligned_depth_to_color/image_raw',lambda m:cb('depth',m),qos_profile_sensor_data)]
s=time.monotonic()
while time.monotonic()-s<10:rclpy.spin_once(n,timeout_sec=.1)
r={'counts_10s':counts}
if 'color' in frames:cv2.imwrite(str(out/'current_color.jpg'),b.imgmsg_to_cv2(frames['color'],'bgr8'))
if 'depth' in frames:
 d=b.imgmsg_to_cv2(frames['depth'],'passthrough');np.save(out/'current_depth.npy',d);r['valid_depth_fraction']=float(np.count_nonzero(d)/d.size)
(out/'camera_status.json').write_text(json.dumps(r,indent=2));print(r);n.destroy_node();rclpy.shutdown()
