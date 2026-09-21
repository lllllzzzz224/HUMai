import time,json
from pathlib import Path
import rclpy,cv2
from rclpy.node import Node
from sensor_msgs.msg import Image,CameraInfo
from std_msgs.msg import String
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data
import tf2_ros
out=Path('/home/li/桌面/9月机械臂实验（算法）/reports/live/single_orange_control/measured_table')
rclpy.init();n=Node('single_orange_measured_table_record');bridge=CvBridge();buf=tf2_ros.Buffer();listener=tf2_ros.TransformListener(buf,n)
start=time.monotonic();last=0.;writer=None;rows=[];latest_diag=None;info=None

def diag(m):
 global latest_diag
 try:latest_diag=json.loads(m.data)
 except ValueError:pass

def caminfo(m):
 global info
 info={'k':list(m.k),'frame':m.header.frame_id,'width':m.width,'height':m.height}

def cb(m):
 global last,writer
 now=time.monotonic()
 if now-last<.19:return
 last=now;im=bridge.imgmsg_to_cv2(m,desired_encoding='bgr8')
 if writer is None:
  writer=cv2.VideoWriter(str(out/'alignment.avi'),cv2.VideoWriter_fourcc(*'MJPG'),5.,(im.shape[1],im.shape[0]))
  if not writer.isOpened():raise RuntimeError('video writer failed')
 row={'elapsed_s':now-start,'frame':m.header.frame_id,'stamp_sec':m.header.stamp.sec,'stamp_nanosec':m.header.stamp.nanosec,'diagnostics':latest_diag}
 for parent,child,key in [('base_link','tcp_link','tcp_base'),(m.header.frame_id,'base_link','base_to_camera')]:
  try:
   t=buf.lookup_transform(parent,child,rclpy.time.Time());p=t.transform.translation;q=t.transform.rotation
   row[key]={'xyz':[p.x,p.y,p.z],'xyzw':[q.x,q.y,q.z,q.w]}
  except Exception:pass
 rows.append(row);writer.write(im)
 if len(rows)%5==1:cv2.imwrite(str(out/('frame_%04d.jpg'%len(rows))),im)
 cv2.imwrite(str(out/'latest.jpg'),im)
subs=[n.create_subscription(Image,'/camera/camera/color/image_raw',cb,qos_profile_sensor_data),n.create_subscription(CameraInfo,'/camera/camera/color/camera_info',caminfo,qos_profile_sensor_data),n.create_subscription(String,'/apple_pick_v2/apple_diagnostics',diag,10)]
print('RECORDER_READY',flush=True)
try:
 while time.monotonic()-start<150 and not (out/'stop').exists():rclpy.spin_once(n,timeout_sec=.1)
finally:
 if writer is not None:writer.release()
 (out/'recording.json').write_text(json.dumps({'camera_info':info,'frames':rows},ensure_ascii=False,indent=2))
 print('RECORDER_SAVED',len(rows),'frames',flush=True);n.destroy_node();rclpy.shutdown()
