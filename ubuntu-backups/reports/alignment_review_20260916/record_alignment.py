from pathlib import Path
import time,json,cv2,rclpy,tf2_ros
from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
P=Path(__file__).resolve().parent
rclpy.init();n=rclpy.create_node('alignment_review_record');b=CvBridge();buf=tf2_ros.Buffer();listener=tf2_ros.TransformListener(buf,n);rows=[];writer=None;last=0;s=time.monotonic()
def cb(m):
 global writer,last
 now=time.monotonic()
 if now-last<.19:return
 last=now;a=b.imgmsg_to_cv2(m,'bgr8')
 if writer is None:
  writer=cv2.VideoWriter(str(P/'alignment.avi'),cv2.VideoWriter_fourcc(*'MJPG'),5.,(a.shape[1],a.shape[0]))
  if not writer.isOpened():raise RuntimeError('Cannot create recording')
 row={'elapsed_s':now-s,'stamp_s':m.header.stamp.sec+m.header.stamp.nanosec*1e-9}
 try:
  t=buf.lookup_transform('base_link','tcp_link',rclpy.time.Time());v=t.transform.translation;row['tcp_m']=[v.x,v.y,v.z]
 except Exception:pass
 writer.write(a);cv2.imwrite(str(P/'latest.jpg'),a)
 if len(rows)%5==0:cv2.imwrite(str(P/f'frame_{len(rows):04d}.jpg'),a)
 rows.append(row)
sub=n.create_subscription(Image,'/camera/camera/color/image_raw',cb,qos_profile_sensor_data)
print('RECORDER_READY',flush=True)
try:
 while time.monotonic()-s<180 and not (P/'stop_recording').exists():rclpy.spin_once(n,timeout_sec=.1)
finally:
 if writer is not None:writer.release()
 (P/'recording.json').write_text(json.dumps(rows,indent=2));print('SAVED',len(rows),flush=True);n.destroy_node();rclpy.shutdown()
