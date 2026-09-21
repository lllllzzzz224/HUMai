import ast,json,time,socket,hashlib,xml.etree.ElementTree as ET
from pathlib import Path
from collections import deque
import numpy as np,cv2,rclpy,tf2_ros
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import Image,CameraInfo
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
OUT=Path(__file__).parent; URDF=Path('/home/li/ros2_ws/install/rm_description/share/rm_description/urdf/rm_65.urdf')
def mat(xyz,r):
 t=np.eye(4);t[:3,:3]=r;t[:3,3]=xyz;return t
joints={j.find('child').attrib['link']:j for j in ET.parse(URDF).getroot().findall('joint')}
def fk(degs):
 chain=[];link='Link6'
 while link!='base_link':
  j=joints[link];chain.append(j);link=j.find('parent').attrib['link']
 t=np.eye(4); vals={f'joint{i+1}':np.deg2rad(v/1000) for i,v in enumerate(degs)}
 for j in reversed(chain):
  o=j.find('origin');xyz=np.fromstring(o.get('xyz','0 0 0'),sep=' ');rpy=np.fromstring(o.get('rpy','0 0 0'),sep=' ')
  t=t@mat(xyz,Rotation.from_euler('xyz',rpy).as_matrix())
  if j.get('type') in ('revolute','continuous'):
   axis=np.fromstring(j.find('axis').get('xyz'),sep=' ');t=t@mat([0,0,0],Rotation.from_rotvec(axis*vals[j.get('name')]).as_matrix())
 return t
HAND_SOURCE=Path('/home/li/hand_eye_calibration/hand_eye_static_tf.launch.py')
args=next(ast.literal_eval(k.value) for node in ast.walk(ast.parse(HAND_SOURCE.read_text())) if isinstance(node,ast.Call) for k in node.keywords if k.arg=='arguments')
assert args[-2:]==['Link6','camera_link']
HAND=mat([float(v) for v in args[:3]],Rotation.from_quat([float(v) for v in args[3:7]]).as_matrix())
def arm():
 start=time.time()
 with socket.create_connection(('192.168.1.18',8080),timeout=2) as s:
  s.sendall(b'{"command":"get_current_arm_state"}\r\n');buf=b''
  while b'\n' not in buf:
   chunk=s.recv(4096)
   if not chunk:raise RuntimeError('Controller closed read-only connection')
   buf+=chunk
   if len(buf)>65536:raise RuntimeError('Oversized read-only controller response')
 data=json.loads(buf.splitlines()[0]);end=time.time()
 if data.get('state')!='current_arm_state' or len(data['arm_state']['joint'])!=6:raise RuntimeError('Invalid arm state')
 if not np.all(np.isfinite(data['arm_state']['joint'])) or any(v!=0 for v in data['arm_state'].get('err',[1])):raise RuntimeError('Invalid joint values or controller error')
 return dict(start_wall=start,end_wall=end,data=data)
rclpy.init();n=rclpy.create_node('branch_registered_readonly_capture');b=CvBridge();tf=tf2_ros.Buffer();listener=tf2_ros.TransformListener(tf,n)
streams={'rgb':deque(maxlen=30),'depth':deque(maxlen=30)};counts={'rgb':0,'depth':0};info={};stats=[]
def cb(k,m):
 now=time.time();stamp=m.header.stamp.sec+m.header.stamp.nanosec/1e9;counts[k]+=1;streams[k].append((stamp,now,m))
 if k=='depth':
  a=b.imgmsg_to_cv2(m,'passthrough');stats.append(dict(stamp=stamp,receipt_wall=now,valid_fraction=float(np.mean(a>0)),shape=list(a.shape),encoding=m.encoding))
subs=[n.create_subscription(Image,'/camera/camera/color/image_raw',lambda m:cb('rgb',m),qos_profile_sensor_data),n.create_subscription(Image,'/camera/camera/aligned_depth_to_color/image_raw',lambda m:cb('depth',m),qos_profile_sensor_data),n.create_subscription(CameraInfo,'/camera/camera/color/camera_info',lambda m:info.update(color=m),qos_profile_sensor_data)]
records=[];errors=[];start=time.time();last=0
try:
 while time.time()-start<12:
  rclpy.spin_once(n,timeout_sec=.05)
  if time.time()-last<2 or 'color' not in info:continue
  last=time.time();pre=arm();deadline=time.time()+1;pair=None
  while time.time()<deadline:
   rclpy.spin_once(n,timeout_sec=.02)
   for rs,rr,rm in reversed(streams['rgb']):
    for ds,dr,dm in reversed(streams['depth']):
     if min(rs,ds)>pre['end_wall'] and abs(rs-ds)<.035 and max(time.time()-rs,time.time()-ds)<.4:pair=(rs,rr,rm,ds,dr,dm);break
    if pair:break
   if pair:break
  if pair is None:errors.append('No fresh bracketed RGBD pair');continue
  post=arm();rs,rr,rm,ds,dr,dm=pair
  a=np.array(pre['data']['arm_state']['joint']);z=np.array(post['data']['arm_state']['joint']);delta=float(np.max(np.abs(a-z)))/1000
  if delta>.02 or post['end_wall']-pre['start_wall']>.6 or max(rs,ds)>post['end_wall']:errors.append('Unstable/slow bracket');continue
  try:tr=tf.lookup_transform('camera_link',rm.header.frame_id,rclpy.time.Time())
  except Exception as e:errors.append(str(e));continue
  q=tr.transform.rotation;v=tr.transform.translation;internal=mat([v.x,v.y,v.z],Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix())
  T=fk((a+z)/2)@HAND@internal;depth=b.imgmsg_to_cv2(dm,'passthrough');rgb=b.imgmsg_to_cv2(rm,'bgr8');i=len(records)
  np.save(OUT/f'depth_{i:02d}.npy',depth);cv2.imwrite(str(OUT/f'rgb_{i:02d}.jpg'),rgb)
  ci=info['color'];records.append(dict(index=i,rgb_stamp=rs,depth_stamp=ds,rgb_receipt=rr,depth_receipt=dr,sync_error_s=abs(rs-ds),pre_arm=pre,post_arm=post,joint_span_deg=delta,bracket_s=post['end_wall']-pre['start_wall'],K=list(ci.k),D=list(ci.d),distortion_model=ci.distortion_model,color_frame=rm.header.frame_id,depth_frame=dm.header.frame_id,depth_encoding=dm.encoding,camera_to_base=T.tolist(),base_to_Link6=fk((a+z)/2).tolist(),camera_link_to_optical=internal.tolist()))
finally:
 (OUT/'capture.json').write_text(json.dumps(dict(duration_s=time.time()-start,counts=counts,records=records,depth_stats=stats,errors=errors,urdf=str(URDF),urdf_sha256=hashlib.sha256(URDF.read_bytes()).hexdigest(),hand_eye=HAND.tolist(),hand_eye_source=str(HAND_SOURCE),hand_eye_source_sha256=hashlib.sha256(HAND_SOURCE.read_bytes()).hexdigest(),calibration_physically_reverified_today=False,time_policy='ROS frame epoch checked against host wall and bracketed by real controller reads; motion threshold .02deg, bracket <=.6s; no fake joint publisher'),indent=2))
 print(json.dumps(dict(output_directory=str(OUT),counts=counts,accepted=len(records),errors=errors[-5:])));n.destroy_node();rclpy.shutdown()
