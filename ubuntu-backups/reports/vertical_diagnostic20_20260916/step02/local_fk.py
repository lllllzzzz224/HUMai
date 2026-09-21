import json,time,socket,hashlib,xml.etree.ElementTree as ET
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
HAND=mat([-.064514447085,.005428183272,.003696935131],Rotation.from_quat([-.020548815920,.728253776074,-.001728512288,-.684997223383]).as_matrix())
def arm():
 start=time.time()
 with socket.create_connection(('192.168.1.18',8080),timeout=2) as s:
  s.sendall(b'{"command":"get_current_arm_state"}\r\n');buf=b''
  while b'\n' not in buf:
   chunk=s.recv(4096)
   if not chunk:raise RuntimeError('Controller closed read-only connection')
   buf+=chunk
 data=json.loads(buf.splitlines()[0]);end=time.time()
 if data.get('state')!='current_arm_state' or len(data['arm_state']['joint'])!=6:raise RuntimeError('Invalid arm state')
 return dict(start_wall=start,end_wall=end,data=data)
