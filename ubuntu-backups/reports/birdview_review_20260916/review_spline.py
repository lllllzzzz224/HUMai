import json,hashlib,xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
from scipy.interpolate import CubicSpline
from scipy.spatial.transform import Rotation
from rclpy.serialization import deserialize_message
from moveit_msgs.msg import RobotTrajectory
p=Path('/home/li/桌面/9月机械臂实验（算法）/reports/birdview_return_20260916');data=(p/'trajectory.cdr').read_bytes();jt=deserialize_message(data,RobotTrajectory).joint_trajectory
t=np.array([s.time_from_start.sec+s.time_from_start.nanosec*1e-9 for s in jt.points]);q=np.array([s.positions for s in jt.points]);cs=CubicSpline(t,q,bc_type=((1,np.zeros(6)),(1,np.zeros(6))));tt=np.unique(np.r_[np.arange(0,t[-1],.02),t]);qq=cs(tt)
u=ET.parse('/home/li/ros2_ws/install/rm_description/share/rm_description/urdf/rm_65.urdf').getroot();joints={j.find('child').get('link'):j for j in u.findall('joint')};chain=[];link='Link6'
while link!='base_link':
 j=joints[link];chain.append(j);link=j.find('parent').get('link')
def fk(qv):
 T=np.eye(4); vals=dict(zip(jt.joint_names,qv))
 for j in reversed(chain):
  o=j.find('origin');M=np.eye(4);M[:3,:3]=Rotation.from_euler('xyz',np.fromstring(o.get('rpy','0 0 0'),sep=' ')).as_matrix();M[:3,3]=np.fromstring(o.get('xyz','0 0 0'),sep=' ');T=T@M
  if j.get('type') in ('revolute','continuous'):
   M=np.eye(4);M[:3,:3]=Rotation.from_rotvec(np.fromstring(j.find('axis').get('xyz'),sep=' ')*vals[j.get('name')]).as_matrix();T=T@M
 return T
corners=np.array([[x,y,z] for x in [-.08,.08] for y in [-.08,.08] for z in [0,.22]]);xyz=[];mins=[]
for v in qq:
 T=fk(v);xyz.append((T@np.array([0,0,.138,1]))[:3]);mins.append((corners@T[:3,:3].T+T[:3,3])[:,2].min())
xyz=np.array(xyz);speed=np.linalg.norm(np.diff(xyz,axis=0),axis=1)/np.diff(tt)
r=dict(hash=hashlib.sha256(data).hexdigest(),points=len(t),duration_s=float(t[-1]),samples=len(tt),spline_min_tcp_z_m=float(xyz[:,2].min()),spline_min_tool_z_m=float(min(mins)),table_clearance_m=float(min(mins)+.055),max_20ms_tcp_segment_speed_m_s=float(speed.max()),array_lengths_valid=all(len(a.velocities)==len(a.accelerations)==len(jt.joint_names) for a in jt.points),strict_times=bool(np.all(np.diff(t)>0)),finite=bool(np.isfinite(qq).all()),final_tcp_m=xyz[-1].tolist(),scope='Independent offline URDF FK of clamped cubic interpolation; does not replace scene collision check; sampled speed not continuous guarantee')
Path('/home/li/桌面/9月机械臂实验（算法）/reports/birdview_review_20260916/spline_review.json').write_text(json.dumps(r,indent=2));print(json.dumps(r,indent=2))
