"""Read-only check of rm_control clamped global cubic spline; no execution."""
import json,copy,hashlib,time
from pathlib import Path
import numpy as np
from scipy.interpolate import CubicSpline
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.serialization import serialize_message,deserialize_message
from moveit_msgs.msg import RobotTrajectory,RobotState
from preflight_alignment import ReadOnly,load_config,P
from local_fk import fk,joints
cfg=load_config(P/'config.yaml');data=(P/'trajectory.cdr').read_bytes();(P/'reverse_before_spline.cdr').write_bytes(data);traj=deserialize_message(data,RobotTrajectory);jt=traj.joint_trajectory
q=np.array([p.positions for p in jt.points]);tt=np.array([p.time_from_start.sec+p.time_from_start.nanosec*1e-9 for p in jt.points]);assert np.all(np.diff(tt)>0)
for pt in jt.points:
 if len(pt.velocities)!=6 or len(pt.accelerations)!=6:raise RuntimeError('driver requires six velocities and accelerations')
def measure(tt):
 cs=CubicSpline(tt,q,axis=0,bc_type=((1,np.zeros(6)),(1,np.zeros(6))))
 times=np.unique(np.r_[np.arange(tt[0],tt[-1],.04),tt]);qq=cs(times)
 for i,name in enumerate(jt.joint_names):
  j=next(v for v in joints.values() if v.get('name')==name);lim=j.find('limit')
  if qq[:,i].min()<float(lim.get('lower')) or qq[:,i].max()>float(lim.get('upper')):raise RuntimeError('spline sampled joint limit exceeded')
 Ts=np.array([fk(np.rad2deg(qqi)*1000) for qqi in qq]);tcp=(Ts@np.array([0,0,.138,1]))[:,:3]
 speed=np.linalg.norm(np.diff(tcp,axis=0),axis=1)/np.diff(times)
 corners=np.array([[x,y,z,1] for x in [-.08,.08] for y in [-.08,.08] for z in [0,.22]])
 low=min(float((T@corners.T)[2].min()) for T in Ts)
 return times,qq,tcp,low,float(speed.max())
times,qq,tcp,low,v=measure(tt);stretch=max(1.,v/.008)*1.1
for pt,t in zip(jt.points,tt):
 ns=round(t*stretch*1e9);pt.time_from_start.sec=ns//1000000000;pt.time_from_start.nanosec=ns%1000000000;pt.velocities=[x/stretch for x in pt.velocities];pt.accelerations=[x/stretch**2 for x in pt.accelerations]
tt=np.array([p.time_from_start.sec+p.time_from_start.nanosec*1e-9 for p in jt.points]);times,qq,tcp,low,after=measure(tt)
if after>.008+1e-7:raise RuntimeError('sampled spline speed remains excessive')
rclpy.init();n=ReadOnly(cfg);out={'status':'STARTED','input_sha256':hashlib.sha256(data).hexdigest(),'method':'rm_control global clamped cubic spline, both endpoint first derivatives zero, sampled <=40ms plus original knots; not continuous proof','spline_stretch':stretch,'original_spline_max_segment_speed_m_s':v,'final_spline_max_segment_speed_m_s':after,'spline_min_tcp_z_m':float(tcp[:,2].min()),'spline_min_tool_box_z_m':low,'spline_tool_table_clearance_m':low+.055,'sample_count':len(times),'duration_s':float(tt[-1]),'joint_limits_sampled':True}
try:
 for c in [n.validity_client,n.scene_client]:
  if not c.wait_for_service(timeout_sec=5):raise RuntimeError('service missing')
 n.apply_protection()
 for i,qi in enumerate(qq):
  st=RobotState();st.joint_state.name=jt.joint_names;st.joint_state.position=qi.tolist();n.state_is_valid(st,'driver cubic sample')
 out['status']='PASS_SAMPLED_DRIVER_SPLINE';final=serialize_message(traj);(P/'trajectory.cdr').write_bytes(final);out['final_sha256']=hashlib.sha256(final).hexdigest()
 np.savez_compressed(P/'spline_samples.npz',time_s=times,joints_rad=qq,tcp_m=tcp)
except Exception as e:out['status']='FAILED';out['error']=str(e)
finally:(P/'spline_validation.json').write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2));n.destroy_node();rclpy.shutdown()
