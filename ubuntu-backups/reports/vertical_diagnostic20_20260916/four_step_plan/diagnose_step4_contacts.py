"""Offline hypothetical endpoint FK solve, then collision contacts only; never executable."""
import sys,json
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
P=Path(__file__).resolve().parent;sys.path.insert(0,str(P.parent))
from local_fk import fk,joints
from preflight_alignment import ReadOnly,load_config
from moveit_msgs.srv import GetStateValidity
import rclpy
plan=json.loads((P/'four_step_feasibility.json').read_text());row=plan['steps'][3];goal=np.array(row['target_m']);seed=np.array(row['start_joints_rad']);R=fk(np.rad2deg(plan['steps'][0]['start_joints_rad'])*1000)[:3,:3]
limits=[next(j for j in joints.values() if j.get('name')==f'joint{i}').find('limit') for i in range(1,7)];lo=[float(x.get('lower')) for x in limits];hi=[float(x.get('upper')) for x in limits]
def fun(q):
 T=fk(np.rad2deg(q)*1000);p=(T@np.array([0,0,.138,1]))[:3];return np.r_[p-goal,.1*Rotation.from_matrix(T[:3,:3]@R.T).as_rotvec()]
x=least_squares(fun,seed,bounds=(lo,hi),xtol=1e-12,ftol=1e-12,gtol=1e-12,max_nfev=100);res=fun(x.x);out={'diagnostic_only_not_executable':True,'requested_new_step':4,'fk_position_error_m':float(np.linalg.norm(res[:3])),'orientation_error_rad':float(np.linalg.norm(res[3:])/.1),'hypothetical_joints_rad':x.x.tolist()}
if out['fk_position_error_m']>1e-5 or out['orientation_error_rad']>1e-4:raise RuntimeError('hypothetical FK solve did not converge')
rclpy.init();n=ReadOnly(load_config(P.parent/'config.yaml'))
try:
 assert n.validity_client.wait_for_service(timeout_sec=5);n.apply_protection();req=GetStateValidity.Request();req.group_name=n.cfg['moveit']['group'];req.robot_state.is_diff=True;req.robot_state.joint_state.name=n.JOINT_NAMES;req.robot_state.joint_state.position=x.x.tolist();f=n.validity_client.call_async(req);rclpy.spin_until_future_complete(n,f,timeout_sec=5);r=f.result();assert r is not None;out['state_valid']=r.valid;out['contacts']=[{'body1':v.contact_body_1,'body2':v.contact_body_2,'depth_m':v.depth,'position_m':[v.position.x,v.position.y,v.position.z]} for v in r.contacts]
finally:(P/'step4_contacts.json').write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2));n.destroy_node();rclpy.shutdown()
